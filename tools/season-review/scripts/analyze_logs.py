"""Offline telemetry triage with WPILib's native DataLogReader (not gameplay review)."""
from pathlib import Path
import bisect
import csv
import hashlib
import json
import math
import struct
from wpiutil.log import DataLogReader

ROOT = Path(__file__).resolve().parents[1]
SIGNALS = {
    '/DriverStation/Enabled', '/DriverStation/Autonomous',
    '/DriverStation/EventName', '/DriverStation/MatchNumber', '/DriverStation/MatchType',
    '/DriverStation/FMSAttached', '/DriverStation/DSAttached', '/DriverStation/EmergencyStop',
    '/DriverStation/AllianceStation', '/DriverStation/ReplayNumber',
    '/SystemStats/BatteryVoltage', '/SystemStats/BrownedOut',
    '/SystemStats/CANBus/OffCount', '/SystemStats/CANBus/Utilization',
    '/RealOutputs/LoggedRobot/FullCycleMS', '/Drive/Gyro/Connected',
    '/RealOutputs/Odometry/Robot', '/RealOutputs/Odometry/Odometry',
    '/Flywheel/FlywheelAngularVelocity', '/RealOutputs/Flywheel/GoalVelocity',
    '/Flywheel/FlywheelLeaderAngularVelocity',
    '/RealOutputs/Shooter/Flywheel/isAtGoal', '/Kicker/KickerVoltage',
    '/HopperIntake/IntakeVoltage', '/Climb/AppliedVolts',
    '/RealOutputs/Vision/cam1/Connected', '/RealOutputs/Vision/cam2/Connected',
    '/RealOutputs/Vision/cam3/Connected',
}

def value_at(series, t, default=None):
    i = bisect.bisect_right(series, (t,))
    # Include updates exactly at t; tuples compare by length before their value.
    while i < len(series) and series[i][0] == t:
        i += 1
    return series[i-1][1] if i else default

def integrate(series, start, end, predicate):
    """Integrate held values; change-only logging must not be sample-counted."""
    points = [(start, value_at(series, start))]
    points += [(t,v) for t,v in series if start < t < end]
    points.append((end, None))
    return sum(b[0]-a[0] for a,b in zip(points,points[1:]) if a[1] is not None and predicate(a[1]))

def summarize_window(data, start, end):
    def duration(key, pred=bool):
        s=data.get(key,[])
        return round(integrate(s,start,end,pred),3) if s else None
    def samples(key):
        s=data.get(key,[]);v=value_at(s,start)
        return ([(start,v)] if v is not None else [])+[(t,v) for t,v in s if start<t<=end]
    battery=samples('/SystemStats/BatteryVoltage')
    cycle=samples('/RealOutputs/LoggedRobot/FullCycleMS')
    posekey=next((k for k in ['/RealOutputs/Odometry/Robot','/RealOutputs/Odometry/Odometry'] if k in data),None)
    poses=samples(posekey) if posekey else []
    distance=0.; jumps=[];max_speed=0.
    for (t,a),(u,b) in zip(poses,poses[1:]):
        d=math.hypot(b[0]-a[0],b[1]-a[1]);dt=u-t
        if dt>0 and d/dt>8:
            jumps.append({'t_s':round(u,3),'delta_m':round(d,3)})
        elif dt>0:
            distance+=d;max_speed=max(max_speed,d/dt)
    metadata={k.rsplit('/',1)[-1]:value_at(data.get(k,[]),end) for k in SIGNALS if k.startswith('/DriverStation/')}
    return dict(start_s=round(start,3),end_s=round(end,3),duration_s=round(end-start,3),
        metadata=metadata,autonomous_s=duration('/DriverStation/Autonomous'),
        battery_min_v=round(min(v for _,v in battery),3) if battery else None,
        battery_below_9v_s=duration('/SystemStats/BatteryVoltage',lambda v:v<9),
        browned_out_s=duration('/SystemStats/BrownedOut'),
        gyro_disconnected_s=duration('/Drive/Gyro/Connected',lambda v:not v),
        loop_over_20ms_s=duration('/RealOutputs/LoggedRobot/FullCycleMS',lambda v:v>20),
        loop_max_ms=round(max(v for _,v in cycle),3) if cycle else None,
        intake_commanded_s=duration('/HopperIntake/IntakeVoltage',lambda v:abs(v)>1),
        kicker_commanded_s=duration('/Kicker/KickerVoltage',lambda v:abs(v)>1),
        flywheel_over_100rad_s=duration('/Flywheel/FlywheelLeaderAngularVelocity' if '/Flywheel/FlywheelLeaderAngularVelocity' in data else '/Flywheel/FlywheelAngularVelocity',lambda v:abs(v)>100),
        climb_commanded_s=duration('/Climb/AppliedVolts',lambda v:abs(v)>1),
        camera_disconnected_s={str(i):duration(f'/RealOutputs/Vision/cam{i}/Connected',lambda v:not v) for i in [1,2,3]},
        pose_signal=posekey,estimated_pose_travel_m=round(distance,2) if poses else None,
        max_pose_speed_mps=round(max_speed,2) if poses else None,pose_jumps=jumps)

def analyze(path):
    path = path.resolve()
    reader=DataLogReader(str(path))
    if not reader.isValid():raise ValueError('Invalid WPILOG header')
    entries={};data={};metadata={};first=None;last=0.;records=0
    parse_error=None
    for record in reader:
        try:
            records+=1
            if record.isStart():
                s=record.getStartData();entries[s.entry]=(s.name,s.type)
            elif record.isFinish():entries.pop(record.getFinishEntry(),None)
            elif not record.isControl():
                t=record.getTimestamp()/1e6;first=t if first is None else min(first,t);last=max(last,t)
                name,typ=entries.get(record.getEntry(),('',''))
                if name not in SIGNALS and not name.startswith('/RealMetadata/'):continue
                if typ=='boolean':v=record.getBoolean()
                elif typ=='double':v=record.getDouble()
                elif typ=='float':v=record.getFloat()
                elif typ=='int64':v=record.getInteger()
                elif typ=='string':v=record.getString()
                elif typ=='struct:Pose2d' and len(record.getRaw())==24:v=struct.unpack('<ddd',record.getRaw())
                else:continue
                if isinstance(v,float) and not math.isfinite(v):continue
                if name.startswith('/RealMetadata/'):metadata[name]=v
                else:data.setdefault(name,[]).append((t,v))
        except Exception as exc:
            parse_error=str(exc)
            break  # Preserve the decoded prefix; do not resynchronize through damaged bytes.
    for series in data.values():series.sort(key=lambda item:item[0])
    windows=[];begin=None
    for t,v in data.get('/DriverStation/Enabled',[]):
        if v and begin is None:begin=t
        if not v and begin is not None:
            windows.append(summarize_window(data,begin,t));begin=None
    if begin is not None:windows.append(dict(summarize_window(data,begin,last),truncated_enabled_end=True))
    return dict(decode_error=parse_error,decoded_prefix_only=parse_error is not None,file=str(path.relative_to(ROOT)),sha256=hashlib.file_digest(path.open('rb'),'sha256').hexdigest(),
        bytes=path.stat().st_size,records=records,start_s=first,end_s=last,
        build_metadata=metadata,signals=sorted(data),enabled_windows=windows,
        driverstation_values={k:sorted(set(v for _,v in s),key=str) for k,s in data.items() if k.startswith('/DriverStation/')})

def main():
    results=[]
    # Focus and electrical viewing derivatives are not additional robot runs.
    for path in sorted((ROOT/'logs/2026').glob('*.wpilog')):
        try:result=analyze(path)
        except Exception as e:
            with path.open('rb') as f:
                digest=hashlib.file_digest(f,'sha256').hexdigest()
            result=dict(file=str(path.relative_to(ROOT)),sha256=digest,bytes=path.stat().st_size,error=str(e))
        results.append(result);print(path.name,len(result.get('enabled_windows',[])),result.get('error','ok'),flush=True)
    out=ROOT/'analysis';out.mkdir(exist_ok=True)
    (out/'log-analysis.json').write_text(json.dumps(results,indent=2)+'\n')
    rows=[]
    for result in results:
        for i,w in enumerate(result.get('enabled_windows',[])):
            rows.append(dict(file=result['file'],window=i,**{k:v for k,v in w.items() if not isinstance(v,(dict,list))},**w['metadata']))
    if rows:
        fields=list(dict.fromkeys(k for r in rows for k in r))
        with (out/'enabled-windows.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)

if __name__=='__main__':main()
