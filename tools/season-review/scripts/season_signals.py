"""Extract held telemetry, clock anchors and bounded season metrics from supplied logs."""
from pathlib import Path
import bisect
import json
import math
import struct
from wpiutil.log import DataLogReader
from analyze_logs import value_at, integrate

ROOT=Path(__file__).resolve().parents[1]
KEYS={
 'enabled':'/DriverStation/Enabled','auto':'/DriverStation/Autonomous',
 'ds':'/DriverStation/DSAttached','fms':'/DriverStation/FMSAttached',
 'clock':'/DriverStation/MatchTime','game_message':'/DriverStation/GameSpecificMessage',
 'battery':'/SystemStats/BatteryVoltage','brownout':'/SystemStats/BrownedOut',
 'current':'/PowerDistribution/TotalCurrent',
 'loop_ms':'/RealOutputs/LoggedRobot/FullCycleMS',
 'pose':'/RealOutputs/Odometry/Robot',
 'odometry':'/RealOutputs/Odometry/Odometry',
 'flywheel':'/Flywheel/FlywheelLeaderAngularVelocity',
 'flywheel_old':'/Flywheel/FlywheelAngularVelocity',
 'flywheel_goal':'/RealOutputs/Flywheel/GoalVelocity',
 'flywheel_ready':'/RealOutputs/Shooter/Flywheel/isAtGoal',
 'flywheel_connected':'/Flywheel/FlywheelLeaderConnected',
 'hood_ready':'/RealOutputs/Shooter/Hood/atGoal',
 'hood':'/Hood/HoodAngle','hood_goal':'/RealOutputs/Hood/goal',
 'kicker':'/Kicker/KickerVoltage','kicker_velocity':'/Kicker/KickerAngularVelocity',
 'intake':'/HopperIntake/IntakeVoltage','floor':'/ActiveFloor/ActiveFloorVoltage',
 'camera1':'/RealOutputs/Vision/cam1/Connected','camera2':'/RealOutputs/Vision/cam2/Connected',
 'camera3':'/RealOutputs/Vision/cam3/Connected',
 'vision_valid':'/Vision/HasEstimate','vision_timestamps':'/Vision/TimestampArray',
 'joy0':'/DriverStation/Joystick0/AxisValues','joy1':'/DriverStation/Joystick1/AxisValues',
 'buttons0':'/DriverStation/Joystick0/ButtonValues',
 'buttons1':'/DriverStation/Joystick1/ButtonValues',
 'driver':'/NetworkInputs/SmartDashboard/Driver Selection',
 'operator':'/NetworkInputs/SmartDashboard/Operator Selection',
 'auto_choice':'/NetworkInputs/SmartDashboard/Starting Position',
 'drive0':'/Drive/Module0/DriveAppliedVolts','drive1':'/Drive/Module1/DriveAppliedVolts',
 'drive2':'/Drive/Module2/DriveAppliedVolts','drive3':'/Drive/Module3/DriveAppliedVolts',
 'gyro':'/Drive/Gyro/Connected',
 'can_util':'/SystemStats/CANBus/Utilization',
 'can_tx_errors':'/SystemStats/CANBus/TransmitErrorCount',
 'can_rx_errors':'/SystemStats/CANBus/ReceiveErrorCount',
 'can_tx_full':'/SystemStats/CANBus/TxFullCount',
 'hood_connected':'/Hood/HoodConnected',
 'intake_connected':'/HopperIntake/IntakeConnected',
 'kicker_connected':'/Kicker/KickerConnected',
 'floor_connected':'/ActiveFloor/ActiveFloorConnected',
}

def collect(path):
    inverse={v:k for k,v in KEYS.items()};entries={};series={};error=None
    reader=DataLogReader(str(path))
    if not reader.isValid():return {},'Invalid WPILOG header'
    try:
        for r in reader:
            if r.isStart():
                s=r.getStartData();entries[s.entry]=(s.name,s.type)
            elif r.isFinish():entries.pop(r.getFinishEntry(),None)
            elif not r.isControl() and r.getEntry() in entries:
                name,typ=entries[r.getEntry()]
                if name not in inverse:continue
                if typ=='boolean':v=r.getBoolean()
                elif typ=='double':v=r.getDouble()
                elif typ=='float':v=r.getFloat()
                elif typ=='int64':v=r.getInteger()
                elif typ=='string':v=r.getString()
                elif typ=='float[]':v=r.getFloatArray()
                elif typ=='double[]':v=r.getDoubleArray()
                elif typ=='struct:Pose2d':v=list(struct.unpack('<ddd',r.getRaw()))
                else:continue
                if isinstance(v,float) and not math.isfinite(v):continue
                series.setdefault(inverse[name],[]).append((r.getTimestamp()/1e6,v))
    except Exception as e:error=str(e)
    for s in series.values():s.sort(key=lambda v:v[0])
    if 'flywheel' not in series and 'flywheel_old'in series:series['flywheel']=series['flywheel_old']
    return series,error

def held_condition(series,keys,start,end,predicate):
    if any(k not in series for k in keys):return None
    boundaries=sorted({start,end}|{t for k in keys for t,_ in series[k] if start<t<end})
    total=0.
    for a,b in zip(boundaries,boundaries[1:]):
        values=[value_at(series[k],a) for k in keys]
        if all(v is not None for v in values) and predicate(*values):total+=b-a
    return round(total,3)

def summarize(series,windows):
    out=[]
    for w in windows:
        a,b=w['start_s'],w['end_s'];d=dict(w)
        d['kicker_active_flywheel_not_ready_s']=held_condition(series,['kicker','flywheel_ready'],a,b,lambda k,r:abs(k)>1 and not r)
        d['kicker_active_hood_not_ready_s']=held_condition(series,['kicker','hood_ready'],a,b,lambda k,r:abs(k)>1 and not r)
        d['driver_translation_input_s']=held_condition(series,['joy0'],a,b,lambda x:len(x)>1 and math.hypot(x[0],x[1])>.1)
        d['drive_command_s']=held_condition(series,['drive0','drive1','drive2','drive3'],a,b,lambda *v:max(abs(x) for x in v)>1)
        d['fms_attached_s']=held_condition(series,['fms'],a,b,bool)
        d['ds_attached_s']=held_condition(series,['ds'],a,b,bool)
        d['disabled_by_ds_s']=held_condition(series,['ds'],a,b,lambda x:not x)
        d['can_silent_s']=held_condition(series,['can_util'],a,b,lambda x:x==0)
        d['flywheel_disconnected_s']=held_condition(series,['flywheel_connected'],a,b,lambda x:not x)
        out.append(d)
    return out

def compact(series,start,end,step=.1):
    keys=list(series);times=[round(start+i*step,3) for i in range(math.floor((end-start)/step)+1)]
    # Binary search by timestamp only; values may be arrays or strings.
    timestamps={k:[x[0] for x in series[k]] for k in keys}
    values={}
    for k in keys:
        vals=[]
        for t in times:
            i=bisect.bisect_right(timestamps[k],t)-1
            v=series[k][i][1] if i>=0 else None
            if isinstance(v,float):v=round(v,5)
            elif isinstance(v,(list,tuple)):v=[round(x,5) for x in v]
            vals.append(v)
        values[k]=vals
    return dict(step_s=step,times=times,values=values)

def main():
    logs=json.loads((ROOT/'analysis/log-analysis.json').read_text())
    matches=json.loads((ROOT/'analysis/match-review.json').read_text())['rows']
    matched={f:m for m in matches for f in m['logs']}
    out=ROOT/'analysis/video-sync/telemetry';out.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for log in logs:
        file=log['file'];path=ROOT/file
        if log.get('error'):
            summaries.append(dict(file=file,error=log['error']));continue
        series,error=collect(path)
        windows=summarize(series,log.get('enabled_windows',[]))
        result=dict(file=file,sha256=log['sha256'],error=error,windows=windows,
            choices={k:sorted(set(v for _,v in series.get(k,[]))) for k in ['driver','operator','auto_choice','game_message']})
        summaries.append(result)
        if file in matched:
            match=matched[file];key=match['match']
            # Include pre-match clock updates and the whole recorded interval;
            # a static/disabled match still gets an honest player and clock data.
            data=compact(series,log['start_s'],log['end_s'])
            data.update(match=key,file=file,sha256=log['sha256'],signals=KEYS,
                clock_records=[dict(t_s=t,clock=v,auto=value_at(series.get('auto',[]),t)) for t,v in series.get('clock',[])],
                enabled_windows=windows,decode_error=error,choices=result['choices'])
            (out/(key+'.json')).write_text(json.dumps(data,separators=(',',':'))+'\n')
        print(path.name,len(windows),error or 'ok',flush=True)
    (ROOT/'analysis/season-metrics.json').write_text(json.dumps(summaries,indent=2)+'\n')

if __name__=='__main__':main()
