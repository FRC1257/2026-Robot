"""Trace module feedback across original logs; durations use held values, not record counts."""
from pathlib import Path
import json, struct, hashlib, bisect, math
from datetime import datetime, timezone
from wpiutil.log import DataLogReader
from analyze_logs import ROOT, value_at

MEASURED='/RealOutputs/SwerveStates/Measured'
COMMAND='/RealOutputs/SwerveStates/SetpointsOptimized'
ENABLED='/DriverStation/Enabled'

def intervals(data, end):
    """Drive1 zero-feedback exposure while enabled and another module exceeds 1 m/s."""
    keys=[ENABLED,MEASURED,COMMAND]
    times=sorted({t for k in keys for t,_ in data.get(k,[])} | {end})
    output=[]; active=None; exposure=0.; normal=0.; example=None
    for t,u in zip(times,times[1:]):
        m=value_at(data.get(MEASURED,[]),t); c=value_at(data.get(COMMAND,[]),t)
        on=value_at(data.get(ENABLED,[]),t)
        if not on or m is None or len(m)!=8:
            if active is not None:output.append([active,t]);active=None
            continue
        peers=max(abs(m[i]) for i in (0,4,6))
        if peers<=1:
            if active is not None:output.append([active,t]);active=None
            continue
        exposure+=u-t
        bad=abs(m[2])<.01
        if bad and active is None:active=t
        if not bad:
            normal+=u-t
            if active is not None:output.append([active,t]);active=None
        if example is None or (c is not None and abs(c[2])>abs(example.get('command_mps') or 0)):
            example={'t_s':t,'measured_mps':m[::2], 'command_mps':c[2] if c and len(c)==8 else None,
                     'module1_voltage':value_at(data.get('/Drive/Module1/DriveAppliedVolts',[]),t),
                     'module1_current':value_at(data.get('/Drive/Module1/DriveCurrentAmps',[]),t)}
    if active is not None:output.append([active,end])
    return {'peer_motion_enabled_s':exposure,'module1_nonzero_during_peer_motion_s':normal,
            'module1_zero_during_peer_motion_s':sum(b-a for a,b in output),
            'zero_intervals_s':[[round(a,6),round(b,6)] for a,b in output if b-a>=.5], 'example':example}

def scan(path, trace_path=None):
    reader=DataLogReader(str(path))
    out={'file':path.name,'sha256':hashlib.file_digest(path.open('rb'),'sha256').hexdigest(),'bytes':path.stat().st_size}
    if not reader.isValid():return {**out,'error':'Invalid WPILOG header'}
    names={};data={};meta={};end=0.;error=None
    for r in reader:
        try:
            if r.isStart():d=r.getStartData();names[d.entry]=(d.name,d.type)
            elif r.isFinish():names.pop(r.getFinishEntry(),None)
            elif not r.isControl():
                t=r.getTimestamp()/1e6;end=max(end,t);k,typ=names.get(r.getEntry(),('',''))
                if not (k.startswith('/Drive/Module') or k.startswith('/RealMetadata/') or k.startswith('/DriverStation/') or k in [MEASURED,COMMAND]):continue
                if typ=='double':v=r.getDouble()
                elif typ=='boolean':v=r.getBoolean()
                elif typ=='int64':v=r.getInteger()
                elif typ=='string':v=r.getString()
                elif k in [MEASURED,COMMAND] and len(r.getRaw())==64:v=struct.unpack('<8d',r.getRaw())
                else:continue
                if k.startswith('/RealMetadata/'):meta[k]=v
                else:data.setdefault(k,[]).append((t,v))
        except Exception as exc:error=str(exc);break
    for s in data.values():s.sort(key=lambda x:x[0])
    if trace_path is not None:
        trace_path.write_text(json.dumps({k:s for k,s in data.items() if k in [MEASURED,COMMAND,ENABLED] or k.startswith('/Drive/Module1/')},separators=(',',':'))+'\n')
    stats={k:{'records':len(s),'min':min(v for _,v in s),'max':max(v for _,v in s),'first_nonzero_s':next((t for t,v in s if abs(v)>.01),None)} for k,s in data.items() if k.startswith('/Drive/Module') and s and isinstance(s[0][1],(int,float))}
    return {**out,'decode_error':error,'metadata':meta,'driverstation':{k:sorted(set(v for _,v in s),key=str) for k,s in data.items() if k.startswith('/DriverStation/')},'module_stats':stats,**intervals(data,end)}

def main():
    out=ROOT/'analysis/electrical/swerve-history.json';results=[]
    for p in sorted((ROOT/'logs/2026').glob('*.wpilog')):
        row=scan(p);results.append(row)
        print(p.name,round(row.get('peer_motion_enabled_s',0),2),round(row.get('module1_zero_during_peer_motion_s',0),2),flush=True)
        out.write_text(json.dumps({'generated_utc':datetime.now(timezone.utc).isoformat(),'method':'Original logs only. Held measured states during DS Enabled, any other module speed >1 m/s. Zero means |Module1 measured speed| <0.01 m/s. Intervals >=0.5s listed; total includes shorter intervals. Missing fields excluded, never treated as zero. Module1 to physical front-right mapping requires matching code revision; dirty build caveat.','logs':results},indent=2)+'\n')
if __name__=='__main__':main()
