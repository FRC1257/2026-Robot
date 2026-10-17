"""Reproduce Q11 electrical findings without exporting radio authentication fields."""
from pathlib import Path
import hashlib,json,struct,bisect
from wpiutil.log import DataLogReader
from electrical_dashboard import ROOT,KEYS,at
from power_details import MOTOR_PREFIXES

def main():
    p=ROOT/'logs/2026/akit_26-03-14_17-33-19_njtab_q11.wpilog'
    names={};series={};radio=[];meta={};ticks=[]
    special={'/RealOutputs/SwerveStates/Measured','/RealOutputs/SwerveStates/SetpointsOptimized','/RealOutputs/LoggedRobot/FullCycleMS','/Timestamp'}
    for r in DataLogReader(str(p)):
        if r.isStart():d=r.getStartData();names[d.entry]=(d.name,d.type)
        elif not r.isControl():
            key,typ=names.get(r.getEntry(),('',''));t=r.getTimestamp()
            if key in ['/RealMetadata/GitSHA','/RealMetadata/GitDirty','/RealMetadata/BuildDate']:meta[key]=r.getString()
            if key=='/RadioStatus/Status' and 405e6<t<480e6:
                row={'t_s':t/1e6}
                try:
                    obj=json.loads(r.getString());net=obj.get('networkStatus6',{})
                    row.update({k:net.get(k) for k in ['isLinked','signalDbm','signalNoiseRatio','rxBytes','txBytes','connectionQuality']})
                except ValueError:row['payload_available']=False
                radio.append(row)
            if key not in KEYS.values() and key not in special and not key.startswith(MOTOR_PREFIXES):continue
            getter={'double':r.getDouble,'float':r.getFloat,'int64':r.getInteger,'boolean':r.getBoolean}.get(typ)
            if getter:value=getter()
            elif key=='/PowerDistribution/ChannelCurrent':value=r.getDoubleArray()
            elif key.startswith('/RealOutputs/SwerveStates/'):
                raw=r.getRaw();value=list(struct.unpack('<'+'d'*(len(raw)//8),raw))
            else:continue
            series.setdefault(key,[]).append((t,value))
    for rows in series.values():rows.sort(key=lambda x:x[0])
    module1={k:{'records':len(rows),'min':min(v for _,v in rows),'max':max(v for _,v in rows)} for k,rows in series.items() if k.startswith('/Drive/Module1/Drive') and rows and isinstance(rows[0][1],(float,int))}
    sample={key:at(series,key,416_000_000) for key in special if key.startswith('/RealOutputs/SwerveStates/')}
    timestamps=[t/1e6 for t,_ in series['/Timestamp'] if 410e6<t<480e6]
    loops=[v for t,v in series['/RealOutputs/LoggedRobot/FullCycleMS'] if 417e6<t<418e6]
    report={'source':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'code_metadata':meta,
      'module1_drive':module1,'swerve_at416_speed_angle_pairs':sample,
      'loop_onset_max_ms':max(loops),'max_cycle_timestamp_gap_410_480_s':max(b-a for a,b in zip(timestamps,timestamps[1:])),
      'raw_pdh_fault_changes':[[t/1e6,v] for t,v in series['/PowerDistribution/Faults']],
      'radio_allowlisted_status':radio,
      'window_410_425':{k:[[t/1e6,v] for t,v in rows if 410e6<=t<=425e6] for k,rows in series.items() if k!='/Timestamp'},
      'limitations':['Different devices sample asynchronously; held values are not simultaneous measurements','No exported raw radio JSON or authentication metadata','Swerve measured states reuse module feedback; they are not independent motion measurements','Code working tree was dirty; declared base revision is not exact deployed source']}
    out=ROOT/'analysis/electrical/q11-diagnosis-evidence.json';out.write_text(json.dumps(report,indent=2)+'\n');print(out)

if __name__=='__main__':main()
