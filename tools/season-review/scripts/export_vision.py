"""Export recorded vision/estimated pose changes without inventing missing frames."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import struct
from wpiutil.log import DataLogReader

ROOT=Path(__file__).resolve().parents[1]

def export(path,out):
    digest=hashlib.file_digest(path.open('rb'),'sha256').hexdigest()
    summary=dict(file=str(path.relative_to(ROOT)),sha256=digest,records=0,unique_ids_by_camera={},error=None)
    reader=DataLogReader(str(path));entries={};ids={}
    if not reader.isValid():
        summary['error']='Invalid WPILOG header';return summary
    destination=out/(path.stem+'.jsonl.gz')
    with gzip.open(destination,'wt') as f:
        f.write(json.dumps(dict(schema='1257-recorded-vision-v1',source_sha256=digest,
            timestamp='raw FPGA seconds; WPILOG record time, not necessarily frame capture time',
            legacy_limit='CameraTargets/Targets3D are conditional cached outputs; not every detected frame',
            pose2d_order=['x_m','y_m','yaw_rad'],pose3d_order=['x_m','y_m','z_m','qw','qx','qy','qz']))+'\n')
        try:
            for record in reader:
                if record.isStart():
                    x=record.getStartData();entries[x.entry]=(x.name,x.type)
                elif record.isFinish():entries.pop(record.getFinishEntry(),None)
                elif not record.isControl():
                    key,typ=entries.get(record.getEntry(),('',''))
                    wanted=(key.startswith('/Vision/') or key.startswith('/RealOutputs/Vision/') or
                            key in ['/RealOutputs/Odometry/Robot','/RealOutputs/Odometry/Odometry','/RealOutputs/Odometry/EstimatedPose'])
                    if not wanted:continue
                    if typ=='int64[]':value=record.getIntegerArray()
                    elif typ=='double[]':value=record.getDoubleArray()
                    elif typ=='double':value=record.getDouble()
                    elif typ=='int64':value=record.getInteger()
                    elif typ=='boolean':value=record.getBoolean()
                    elif typ.startswith('struct:Pose2d') or typ.startswith('struct:Pose3d'):
                        size=3 if 'Pose2d' in typ else 7
                        raw=record.getRaw()
                        if len(raw)%(size*8):raise ValueError('Malformed pose payload')
                        value=[list(x) for x in struct.iter_unpack('<'+'d'*size,raw)]
                    elif typ.startswith('struct:Rotation2d'):
                        value=[x[0] for x in struct.iter_unpack('<d',record.getRaw())]
                    else:continue
                    # Keep exact values/ordering, including unknown tag IDs.
                    if key.startswith('/Vision/CameraTargets/') and typ=='int64[]':
                        ids.setdefault(key.rsplit('/',1)[-1],set()).update(value)
                    f.write(json.dumps(dict(t_s=record.getTimestamp()/1e6,key=key,type=typ,value=value),allow_nan=False)+'\n')
                    summary['records']+=1
        except Exception as exc:summary['error']=str(exc)+'; decoded prefix only'
    summary['unique_ids_by_camera']={k:sorted(v) for k,v in ids.items()}
    summary['export']=str(destination.relative_to(ROOT))
    return summary

def main():
    out=ROOT/'analysis/vision-records';out.mkdir(exist_ok=True)
    rows=[]
    for path in sorted((ROOT/'logs').rglob('*.wpilog')):
        row=export(path,out);rows.append(row);print(path.name,row['records'],row['error'] or 'ok',flush=True)
    (ROOT/'analysis/vision-export-index.json').write_text(json.dumps(rows,indent=2)+'\n')
    with (ROOT/'analysis/tag-coverage.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['log','recorded_camera_index','unique_recorded_ids','exported_records','decode_error'])
        for row in rows:
            for camera,ids in row['unique_ids_by_camera'].items():w.writerow([row['file'],camera,' '.join(map(str,ids)),row['records'],row['error']])

if __name__=='__main__':main()
