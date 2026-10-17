"""Create a transparent focus excerpt, keeping original FPGA times and bytes."""
from pathlib import Path
from wpiutil.log import DataLogReader
import struct
import json,hashlib
ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'logs/2026/akit_26-03-14_17-33-19_njtab_q11.wpilog'
start,end=410_000_000,425_000_000
folder=ROOT/'logs/focus';folder.mkdir(exist_ok=True)
target=folder/'Q11_410-425_brownout.wpilog'
definitions={};starts={};held={};records=[]
for r in DataLogReader(str(source)):
    if r.isStart():
        d=r.getStartData();definitions[d.entry]=(d.name,d.type,d.metadata);starts[d.entry]=bytes(r.getRaw())
    elif not r.isControl():
        t=r.getTimestamp();i=r.getEntry()
        if t<start:held[i]=(t,bytes(r.getRaw()))
        elif t<=end:records.append((t,i,bytes(r.getRaw())))
# WPILOG record header: 4-byte entry ID, 4-byte payload size, 8-byte timestamp.
# Keep the source header (including AdvantageKit extra header) unchanged.
with source.open('rb') as original:
    header=original.read(12);extra=original.read(int.from_bytes(header[8:12],'little'))
with target.open('wb') as output:
    output.write(header+extra)
    def write_record(entry,payload,timestamp):
        output.write(struct.pack('<BIIQ',0x7f,entry,len(payload),timestamp));output.write(payload)
    for i,raw in starts.items():write_record(0,raw,start)
    for i,(t,raw) in held.items():write_record(i,raw,start)
    for t,i,raw in records:write_record(i,raw,t)
# Verify exact timestamp/payload preservation for every original in-window record.
ent={};actual=[];boot=[]
for r in DataLogReader(str(target)):
    if r.isStart():d=r.getStartData();ent[d.entry]=d.name
    elif not r.isControl():
        row=(r.getTimestamp(),ent[r.getEntry()],bytes(r.getRaw()))
        if r.getTimestamp()==start:boot.append(row)
        else:actual.append(row)
expected=[(t,definitions[i][0],raw) for t,i,raw in records if t!=start]
assert actual==expected,'Excerpt differs from source data'
receipt=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.file_digest(source.open('rb'),'sha256').hexdigest(),output=str(target.relative_to(ROOT)),output_sha256=hashlib.file_digest(target.open('rb'),'sha256').hexdigest(),range_s=[410,425],preserved_original_data_records=len(records),synthesized_held_state_records=len(held),verification='Every original data record strictly after410 through425 retains identical signal name, timestamp and raw payload, in original order.',video_source='videos/durFirbjol0.mp4',video_start_s=25.943,video_duration_s=15,video_offset_s=384.057,warning='Pre-window values are synthesized holds at410s; this is not a new robot run. Video alignment approximately±1s. Do not use AUTO-end detection on this teleop-only excerpt.')
(ROOT/'analysis/video-sync/q11-focus-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(target,len(records),'original records verified')
