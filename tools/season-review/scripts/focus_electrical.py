"""Focus Q11's augmented log at original FPGA seconds 410–425, without rebasing."""
from pathlib import Path
import hashlib,json
from wpiutil.log import DataLogReader
from electrical_dashboard import ROOT,record

def main():
    source=ROOT/'logs/electrical/electrical_akit_26-03-14_17-33-19_njtab_q11.wpilog'
    target=ROOT/'logs/focus/Q11-electrical-focus.wpilog';target.parent.mkdir(exist_ok=True)
    starts={};held={};rows=[];first=410_000_000;last=425_000_000
    for r in DataLogReader(str(source)):
        raw=bytes(r.getRaw());t=r.getTimestamp()
        if r.isStart():starts[r.getStartData().entry]=raw
        elif not r.isControl():
            if t<first:held[r.getEntry()]=raw
            elif t<=last:rows.append((r.getEntry(),raw,t))
    with source.open('rb') as f:header=f.read(12);extra=f.read(int.from_bytes(header[8:12],'little'))
    with target.open('wb') as f:
        f.write(header+extra)
        for raw in starts.values():record(f,0,raw,first)
        for entry,raw in held.items():record(f,entry,raw,first)
        for entry,raw,t in rows:record(f,entry,raw,t)
    expected=hashlib.sha256();actual=hashlib.sha256()
    import struct
    for entry,raw,t in rows:
        if t>first:expected.update(struct.pack('<IQI',entry,t,len(raw))+raw)
    for r in DataLogReader(str(target)):
        if not r.isControl() and r.getTimestamp()>first:
            raw=bytes(r.getRaw());actual.update(struct.pack('<IQI',r.getEntry(),r.getTimestamp(),len(raw))+raw)
    assert expected.digest()==actual.digest()
    receipt=dict(source=str(source.relative_to(ROOT)),output=str(target.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),range_original_fpga_s=[410,425],timestamps_rebased=False,synthesized_initial_holds=len(held),preserved_in_window_records=len(rows),verified_record_digest=actual.hexdigest(),video='videos/Q11_410-425_brownout.mov',video_frame1_log_s=410,video_alignment_uncertainty_s=1)
    (ROOT/'analysis/electrical/q11-extended-focus.json').write_text(json.dumps(receipt,indent=2)+'\n');print(target)

if __name__=='__main__':main()
