"""Make the focus excerpt start at zero for native video locking; source time=+410s."""
from pathlib import Path
from wpiutil.log import DataLogReader
import struct,json,hashlib
ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'logs/focus/Q11_410-425_brownout.wpilog';target=source.with_name('Q11_brownout_focus_ZERO_EQUALS_410s.wpilog');base=410000000
names={};expected=[];count=0
with source.open('rb') as f:header=f.read(12);extra=f.read(int.from_bytes(header[8:12],'little'))
with target.open('wb') as out:
 out.write(header+extra)
 for r in DataLogReader(str(source)):
  if r.isStart():d=r.getStartData();names[d.entry]=d.name
  t=max(0,r.getTimestamp()-base);raw=bytes(r.getRaw());entry=r.getEntry()
  if not r.isControl() and names.get(entry)=='/Timestamp':raw=struct.pack('<q',max(0,r.getInteger()-base))
  out.write(struct.pack('<BIIQ',0x7f,entry,len(raw),t));out.write(raw)
  if not r.isControl():expected.append((t,names[entry],raw))
observed=[];entries={}
for r in DataLogReader(str(target)):
 if r.isStart():d=r.getStartData();entries[d.entry]=d.name
 elif not r.isControl():observed.append((r.getTimestamp(),entries[r.getEntry()],bytes(r.getRaw())))
assert observed==expected
p=ROOT/'analysis/video-sync/q11-focus-receipt.json';d=json.loads(p.read_text());d.update(relative_view_file=str(target.relative_to(ROOT)),relative_view_sha256=hashlib.file_digest(target.open('rb'),'sha256').hexdigest(),relative_view_rule='Original FPGA seconds = displayed seconds +410; record timestamps and AdvantageKit /Timestamp shifted by410s, all other signal payloads unchanged. Initial held /Timestamp set0.',native_video_file='videos/Q11_410-425_brownout.mov',native_video_start_log_s=0,relative_view_verified_records=len(expected));p.write_text(json.dumps(d,indent=2)+'\n');print(target)
