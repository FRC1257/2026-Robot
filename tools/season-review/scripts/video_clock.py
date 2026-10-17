"""Read broadcast clocks locally, preserving OCR evidence for log synchronization."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import re
import statistics
import subprocess
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]

def parse_clock(text):
    m=re.search(r'([0-2])\s*:\s*([0-5][0-9])',text)
    return int(m[1])*60+int(m[2]) if m else None

def ocr(path):
    text=subprocess.check_output(['tesseract',str(path),'stdout','--psm','7','-c','tessedit_char_whitelist=0123456789:'],stderr=subprocess.DEVNULL,text=True).strip()
    return text,parse_clock(text)

def run(match):
    key=match['match'];vid=next(v['key'] for v in match['video_pointers'] if v['type']=='youtube')
    video=ROOT/'videos'/f'{vid}.mp4';info=ROOT/'videos'/f'{vid}.info.json'
    if not video.exists():return dict(match=key,status='video_missing')
    metadata=json.loads(info.read_text());folder=ROOT/'videos/clock-frames'/key;folder.mkdir(parents=True,exist_ok=True)
    if not (folder/'first-00090.png').exists():
        # Narrow crop excludes the small shift timer. Keep exact time sampling
        # metadata; ffmpeg's 2 Hz filter has at most 0.25 s rounding uncertainty.
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(video),'-t','45',
            '-vf','fps=2:round=near,crop=iw*0.12:ih*0.078:iw*0.44:ih*0.15,scale=450:-1',
            str(folder/'first-%05d.png')],check=True)
    paths=sorted(folder.glob('first-*.png'))
    with ThreadPoolExecutor(max_workers=4) as pool:read=list(pool.map(ocr,paths))
    samples=[dict(video_s=(int(p.stem.split('-')[-1])-1)/2,text=t,clock=v,image=str(p.relative_to(ROOT))) for p,(t,v) in zip(paths,read)]
    # Check the later teleop clock too, rather than trusting a single early anchor.
    for sec in [60,90,120,150,170]:
        if sec>=metadata['duration']:continue
        target=folder/f'check-{sec}.png'
        if not target.exists():subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss',str(sec),'-i',str(video),'-frames:v','1','-vf','crop=iw*0.12:ih*0.078:iw*0.44:ih*0.15,scale=450:-1',str(target)],check=True)
        text,value=ocr(target);samples.append(dict(video_s=sec,text=text,clock=value,image=str(target.relative_to(ROOT))))
    receipt=dict(match=key,url=f'https://www.youtube.com/watch?v={vid}',video_id=vid,title=metadata.get('title'),publisher=metadata.get('uploader'),duration_s=metadata.get('duration'),upload_date=metadata.get('upload_date'),
        file=str(video.relative_to(ROOT)),video_sha256=hashlib.file_digest(video.open('rb'),'sha256').hexdigest(),
        recorded_at_utc=datetime.now(timezone.utc).isoformat(),rights='Local research copy; redistribution permission not established',
        ocr_engine=subprocess.check_output(['tesseract','--version'],text=True).splitlines()[0],samples=samples)
    (ROOT/'analysis/video-sync'/f'{key}-clock.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(key,receipt['title'],[(x['video_s'],x['clock']) for x in samples[:12]],flush=True)
    return receipt

def main():
    matches=json.loads((ROOT/'analysis/match-review.json').read_text())['rows']
    for match in matches:
        if '_qm' in match['match'] and match['logs']:run(match)

if __name__=='__main__':main()
