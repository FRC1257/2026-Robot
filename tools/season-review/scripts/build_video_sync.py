"""Build reusable, robust broadcast-clock alignments; do not infer missing gameplay."""
from pathlib import Path
import csv,json,statistics
ROOT=Path(__file__).resolve().parents[1]

def align(samples,records):
    # First sample of each integer avoids giving repeated held OCR values extra weight.
    video={};log={}
    for s in samples:
        c=s.get('clock')
        if c is not None and 3<=c<=18 and s['video_s']<23:video.setdefault(c,s['video_s'])
    for r in records:
        if r['auto'] and 3<=r['clock']<=18:log.setdefault(r['clock'],r['t_s'])
    anchors=[dict(clock=c,video_s=t,log_s=log[c],offset_s=log[c]-t) for c,t in video.items() if c in log]
    if len(anchors)<6:return None,anchors,[]
    mid=statistics.median(a['offset_s'] for a in anchors)
    retained=[a for a in anchors if abs(a['offset_s']-mid)<=1.0]
    if len(retained)<6:return None,anchors,[]
    offset=statistics.median(a['offset_s'] for a in retained)
    for a in anchors:a['accepted']=a in retained;a['residual_s']=round(a['offset_s']-offset,3)
    # Later countdown samples check drift independently. Integers describe a one-second
    # interval, so record distance to the nearest matching logged countdown interval.
    checks=[]
    for s in samples:
        if s['video_s']<60 or s.get('clock') is None:continue
        t=s['video_s']+offset
        candidates=[]
        for i,r in enumerate(records[:-1]):
            if not r['auto'] and r['clock']==s['clock']:
                end=records[i+1]['t_s']
                if end-r['t_s']<=2:candidates.append(max(r['t_s']-t,0,t-end))
        if candidates:checks.append(dict(video_s=s['video_s'],clock=s['clock'],nearest_interval_error_s=round(min(candidates),3)))
    return round(offset,3),anchors,checks

def main():
    matches=json.loads((ROOT/'analysis/match-review.json').read_text())['rows'];out=[]
    for m in matches:
        key=m['match'];p=ROOT/'analysis/video-sync'/f'{key}-clock.json'
        if not p.exists():continue
        receipt=json.loads(p.read_text());data=json.loads((ROOT/'analysis/video-sync/telemetry'/f'{key}.json').read_text())
        offset,anchors,checks=align(receipt['samples'],data['clock_records'])
        row={k:receipt[k] for k in ['match','url','video_id','title','publisher','duration_s','upload_date','file','video_sha256','recorded_at_utc','rights']}
        row.update(log_file=data['file'],log_sha256=data['sha256'],log_start_s=data['times'][0],log_end_s=data['times'][-1],
            alliance=m['alliance'],result=m['result'],alliance_score=m['alliance_score'],opponent_score=m['opponent_score'],
            offset_s=offset,status='clock_aligned' if offset is not None else 'pre_match_only_no_gameplay_alignment',
            formula='log_s = video_s + offset_s',uncertainty_s=1.0 if offset is not None else None,
            method='Median of first observed AUTO countdown integers 3–18; reject offsets >1s from median; 2Hz OCR, integer-clock quantization; not frame-exact',
            anchors=anchors,later_checks=checks,enabled_windows=data['enabled_windows'],decode_error=data['decode_error'])
        if offset is None:row['note']='Q31 contains no enabled gameplay and no decreasing AUTO countdown. Do not extrapolate this pre-match log into the video.'
        if checks and max(c['nearest_interval_error_s'] for c in checks)>1.5:row['review_flag']='Later clock disagreement; manually verify broadcast edits or OCR.'
        out.append(row)
    target=ROOT/'analysis/video-sync/manifest.json';target.write_text(json.dumps(dict(schema=1,rows=out),indent=2)+'\n')
    with (target.parent/'alignments.csv').open('w') as f:
        fields=['match','status','offset_s','uncertainty_s','log_file','url','alliance_score','opponent_score'];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(out)
    for x in out:print(x['match'],x['offset_s'],x['later_checks'],x.get('review_flag',''))
if __name__=='__main__':main()
