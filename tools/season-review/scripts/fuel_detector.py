"""Generate a video-linked review queue. Candidate events are NOT counted balls."""
from pathlib import Path
import json,csv,bisect,statistics,hashlib,math
from datetime import datetime,timezone
from season_signals import collect,KEYS
from ball_activity_probe import sample,dips
ROOT=Path(__file__).resolve().parents[1]

def intake_events(volts,current,speed):
    result=[];last=-100
    for i in range(15,len(volts)-5):
        if i-last<15:continue
        if any(any(x is None or not math.isfinite(x) for x in (current[j],volts[j],speed[j])) for j in range(i-10,i+1)):continue
        if max(volts[i-10:i+1])>=-1:continue # forward direction from retained code
        baseline=statistics.median(current[i-10:i])
        if current[i]-baseline<8:continue
        previous=statistics.median(abs(s) for s in speed[i-10:i])
        if previous<10 or abs(speed[i])>previous*.8:continue
        result.append(i);last=i
    return result

def main():
    align=list(csv.DictReader((ROOT/'analysis/video-sync/alignments.csv').open()))
    logs=json.loads((ROOT/'analysis/season-metrics.json').read_text());events=[];coverage=[]
    for log in logs:
        if not log.get('windows') or not any(k in log['file'] for k in ['njtab_q','paben_q','paben_e']):continue
        path=ROOT/log['file'];series,error=collect(path)
        match=next((a for a in align if a['log_file']==log['file']),None)
        file_hash=hashlib.sha256(path.read_bytes()).hexdigest()
        total={'file':log['file'],'sha256':file_hash,'error':error,'candidate_launches':0,'candidate_intake_contacts':0}
        for w in log['windows']:
            start,end=w['start_s'],w['end_s'];times=[start+i*.02 for i in range(int((end-start)/.02))]
            fields=['flywheel','flywheel_goal','kicker','battery','intake','intake_current','intake_speed','kicker_current','pose']
            v={k:sample(series.get(k,[]),times) for k in fields}
            shots=dips(v['flywheel'],v['flywheel_goal'],v['kicker'],v['battery'],10,False)
            stable=set(dips(v['flywheel'],v['flywheel_goal'],v['kicker'],v['battery'],10,True))
            contacts=intake_events(v['intake'],v['intake_current'],v['intake_speed'])
            for kind,indices in [('launch',shots),('intake_contact',contacts)]:
                for i in indices:
                    t=times[i];offset=float(match['offset_s']) if match and match['offset_s'] else None
                    video=t-offset if offset is not None else None
                    reasons=[]
                    if kind=='launch' and i not in stable:reasons.append('voltage-confounded')
                    if kind=='intake_contact':reasons.append('contact-or-jam-not-confirmed-acquisition')
                    if abs(v['kicker'][i] or 0)>1 and kind=='intake_contact':reasons.append('simultaneous-feeding')
                    before=[x for x in v['kicker_current'][max(0,i-5):i] if x is not None]
                    cur=v['kicker_current'][i]
                    corroborated=bool(before and cur is not None and cur-statistics.median(before)>3)
                    event={'id':file_hash[:12]+f'-{kind}-{t:.3f}','kind':kind,'match':match['match'] if match else Path(log['file']).stem,'phase':'auto' if w['autonomous_s'] else 'teleop','log_s':round(t,3),'video_s':round(video,3) if video is not None else None,'video_id':match['url'].split('v=')[-1] if match else None,'source_url':match['url'] if match else None,'alignment_uncertainty_s':float(match['uncertainty_s']) if match and match['uncertainty_s'] else None,'battery_v':v['battery'][i],'pose_estimate':v['pose'][i],'flywheel_rad_s':v['flywheel'][i],'kicker_current_corroboration':corroborated,'warnings':reasons,'status':'unreviewed','ball_count':None}
                    events.append(event)
            total['candidate_launches']+=len(shots);total['candidate_intake_contacts']+=len(contacts)
        coverage.append(total)
    out=ROOT/'analysis/fuel-detector';out.mkdir(parents=True,exist_ok=True)
    data={'generated_utc':datetime.now(timezone.utc).isoformat(),'calibrated':False,'ball_count_estimate':None,'method':'Launch: 10rad/s recovered speed drops while feeding, stable goal; 160ms separation. Intake: forward voltage continuously >1V magnitude for200ms, >=8A rise over previous200ms median and >=20% speed reduction;300ms separation. Thresholds uncalibrated. Held20ms grid. Events may merge balls or count oscillation, jams or contact; no precision/recall claims. Pose is estimator output, not ground truth.','coverage':coverage,'events':events}
    (out/'events.json').write_text(json.dumps(data,indent=2))
    print(json.dumps({'logs':len(coverage),'events':len(events),'candidate_launches':sum(r['candidate_launches'] for r in coverage),'candidate_intake_contacts':sum(r['candidate_intake_contacts'] for r in coverage)}))
if __name__=='__main__':main()
