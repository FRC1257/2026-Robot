"""Exploratory load-transient counts, NOT ball counts; 20 ms held grid."""
import json, math, bisect, hashlib, statistics
from pathlib import Path
from datetime import datetime, timezone
from season_signals import collect, KEYS, held_condition
ROOT=Path(__file__).resolve().parents[1]
KEYS.update({'fly_current':'/Flywheel/FlywheelLeaderCurrent','fly_current_old':'/Flywheel/FlywheelCurrent','intake_current':'/HopperIntake/IntakeCurrent','intake_speed':'/HopperIntake/IntakeVelocity','kicker_current':'/Kicker/KickerCurrent'})
def sample(series,times):
 ts=[t for t,v in series];vs=[v for t,v in series]
 return [vs[i] if (i:=bisect.bisect_right(ts,t)-1)>=0 else None for t in times]
def dips(speed,goal,kicker,battery,threshold,guard=True):
 events=[];last=-100
 for i in range(10,len(speed)-20):
  if i-last<8:continue
  span=range(i-5,i+6)
  if any(speed[j] is None or goal[j] is None or kicker[j] is None or battery[j] is None for j in span):continue
  if speed[i]<100 or abs(kicker[i])<=1:continue
  if max(goal[j] for j in span)-min(goal[j] for j in span)>2:continue
  drop=max(speed[j] for j in range(i-5,i))-speed[i]
  if drop<threshold:continue
  if guard and (min(battery[j] for j in span)<9 or max(battery[j] for j in span)-min(battery[j] for j in span)>.5):continue
  # A recovered depression; reject sustained spin-down. Nearby balls can merge.
  future=[v for v in speed[i+1:i+21] if v is not None]
  if not future or max(future)<speed[i]+threshold*.5:continue
  events.append(i);last=i
 return events

def main():
 logs=json.loads((ROOT/'analysis/season-metrics.json').read_text());out=[]
 for log in logs:
  if not log.get('windows') or not any(s in log['file'] for s in ['njtab_q','paben_q','paben_e']):continue
  p=ROOT/log['file'];s,error=collect(p);row={'file':log['file'],'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'decode_error':error,'windows':[]}
  if 'fly_current' not in s and 'fly_current_old'in s:s['fly_current']=s['fly_current_old']
  for w in log['windows']:
   a,b=w['start_s'],w['end_s'];times=[a+i*.02 for i in range(int((b-a)/.02))]
   v={k:sample(s.get(k,[]),times) for k in ['flywheel','flywheel_goal','kicker','battery','intake_current','intake_speed','intake']}
   counts={str(th):len(dips(v['flywheel'],v['flywheel_goal'],v['kicker'],v['battery'],th)) for th in [5,10,20]}
   unguard={str(th):len(dips(v['flywheel'],v['flywheel_goal'],v['kicker'],v['battery'],th,False)) for th in [5,10,20]}
   candidates=dips(v['flywheel'],v['flywheel_goal'],v['kicker'],v['battery'],10)
   row['windows'].append({'phase':'auto' if w['autonomous_s']>0 else 'teleop','start_s':a,'end_s':b,'intake_active_s':w['intake_commanded_s'],'intake_negative_s':held_condition(s,['intake'],a,b,lambda x:x < -1),'intake_positive_s':held_condition(s,['intake'],a,b,lambda x:x > 1),'kicker_active_s':w['kicker_commanded_s'],'flywheel_and_feed_s':held_condition(s,['kicker','flywheel'],a,b,lambda k,f:abs(k)>1 and f>100),'candidate_dips_guarded':counts,'candidate_dips_without_voltage_guard':unguard,'candidate_10rad_timestamps':[round(times[i],3) for i in candidates],'intake_current_range_a':[min(q),max(q)] if (q:=[x for x in v['intake_current'] if x is not None]) else None,'intake_speed_range_rad_s':[min(q),max(q)] if (q:=[x for x in v['intake_speed'] if x is not None]) else None})
  out.append(row)
 result={'generated_utc':datetime.now(timezone.utc).isoformat(),'method':'20 ms held grid. Flywheel >100 rad/s, |kicker volts|>1, goal range <=2 rad/s over +/-100ms; speed drop from preceding100ms peak, recovery of half threshold within400ms;160ms refractory. Guard: battery>=9V and range<=0.5V over +/-100ms. Threshold sensitivity 5,10,20rad/s. No validated single-ball calibration. Neither precision nor recall known. Counts are NOT balls, bounds, or successful scores. Change-only fields are held; timestamps are logging timestamps, not certified sensor freshness. Intake current/speed availability checked; no calibrated intake detector.','logs':out}
 (ROOT/'analysis/programming-plan/ball-activity-probe.json').write_text(json.dumps(result,indent=2))
 for r in out:
  for w in r['windows']:
   print(Path(r['file']).name,w['phase'],w['intake_active_s'],w['kicker_active_s'],w['flywheel_and_feed_s'],w['candidate_dips_guarded'],w['candidate_dips_without_voltage_guard'])
if __name__=='__main__':main()
