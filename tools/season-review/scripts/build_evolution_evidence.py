"""Retain immutable local Git evidence and existing log-derived summaries for season review."""
from pathlib import Path
import subprocess,json,hashlib
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parent.parent;OUT=ROOT/'provenance/software-evolution'
REVS=['83e550c','f085965','aa2546d','6b19b9d','928345b','6b3cf89','e1e36a0','eeaaa64','4fef0b4','e14861b','4e7c726','45b3940','ad5d9ba','1760063','f0e5fc8','4e45023','fd60dd9','7ea645a','01c9126','975d4a7','a10f56f','479d930','4b3c573','3189eff','9004231','4517e0c','90560c0','5083ee8','eaa6f81','192680a','cd7df74','4b7ed85']
FILES=['RobotContainer.java','util/autonomous/AutoChooser.java','subsystems/Shooter/Flywheel/Flywheel.java','subsystems/Shooter/Flywheel/FlywheelConstants.java','subsystems/Shooter/Flywheel/FlywheelIOSparkMax.java','subsystems/Hopper/HopperPivot/HopperPivot.java','subsystems/drive/Drive.java','subsystems/drive/ModuleIOSparkMax.java','subsystems/vision/VisionIOPhoton.java']
def git(*args):return subprocess.check_output(['git','-C',str(REPO),*args])
def main():
 OUT.mkdir(parents=True,exist_ok=True);now=datetime.now(timezone.utc).isoformat();receipts=[];commits=[]
 def retain(path,data,url,rev):
  target=OUT/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
  receipts.append({'path':str(target.relative_to(ROOT)),'url':url,'sha256':hashlib.sha256(data).hexdigest(),'retrieved_utc':now,'revision':rev,'method':'read local Git object; no claim of exact dirty deployed tree'})
 for short in REVS:
  rev=git('rev-parse',short).decode().strip();meta=git('show','-s','--format=%H%n%aI%n%cI%n%s',rev).decode().splitlines()
  commits.append(dict(zip(['sha','author_date','committer_date','subject'],meta)))
  retain(short+'.patch',git('show','--format=fuller','--no-ext-diff',rev),f'https://github.com/FRC1257/2026-Robot/commit/{rev}',rev)
 for short in ['f0e5fc8','4e45023','5083ee8','eaa6f81']:
  rev=git('rev-parse',short).decode().strip()
  for name in FILES:
   path='src/main/java/frc/robot/'+name
   retain(Path(short)/name,git('show',rev+':'+path),f'https://github.com/FRC1257/2026-Robot/blob/{rev}/{path}',rev)
 (OUT/'sources.json').write_text(json.dumps(receipts,indent=2)+'\n')
 (OUT/'commits.json').write_text(json.dumps(commits,indent=2)+'\n')
 retain(Path('2026-history.txt'),git('log','--all','--since=2026-01-01','--date=iso-strict','--format=%H %ad %s'), 'https://github.com/FRC1257/2026-Robot/commits/',None)
 (OUT/'sources.json').write_text(json.dumps(receipts,indent=2)+'\n')
 logs=json.loads((ROOT/'analysis/log-analysis.json').read_text());metrics=json.loads((ROOT/'analysis/season-metrics.json').read_text());index={r['file']:r for r in logs};rows=[]
 for r in metrics:
  if not r.get('windows') or not any(s in r['file'] for s in ['njtab_q','paben_q','paben_e']):continue
  src=index[r['file']];row={'file':r['file'],'sha256':r.get('sha256'),'build_metadata':src.get('build_metadata'),'decode_error':r.get('error'),'windows':r['windows']}
  rows.append(row)
 artifacts=['analysis/log-analysis.json','analysis/season-metrics.json','analysis/electrical/swerve-history.json']
 summary={'generated_utc':now,'source_artifacts':[{'path':p,'sha256':hashlib.sha256((ROOT/p).read_bytes()).hexdigest()} for p in artifacts], 'match_log_evidence':rows,'limitations':['Original match logs have dirty builds; committed source is not exact deployed code','Kicker command while readiness false does not count missed shots or establish that timeout caused the interval','Readiness is a logged tolerance predicate, not a ball outcome','Below 9 V is exploratory, not brownout detection','No logged outcomes after March 29; later branch commits are unvalidated by this archive']}
 (ROOT/'analysis/software-evolution-evidence.json').write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__':main()
