"""Open a match in the configured AdvantageScope hub, using its AdvantageKit hook."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile
import argparse

ROOT=Path(__file__).resolve().parents[1]
APP=Path.home()/'Applications/AdvantageScope.app'
default='akit_26-03-29_13-40-03_paben_q48.wpilog'
parser=argparse.ArgumentParser()
parser.add_argument('name',nargs='?',default=default)
parser.add_argument('--layout',choices=['1257-season-review.json','1257-electrical-dashboard.json'],default='1257-season-review.json')
args=parser.parse_args();name=args.name
matches=[p for p in (ROOT/'logs').rglob('*.wpilog') if p.name==name]
if len(matches)!=1:
    raise SystemExit('Expected one exact log filename, found '+str(len(matches)))
log=matches[0]
running=subprocess.run(['pgrep','-f',str(APP/'Contents/MacOS/AdvantageScope')],capture_output=True).returncode==0
if running:
    # AdvantageScope documents this same local integration for AdvantageKit.
    (Path(tempfile.gettempdir())/'ascope-log-path.txt').write_text(str(log)+'\n')
    subprocess.run(['open','-a',str(APP)],check=True)
else:
    state=Path.home()/'Library/Application Support/AdvantageScope/state-26_0_2.json'
    if state.exists():shutil.copy2(state,ROOT/'analysis/last-session-layout.json')
    layout=json.loads((ROOT/'analysis'/args.layout).read_text());layout.pop('version',None)
    state.parent.mkdir(parents=True,exist_ok=True);state.write_text(json.dumps(layout))
    subprocess.run(['open','-a',str(APP),'--args',str(log)],check=True)
