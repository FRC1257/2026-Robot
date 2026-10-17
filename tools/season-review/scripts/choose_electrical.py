"""Interactive local match picker for the electrical AdvantageScope dashboard."""
from pathlib import Path
import json
import subprocess
import sys
from electrical_dashboard import build,layout,ROOT

def main():
    rows=json.loads((ROOT/'analysis/log-analysis.json').read_text())
    choices=[r for r in rows if r.get('enabled_windows') and '/2026/' in r['file']]
    choices.sort(key=lambda r:r['file'])
    print('\n1257 electrical dashboard — recorded robot runs\n')
    for i,r in enumerate(choices,1):print(f'{i:2}. {Path(r["file"]).name}')
    default=next(i for i,r in enumerate(choices,1) if r['file'].endswith('_njtab_q11.wpilog'))
    answer=input(f'\nChoose a run [Enter = Q11, {default}]: ').strip()
    try:index=int(answer) if answer else default
    except ValueError:raise SystemExit('Enter a number from the list')
    if not 1<=index<=len(choices):raise SystemExit('Run number outside the list')
    source=ROOT/choices[index-1]['file']
    print('\nPreparing a viewing copy; the source log remains unchanged.',flush=True)
    target=build(source);layout(source)
    subprocess.run([sys.executable,str(ROOT/'scripts/open_review.py'),target.name,'--layout','1257-electrical-dashboard.json'],check=True)
    print('\nOpened in AdvantageScope. If another layout is active, import analysis/1257-electrical-dashboard.json once, then rerun this launcher.')

if __name__=='__main__':main()
