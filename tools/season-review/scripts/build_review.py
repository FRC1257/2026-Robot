"""Join offline robot telemetry to retained TBA snapshots; no individual scoring inference."""
from pathlib import Path
import csv
import json

ROOT=Path(__file__).resolve().parents[1]

def main():
    logs=json.loads((ROOT/'analysis/log-analysis.json').read_text())
    folder=ROOT/'provenance/tba/team/frc1257/matches/2026'
    receipt=json.loads((folder/'latest.json').read_text())
    matches=json.loads((folder/receipt['response_file']).read_text())
    mapped={}
    for log in logs:
        ds=log.get('driverstation_values',{})
        event=next((v.lower() for v in ds.get('/DriverStation/EventName',[]) if v),'')
        number=next((v for v in ds.get('/DriverStation/MatchNumber',[]) if v),None)
        typ=next((v for v in ds.get('/DriverStation/MatchType',[]) if v),None)
        if not event or number is None or typ not in [2,3]:continue
        # 2026 double-elimination logs enumerate playoff matches as eN.
        # Verify this mapping against the team's actual TBA lineup, not just a filename.
        key=f'2026{event}_'+(f'qm{number}' if typ==2 else f'sf{number}m1')
        if key in {m['key'] for m in matches}:mapped.setdefault(key,[]).append(log)
    observation_file=ROOT.parent/'research/observations/2026-1257/season-sampled-frames.json'
    sampled={}
    if observation_file.exists():
        for o in json.loads(observation_file.read_text()).get('observations',[]):
            sampled[o['match_key']]=sampled.get(o['match_key'],0)+1
    rows=[]
    for m in sorted(matches,key=lambda m:(m['event_key'],m.get('actual_time') or m.get('time') or 0)):
        alliance=next(k for k,a in m['alliances'].items() if 'frc1257' in a['team_keys'])
        other='red' if alliance=='blue' else 'blue'
        candidates=mapped.get(m['key'],[])
        # FMSAttached can fall false at a disable/disconnect boundary. Metadata is
        # sampled at window end, so do not discard that diagnostic enabled period.
        windows=[w for c in candidates for w in c.get('enabled_windows',[])
                 if w['metadata'].get('EventName','').lower()==m['event_key'][4:]
                 and True in c.get('driverstation_values',{}).get('/DriverStation/FMSAttached',[])]
        rows.append(dict(match=m['key'],alliance=alliance,alliance_score=m['alliances'][alliance]['score'],opponent_score=m['alliances'][other]['score'],result='tie' if not m['winning_alliance'] else 'win' if m['winning_alliance']==alliance else 'loss',logs=[c['file'] for c in candidates],enabled_s=round(sum(w['duration_s'] for w in windows),3),enabled_windows=windows,video_pointers=m.get('videos',[]),video_reviewed=False,full_match_reviewed=False,sampled_frames_reviewed=sampled.get(m['key'],0)))
    (ROOT/'analysis/match-review.json').write_text(json.dumps({'tba_source_id':receipt['source_id'],'rows':rows},indent=2)+'\n')
    with (ROOT/'analysis/match-coverage.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['match','alliance','alliance_score','opponent_score','result','enabled_seconds','log_files'])
        for r in rows:writer.writerow([r['match'],r['alliance'],r['alliance_score'],r['opponent_score'],r['result'],r['enabled_s'],';'.join(r['logs'])])
    empty=[l for l in logs if l.get('bytes')==0]
    partial=[l for l in logs if l.get('decode_error')]
    failures=[l for l in logs if 'error' in l]
    selected=['2026njtab_qm11','2026paben_qm48','2026paben_qm42','2026paben_qm34','2026paben_qm53','2026njtab_qm63','2026paben_sf1m1','2026paben_sf5m1']
    notes={
        '2026njtab_qm11':'Investigate the 4.04 V logged dip and the split enabled periods. Check DS connection, brownout, loop time and video before assigning a cause.',
        '2026paben_qm48':'Start here for sustained low-voltage review. Compare battery with current, drivetrain demand, and flywheel recovery. Alliance score is not a measurement of 1257 scoring.',
        '2026paben_qm42':'Contrast case: unusually flat battery voltage. Check whether the robot actually moves, whether motors receive commands, and whether telemetry is stale before calling this healthy operation.',
        '2026paben_qm34':'Winning alliance comparison with long enabled telemetry. Use it as a comparison for Q48 rather than treating the win as evidence of a particular mechanism advantage.',
        '2026paben_qm53':'Autonomous enabled period is shorter than the ~20 s segments in most other logs. Investigate enable/disable transitions; the log alone does not establish why.',
        '2026njtab_qm63':'Close alliance win (145–140) with limited enabled telemetry. Good video follow-up; do not equate the short log segment to the full match.',
        '2026paben_sf1m1':'Playoff comparison with a long teleop record. Compare pre-match battery, demand, and subsystem readiness against qualification runs.',
        '2026paben_sf5m1':'Short playoff enabled record. Determine whether logging stopped, the robot disconnected, or the robot was disabled; these remain competing hypotheses.',
    }
    lines=['# Team 1257 — 2026 season log review','',
        'Automated telemetry triage prepared October 16, 2026. AI review covers 18 paused frames across three qualifications; no full match has been continuously reviewed. See MATCH_WALKTHROUGH.md for synchronized videos, bounded observations and strategy/software recommendations.','',
        f'Inspected all {len(logs)} supplied files: {len(logs)-len(failures)-len(partial)} decoded without a reported exception, {len(partial)} retained decoded prefixes with a decode error, and {len(empty)} empty files. The native reader can also stop at truncated trailing records without raising; a successful decode does not certify byte-level completeness.',
        f'TBA lists {len(rows)} match appearances across two events; {sum(bool(r["enabled_windows"]) for r in rows)} have enabled telemetry in FMS-identified match logs in this archive. Other match logs are absent or contain only disabled data. Practice logs are analyzed separately in log-analysis.json.','',
        '## How to walk through a match','',
        '1. Double-click `../Open Season Review.command`. It opens Bensalem Q48 in AdvantageScope. To change matches while keeping the layout, use File → Open Log(s), or one of the launchers in `../matches/`.',
        '2. Start in **1 Match + Battery**. Blue is battery volts, orange is total current. The discrete rows mark Enabled, Autonomous, and BrownedOut. Click a time in the graph, then press Space to play/pause. Left/Right step by recorded cycle while paused. Right-click Play for slower playback.',
        '3. Switch to **2 Robot 3D** to see the same selected time on the 2026 field. The supplied approximate model and intake/hood component pose array are connected. The model is illustrative; its alignment is not CAD validated.',
        '4. Use **3 Field + Auto** for estimated robot pose versus the trajectory setpoint. Pose corrections and resets can look like robot jumps; they are not proof of physical motion.',
        '5. Use **4 Shooter** to compare measured angular velocity with its goal, readiness, and kicker command. Early-season logs use FlywheelAngularVelocity; later ones use FlywheelLeaderAngularVelocity. One may be absent. A spinning flywheel or kicker command is not a counted scored ball.',
        '6. Use **5 Intake + Hood** for hood/pivot angles and intake/kicker voltages, and **6 Timing + Sensors** for loop duration, CAN use, gyro and camera connection flags. Missing signals remain missing, not zero.',
        '7. Command+Left / Command+Right switch tabs. For another window or a changed layout, use File → Import Layout and choose `1257-season-review.json`. The layout is portable within AdvantageScope 26.0.2; the custom model must also be installed.','',
        '## Matches worth reviewing first','',
        'Times below are raw WPILOG timestamps in seconds (FPGA time since boot), not video time or match countdown. For these unmerged logs, AdvantageScope uses the recorded timestamp directly; use the raw times below. A multi-log merge or manual time alignment can change the displayed reference. Window boundaries may include scheduler/DS update latency.','']
    (ROOT/'matches').mkdir(exist_ok=True)
    for key in selected:
        r=next(x for x in rows if x['match']==key)
        lines += [f'### {key} — alliance {r["result"]}, {r["alliance_score"]}–{r["opponent_score"]}', '',notes[key],'']
        for file in r['logs']:
            l=next(l for l in logs if l['file']==file)
            lines += [f'Log: `{Path(file).name}`; source `wpilog:sha256:{l["sha256"]}`; first record **{l["start_s"]:.3f} s**.','']
            launcher=ROOT/'matches'/f'{key}.command'
            launcher.write_text('#!/bin/zsh\nset -e\nworkspace_dir="${0:A:h:h}"\nexec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "'+Path(file).name+'"\n');launcher.chmod(0o755)
        for w in r['enabled_windows']:
            lines.append(f'- Raw {w["start_s"]:.3f}–{w["end_s"]:.3f} s; enabled {w["duration_s"]:.3f} s; autonomous {w["autonomous_s"]} s; min battery {w["battery_min_v"]} V; below 9 V {w["battery_below_9v_s"]} s; logged brownout {w["browned_out_s"]} s; intake commanded {w["intake_commanded_s"]} s; kicker commanded {w["kicker_commanded_s"]} s.'+(' **Log ends while enabled.**' if w.get('truncated_enabled_end') else ''))
        lines+=['',f'[TBA match](https://www.thebluealliance.com/match/{key}) · '+(f'{sampled[key]} paused frames reviewed by AI; no continuous match review.' if key in sampled else 'video remains unviewed.'),'']
    lines += ['## Evidence and limits','',
        f'TBA result source: `{receipt["source_id"]}`, retrieved `{receipt["retrieved_at_utc"]}`; raw response and receipt under `../provenance/tba/team/frc1257/matches/2026/`. Scores are alliance outcomes. The playoff eN → sfNm1 mapping was checked against this team’s recorded lineup appearances.',
        'Raw ZIPs remain in Downloads; extracted originals remain in logs/ and models/. Each analyzed file has a SHA-256 and byte count in log-analysis.json, including empty/error files. Source archives have receipts in provenance/receipts.json.',
        'Enabled durations and below-threshold durations use sample-and-hold integration, not sample percentages. A 9 V threshold is an exploratory diagnostic choice, not the roboRIO brownout threshold. BrownedOut is reported separately. Missing signals return null. No extrapolation is made beyond the log end. Window metadata contains the values held at the window end: Enabled may read false, and FMSAttached may fall false at a disconnect. Such periods are retained when the event/match identity and log-level FMS evidence match.',
        'Odometry travel is an estimate; steps above 8 m/s are omitted and recorded as pose jumps. Smaller estimator corrections can still inflate travel, so do not rank driving quality using this number.',
        'Log build GitSHA/dirty metadata is retained. Current cloned source is a later revision and is not automatically the code that generated each log. Full-season coverage, motor physics, camera accuracy, exact geometry and actual ball counts are not established by this analysis.','',
        '## Decode gaps','']
    for l in failures+partial:lines.append(f'- `{Path(l["file"]).name}`: {l.get("error") or l.get("decode_error")}; {l["bytes"]} bytes. '+('Only its decoded prefix is usable.' if l.get('decoded_prefix_only') else 'No usable decoded record.'))
    (ROOT/'analysis/SEASON_REVIEW.md').write_text('\n'.join(lines)+'\n')

if __name__=='__main__':main()
