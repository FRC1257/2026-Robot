# Experimental fuel event detector

This is a motor-transient candidate detector plus a synchronized video review queue. **It does not yet estimate the number of balls acquired, launched, or scored.** `ball_count_estimate` stays null. The 13 available competition logs yield 572 possible launch signatures and 87 intake-contact signatures. These are neither upper nor lower bounds on balls.

## Reproduce and review

From the season-review directory, install requirements in a virtual environment and supply original logs at the paths in `analysis/season-metrics.json` (normally `logs/2026/`). Run:

```
python scripts/fuel_detector.py
python scripts/serve_review.py 8831 --no-browser
```

Open http://127.0.0.1:8831/review/fuel-detector.html. Supply local recordings as `videos/<video_id>.mp4` using the retained source URLs. Select a match and event kind; each candidate seeks two seconds before its estimated video time. Read alignment uncertainty and warnings. Label visible launches, visible pickups, non-ball disturbances, or uncertainty. Leave count blank when unresolved. Export labels before closing the page. Exported labels are not automatically treated as calibrated detector truth.

## Definitions and limitations

Launch candidates require flywheel speed above 100 rad/s, feeding voltage magnitude above 1 V, a stable goal, a drop of at least 10 rad/s relative to the preceding 100 ms peak and recovery by at least 5 rad/s within 400 ms. Candidates are separated by at least 160 ms. Voltage below 9 V or variation above 0.5 V in the nearby window is flagged. Removing confounded events may also remove real shots.

Intake candidates require continuously negative intake voltage below −1 V for the preceding 200 ms, a current rise of at least 8 A over the preceding median and at least a 20% speed reduction. They are separated by 300 ms. Negative voltage is the forward direction in the retained code. Startup transitions, reverse commands and missing/nonfinite samples are rejected. Contact with fuel, a jam, drag, or another disturbance can meet these criteria; more than one ball can cross per event.

The 20 ms grid holds the most recent logged values. Logger timestamps do not establish physical sensor freshness. Thresholds are engineering guesses, not measured ball signatures. The pose is the robot's estimate and can be wrong. Kicker-current corroboration is another uncalibrated clue, not confirmation.

## Q48 review

`video-observations.json` records two candidate windows reviewed as five extracted frames at 1 fps and a central full-resolution frame. The intake candidate at log 168.554 s / video 33.368 s has 9.12 V battery and essentially stationary flywheel; a contact/drag interpretation is plausible but not proven. The launch candidate at log 197.574 s / video 62.388 s has 301.75 rad/s flywheel, a kicker-current rise, and voltage confounding. Individual 1257 ball crossings and bumper identity were not sufficiently resolved in these wide broadcast samples. Both counts remain unknown. This is not continuous match review or evidence of scoring.

## Calibration acceptance plan

1. On unchanged hardware, film counted trials of 0, 1, 5, and 10 balls with empty intake, normal collection, jams, reversals, and simultaneous drive/shoot conditions. Log at the same rate; use a visible synchronization cue.
2. Annotate exact intake and exit crossings in continuous video, including intervals without candidates. Distinguish acquired, ejected, launched, and scored balls; record occlusion and initial inventory.
3. Split by whole match/trial, tune thresholds on training trials only, then match predicted events to held-out crossings within a declared timing tolerance. Measure precision, recall, per-match count error, missed bursts and false events during sag. Multiple-ball bursts need a separate counting model.
4. Publish a count estimate only after held-out validation, with uncertainty and coverage. Never substitute total alliance score for this robot's shot count.
