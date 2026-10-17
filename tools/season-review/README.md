# 2026 season review workspace

Contains the AdvantageScope 26 layouts, original and configured illustrative robot/camera-rig GLBs, analysis and detector scripts, tests, reports and retained Git evidence. The model is not fabrication CAD. Camera transforms are code-derived; physical calibration remains required.

## Setup

From this directory create a Python 3.12 virtual environment and install `requirements.txt`. Place original team logs in `logs/2026/`; do not upload raw logs without sanitizing radio credentials. Import `analysis/1257-electrical-dashboard.json` in AdvantageScope 26 and install the directories under `models/configured/` as custom assets. `scripts/configure_model.py` reproduces the configured model. On macOS `scripts/open_review.py` opens an exact log filename; on other systems open logs manually.

Run `python -m unittest discover -s scripts -p 'test_*.py'`. Analyses read the preserved original logs; they are not bundled. `scripts/ball_activity_probe.py` produces candidate disturbances, not calibrated ball counts. The plan and acceptance tests are under `analysis/programming-plan/`.

Full broadcasts, original WPILOGs, installed WPILib/AdvantageScope binaries, virtual environments, temporary sessions and credentials are intentionally local. Video source/alignment manifests identify the recordings needed for local review. Original receipt hashes refer to original local source bytes; machine-path/personal-choice redaction means bundled summary bytes may differ. Run-specific absolute paths must be configured locally.

Public illustrated report: https://frc1257-2026-season-review.akashdub06.chatgpt.site

The scripts were developed against the archived season. Some historical reconstruction scripts intentionally read fixed Git revisions. No claim is made that old dirty deployment sources are fully recoverable. See report caveats before using PDH channels or candidate ball events as facts.

Current branch changes and release tests: `IMPLEMENTATION.md`. Fuel review instructions and calibration limits: `analysis/fuel-detector/README.md`. The delivered season HTML and authored report sources are retained under `reports/`; that report predates these implementation commits.
