#!/bin/zsh
cd -- "${0:A:h}"
if [[ ! -f logs/focus/Q11-electrical-focus.wpilog ]]; then
  .venv/bin/python scripts/electrical_dashboard.py || exit 1
  .venv/bin/python scripts/focus_electrical.py || exit 1
fi
.venv/bin/python scripts/open_review.py Q11-electrical-focus.wpilog --layout 1257-electrical-dashboard.json
echo 'Video clip: videos/Q11_410-425_brownout.mov. Lock frame 1 to log time 410.000.'
