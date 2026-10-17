#!/bin/zsh
set -e
workspace_dir="${0:A:h:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "akit_26-03-15_16-37-51_njtab_q63.wpilog"
