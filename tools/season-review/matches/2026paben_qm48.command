#!/bin/zsh
set -e
workspace_dir="${0:A:h:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "akit_26-03-29_13-40-03_paben_q48.wpilog"
