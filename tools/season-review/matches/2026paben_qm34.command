#!/bin/zsh
set -e
workspace_dir="${0:A:h:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "akit_26-03-28_21-18-11_paben_q34.wpilog"
