#!/bin/zsh
set -e
workspace_dir="${0:A:h:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "akit_26-03-29_19-37-09_paben_e5.wpilog"
