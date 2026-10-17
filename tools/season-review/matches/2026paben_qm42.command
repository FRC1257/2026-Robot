#!/bin/zsh
set -e
workspace_dir="${0:A:h:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "akit_26-03-28_22-29-34_paben_q42.wpilog"
