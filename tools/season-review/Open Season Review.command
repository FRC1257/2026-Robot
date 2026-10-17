#!/bin/zsh
set -e
workspace_dir="${0:A:h}"
exec "$workspace_dir/.venv/bin/python" "$workspace_dir/scripts/open_review.py" "$@"
