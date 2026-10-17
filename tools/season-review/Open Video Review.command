#!/bin/zsh
cd -- "${0:A:h}"
.venv/bin/python scripts/serve_review.py 8826
