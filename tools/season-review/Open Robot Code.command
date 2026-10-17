#!/bin/zsh
set -e
workspace_dir="${0:A:h}"
exec "$HOME/wpilib/2026/vscode/Visual Studio Code.app/Contents/Resources/app/bin/code" "$workspace_dir/../.."
