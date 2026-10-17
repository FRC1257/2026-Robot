#!/bin/zsh
set -e
workspace_dir="${0:A:h}"
export JAVA_HOME="$HOME/wpilib/2026/jdk"
cd "$workspace_dir/../.."
exec ./gradlew simulateJava
