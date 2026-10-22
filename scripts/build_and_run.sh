#!/usr/bin/env bash
# Syncs the project from Windows (D:) to the fast Linux filesystem, builds fpv_lab2 and runs the flight.
# Usage: ./scripts/build_and_run.sh [-p delta_x:=5.2 ...]   (ROS parameters are optional)
set -e
SRC="$(cd "$(dirname "$0")/.." && pwd)"
DST="$HOME/fpv_labs"
mkdir -p "$DST"
rsync -a -c --delete --exclude build --exclude install --exclude log --exclude .git --exclude __pycache__ "$SRC/" "$DST/"

source /opt/ros/humble/setup.bash
source /opt/ardu_ws/install/setup.bash
cd "$DST"
colcon build --symlink-install --packages-select fpv_lab2
source "$DST/install/setup.bash"

mkdir -p "$HOME/sim_logs"
ros2 run fpv_lab2 flight_test --ros-args "$@" 2>&1 | tee "$HOME/sim_logs/flight_$(date +%H%M%S).log"
