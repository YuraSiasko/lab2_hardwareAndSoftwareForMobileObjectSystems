#!/usr/bin/env bash
# Starts ArduPilot SITL + Gazebo (maze world) + ROS 2 bridge + DDS agent in the background.
# Usage:  ./scripts/start_sim.sh          (waits until /ap/v1 services appear)
source /opt/ros/humble/setup.bash
source /opt/ardu_ws/install/setup.bash
export GZ_VERSION=harmonic
export PATH="$HOME/.local/bin:$PATH"   # mavproxy.py lives here
# libsdformat12 (used by sdformat_urdf on Humble) resolves model:// through SDF_PATH
export SDF_PATH="$SDF_PATH:/opt/ardu_ws/install/ardupilot_gz_description/share/ardupilot_gz_description/models"

LOGDIR="$HOME/sim_logs"; mkdir -p "$LOGDIR"

if pgrep -f "iris_maze.launch.py" >/dev/null; then
  echo "Simulation already running. Use ./scripts/stop_sim.sh first."; exit 1
fi

# RViz is not needed for the lab and uses a lot of RAM: disable it if the launch file allows
# Spawn point inside a free corridor of the maze (the straight path to the
# target point must not cross the 4 m high walls). Override: SPAWN_X / SPAWN_Y.
ARGS="x:=${SPAWN_X:-6.0} y:=${SPAWN_Y:--2.2}"
if ros2 launch ardupilot_gz_bringup iris_maze.launch.py --show-args 2>/dev/null | grep -q "rviz"; then
  ARGS="$ARGS rviz:=false"
fi

echo "[start_sim] launching iris_maze $ARGS ..."
nohup ros2 launch ardupilot_gz_bringup iris_maze.launch.py $ARGS > "$LOGDIR/sim.log" 2>&1 &
echo $! > "$LOGDIR/sim.pid"

echo "[start_sim] waiting for ArduPilot ROS 2 services (up to 120 s) ..."
for i in $(seq 1 60); do
  if pgrep -x arducopter >/dev/null && ros2 service list 2>/dev/null | grep -q "/ap/v1/arm_motors"; then
    echo "[start_sim] ready. Log: $LOGDIR/sim.log"; exit 0
  fi
  sleep 2
done
echo "[start_sim] services did not appear, see $LOGDIR/sim.log"; exit 2
