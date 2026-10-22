#!/usr/bin/env bash
# Stops everything started by start_sim.sh (ROS 2 launch, Gazebo, SITL, DDS agent).
echo "[stop_sim] stopping ..."
pkill -f "fpv_lab2/flight_test" 2>/dev/null; pkill -f "flight_test --ros-args" 2>/dev/null
pkill -INT -f "ros2 launch ardupilot_gz_bringup" 2>/dev/null
sleep 3
for p in "gz sim" ruby arducopter micro_ros_agent mavproxy rviz2 robot_state_publisher parameter_bridge; do
  pkill -f "$p" 2>/dev/null
done
sleep 1
rm -f "$HOME/sim_logs/sim.pid"
echo "[stop_sim] left processes (should be empty):"
pgrep -fa "gz sim|arducopter|micro_ros_agent|ardupilot_gz" || echo "  none"
