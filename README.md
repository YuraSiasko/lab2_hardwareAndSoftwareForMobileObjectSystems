# Lab 2: Gazebo + ArduPilot SITL + ROS 2 (variant 23)

Flight to a target point and automatic landing.
Target: `x_target = x0 - 5.2`, `y_target = y0 - 0.8` (variant 23), tolerance 0.2 m.

## Layout
- `src/fpv_lab2/` - ROS 2 Python package (`flight_test` FSM + new `NAVIGATING` state)
- `scripts/start_sim.sh`, `scripts/stop_sim.sh` - start / stop the simulation
- `scripts/build_and_run.sh` - build the package and run the flight

## Run (Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, ArduPilot SITL in /opt/ardu_ws)
```bash
./scripts/start_sim.sh        # Gazebo + SITL + DDS agent
./scripts/build_and_run.sh    # colcon build + ros2 run fpv_lab2 flight_test
./scripts/stop_sim.sh
```
Parameters: `-p delta_x:=5.2 -p delta_y:=0.8 -p max_speed:=0.4 -p tolerance:=0.2`.

## Notes
- The drone is spawned at `x=6.0, y=-2.2` in the maze world (override with `SPAWN_X`, `SPAWN_Y`):
  at the default spawn point (0, 0) there is a 4 m high wall 1 m to the west that blocks the path to the target.
- The simulation runs at about 10-20 % real time under WSL2, so all timeouts are wall-clock and generous.
- Pre-arm check is retried, then a 60 s settling wait precedes arming; takeoff waits for the altitude.
- Velocity commands go to `/ap/v1/cmd_vel` with `frame_id: map` (ENU).
- Flight log of the reported run: `logs/flight_variant23.log`.
- Unit tests: `cd src/fpv_lab2 && python3 -m pytest test`.
