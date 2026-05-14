# Robot Control Sim

Lightweight C++ simulator of a PID-controlled robot in 2D. Includes a Kalman filter for smoothing noisy sensor readings.

## Model Predictive Controller

The `MPC` class (`include/mpc.h`) implements a linear-quadratic receding horizon
controller for the simulator's 4D double-integrator model. Configure it with a
sample time and horizon, optionally tune the `Q`/`R` weights and acceleration
limits, then call `computeControl` with the estimated state (e.g. from the
Kalman filter) to obtain optimal acceleration commands. The class is fully
self-contained and builds as part of `robot_sim_lib`.

Build:

```bash
cmake -S . -B build && cmake --build build
```

Binary: `bin/robot_sim`

## ROS 2

A ROS 2 Jazzy package wraps the same controllers and exposes them as nodes.
See [`ros2_ws/src/robot_control_sim_ros2/README.md`](ros2_ws/src/robot_control_sim_ros2/README.md).

### Docker

For machines without a native ROS 2 install, a Dockerfile + Makefile provide a
Jazzy environment with the workspace bind-mounted:

```bash
make image     # build the robot_control_sim:jazzy image
make ws        # colcon build inside the container
make shell     # interactive shell with ROS sourced
make launch    # run the full stack with RViz (X11 forwarded)
make clean     # remove ros2_ws build/install/log
```

The compose service in `docker/compose.yaml` forwards `$DISPLAY` and
mounts `/tmp/.X11-unix`; on first `make launch` you may need
`xhost +local:docker`.

TODO: add a `pybind` wrapper
