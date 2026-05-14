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

## ROS 2 (v1)

A ROS 2 Jazzy package wraps the same controllers and exposes them as nodes.
See [`ros2_ws/src/robot_control_sim_ros2/README.md`](ros2_ws/src/robot_control_sim_ros2/README.md).
Local execution requires ROS 2 Jazzy; v2 will provide a Docker environment
for machines without a native install.

TODO: add a `pybind` wrapper
