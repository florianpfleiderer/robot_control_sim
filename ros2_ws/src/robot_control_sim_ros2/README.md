# robot_control_sim_ros2

ROS 2 Jazzy wrapper around the standalone `robot_sim_lib` controllers.

## Build (requires ROS 2 Jazzy)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch robot_control_sim_ros2 robot_sim.launch.py
```

Override the controller at runtime:

```bash
ros2 param set /controller_node controller_type pid     # or mpc4d / mpc6d
```

## Topics

| Topic                | Type                          | Direction          |
|----------------------|-------------------------------|--------------------|
| `/robot/true_pose`   | `geometry_msgs/PoseStamped`   | plant → *          |
| `/robot/true_twist`  | `geometry_msgs/TwistStamped`  | plant → *          |
| `/robot/measurement` | `geometry_msgs/PointStamped`  | sensor → estimator |
| `/robot/odom`        | `nav_msgs/Odometry`            | estimator → ctrl   |
| `/robot/disturbance` | `geometry_msgs/Vector3Stamped` | estimator → ctrl   |
| `/cmd_accel`         | `geometry_msgs/AccelStamped`   | controller → plant |

## Frames

- `map` — world frame, fixed.
- `odom` — static identity relative to `map` in v1.
- `base_link` — broadcast by `estimator_node` from `/odom` pose.

## Local development without ROS

ROS 2 cannot be installed on every dev machine. v2 of this feature ships a
Dockerfile that provides a Jazzy environment. Until then, the standalone CMake
project at the repo root (`cmake -S . -B build && cmake --build build`) remains
the verifiable surface — it compiles the same math library this package wraps.
