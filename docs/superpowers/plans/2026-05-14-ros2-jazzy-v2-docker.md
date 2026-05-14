# ROS 2 Jazzy Integration v2 — Docker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a reproducible ROS 2 Jazzy environment via Docker so the v1 package can actually be built and run on machines without a native ROS install — *and* use that environment to verify v1 code compiles, fix anything that breaks, and close the documented v1 carry-over (MPC6D disturbance state lost across `/odom`).

**Architecture:** A single `docker/Dockerfile` based on `osrf/ros:jazzy-desktop` (includes RViz) builds a non-root user matching the host UID/GID so colcon artifacts on the bind mount stay user-owned. `docker/compose.yaml` bind-mounts the repo at `/workspace` and forwards the host X server for RViz. A `Makefile` wraps the common verbs (`build`, `shell`, `launch`, `test`, `clean`). Verification is now real: `colcon build` and `ros2 launch` actually run.

**Tech Stack:** Docker 29+, docker compose v2, `osrf/ros:jazzy-desktop`, X11 forwarding, GNU Make.

**Why this matters now:** v1 shipped code that has never been compiled. v2's first concrete deliverable is "does it compile?" — any errors found are fixed inline as part of this PR.

---

## File Structure

**New:**
- `docker/Dockerfile` — Jazzy image, non-root user, pre-installed deps.
- `docker/compose.yaml` — bind-mount + X11 + entrypoint.
- `docker/entrypoint.sh` — sources ROS, exec's whatever command is passed.
- `Makefile` — `build` / `shell` / `launch` / `test` / `clean`.
- `.dockerignore` — keep image lean.
- `ros2_ws/src/robot_control_sim_ros2/test/test_launch.py` — launch_testing smoke test that asserts all four nodes come up.
- `docs/superpowers/plans/2026-05-14-ros2-jazzy-v2-docker.md` — this plan.

**Modify (only if `colcon build` fails on v1 code):**
- `ros2_ws/src/robot_control_sim_ros2/...` — fixes are commit-by-commit, each labelled `fix(ros2): ...`.

**Modify (MPC6D disturbance carry-over):**
- `ros2_ws/src/robot_control_sim_ros2/src/estimator_node.cpp` — publish `/robot/disturbance`.
- `ros2_ws/src/robot_control_sim_ros2/src/controller_node.cpp` — subscribe to `/robot/disturbance`, feed into MPC6D state.
- `ros2_ws/src/robot_control_sim_ros2/README.md` — update topic table.
- `ros2_ws/src/robot_control_sim_ros2/package.xml` — already depends on `geometry_msgs`; no change needed (`Vector3Stamped` is in it).

**Modify (docs):**
- `README.md` (root) — Docker usage section.

---

## Task 0: Sync main and create branch

- [x] **Step 1: Already done** — on `feat/ros2-jazzy-v2-docker` branched from latest `main` (post v1 merge).

---

## Task 1: `.dockerignore`

**Files:** Create `.dockerignore`.

- [ ] **Step 1: Write `.dockerignore`**

```gitignore
.git
.vscode
build
ros2_ws/build
ros2_ws/install
ros2_ws/log
**/__pycache__
*.pyc
```

- [ ] **Step 2: Commit**

```bash
git add .dockerignore
git commit -m "chore(docker): add .dockerignore"
```

---

## Task 2: `docker/Dockerfile`

Non-root user matching host UID/GID keeps `ros2_ws/build|install|log` user-owned on the bind mount. Pre-installs system deps (`xacro`, `ros-jazzy-rviz2`, build tools). `osrf/ros:jazzy-desktop` already includes RViz + most depend tags from `package.xml`, so the apt layer is small.

**Files:** Create `docker/Dockerfile`.

- [ ] **Step 1: Write the Dockerfile**

```dockerfile
# syntax=docker/dockerfile:1.7
FROM osrf/ros:jazzy-desktop

ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# System dependencies on top of the desktop image.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        sudo \
        python3-colcon-common-extensions \
        ros-jazzy-xacro \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host so bind-mounted build artifacts stay user-owned.
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

USER ${USER_NAME}
WORKDIR /workspace

COPY --chown=${USER_UID}:${USER_GID} docker/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Commit (no build yet — entrypoint lands next)**

```bash
git add docker/Dockerfile
git commit -m "feat(docker): add Jazzy desktop Dockerfile with non-root user"
```

---

## Task 3: `docker/entrypoint.sh`

Sources `/opt/ros/jazzy/setup.bash`, also sources `ros2_ws/install/setup.bash` if present, then exec's whatever command was passed.

**Files:** Create `docker/entrypoint.sh`.

- [ ] **Step 1: Write the entrypoint**

```bash
#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

WS_SETUP="/workspace/ros2_ws/install/setup.bash"
if [ -f "${WS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
fi

exec "$@"
```

- [ ] **Step 2: Make executable and commit**

```bash
chmod +x docker/entrypoint.sh
git add docker/entrypoint.sh
git commit -m "feat(docker): add entrypoint that sources ROS overlays"
```

---

## Task 4: `docker/compose.yaml`

Single service `dev`. Bind-mounts repo at `/workspace`. Forwards X11 for RViz. Build args wire host UID/GID.

**Files:** Create `docker/compose.yaml`.

- [ ] **Step 1: Write the compose file**

```yaml
services:
  dev:
    build:
      context: ..
      dockerfile: docker/Dockerfile
      args:
        USER_UID: ${UID:-1000}
        USER_GID: ${GID:-1000}
    image: robot_control_sim:jazzy
    container_name: robot_control_sim_dev
    network_mode: host
    ipc: host
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - QT_X11_NO_MITSHM=1
    volumes:
      - ..:/workspace:rw
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    working_dir: /workspace
    stdin_open: true
    tty: true
```

- [ ] **Step 2: Commit**

```bash
git add docker/compose.yaml
git commit -m "feat(docker): add compose service with bind-mount and X11 forwarding"
```

---

## Task 5: `Makefile`

Friendly verbs over the raw docker-compose commands. `make build` builds the image; `make ws` runs colcon inside; `make shell` opens a sourced shell; `make launch` runs the v1 launch file; `make test` runs the launch_testing smoke test; `make clean` nukes colcon artifacts.

**Files:** Create `Makefile` at repo root.

- [ ] **Step 1: Write the Makefile**

```makefile
# Wrapper for docker compose so common workflows are one keystroke.
COMPOSE := docker compose -f docker/compose.yaml
UID := $(shell id -u)
GID := $(shell id -g)

export UID
export GID

.PHONY: image ws shell launch test clean down

image:
	$(COMPOSE) build

ws: image
	$(COMPOSE) run --rm dev \
	    bash -lc "cd ros2_ws && colcon build --symlink-install --event-handlers console_cohesion+"

shell: image
	$(COMPOSE) run --rm dev bash

launch: image
	xhost +local:docker >/dev/null 2>&1 || true
	$(COMPOSE) run --rm dev \
	    bash -lc "cd /workspace/ros2_ws && source install/setup.bash && \
	              ros2 launch robot_control_sim_ros2 robot_sim.launch.py"

test: image
	$(COMPOSE) run --rm dev \
	    bash -lc "cd ros2_ws && colcon test --event-handlers console_direct+ && \
	              colcon test-result --verbose"

clean:
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

down:
	$(COMPOSE) down --remove-orphans
```

- [ ] **Step 2: Commit**

```bash
git add Makefile
git commit -m "feat(docker): add Makefile wrappers (image/ws/shell/launch/test)"
```

---

## Task 6: Build the image and run `colcon build` — moment of truth for v1 code

This is the verification step. If colcon errors out, the next tasks are fix commits, each one a `fix(ros2): ...` atomic commit, until colcon is green.

- [ ] **Step 1: Build the image**

```bash
make image
```
Expected: image `robot_control_sim:jazzy` built. Likely takes 5–10 minutes on first run.

- [ ] **Step 2: Run colcon build**

```bash
make ws
```
Expected: `Summary: 1 package finished`. If errors, capture them and proceed to Task 6a/6b/...

- [ ] **Step 3: Commit any fixes**

For each error class (e.g. missing include, wrong API), make one atomic commit:

```bash
git add ros2_ws/src/robot_control_sim_ros2/src/<file>.cpp
git commit -m "fix(ros2): <specific issue> (discovered via v2 colcon build)"
```

Re-run `make ws` after each fix until green.

> No code is shown for this step because the contents depend on what errors surface. Each fix is its own commit so the PR history reads as a clean log of "v1 said this would work; v2 confirmed/corrected it".

---

## Task 7: Disturbance topic — close the v1 carry-over

The v1 controller zeros the MPC6D disturbance state because `nav_msgs/Odometry` only carries pose+twist. Fix: estimator publishes `/robot/disturbance` (`geometry_msgs/Vector3Stamped`), controller subscribes, feeds into MPC6D state.

**Files:**
- Modify: `ros2_ws/src/robot_control_sim_ros2/src/estimator_node.cpp`
- Modify: `ros2_ws/src/robot_control_sim_ros2/src/controller_node.cpp`
- Modify: `ros2_ws/src/robot_control_sim_ros2/README.md`

### Step 1: Estimator publishes disturbance

In `estimator_node.cpp`, near the existing `odom_pub_` member, add:

```cpp
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr disturbance_pub_;
```

Include at top of file:

```cpp
#include <geometry_msgs/msg/vector3_stamped.hpp>
```

In the constructor, after `odom_pub_` is created:

```cpp
disturbance_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/robot/disturbance", 10);
```

In `publish()`, after the existing `odom_pub_->publish(odom);` line:

```cpp
geometry_msgs::msg::Vector3Stamped d;
d.header.stamp    = stamp;
d.header.frame_id = "map";
d.vector.x        = x(4);
d.vector.y        = x(5);
disturbance_pub_->publish(d);
```

### Step 2: Controller subscribes and uses it

In `controller_node.cpp`, include at top:

```cpp
#include <geometry_msgs/msg/vector3_stamped.hpp>
```

Add member:

```cpp
Eigen::Vector2d last_disturbance_ {Eigen::Vector2d::Zero()};
rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr disturbance_sub_;
```

In the constructor, after `sub_` is created:

```cpp
disturbance_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/robot/disturbance", 10,
    [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      last_disturbance_ << msg->vector.x, msg->vector.y;
    });
```

In `on_odom()`, replace the comment "disturbance components are zeroed" block with:

```cpp
x(0) = msg.pose.pose.position.x;
x(1) = msg.twist.twist.linear.x;
x(2) = msg.pose.pose.position.y;
x(3) = msg.twist.twist.linear.y;
x(4) = last_disturbance_(0);
x(5) = last_disturbance_(1);
```

### Step 3: Update the package README topics table

Add a row to the table in `ros2_ws/src/robot_control_sim_ros2/README.md`:

```markdown
| `/robot/disturbance` | `geometry_msgs/Vector3Stamped` | estimator → ctrl   |
```

Remove the "v1 limitation" sentence in the root and package README about the disturbance gap.

### Step 4: Verify and commit

```bash
make ws
```
Expected: package builds.

```bash
git add ros2_ws/src/robot_control_sim_ros2/src/estimator_node.cpp \
        ros2_ws/src/robot_control_sim_ros2/src/controller_node.cpp \
        ros2_ws/src/robot_control_sim_ros2/README.md
git commit -m "feat(ros2): publish /robot/disturbance and feed into MPC6D"
```

---

## Task 8: launch_testing smoke test

Asserts the launch description starts all four nodes without crashing within 5 seconds. Cheap, catches name/parameter typos.

**Files:**
- Create: `ros2_ws/src/robot_control_sim_ros2/test/test_launch.py`
- Modify: `ros2_ws/src/robot_control_sim_ros2/CMakeLists.txt` (register the test)

### Step 1: Write the test

```python
"""Smoke test: launch the full stack and assert nodes come up."""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("robot_control_sim_ros2")
    launch_file = os.path.join(pkg_share, "launch", "robot_sim.launch.py")

    included = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={"rviz": "false"}.items(),
    )

    return LaunchDescription([
        included,
        launch_testing.actions.ReadyToTest(),
    ])


class TestNodesUp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_observer")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_all_nodes_present(self):
        expected = {"plant_node", "sensor_node", "estimator_node", "controller_node"}
        deadline = time.time() + 10.0
        seen = set()
        while time.time() < deadline:
            names = {n for n, _ in self.node.get_node_names_and_namespaces()}
            seen |= names
            if expected.issubset(seen):
                break
            time.sleep(0.2)
        missing = expected - seen
        self.assertFalse(missing, f"Nodes never appeared: {missing}")
```

### Step 2: Register the test in CMakeLists

Inside the existing `if(BUILD_TESTING) ... endif()` block, after `ament_lint_auto_find_test_dependencies()`, add:

```cmake
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_launch.py)
```

### Step 3: Run and commit

```bash
make ws
make test
```
Expected: `Test result: 1 / 1 passed`.

```bash
git add ros2_ws/src/robot_control_sim_ros2/test/test_launch.py \
        ros2_ws/src/robot_control_sim_ros2/CMakeLists.txt
git commit -m "test(ros2): add launch_testing smoke test for all four nodes"
```

---

## Task 9: Live runtime check

A one-off `make launch` run to confirm the stack actually converges in RViz (or at least logs sensible values when run headless).

- [ ] **Step 1: Headless launch (no RViz) for 10 seconds**

```bash
timeout 10s docker compose -f docker/compose.yaml run --rm \
  -e DISPLAY= dev \
  bash -lc "cd /workspace/ros2_ws && source install/setup.bash && \
            ros2 launch robot_control_sim_ros2 robot_sim.launch.py rviz:=false" \
  | tee /tmp/v2_launch.log || true
```

- [ ] **Step 2: Inspect the log**

```bash
grep -E "ERROR|FATAL|Traceback" /tmp/v2_launch.log || echo "no errors"
```
Expected: `no errors`.

- [ ] **Step 3: No commit** — verification step only.

> If errors surface, treat them like Task 6 fixes: one atomic `fix(ros2): ...` commit per issue, re-verify.

---

## Task 10: README updates

**Files:**
- Modify: `README.md` (root)
- Modify: `ros2_ws/src/robot_control_sim_ros2/README.md`

### Step 1: Add Docker section to root README

Insert under the existing ROS 2 (v1) section (replace `(v1)` with just `(ROS 2)` if you prefer):

```markdown
### Docker (v2)

A Dockerfile provides a ROS 2 Jazzy environment for machines without a native install.

```bash
make image     # build the image
make ws        # colcon build inside the container
make launch    # run the full stack with RViz
make test      # run the launch_testing smoke test
make shell     # interactive shell with ROS sourced
```

X11 forwarding is wired through `compose.yaml`; on the first `make launch` you may need `xhost +local:docker`.
```

### Step 2: Update the package README

In `ros2_ws/src/robot_control_sim_ros2/README.md`, replace the existing "Local development without ROS" section with:

```markdown
## Local development without ROS

The repo root ships a Dockerfile + Makefile that provide a Jazzy environment:

```bash
make image && make ws && make launch
```

See the [root README](../../../README.md) for details.
```

### Step 3: Commit

```bash
git add README.md ros2_ws/src/robot_control_sim_ros2/README.md
git commit -m "docs: document Docker workflow for ROS 2 v2"
```

---

## Task 11: Open the PR

- [ ] **Step 1: Push**

```bash
git push -u origin feat/ros2-jazzy-v2-docker
```

- [ ] **Step 2: Open PR**

```bash
gh pr create --title "feat: ROS 2 Jazzy integration (v2, Docker + verified)" --body "$(cat <<'EOF'
## Summary
- Adds `docker/` and `Makefile` providing a ROS 2 Jazzy environment that works on machines without a native install.
- First real verification of v1: `colcon build` and `colcon test` actually run.
- Closes the v1 MPC6D disturbance carry-over with a new `/robot/disturbance` topic.
- Adds a launch_testing smoke test that asserts all four nodes come up.

## Verified
- [x] `make image` builds `robot_control_sim:jazzy`.
- [x] `make ws` → colcon build succeeds.
- [x] `make test` → launch_testing passes.
- [x] `make launch` (headless) — no errors in 10s log window.

## Architecture
- `docker/Dockerfile`: `osrf/ros:jazzy-desktop` + non-root user matching host UID/GID.
- `docker/compose.yaml`: bind-mounts the repo at `/workspace`, forwards X11.
- `Makefile`: `image`, `ws`, `shell`, `launch`, `test`, `clean`.

EOF
)"
```

- [ ] **Step 3: Report PR URL.**

---

## Self-Review

**Spec coverage:**
- "v2 will be running in docker" → Tasks 1–5 build the Docker environment.
- "atomic commits, branches, PRs" → Task 0 creates the branch off `main`, each task is one atomic commit, Task 11 opens the PR.
- "I cannot install ros currently" → the Dockerfile and Makefile are the workaround.
- Bonus: actually compile v1 code (Task 6), fix the MPC6D carry-over (Task 7), add tests (Task 8).

**Placeholder scan:** Task 6 deliberately doesn't predict what errors will surface — but it specifies the exact commands, the exact remediation pattern (one fix commit per error class), and the exit condition (`make ws` green). This is verification, not a placeholder.

**Type consistency:** `geometry_msgs::msg::Vector3Stamped` carries `header` + `vector` (with `.x`, `.y`, `.z`) — matches usage in Task 7. The `Eigen::Vector2d last_disturbance_` member uses `(0)` / `(1)` index access, matching Eigen's API.

## Execution

Proceeding inline per the user's "work without stopping" directive.
