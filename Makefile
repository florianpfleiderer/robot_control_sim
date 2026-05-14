# Wrapper for docker compose so common workflows are one keystroke.
COMPOSE := docker compose -f docker/compose.yaml
UID := $(shell id -u)
GID := $(shell id -g)
VIDEO_GID := $(shell getent group video  | cut -d: -f3)
RENDER_GID := $(shell getent group render | cut -d: -f3)

export UID
export GID
export VIDEO_GID
export RENDER_GID

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
