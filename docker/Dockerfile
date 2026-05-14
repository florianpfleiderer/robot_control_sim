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
