# syntax=docker/dockerfile:1
#
# Multi-stage image:
#   base     ROS 2 + all the dependencies resolved by rosdep (cached as long as no
#            package.xml changes)
#   dev      base + developer tools (used by the devcontainer)
#   build    compiles and tests the workspace
#   runtime  base + installed workspace, default command = Gazebo demo
#
# Build from the repository root:
#   docker build -f docker/Dockerfile -t nav2_mpc_controller .

ARG ROS_DISTRO=lyrical

FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}
WORKDIR /ws

# Only the manifests: the dependency layer is rebuilt only when dependencies change.
COPY mpc_core/package.xml src/nav2_mpc_controller/mpc_core/
COPY nav2_mpc_controller/package.xml src/nav2_mpc_controller/nav2_mpc_controller/
COPY mpc_benchmark/package.xml src/nav2_mpc_controller/mpc_benchmark/
COPY mpc_bringup/package.xml src/nav2_mpc_controller/mpc_bringup/
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
 && apt-get install -y --no-install-recommends python3-colcon-common-extensions ccache \
 && rm -rf /var/lib/apt/lists/*

FROM base AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
      clang-format clang-tidy gdb git less nano pre-commit python3-matplotlib python3-pandas \
 && rm -rf /var/lib/apt/lists/*

FROM base AS build
COPY . src/nav2_mpc_controller
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DMPC_WARNINGS_AS_ERRORS=ON \
 && rm -rf build log

FROM base AS runtime
COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mpc_bringup", "demo.launch.py"]
