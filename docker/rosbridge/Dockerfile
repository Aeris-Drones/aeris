FROM --platform=$TARGETPLATFORM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

# Install rosbridge + build tools for message package
RUN apt-get update -qq \
  && apt-get install -y -qq --no-install-recommends \
    ros-humble-rosbridge-suite \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Build aeris_msgs once at image build-time so rosbridge can resolve custom types.
WORKDIR /opt/aeris_ws
RUN mkdir -p src
COPY software/edge/src/aeris_msgs/ /opt/aeris_ws/src/aeris_msgs/
RUN source /opt/ros/humble/setup.bash \
  && colcon build --packages-select aeris_msgs --symlink-install

COPY docker/rosbridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
