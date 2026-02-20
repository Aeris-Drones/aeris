FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-pip \
    python3-colcon-common-extensions \
    python3-empy \
    python3-jinja2 \
    python3-jsonschema \
    python3-numpy \
    python3-serial \
    python3-setuptools \
    python3-wheel \
    python3-yaml \
    build-essential \
    ccache \
    cmake \
    ninja-build \
    pkg-config \
    wget \
    zip \
    unzip \
    rsync \
    exiftool \
    xz-utils \
    file \
    libxml2-utils \
    libopencv-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libgtest-dev \
    libtinyxml2-dev \
    libbenchmark-dev \
    protobuf-compiler \
    libprotobuf-dev \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-image \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    future \
    kconfiglib \
    pymavlink \
    pyros-genmsg

RUN git clone --recursive --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && cd /opt/PX4-Autopilot \
 && DONT_RUN=1 make px4_sitl_default

RUN ln -s /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 /usr/local/bin/px4

ENV PX4_BIN=/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4
ENV ROS_DISTRO=humble

RUN groupadd --gid 1000 aeris \
 && useradd --uid 1000 --gid aeris --create-home --shell /bin/bash aeris \
 && mkdir -p /workspace \
 && chown -R aeris:aeris /workspace /opt/PX4-Autopilot/build/px4_sitl_default

WORKDIR /workspace

USER aeris

CMD ["bash"]
