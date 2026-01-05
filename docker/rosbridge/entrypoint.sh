#!/bin/bash

set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/aeris_ws/install/setup.bash

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
