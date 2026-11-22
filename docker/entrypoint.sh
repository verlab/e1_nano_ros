#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash
export E1_DESCRIPTION_DIR=${E1_DESCRIPTION_DIR:-/ws/install/e1_description/share/e1_description}
exec "$@"
