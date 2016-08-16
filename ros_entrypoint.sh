#!/bin/bash
set -e

# Setup ROS environment
source "$ROS_INSTALL_DIR/setup.bash"
exec "$@"
