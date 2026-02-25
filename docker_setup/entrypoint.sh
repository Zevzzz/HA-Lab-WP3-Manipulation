#!/bin/bash
set -e
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/ros/fastdds_profile.xml}
source "/opt/ros/jazzy/setup.bash"
if [ -f "/home/ros/ws/install/setup.bash" ]; then
  source "/home/ros/ws/install/setup.bash"
fi
exec "$@"
