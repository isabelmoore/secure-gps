#!/bin/bash

# Source ROS Humble
. /opt/ros/humble/setup.bash

# Forward the ROS IP and HOSTNAME only if they are set (by docker compose)
if [ ! -z $HOST_ROS_IP ]; then
    export ROS_IP=$HOST_ROS_IP
fi

if [ ! -z $HOST_ROS_HOSTNAME ]; then
    export ROS_HOSTNAME=$HOST_ROS_HOSTNAME
fi

exec "$@"