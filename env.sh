#!/usr/bin/env sh
export ROS_MASTER_URI=http://$1:11311
export ROS_IP=`hostname -I`

#env | grep ROS
shift
exec /catkin_ws/devel/env.sh "$@"
