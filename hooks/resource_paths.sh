#! /bin/sh

. /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${ROS_WORKSPACE}/src/ros_follow_line/models"
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:${ROS_WORKSPACE}/src/ros_follow_line/worlds"
