#!/bin/bash

# remember to source the script to run it in the current shell (not the subshell) 

source ../../install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
