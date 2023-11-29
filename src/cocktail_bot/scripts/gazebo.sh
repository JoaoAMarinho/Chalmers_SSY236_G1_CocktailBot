#!/bin/bash

# Export path
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/src/cocktail_bot

# Source workspace
source ./devel/setup.bash

# Launch gazebo
roslaunch cocktail_bot gazebo.launch