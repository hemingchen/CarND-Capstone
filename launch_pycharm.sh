#!/bin/bash
# Activate the ROS environment and then launch pycharm

PYCHARM_DIR=pycharm-community-2017
source ros/devel/setup.sh && /opt/${PYCHARM_DIR}/bin/pycharm.sh &
