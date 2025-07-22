#!/bin/bash
source /opt/ros/jazzy/setup.bash

# If no command is provided, run the analyzer by default
if [ $# -eq 0 ]; then
    python3 ros_array_analyzer.py
else
    exec "$@"
fi 