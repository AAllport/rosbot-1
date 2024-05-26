#!/bin/bash

pio run -t upload
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v5