#!/bin/bash

source install/local_setup.bash
tmux \
    new-session 'sleep 10 && ros2 launch amp_kart_bringup nav2_bringup.launch.py' \; \
    split-window -h 'ros2 launch amp_kart_bringup kart.launch.py serial_tty:=/dev/ttyACM0' \; \
    split-window -v 'ros2 launch amp_kart_bringup twist_mux.launch.py' \
