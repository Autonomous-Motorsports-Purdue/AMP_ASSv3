# `amp_launch_tests`

Contains files to run on-ground tests.

## Directory Tree

- `config/`
  - `xbox.config.yaml` _XBox One controller setup for teleop joy_
- `launch/`
  - `perception_bringup.launch.py` _Launch ZED and VLP16 sensors_
  - `teleop_joy_micro_ros.launch.py` _Launch teleop_twist_joy and micro-ros agent_
  - `teleop_joy_micro_ros_client.launch.py` _Launch micro-ros agent, run on
    on-kart Jetson_
  - `teleop_joy_micro_ros_server.launch.py` _Launch teleop_twist_joy, run on
    Remote Computer_
