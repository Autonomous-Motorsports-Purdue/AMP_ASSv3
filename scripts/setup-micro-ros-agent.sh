#! /bin/bash

sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -ry

sudo apt-get install python3-pip

colcon build --symlink-install
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo "Remember to source install/local_setup.bash"
