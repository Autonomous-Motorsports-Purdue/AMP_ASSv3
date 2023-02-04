# `amp_kart_bringup`

Contains files needed to start the kart.

## Directory Tree

- `launch`
  - `bringup.launch.py` _Start the high-level navigation software stack_
  - `localization.launch.py` _Initialize map_server and amcl_
  - `navigation.launch.py` _Initialize Nav2_
  - `rviz.launch.py` _Initialize RViz with Nav2 configuration_
  - `slam.launch.py` _Initialize map_server and slam_toolbox_
  - `VLP16.launch.py` _Initialize Velodyne LIDAR VLP16 nodes_
- `maps`
  - `turtlebot3.world.pgm`
  - `turtlebot3.world.yaml`
- `params`
  - `VLP16.params.yaml` _VLP16 parameters_
  - `VLP16db.params.yaml` _VLP16 calibration parameters_
  - `nav2.params.yaml` _Nav2 parameters_
- `rviz`
  - `nav2_default_view.rviz` _RViz config for Nav2_
