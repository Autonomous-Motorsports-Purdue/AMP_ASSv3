# `amp_kart_bringup`

Contains files needed to start the kart.

## Directory Tree

- `config/`
  - `xbox.config.yaml` _config file for teleop twist joy node_
- `launch/`
  - `kart.launch.py` _Start sensors and micro ros_
  - `kart_bringup.launch.py` _Start kart related items; currently sensors, micro ros, and nav2_
  - `laptop_bringup.launch.py` _Start laptop related items; currently rviz_
  - `localization.launch.py` _Initialize map_server and amcl_
  - `nav2_bringup.launch.py` _Start the high-level navigation software stack_
  - `navigation.launch.py` _Initialize Nav2_
  - `rviz.launch.py` _Initialize RViz with Nav2 configuration_
  - `slam.launch.py` _Initialize map_server and slam_toolbox_
  - `VLP16.launch.py` _Initialize Velodyne LIDAR VLP16 nodes_
- `map/`
  - `empty_map.yaml` _Default empty map_
- `params/`
  - `VLP16.params.yaml` _VLP16 parameters_
  - `VLP16db.params.yaml` _VLP16 calibration parameters_
  - `nav2.params.yaml` _Nav2 parameters_
- `rviz/`
  - `nav2_default_view.rviz.yaml` _RViz config for Nav2_
