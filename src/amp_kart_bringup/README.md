# `amp_kart_bringup`

Contains files needed to start the kart.

## Mux structure

The muxes used are [`twist_mux`](http://wiki.ros.org/twist_mux).
They are laid out as can be seen below.

```
                   /joy_disable      /joy_enable
                  |                 |
                  |                 |   /stop
                 ____               |  |
                |    |  /mux1_vel   ____
   /joy_vel ____|    |_____________|    |
                |    |             |    |
                |____|             |    |____ /cmd_vel
                 mux1              |    |
                       /nav_vel ___|    |
                                   |____|
                                    mux2
```

Priorities for the muxes are listed below, as well as the corresponding topic
and (expected) topic publishers:

- `mux1` &rarr; `/mux1_vel`
  - `joy_disable` ~ `kart_commander`: 150
  - `joy_vel` ~ `teleop_twist_joy_node`: 100
- `mux2` &rarr; `/cmd_vel`
  - `stop` ~ `kart_commander`: 255
  - `joy_enable` ~ `kart_commander`: 50
  - `nav_vel` ~ `nav2`: 10
  - `mux1_vel` ~ `mux1`: 100

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
  - `teleop.launch.py` _Launch joy teleop nodes_
  - `twist_mux.launch.py` _Launch both twist muxex_
  - `VLP16.launch.py` _Initialize Velodyne LIDAR VLP16 nodes_
  - `zed.launch.py` _Zed camera launch file_
- `map/`
  - `empty_map.yaml` _Default empty map_
- `params/`
  - `VLP16.params.yaml` _VLP16 parameters_
  - `VLP16db.params.yaml` _VLP16 calibration parameters_
  - `nav2.params.yaml` _Nav2 parameters_
  - `patchworkpp.params.yaml` _Patchwork++ parameters_
  - `twist_mux_1.params.yaml`
  - `twist_mux_2.params.yaml`
- `rviz/`
  - `nav2_default_view.rviz.yaml` _RViz config for Nav2_
