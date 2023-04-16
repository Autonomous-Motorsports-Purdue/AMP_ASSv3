# `amp_kart_bringup`

Contains files needed to start the kart.

## Mux structure

There are two control sources that all produce `twist_mux` messages to "cmd_vel": joy_vel (teleop controller) and nav_vel (autnomous navigation).
The RCS provides some track states (safe stop, estop, teleop, auto) that control which input we select. It's important that the states can override each other in this priority:

- ESTOP/SAFE STOP
- TELEOP
- AUTO

The priorities are implemented with a [`twist_mux`](http://wiki.ros.org/twist_mux).
and laid out as can be seen below.

```


                          /joy_only   /stop (255)
                                    \   /
                                     | |
                                    _|_|_
                       /joy_vel____|     |
                                   |     |____ /cmd_vel
                                   |     |
                       /nav_vel ___|     |
                                   |_____|

```

Priorities for each topic and lock are listed, as well as the expected topic publishers:

- `stop` ~ `kart_commander`: 255
- `joy_vel` ~ `teleop_twist_joy_node`: 50
- `joy_only_publisher` ~ `kart_commander`: 50
- `nav_vel` ~ `nav2`: 10

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
  - `twist_mux.launch.py` _Launch both twist muxex and kart_commander_
  - `VLP16.launch.py` _Initialize Velodyne LIDAR VLP16 nodes_
- `map/`
  - `empty_map.yaml` _Default empty map_
- `params/`
  - `VLP16.params.yaml` _VLP16 parameters_
  - `VLP16db.params.yaml` _VLP16 calibration parameters_
  - `nav2.params.yaml` _Nav2 parameters_
  - `twist_mux.params.yaml`
- `rviz/`
  - `nav2_default_view.rviz.yaml` _RViz config for Nav2_
