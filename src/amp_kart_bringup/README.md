# `amp_kart_bringup`

Contains files needed to start the kart.

## Mux structure

There are two control sources that both produce `twist_mux` messages to `cmd_vel`: joy_vel (teleop controller) and nav_vel (autnomous navigation).
The RCS provides some track states (safe stop, estop, teleop, auto) that control which input we select. The kart_commander uses these states to determine which control source to let through, or neither.

The priorities are implemented with `switch_mux`, a node defined in `kart_commander`

```
              /topic_select
                   |
                   |
                  _|___
     /joy_vel____|     |
                 |     |____ /cmd_vel
                 |     |
                switch_mux
                 |     |
     /nav_vel ___|     |
                 |_____|
```

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
  - `kart_commander.launch.py` _Launch kart_commander and the twist muxes_
  - `VLP16.launch.py` _Initialize Velodyne LIDAR VLP16 nodes_
  - `zed.launch.py` _Zed camera launch file_
- `map/`
  - `empty_map.yaml` _Default empty map_
- `params/`
  - `VLP16.params.yaml` _VLP16 parameters_
  - `VLP16db.params.yaml` _VLP16 calibration parameters_
  - `nav2.params.yaml` _Nav2 parameters_
  - `kart_commander.params.yaml`
  - `patchworkpp.params.yaml` _Patchwork++ parameters_
  - `switch_mux.params.yaml`
- `rviz/`
  - `nav2_default_view.rviz.yaml` _RViz config for Nav2_
