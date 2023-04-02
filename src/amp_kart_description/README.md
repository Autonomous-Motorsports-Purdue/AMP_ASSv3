# `amp_kart_description`

Contains a [URDF model](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/URDF-Main.html)
of the kart.

## Directory Tree

- `CMakeLists.txt`
- `launch/`
  - `display_kart.launch.py` _Run robot_state_publisher and joint_state_publisher
    and view the URDF in RViz_
- `meshes/`
  - `chassis.dae`
  - `chassis.STL`
  - `cone.dae`
  - `hokuyo.dae`
  - `left_front_wheel.dae`
  - `left_front_wheel.STL`
  - `left_rear_wheel.dae`
  - `left_rear_wheel.STL`
  - `left_steering_hinge.dae`
  - `left_steering_hinge.STL`
  - `parking_1.dae`
  - `right_front_wheel.dae`
  - `right_front_wheel.STL`
  - `right_rear_wheel.dae`
  - `right_rear_wheel.STL`
  - `right_steering_hinge.dae`
  - `right_steering_hinge.STL`
  - `walker_racecourse.dae`
- `package.xml`
- `README.md`
- `rviz/`
  - `urdf.rviz.yaml` _RViz config to view URDF_
- `urdf/`
  - `kart.urdf` _Kart URDF file_
  - `macros.xacro` _macros XACRO file_
  - `materials.xacro` _material XACRO file_
  - `racecar.gazebo` _racecar GAZEBO file_
  - `racecar.xacro` _Kart V2 XACRO file_
