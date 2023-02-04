# AMP's Autonomous Software Stack

[![Build and Test the Software Stack](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSV3/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSV3/actions/workflows/ci.yaml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/Autonomous-Motorsports-Purdue/AMP_ASSV3/main.svg)](https://results.pre-commit.ci/latest/github/Autonomous-Motorsports-Purdue/AMP_ASSV3/main)

Autonomous Motorsports Purdue's software stack for the go-kart which will race
in the [Autonomous Karting Series](https://autonomouskartingseries.com/).
It is built on top of [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) as
it is the latest, and last, ROS2 distro that can run natively on the team's
NVIDIA Xavier NX computer running Ubuntu 20.04, which is limited by the distro
packaged with Jetpack: [JetPack SDK 5.0.2](https://developer.nvidia.com/embedded/jetpack-sdk-502).

**NOTE:** This repository is a ROS workspace.

## Development Setup

Make sure [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) is
installed in your system and you have the proper environment setup.

**NOTE:** Laptops without an NVIDIA GPU or a compatible CUDA version you will
not be able to install the [ZED SDK](https://www.stereolabs.com/developers/release/),
which is required to interact with the [ZED stereo camera](https://www.mybotshop.de/Datasheet/zed-camera-datasheet.pdf).

### Downloading and Setting up the Repo

Clone the repo (change the link to an SSH link if you have SSH keys setup):

```bash
git clone https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSV3.git
cd AMP_ASSV3
```

Initialize the submodules:

```bash
# Clone the submodule to get micro-ros
git submodule update --init src/micro-ros-setup/
# Clone the submodule to get the ZED ROS2 wrapper, do not install if you
# do not have the ZED SDK installed.
git submodule update --init src/zed-ros2-wrapper/
```

#### Setting up `micro-ros`

Run the `setup-micro-ros-agent.sh` script:

```
./scripts/setup-micro-ros-agent.sh
```

#### Setting up [`pre-commit`](https://pre-commit.com/)

The project has a [pre-commit](https://pre-commit.com/) setup to allow
consistent code formatting and good practices throughout the repo. Code reviews
via [pull requests](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)
will still be used to make sure that the changes being made are proper.

Follow installation instructions for `pre-commit` [here](https://pre-commit.com/#install).

Then, run:

```
pre-commit install --install-hooks
```

Now whenever you commit changes, `pre-commit` will run formatters and clean up
your code. Remember to add the changes made by it when you commit!

### Installing Dependencies and Building

```
rosdep update
rosdep install --from-paths src -iry
colcon build --symlink-install
```

### Running the Default Nav2 Sim

To setup the environment, source the `local_setup` script, and run the launch
file.

```
source install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
ros2 launch amp_kart_simulate tb3_simulation.launch.py
```

## Project Setup

Under `src`, on clone, there should be 7 modules (8 when micro-ros-agent is setup):

- [`src/amp_kart_bringup`](src/amp_kart_bringup)
- [`src/amp_kart_bt_navigator`](src/amp_kart_bt_navigator)
- [`src/amp_kart_description`](src/amp_kart_description)
- [`src/amp_kart_simulate`](src/amp_kart_simulate)
- [`src/amp_launch_tests`](src/amp_launch_tests)
- `src/micro-ros-setup`
- `src/zed-ros2-wrapper`
