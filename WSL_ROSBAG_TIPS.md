# ROSbag Tips

Some solutions to issues I encountered to running rosbags.

These bags really helped during testing since we often can't get racing data or have to stick to the basement of BIDC -\_-

If you're looking for vehicle lidar data from residential areas, try the [Kitti Dataset](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608), or [Jackal Dataset](https://www.cvlibs.net/datasets/kitti/).
This bag has some [coned FSAE pointclouds](https://www.dropbox.com/sh/4116xoc7srps6a5/AAC3q1h50swG7fRMI3USNn2la?dl=0)

## Running ROS1 bags on WSL

The bag file utility in ros2 `ros2 bag` has compatibility settings for running bags recorded with ROS1. The command is `ros2 bag play -s rosbag_v2 bagfile.bag`, but chances are you do not have the `rosbag_v2` plugin installed

1. First, add the (noetic apt repository)[http://wiki.ros.org/noetic/Installation/Ubuntu], or that of another ROS1 version.

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt update` 

2. Then install the bag compatibility plugin `sudo apt install ros-<distro>-rosbag2-bag-v2-plugins`. This will install its dependency, `ros-distro-ros1-bridge`, which further requires some `noetic` packages 

3. In order to have ROS 1 and 2 active together, we must source `/opt/ros/noetic/setup.bash` and `/opt/ros/foxy/setup.bash`, _in that order_. If you are sourcing foxy in your `.bashrc` already, add this before it:

```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

4. ROS distros are activated until the end of the shell session, which means `source ~/.bashrc` won't reload them correctly. Instead, you should log out and log back in. For WSL users, `wsl --shutdown` does not actually perform a full log out. You can simulate relogin this by running `sudo su - $USER` manually in the terminal with `ros2 bag`. [see why](https://superuser.com/questions/1733537/re-login-wsl-ubuntu)
5. Finally, the command `ros2 bag play -s rosbag_v2 bagfile.bag` should work
6. Note that while `source /opt/ros/noetic/setup.bash` is still in your `.bashrc`, you'll have to run `sudo su - $USER` on every new shell. There not good solution other than to revert your `.bashrc` after you finish your rosbag business.
