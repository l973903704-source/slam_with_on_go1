# slam_with_on_go1

ROS Melodic workspace package for odometry-assisted 2D LaserScan SLAM and navigation on a Unitree Go1-style robot platform.

This repository currently contains `slam_v44`, an experimental SLAM package that keeps the startup pose fixed at the `map` origin, uses odometry as the primary short-term motion prediction, and applies scan matching only as a local correction around the odometry estimate.

## Features

- 2D LaserScan occupancy-grid mapping.
- Odometry-assisted pose prediction.
- Local scan matching correction.
- Standard ROS TF layout: `map -> odom -> base_link`.
- A* global path planning.
- Lookahead-based local velocity control.
- Persistent obstacle visualization markers for RViz.
- Safe navigation debugging through `/slam_v44/debug_cmd_vel` by default.

## Repository Layout

```text
slam_with_on_go1/
  README.md
  slam_v44/
    CMakeLists.txt
    package.xml
    config/
    include/
    launch/
    src/
    README.md
    README_algorithm_v44.md
    todolist.md
```

## Requirements

- Ubuntu 18.04 or another environment compatible with ROS Melodic.
- ROS Melodic with `catkin`.
- A running robot SDK or driver that publishes LaserScan and odometry topics.
- Expected default topics:
  - `/slamware_ros_sdk_server_node/scan`
  - `/slamware_ros_sdk_server_node/odom`

The package depends on these ROS packages:

- `roscpp`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf`
- `visualization_msgs`

## Install

Clone this repository into a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/l973903704-source/slam_with_on_go1.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Verify that ROS can find the package:

```bash
rospack find slam_v44
```

## Run SLAM

Start mapping with the default LaserScan and odometry topics:

```bash
roslaunch slam_v44 slam_v44.launch
```

Use custom topics when your robot publishes different names:

```bash
roslaunch slam_v44 slam_v44.launch scan_topic:=/scan odom_topic:=/odom
```

If your robot does not publish `odom -> base_link`, disable the default `map -> odom` correction mode:

```bash
roslaunch slam_v44 slam_v44.launch publish_map_to_odom_tf:=false
```

## Run With RViz

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true
```

The main visualization topics are:

- `/slam_v44/map`
- `/slam_v44/pose`
- `/slam_v44/slam_path`
- `/slam_v44/persistent_obstacle_markers`

Use `map` as the RViz fixed frame.

## Navigation Debugging

Start SLAM with the navigation node:

```bash
roslaunch slam_v44 slam_v44.launch start_navigation:=true
```

By default, velocity commands are published to:

```text
/slam_v44/debug_cmd_vel
```

After verifying the path and velocity behavior, you can switch to the real robot command topic:

```bash
roslaunch slam_v44 slam_v44.launch start_navigation:=true cmd_vel_topic:=/cmd_vel
```

## Useful Checks

```bash
rostopic echo -n 1 /slamware_ros_sdk_server_node/scan
rostopic echo -n 1 /slamware_ros_sdk_server_node/odom
rostopic echo -n 1 /slam_v44/map
rostopic echo -n 1 /slam_v44/pose
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
```

## Documentation

- Package run guide: [`slam_v44/README.md`](slam_v44/README.md)
- Algorithm notes: [`slam_v44/README_algorithm_v44.md`](slam_v44/README_algorithm_v44.md)
- Development notes: [`slam_v44/todolist.md`](slam_v44/todolist.md)

## Status

This project is an experimental robotics package intended for SLAM, mapping, and navigation testing. Verify topic names, TF frames, and velocity output carefully before commanding a real robot.

## License

The ROS package declares the MIT license in `slam_v44/package.xml`.


硬件还是得看上古真神c++
锻炼情绪稳定性，就来搞真机，搞过都说好，哈哈（咬牙切齿
