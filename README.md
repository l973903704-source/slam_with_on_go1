# slam_on_go1

ROS Melodic workspace package for odometry-assisted 2D LaserScan SLAM and navigation on a Unitree Go1-style robot platform.


## Core Idea

```text
Odometry prediction
        +
Local scan matching correction
        +
Fixed-frame occupancy grid mapping
```

Instead of relying only on scan matching, `slam_v44` uses odometry to provide a stable pose prediction. Scan matching then corrects small errors around that predicted pose.

## Startup Pose

When the node starts, the first odometry pose is saved as the reference pose.

After that, all odometry poses are converted into motion relative to the startup pose:

```text
relative_odom = inverse(start_odom) * current_odom
```

This means the robot starts at the origin of the `map` frame:

```text
x = 0
y = 0
theta = 0
```

## SLAM Workflow

For each incoming LaserScan frame, the system follows this process:

```text
1. Receive odometry
2. Compute relative odometry from the startup pose
3. Predict the current pose in the map frame
4. Run scan matching near the predicted pose
5. Use the corrected pose if the match score is good
6. Fall back to odometry prediction if scan matching is unreliable
7. Insert the LaserScan points into the occupancy grid map
8. Publish map, pose, path, and TF
```

## Odometry-Based Prediction

Odometry provides the main motion estimate.

This helps keep the robot pose continuous, especially when the environment is not good for scan matching, such as:

- Open spaces
- Few obstacles
- Repetitive structures
- Noisy LaserScan data

## Local Scan Matching

Scan matching is only used as a local correction step.

The system searches in a small window around the odometry-predicted pose:

```text
small x offset
small y offset
small theta offset
```

The candidate pose with the best map correlation score is selected as the corrected pose.

If the score is too low, the system ignores the scan matching result and keeps the odometry prediction.

## Occupancy Grid Mapping

After the current pose is determined, LaserScan points are transformed into the fixed `map` frame.

For each laser beam:

- Cells between the robot and the laser endpoint are marked as free space.
- The laser endpoint is marked as occupied.
- The map is updated probabilistically using log-odds.

The final map is published as a ROS `nav_msgs/OccupancyGrid`.

## TF Structure

The default TF structure is:

```text
map -> odom -> base_link
```

Frame meanings:

- `map`: fixed global map frame
- `odom`: continuous odometry frame, may drift over time
- `base_link`: robot body frame

By default, `slam_v44` publishes the `map -> odom` correction.  
The robot base or SDK should publish `odom -> base_link`.

## Navigation

The navigation module uses the generated occupancy grid map.

The navigation process is:

```text
1. Receive a goal
2. Plan a global path with A*
3. Track the path with a lookahead-based local controller
4. Monitor the front LaserScan distance
5. Slow down or stop when an obstacle is too close
```

By default, velocity commands are published to a safe debug topic:

```text
/slam_v44/debug_cmd_vel
```

After confirming the path and command behavior, the output can be switched to:

```text
/cmd_vel
```

## Summary

`slam_v44` is designed around one main principle:

> Odometry provides stable motion prediction, while scan matching only performs small local corrections.

This makes the SLAM process more stable than relying on scan matching alone, especially on a real robot where odometry is available but LaserScan matching can be affected by the environment.


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

<details>
<summary>耶duang~</summary>

硬件还是得看上古真神 C++ <br>
锻炼情绪稳定性，就来搞真机，搞过都说好，哈哈（咬牙切齿）

</details>

