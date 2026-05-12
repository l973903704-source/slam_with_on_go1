# slam_on_go1

ROS Melodic workspace package for odometry-assisted 2D LaserScan SLAM and navigation on a Unitree Go1-style robot platform.

<details>
<summary>Notes Before Running on a Real Robot</summary>

## 1. Understand the Required Background Knowledge

Before running the system on a real robot, it is helpful to understand the basic concepts below. You do not need to master all of them at the beginning, but you should at least know what role each one plays in the whole system:

- **ROS basics**: Understand the basic usage of `roscore`, `roslaunch`, `rosrun`, `rosnode`, `rostopic`, and `rosparam`.
- **Topic communication**: Know that ROS nodes exchange data through topics, such as `/scan`, `/cmd_vel`, `/map`, and `/odom`.
- **Common ROS message types**: Understand what `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `nav_msgs/OccupancyGrid`, `geometry_msgs/PoseStamped`, and `geometry_msgs/Twist` represent.
- **TF coordinate transforms**: Understand the relationship between frames such as `map`, `odom`, and `base_link`, and why the robot pose, map, and goal must be in a consistent coordinate frame.
- **2D LiDAR basics**: Know how LaserScan `range` and `angle` values are converted into 2D points `(x, y)`.
- **Odometry**: Understand that odometry estimates the robot motion from its own sensors. It is usually continuous and smooth, but it may drift over time.
- **Occupancy grid maps**: Know that the map is represented by many grid cells, where each cell can be unknown, free, or occupied.
- **Basic SLAM workflow**: Understand the loop of “sensor data -> pose estimation -> map update -> publish map and pose”.
- **Scan matching**: Know that scan matching compares the current laser scan with the existing map to correct the robot pose.
- **A\* path planning**: Here, I used A*, but the result was not satisfactory. It is recommended to consider other search algorithms.
- **Velocity control with `cmd_vel`**: Know that `/cmd_vel` usually contains linear and angular velocity commands, and it should be tested with a debug topic before connecting to the real robot.
- **Linux and terminal basics**: Be comfortable using commands such as `cd`, `ls`, `catkin_make`, and `source devel/setup.bash`, and learn how to read terminal errors.
- **Network configuration**: Understand that SSH, `ROS_MASTER_URI`, `ROS_IP`, and `ROS_HOSTNAME` can affect remote ROS communication.

## 2. If You Are Using Unitree Go1, C++ Is Recommended

For hardware-related communication, real-time control, and ROS node debugging, C++ is usually more stable and closer to many official SDK examples.

## 3. Run the Whole Pipeline First, Then Modify Algorithm Details

First make sure the whole process works: LiDAR, odometry, TF, map publishing, RViz visualization, navigation goals, and velocity output. After that, modify details such as scan matching, map updating, or path planning. Otherwise, it is easy to confuse algorithm bugs with topic, TF, network, or robot-driver issues.

## 4. Remote Connection Stability Can Be Affected by Heat

SSH or remote desktop connections may become unstable when the robot computer is hot, overloaded, or under poor network conditions. If you really cannot connect, shut it down, let it cool for a while, and then restart it.

## 5. Keep emotions stable and good luck!


</details>

<details>
<summary>Core Idea</summary>

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

</details>


<details>
<summary>Project Overview</summary>

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

</details>


<details>     
<summary>Run SLAM</summary>


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

</details>


<details>
<summary>耶duang~</summary>

硬件还是得看上古真神 C++ <br>
锻炼情绪稳定性，就来搞真机，搞过都说好，哈哈（咬牙切齿）

</details>

