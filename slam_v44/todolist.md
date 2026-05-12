# slam_v44 上机 todolist

## 0. 上机前准备

- [ ] 把 `slam_v44` 文件夹拷贝到机器狗主机：

```bash
scp -r slam_v44 unitree@<DOG_IP>:~/catkin_ws/src/
```

- [ ] 登录机器狗主机：

```bash
ssh unitree@<DOG_IP>
```

- [ ] 编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find slam_v44
```

- [ ] 确认 `rospack find slam_v44` 输出类似：

```text
/home/unitree/catkin_ws/src/slam_v44
```

## 1. 终端 1：先启动老师的 slamware SDK

必须先启动这个，否则 `slam_v44` 拿不到老师发布的雷达和 odom。

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch
```

检查雷达和 odom：

```bash
rostopic echo -n 1 /slamware_ros_sdk_server_node/scan
rostopic echo -n 1 /slamware_ros_sdk_server_node/odom
rostopic hz /slamware_ros_sdk_server_node/scan
rostopic hz /slamware_ros_sdk_server_node/odom
```

## 2. 终端 2：启动 slam_v44 建图 + RViz

先只建图和显示，不导航，不让机器狗动。

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true start_navigation:=false
```

RViz 里确认：

```text
Fixed Frame: map
Map: /slam_v44/map
Pose: /slam_v44/pose
Path: /slam_v44/slam_path
Marker: /slam_v44/persistent_obstacle_markers
TF: map / odom / base_link
```

检查建图输出：

```bash
rostopic echo -n 1 /slam_v44/map
rostopic echo -n 1 /slam_v44/pose
rostopic hz /slam_v44/map
rostopic hz /slam_v44/pose
```

检查 TF：

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
```

如果 `odom -> base_link` 没有，改用：

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true start_navigation:=false publish_map_to_odom_tf:=false
```

然后检查：

```bash
rosrun tf tf_echo map base_link
```

## 3. 手动推着/遥控机器狗走一圈建图

- [ ] 观察 `/slam_v44/pose` 是否随机器狗运动连续变化。
- [ ] 观察 RViz 轨迹 `/slam_v44/slam_path` 是否和真实移动方向一致。
- [ ] 观察地图墙体/障碍物是否稳定，不要都堆到机器人附近。
- [ ] 走回起点附近时，看轨迹是否大致回到原点附近。

如果地图乱：

```bash
rostopic echo -n 1 /slamware_ros_sdk_server_node/odom
rostopic echo -n 1 /slamware_ros_sdk_server_node/scan
rosrun tf tf_echo odom base_link
```

## 4. 终端 3：导航前启动 Unitree twist_sub

执行导航前必须新开一个终端启动它。它负责接收 ROS 的 Twist 速度并转给机器狗底层。

```bash
source ~/catkin_ws/devel/setup.bash
rosrun unitree_legged_real twist_sub
```

## 5. 终端 4：先启动导航调试，不接真实 /cmd_vel

先不让机器狗真的动，只看规划路径和速度输出是否合理。

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_v44 slam_v44.launch \
  start_slam:=false \
  start_persistent_obstacles:=false \
  start_rviz:=false \
  start_navigation:=true
```

默认速度输出是：

```text
/slam_v44/debug_cmd_vel
```

在 RViz 里用 `2D Nav Goal` 点目标，然后检查：

```bash
rostopic echo /slam_v44/global_path
rostopic echo /slam_v44/debug_cmd_vel
```

确认：

- [ ] `/slam_v44/global_path` 有路径。
- [ ] `/slam_v44/debug_cmd_vel` 速度方向合理。
- [ ] 目标点在地图已知可通行区域。
- [ ] 地图、目标、机器人位姿都在 `map` 坐标系下。

## 6. 真正接入 /cmd_vel 让机器狗导航

确认 debug 正常后，再启动真实速度输出。

先停止上一步导航终端，然后重新启动：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_v44 slam_v44.launch \
  start_slam:=false \
  start_persistent_obstacles:=false \
  start_rviz:=false \
  start_navigation:=true \
  cmd_vel_topic:=/cmd_vel
```

安全停止命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

## 7. 常见问题排查

### 没有地图

```bash
rostopic hz /slamware_ros_sdk_server_node/scan
rostopic hz /slamware_ros_sdk_server_node/odom
rosnode info /slam_v44_node
rosparam get /slam_v44_node/scan_topic
rosparam get /slam_v44_node/odom_topic
```

如果日志提示等待 odom：

```text
waiting for odom before inserting scans into fixed map frame
```

说明 odom 没收到。优先改 `odom_topic`，临时测试可用：

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true require_odom_for_mapping:=false
```

### RViz 看不到机器人

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rostopic echo -n 1 /slam_v44/pose
```

如果没有 `odom -> base_link`：

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true publish_map_to_odom_tf:=false
```

### 点目标后没有路径

```bash
rostopic echo -n 1 /slam_v44/map
rostopic echo -n 1 /slam_v44/pose
rostopic echo -n 1 /move_base_simple/goal
rostopic echo /slam_v44/global_path
```

重点看：

- [ ] `/move_base_simple/goal` 的 `header.frame_id` 是否是 `map`。
- [ ] 目标点是否落在地图范围内。
- [ ] 目标点是否在未知区域或障碍物里。
- [ ] `planner_allow_unknown` 是否需要临时改成 `true`。

### 机器狗不动

```bash
rosnode list | grep unitree
rostopic echo /cmd_vel
rosrun unitree_legged_real twist_sub
```

确认：

- [ ] `twist_sub` 已经启动。
- [ ] 导航启动时用了 `cmd_vel_topic:=/cmd_vel`。
- [ ] `/cmd_vel` 有非零速度。
- [ ] 机器狗本体处于允许运动状态。

## 8. 推荐完整顺序

1. 终端 1：

```bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch
```

2. 终端 2：

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true start_navigation:=false
```

3. 建图并确认 `/slam_v44/map`、`/slam_v44/pose`、TF 正常。

4. 终端 3：

```bash
rosrun unitree_legged_real twist_sub
```

5. 终端 4，先 debug：

```bash
roslaunch slam_v44 slam_v44.launch start_slam:=false start_persistent_obstacles:=false start_rviz:=false start_navigation:=true
```

6. 确认 debug 正常后，终端 4 改真实速度：

```bash
roslaunch slam_v44 slam_v44.launch start_slam:=false start_persistent_obstacles:=false start_rviz:=false start_navigation:=true cmd_vel_topic:=/cmd_vel
```
