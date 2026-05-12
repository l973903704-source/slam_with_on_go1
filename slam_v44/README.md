# slam_v44 上机说明

`slam_v44` 是一个 ROS Melodic 下的二维 LaserScan SLAM 和导航实验包。这个版本相对上一版的核心变化是：**启动点固定为 `map` 原点，机器人运动主要由 odom 给出，scan matching 只在 odom 预测附近做小范围修正，地图始终写入固定 `map` 坐标系**。

## 一、主要算法

### 1. 坐标结构

默认坐标树按 ROS 常用结构组织：

```text
map -> odom -> base_link
```

- `map`：全局固定地图坐标系，启动点默认是 `(0, 0, 0)`。
- `odom`：机器人启动后由底盘/SDK 根据自身运动估计出的连续里程计坐标系。
- `base_link`：机器人本体坐标系。

`slam_v44` 默认发布 `map -> odom` 修正，底盘或 SDK 负责发布 `odom -> base_link`。如果机器狗没有发布 `odom -> base_link`，可以临时用 `publish_map_to_odom_tf:=false`，让本包直接发布 `map -> base_link`。

### 2. odom 固定启动参考点

节点启动后收到第一帧 `/slamware_ros_sdk_server_node/odom` 时，会把这帧 odom 锁定为参考点。之后每一帧 odom 都会计算成“相对启动点”的位姿变化：

```text
relative_odom = inverse(start_odom) * current_odom
```

这样机器人刚启动的位置就是 `map` 下的原点，移动后的位置是相对这个原点的位移。

### 3. odom 预测 + scan matching 小范围修正

每帧雷达到来时，流程是：

```text
odom 相对位移 -> 预测 map 下当前位姿 -> 在预测位姿附近 scan matching -> 得到修正位姿 -> 写入地图
```

scan matching 不再独自估计长距离运动，只负责小范围纠偏。默认搜索窗口：

```yaml
linear_search_window: 0.10
angular_search_window: 0.08
linear_step: 0.01
angular_step: 0.01
```

这样可以减少“机器人实际移动了，但地图还把障碍物画在旧位置附近”的问题。

### 4. 栅格地图

地图是 `nav_msgs/OccupancyGrid`：

- 默认话题：`/slam_v44/map`
- 分辨率：`0.05 m/cell`
- 地图大小：`40m x 40m`
- 原点：`(-20, -20)`

雷达点会通过当前修正后的 `Pose2D(x, y, theta)` 转换到固定 `map` 坐标，再用 log-odds 更新占据概率。

### 5. 导航

导航节点使用：

- A* 全局路径规划。
- 栅格膨胀避障。
- lookahead 本地控制。
- 前方雷达距离过近时停止。

默认不会直接向真实 `/cmd_vel` 发速度，而是发到：

```text
/slam_v44/debug_cmd_vel
```

确认路径和速度合理后，再手动切到真实 `/cmd_vel`。

### 6. 持久障碍物显示

持久障碍物节点订阅 `/slam_v44/map`，记住历史上出现过的障碍物边界，并发布：

```text
/slam_v44/persistent_obstacle_markers
```

这个只用于 RViz 可视化，不参与 A* 规划。

## 二、目录结构

拷到机器狗主机后，建议路径是：

```text
~/catkin_ws/src/slam_v44
```

包内主要文件：

```text
slam_v44/
  CMakeLists.txt
  package.xml
  config/slam_v44.yaml
  launch/slam_v44.launch
  include/slam_v44/*.h
  src/*.cpp
```

## 三、上机前确认话题

先在机器狗主机上确认 SDK 已经启动，并检查雷达和 odom：

```bash
rostopic list | grep slamware
rostopic echo -n 1 /slamware_ros_sdk_server_node/scan
rostopic echo -n 1 /slamware_ros_sdk_server_node/odom
```

确认 TF：

```bash
rosrun tf tf_echo odom base_link
```

如果这个命令一直报错，说明底盘可能没有发布 `odom -> base_link`。此时启动时可以加：

```bash
publish_map_to_odom_tf:=false
```

## 四、拷贝到机器狗主机

在本地电脑上，如果机器狗 IP 是 `<DOG_IP>`，用户名是 `unitree`：

```bash
scp -r slam_v44 unitree@<DOG_IP>:~/catkin_ws/src/
```

如果主机上还没有 catkin 工作空间：

```bash
ssh unitree@<DOG_IP>
mkdir -p ~/catkin_ws/src
exit
scp -r slam_v44 unitree@<DOG_IP>:~/catkin_ws/src/
```

也可以先压缩再拷贝：

```bash
tar -czf slam_v44.tar.gz slam_v44
scp slam_v44.tar.gz unitree@<DOG_IP>:~/catkin_ws/src/
ssh unitree@<DOG_IP>
cd ~/catkin_ws/src
tar -xzf slam_v44.tar.gz
```

## 五、编译

进入机器狗主机：

```bash
ssh unitree@<DOG_IP>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find slam_v44
```

如果 `rospack find slam_v44` 能输出：

```text
/home/unitree/catkin_ws/src/slam_v44
```

说明包已经被 ROS 找到了。

建议写入 `~/.bashrc`：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 六、启动建图

默认启动 SLAM 和持久障碍物显示，不启动导航，不启动 RViz：

```bash
roslaunch slam_v44 slam_v44.launch
```

等几秒后检查：

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

如果机器狗没有发布 `odom -> base_link`，用这个启动：

```bash
roslaunch slam_v44 slam_v44.launch publish_map_to_odom_tf:=false
```

此时检查：

```bash
rosrun tf tf_echo map base_link
```

## 七、指定雷达或 odom 话题

如果实际雷达话题不是默认值，用：

```bash
roslaunch slam_v44 slam_v44.launch scan_topic:=/scan
```

如果实际 odom 话题不是默认值，用：

```bash
roslaunch slam_v44 slam_v44.launch odom_topic:=/odom
```

两个一起指定：

```bash
roslaunch slam_v44 slam_v44.launch scan_topic:=/scan odom_topic:=/odom
```

## 八、RViz 查看

如果机器狗主机有图形界面，可以直接：

```bash
roslaunch slam_v44 slam_v44.launch start_rviz:=true
```

如果 RViz 在另一台电脑上运行，先保证两台机器 ROS 网络互通，然后在电脑上：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun rviz rviz -d $(rospack find slam_v44)/config/slam_v44_persistent_obstacles.rviz
```

RViz 里 Fixed Frame 设置为：

```text
map
```

主要显示：

```text
Map      -> /slam_v44/map
Pose     -> /slam_v44/pose
Path     -> /slam_v44/slam_path
Marker   -> /slam_v44/persistent_obstacle_markers
TF       -> map/odom/base_link
```

## 九、导航调试

先只调试，不让机器狗真实动。默认速度输出是安全话题：

```text
/slam_v44/debug_cmd_vel
```

启动建图 + 导航调试：

```bash
roslaunch slam_v44 slam_v44.launch start_navigation:=true
```

在 RViz 里用 `2D Nav Goal` 点目标，检查路径和速度：

```bash
rostopic echo /slam_v44/global_path
rostopic echo /slam_v44/debug_cmd_vel
```

确认没有问题后，再切到真实 `/cmd_vel`：

```bash
roslaunch slam_v44 slam_v44.launch start_navigation:=true cmd_vel_topic:=/cmd_vel
```

安全停止：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

## 十、常用排查命令

检查节点：

```bash
rosnode list | grep slam_v44
rosnode info /slam_v44_node
rosnode info /slam_v44_navigation_node
```

检查参数：

```bash
rosparam get /slam_v44_node/scan_topic
rosparam get /slam_v44_node/odom_topic
rosparam get /slam_v44_node/use_odom_prediction
rosparam get /slam_v44_node/require_odom_for_mapping
rosparam get /slam_v44_node/publish_map_to_odom_tf
```

检查地图没有更新的原因：

```bash
rostopic hz /slamware_ros_sdk_server_node/scan
rostopic hz /slamware_ros_sdk_server_node/odom
rostopic echo -n 1 /slam_v44/pose
rostopic echo -n 1 /slam_v44/map
```

如果日志出现：

```text
waiting for odom before inserting scans into fixed map frame
```

说明没有收到 odom。要么修正 `odom_topic`，要么临时允许无 odom 建图：

```bash
roslaunch slam_v44 slam_v44.launch require_odom_for_mapping:=false
```

检查规划失败：

```bash
rostopic echo -n 1 /slam_v44/map
rostopic echo -n 1 /slam_v44/pose
rosrun tf tf_echo map base_link
```

如果目标点在未知区域，默认 `planner_allow_unknown: false` 会拒绝规划。可以在 `config/slam_v44.yaml` 里临时改成：

```yaml
planner_allow_unknown: true
```

## 十一、关键配置文件

配置文件：

```text
config/slam_v44.yaml
```

常改参数：

```yaml
scan_topic: "/slamware_ros_sdk_server_node/scan"
odom_topic: "/slamware_ros_sdk_server_node/odom"

map_frame: "map"
odom_frame: "odom"
base_frame: "base_link"

publish_tf: true
publish_map_to_odom_tf: true
use_odom_prediction: true
require_odom_for_mapping: true

map_resolution: 0.05
map_width_m: 40.0
map_height_m: 40.0
map_origin_x: -20.0
map_origin_y: -20.0
```

## 十二、推荐第一次上机流程

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find slam_v44

rostopic echo -n 1 /slamware_ros_sdk_server_node/scan
rostopic echo -n 1 /slamware_ros_sdk_server_node/odom

roslaunch slam_v44 slam_v44.launch start_navigation:=false

rostopic echo -n 1 /slam_v44/pose
rostopic echo -n 1 /slam_v44/map
rosrun tf tf_echo map odom
```

如果这些都正常，再开 RViz 和导航调试。


