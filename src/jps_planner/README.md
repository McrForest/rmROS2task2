# jps_planner

基于 JPS（Jump Point Search，跳点搜索）算法的 ROS 2 路径规划演示包（C++）。

## 功能

- 在 RViz 中加载 PGM/YAML 栅格地图（`/map` 话题，`nav_msgs/OccupancyGrid`）
- **障碍物膨胀**：机器人按 `robot_size × robot_size` 栅格的方形足迹处理（默认 10×10），障碍物向外膨胀 `robot_size/2` 格（切比雪夫距离，多源 BFS），规划在膨胀地图上以质点进行，保证路径不贴墙
- 膨胀结果另发 `/inflated_map` 话题，RViz 里勾选 InflatedMap 图层即可查看（黑=原障碍，灰=膨胀区）
- 用 RViz 的 **2D Pose Estimate** 标定初始位姿
- 用 RViz 的 **2D Goal Pose** 标定目标点，自动调用 JPS 算法寻路
- 规划出的路径以 `/planned_path`（`nav_msgs/Path`）显示在地图上，起点（绿球）、终点（红球）以 Marker 显示

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/Workspace/RM/jps_planner_ws
colcon build
```

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/Workspace/RM/jps_planner_ws/install/setup.bash
ros2 launch jps_planner jps_rviz.launch.py
```

默认地图为 `~/Workspace/RM/rosBasicsHomework2/mymap.yaml`，可用参数指定其它地图：

```bash
ros2 launch jps_planner jps_rviz.launch.py map_yaml:=/path/to/your_map.yaml
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 原始栅格地图（transient local） |
| `/inflated_map` | `nav_msgs/OccupancyGrid` | 发布 | 膨胀后的规划地图（100=障碍，70=膨胀区） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | RViz 2D Pose Estimate |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 订阅 | RViz 2D Goal Pose |
| `/planned_path` | `nav_msgs/Path` | 发布 | JPS 规划结果 |
| `/jps_markers` | `visualization_msgs/MarkerArray` | 发布 | 起点/终点标记 |

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_yaml` | `~/Workspace/RM/rosBasicsHomework2/mymap.yaml` | 地图 yaml 路径 |
| `map_frame` | `map` | 地图坐标系 |
| `robot_size` | `10` | 机器人足迹边长（栅格数），膨胀半径取 `(robot_size+1)/2` |

改机器人大小示例：`ros2 run jps_planner jps_planner_node --ros-args -p robot_size:=14`

## 说明

- JPS 实现遵循 Harabor & Grastien 的经典规则：邻居剪枝 + 强迫邻居（forced neighbors）+ 跳点递归，开放列表采用欧氏距离启发式。
- 起点/终点若点到了障碍物上，会自动吸附到最近的自由栅格（半径约 0.5 m 内）。
- 未知区域（-1）按障碍物处理。
