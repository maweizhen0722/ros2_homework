# 导航组第三次课作业

地图文件是 `mymap.pgm` 和 `mymap.yaml`。

程序读地图，按阈值分清墙、空地和没探明的地方，把墙往外加粗一点，给车的宽度留余量。
在 RVIZ 里用 `2D Goal Pose` 点一个终点，用 A\* 找一条绕开墙的最短路，
再把算出来的格子换成米、发到 `/planned_path`，RVIZ 把相邻点连成红线。

## 跑

```bash
source /opt/ros/<发行版>/setup.bash
colcon build --packages-select rm_nav_assignment
source install/setup.bash
ros2 launch rm_nav_assignment homework.launch.py
```

启动后在白色空地上点终点就出红线；`2D Pose Estimate` 可以改起点。
在 VMware 虚拟机（Ubuntu + ROS2）里跑过。

`rviz_result.png` 是在虚拟机里运行时截的图，里面有规划出来的路径。
