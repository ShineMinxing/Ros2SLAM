# Ros2SLAM 🛰️

面向 **ROS 2 Humble / Ubuntu 22.04** 的多 SLAM 算法集成仓库，当前重点支持 **Unitree Go2 / 移动机器人平台**。本仓库将 SLAM 工作流封装为可复现的 ROS 2 包，统一输入/输出结构，便于在实验环境中快速部署与扩展。

> 当前版本已提供：**Cartographer 3D SLAM（Lidar + IMU + Odom）完整流程**：在线建图 → 自动录包 → 话题重命名 → 保存 pbstream → 离线导出 3D 点云。

---

## 📦 环境与依赖

* Ubuntu 22.04
* ROS 2 Humble（建议安装 `ros-humble-desktop`）
* 已配置工作空间示例：`~/ros2_ws/LeggedRobot`

### 1️⃣ Cartographer 与基础工具

```bash
sudo apt update
sudo apt install \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-cartographer-rviz \
  ros-humble-rviz2 \
  ros-humble-ros2bag \
  ros-humble-rosbag2-storage-default-plugins
```

### 2️⃣ 必要配套仓库：Ros2Tools

`Ros2SLAM` 依赖 [Ros2Tools](https://github.com/ShineMinxing/Ros2Tools) 中的 **rosbagtopic_rename** 节点，用于将录制的 rosbag 话题重命名为 Cartographer 期望的标准名称。

安装方式示例：

```bash
cd ~/ros2_ws/LeggedRobot/src

# SLAM 主仓库
git clone --recursive https://github.com/ShineMinxing/Ros2SLAM.git

# 配套工具仓库（包含 rosbagtopic_rename）
git clone --recursive https://github.com/ShineMinxing/Ros2Tools.git

cd ~/ros2_ws/LeggedRobot
colcon build --packages-select cartographer_3d rosbagtopic_rename
source install/setup.bash
```

> 如需使用 Ros2Tools 中的其他工具，可直接对整个工作空间执行 `colcon build`。

---

## 📂 仓库结构

```text
Ros2SLAM/
├── cartographer_3d/      # 已实现的 3D SLAM 工作流
│   ├── config/           # Lua 配置（SLAM + assets writer）
│   ├── launch/           # 在线 SLAM + 自动录包
│   └── rviz/             # RViz2 配置
│
├── <future_slam_x>/      # 预留的其他 SLAM 方法目录（结构与 cartographer_3d 一致）
│   ├── config/
│   ├── launch/
│   └── rviz/
│
├── local/                # SLAM 结果与中间文件
│   ├── go2_slam_bag_*    # 在线录制的 rosbag 目录
│   ├── go2_slam.pbstream # 保存的 Cartographer SLAM 状态
│   └── go2_slam_assets*  # 离线导出的 3D 点云（PLY）
│
└── README.md
```

> 建议将 `local/` 保持为可写目录，用于存放所有实验生成的数据与中间结果。

---

## 🛰️ Cartographer 3D SLAM 工作流

### 输入 / 输出概览

**主要输入话题（来自 Go2 / 机器人底盘）：**

| 话题名             | 类型                        | 说明           |
| --------------- | ------------------------- | ------------ |
| `/SMX/Go2Lidar` | `sensor_msgs/PointCloud2` | 3D 激光点云      |
| `/SMX/Go2IMU`   | `sensor_msgs/Imu`         | 惯性测量单元       |
| `/SMX/Odom`     | `nav_msgs/Odometry`       | 轮式或融合里程计     |
| `/tf`           | `tf2_msgs/TFMessage`      | 动态 TF 树      |
| `/tf_static`    | `tf2_msgs/TFMessage`      | 静态 TF（传感器标定） |

**关键输出：**

| 结果            | 形式              | 存放位置（默认）                                           |
| ------------- | --------------- | -------------------------------------------------- |
| 在线 SLAM 地图    | TF / topic      | RViz2 中查看 `/map`、`/map_SLAM` 等                     |
| SLAM 状态       | `*.pbstream` 文件 | `local/go2_slam.pbstream`                          |
| 离线 3D 点云      | `*.ply` 文件      | `local/go2_slam_assets_points.ply` 等               |
| 原始/重命名 rosbag | 目录 + `.db3`     | `local/go2_slam_bag*/` / `local/go2_slam_bag_0/` 等 |

---

### 🔄 完整 SLAM 使用流程（精简版）

以下步骤假定工作空间为 `~/ros2_ws/LeggedRobot`，且已完成上文中的依赖安装与编译。

#### ① 启动在线 3D SLAM（自动录包）

```bash
ros2 launch cartographer_3d cartographer_3d.launch.py
```

该 launch 将执行：

1. 启动 Cartographer 3D 节点，订阅 `/SMX/Go2Lidar`、`/SMX/Go2IMU`、`/SMX/Odom`、`/tf`、`/tf_static`。
2. 同时启动 `ros2 bag record`，自动将上述话题录制到：

   * `~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_bag_*`
3. 如使用配套的 RViz2 配置，可实时查看地图构建与轨迹。

> 建议让机器人在目标环境中稳定行走一圈，以获得覆盖较好的点云与里程计轨迹。

---

#### ② 使用 Ros2Tools 重命名 rosbag 话题

为了让离线 assets writer 使用统一的话题名，需要将录制的 rosbag 中自定义话题重命名为：`/points2`、`/imu`、`/odom`。

1. 在 `Ros2Tools/config.yaml` 中配置：

```yaml
rosbagtopic_rename_node:
  ros__parameters:
    input_bag:  "~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_bag"
    output_bag: "~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_bag_0"
    topic_mappings:
      "/SMX/Go2Lidar": "/points2"
      "/SMX/Go2IMU":   "/imu"
      "/SMX/Odom":     "/odom"
```

2. 运行重命名节点（离线工具，无需其他节点运行）：

```bash
cd ~/ros2_ws/LeggedRobot
source install/setup.bash
ros2 run rosbagtopic_rename rosbagtopic_rename_node
```

完成后，会生成：

```text
~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_bag_0/
  ├── go2_slam_bag_0_0.db3
  └── metadata.yaml
```

---

#### ③ 保存 Cartographer SLAM 状态（pbstream）

在在线 SLAM 运行完成后，通过服务调用将图优化结果写入 pbstream 文件：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam.pbstream', include_unfinished_submaps: true}"
```

执行成功后，将在 `local/` 目录下生成：

```text
~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam.pbstream
```

> 可在多次实验中复用同一个 pbstream，或按日期与场景进行归档。

---

#### ④ 使用 assets writer 导出最终 3D 点云（PLY）

利用 Cartographer 自带的 assets writer，将 pbstream 与重命名后的 rosbag 组合，离线生成 3D 点云：

```bash
ros2 run cartographer_ros cartographer_assets_writer -- \
  -configuration_directory ~/ros2_ws/LeggedRobot/src/Ros2SLAM/cartographer_3d/config \
  -configuration_basename go2_assets_writer_3d.lua \
  -bag_filenames ~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_bag_0/go2_slam_bag_0_0.db3 \
  -pose_graph_filename ~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam.pbstream \
  -output_file_prefix ~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/go2_slam_assets
```

完成后，`local/` 目录中将包含：

```text
~/ros2_ws/LeggedRobot/src/Ros2SLAM/local/
  ├── go2_slam_assets_points.ply   # 主要 3D 点云结果
  └── （可能附带其他中间文件）
```

可使用 **CloudCompare / MeshLab / RViz2** 对 `*.ply` 文件进行可视化与后处理。

---

## 🧩 扩展与复用建议

* 其他 SLAM 算法（如基于视觉、紧耦合激光-惯导等）可按照 `cartographer_3d/` 的目录结构，新增到 `<future_slam_x>/` 中，以保持统一的 `config/ + launch/ + rviz/` 组织方式。
* 建议保持话题命名与数据记录路径的一致性（如统一使用 `local/` 目录），以便在对比不同 SLAM 算法时共享录制数据与可视化脚本。

---

## 👤 Author

**Minxing Sun (ShineMinxing)**

GitHub: [https://github.com/ShineMinxing](https://github.com/ShineMinxing)

相关仓库：

* Ros2Tools：传感器接口与实用工具集（包含 rosbagtopic_rename 等）
* Ros2Go2Base / Ros2Go2Estimator：控制与状态估计框架

本仓库将根据实验需求持续完善，欢迎通过 Issue / PR 交流使用体验与改进建议。
