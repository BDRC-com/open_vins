# OpenVINS 配置、运行与测试说明

本文档面向当前工作区 `/home/ros2/slam`，用于说明 OpenVINS 在本地 ROS 2 Jazzy 环境下的常用配置方式、运行入口、测试方法和排障思路。

## 1. 代码结构

当前仓库根目录为 `/home/ros2/slam/src/open_vins`。

与日常开发最相关的目录如下：

1. `ov_msckf`
   主体 VIO 运行包，ROS 订阅、状态估计、RViz 可视化都在这里。
2. `ov_init`
   初始化模块，包含静态初始化和动态初始化逻辑。
3. `ov_core`
   核心公共库，包含前端特征跟踪、特征数据库、相机模型和公共工具。
4. `config`
   不同数据集和传感器的配置文件目录。
5. `docs`
   官方 doxygen 文档源码。

## 2. 环境准备

本工作区统一通过根目录环境脚本进入：

```bash
source /home/ros2/slam/slam_dev_env.sh
```

这个脚本会优先加载 ROS 2 Jazzy，并自动暴露 `OPEN_VINS_ROOT` 以及 `openvins_ws` 辅助函数。

常用进入方式：

```bash
source /home/ros2/slam/slam_dev_env.sh
openvins_ws
```

## 3. 配置文件结构

OpenVINS 主配置文件一般命名为 `estimator_config.yaml`。

当前仓库中最常用的几份配置有：

1. `config/euroc_mav/estimator_config.yaml`
   官方默认 EuRoC 配置。
2. `config/euroc_mav/estimator_config_mh01_launch.yaml`
   针对当前工作区 MH_01_easy bring-up 调整过的 KLT 配置。
3. `config/euroc_mav/estimator_config_mh01_launch_orb.yaml`
   用于对照测试的 ORB 前端配置。

主配置可以分成下面几组。

### 3.1 估计器总体开关

典型字段：

- `use_fej`
- `integration`
- `use_stereo`
- `max_cameras`

作用：

1. `use_fej` 控制是否使用 first-estimate Jacobians。
2. `integration` 控制 IMU 离散积分方式，例如 `rk4`。
3. `use_stereo` 控制双目是否按 stereo 模式联合使用。
4. `max_cameras` 控制系统中启用的相机数量。

### 3.2 在线标定开关

典型字段：

- `calib_cam_extrinsics`
- `calib_cam_intrinsics`
- `calib_cam_timeoffset`
- `calib_imu_intrinsics`
- `calib_imu_g_sensitivity`

作用：

1. `true` 表示这些量会进入状态并在运行中被优化。
2. `false` 表示使用外部配置值并固定不动。

在本工作区的 bring-up 配置里，通常先关闭相机外参、内参和时间偏移在线标定，以降低初始化负担并缩小排障范围。

### 3.3 滑窗与更新规模

典型字段：

- `max_clones`
- `max_slam`
- `max_slam_in_update`
- `max_msckf_in_update`
- `dt_slam_delay`

作用：

1. 控制滑动窗口克隆数量。
2. 控制状态中保留多少 SLAM 特征。
3. 控制单次更新参与的特征数量。
4. 在实时性和精度之间做平衡。

### 3.4 初始化参数

典型字段：

- `init_window_time`
- `init_imu_thresh`
- `init_max_disparity`
- `init_max_features`
- `init_dyn_use`
- `init_dyn_min_deg`

初始化逻辑位于 `ov_init/src/init/InertialInitializer.cpp`。

OpenVINS 会综合使用：

1. IMU 激励
2. 图像特征视差
3. 静态初始化条件
4. 动态初始化条件

来判断当前是否可以进入初始化。

当前工作区里，MH_01_easy 在默认 EuRoC 配置下容易卡静态初始化，因此 bring-up 配置中将：

```yaml
init_dyn_use: true
```

以允许系统在起步阶段使用动态初始化。

### 3.5 前端特征跟踪参数

典型字段：

- `use_klt`
- `num_pts`
- `fast_threshold`
- `grid_x`
- `grid_y`
- `min_px_dist`
- `knn_ratio`
- `track_frequency`
- `histogram_method`

其中最重要的是 `use_klt`：

1. `true` 表示使用 KLT 光流前端。
2. `false` 表示使用 ORB descriptor 匹配前端。

代码分支位于 `ov_msckf/src/core/VioManager.cpp`。

经验上：

1. KLT 更适合作为先跑通系统的默认前端。
2. ORB 更适合作为对照实验或前端鲁棒性测试。

### 3.6 外部传感器配置入口

典型字段：

- `relative_config_imu`
- `relative_config_imucam`

EuRoC 默认对应：

1. `config/euroc_mav/kalibr_imu_chain.yaml`
2. `config/euroc_mav/kalibr_imucam_chain.yaml`

这些文件负责提供：

1. IMU 噪声参数。
2. 相机内参与畸变。
3. 相机与 IMU 外参。
4. 相机 topic 名称。
5. 相机与 IMU 时间偏移。

## 4. 配置如何进入程序

ROS 2 真机/rosbag 入口位于：

- `ov_msckf/src/run_subscribe_msckf.cpp`

程序启动流程为：

1. 读取 `config_path`。
2. 用 `YamlParser` 打开配置。
3. 构造 `VioManagerOptions`。
4. 调用 `print_and_load(parser)` 载入估计器、前端、噪声和状态参数。
5. 构造 `VioManager`。
6. 构造 `ROS2Visualizer`。
7. 建立 ROS 订阅并进入 executor spin。

参数与 YAML 的映射定义主要在：

- `ov_msckf/src/core/VioManagerOptions.h`

如果要查某个 YAML 字段实际被谁使用，优先从这里找。

## 5. 运行入口

ROS 2 launch 入口位于：

- `ov_msckf/launch/subscribe.launch.py`

它会：

1. 启动 `run_subscribe_msckf`。
2. 根据 `rviz_enable` 决定是否启动 RViz2。
3. 根据 `config` 或 `config_path` 确定加载哪份配置。

最重要的 launch 参数有：

1. `namespace`
   默认是 `ov_msckf`。
2. `rviz_enable`
   控制是否启动 RViz2。
3. `config`
   使用 `config/<name>/estimator_config.yaml`。
4. `config_path`
   直接指定一份具体配置文件。

ROS 2 RViz 配置位于：

- `ov_msckf/launch/display_ros2.rviz`

默认会查看的关键 topic 包括：

1. `/ov_msckf/pathimu`
2. `/ov_msckf/pathgt`
3. `/ov_msckf/trackhist`
4. `/ov_msckf/points_msckf`

## 6. 构建方法

先加载环境：

```bash
source /home/ros2/slam/slam_dev_env.sh
```

常用增量构建：

```bash
colcon build --packages-select ov_core ov_init ov_msckf
```

需要时也可以全量构建：

```bash
colcon build
```

## 7. 常用运行方法

### 7.1 默认 EuRoC 配置运行

```bash
source /home/ros2/slam/slam_dev_env.sh
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config.yaml \
  rviz_enable:=true
```

### 7.2 当前工作区推荐的 KLT bring-up 运行方式

```bash
source /home/ros2/slam/slam_dev_env.sh
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config_mh01_launch.yaml \
  rviz_enable:=true
```

### 7.3 ORB 对照运行方式

```bash
source /home/ros2/slam/slam_dev_env.sh
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config_mh01_launch_orb.yaml \
  rviz_enable:=true
```

### 7.4 EuRoC MH_01_easy bag 播放

```bash
source /home/ros2/slam/slam_dev_env.sh
ros2 bag play /home/ros2/slam_datasets/machine_hall/MH_01_easy/MH_01_easy \
  --clock 100.0 \
  --topics /imu0 /cam0/image_raw /cam1/image_raw
```

## 8. 仿真运行与测试

仿真入口位于：

- `ov_msckf/src/run_simulation.cpp`

常用仿真配置位于：

- `config/rpng_sim/estimator_config.yaml`

适合用于：

1. 检查算法流程本身是否正常。
2. 验证仿真 RMSE、NEES 和 timing 输出。
3. 在不依赖真实 bag 的情况下做快速回归。

## 9. 真实数据测试建议

当前工作区中已经验证过的真实数据路径为：

- `/home/ros2/slam_datasets/machine_hall/MH_01_easy/MH_01_easy`

建议测试顺序如下：

1. 先用 `estimator_config_mh01_launch.yaml` 运行，确认 KLT 前端路径稳定。
2. 再用 `estimator_config_mh01_launch_orb.yaml` 做 ORB 分支对照。
3. 检查 RViz 是否能看到 `/ov_msckf/pathimu`、`/ov_msckf/trackhist` 和 `points_msckf`。
4. 检查控制台是否持续输出 `q_GtoI`、`p_IinG` 和 `dist`。

## 10. 如何判断系统是否真的跑起来了

### 10.1 看节点

```bash
ros2 node list
```

正常至少应看到：

1. `/ov_msckf/run_subscribe_msckf`
2. 如果开了 RViz，还应有 `/rviz`

### 10.2 看 topic

```bash
ros2 topic info /ov_msckf/pathimu
ros2 topic info /ov_msckf/poseimu
ros2 topic info /ov_msckf/trackhist
```

### 10.3 看控制台状态输出

如果只是一直初始化失败，通常会反复出现 `[init]: ...` 日志。

如果已经进入估计阶段，会出现持续状态输出，例如：

```text
q_GtoI = ... | p_IinG = ... | dist = ...
```

这是最可靠的运行信号。

## 11. 常见日志解释

### 11.1 `failed static init: no accel jerk detected`

含义：

1. 静态初始化没有检测到足够的 IMU 激励。
2. 启动段不够静止，或者不满足 jerk 判据。

常见处理：

1. 打开 `init_dyn_use`。
2. 调整起始 bag 段。
3. 先关闭在线标定参数，减少初始化自由度。

### 11.2 `not enough feats to compute disp: 0,0 < 15`

含义：

1. 初始化想通过特征视差判断运动状态。
2. 但两个时间窗口里可用的跨帧特征轨迹不足 15 条。

注意：

1. 这不等于当前图像完全没有角点。
2. 它表示当前数据库里还没形成足够多可用于视差统计的有效轨迹。

如果只在启动初期短暂出现，一般是正常现象。
如果一直出现并且始终没有位姿输出，才需要进一步调前端参数。

### 11.3 RViz 没有轨迹

优先检查：

1. 是否使用了 launch 入口而不是直接 `ros2 run`。
2. 当前 namespace 是否是 `ov_msckf`。
3. RViz 订阅的是 `/ov_msckf/pathimu`，而不是 `/pathimu`。

## 12. 当前工作区的已知兼容性修复

当前仓库在 ROS 2 Jazzy 下已经完成以下适配：

1. ROS 2 头文件更新到 `.hpp` 路径。
2. `ov_msckf` 显式补齐 `message_filters`、`tf2`、`visualization_msgs` 依赖。
3. `ov_init` ROS 2 测试目标补齐 `ament_libraries`。
4. `run_simulation` 和 `run_subscribe_msckf` 在 `rclcpp::shutdown()` 前显式 reset，避免退出阶段插件卸载顺序导致崩溃。
5. ORB 前端在 `use_klt: false` 时，`TrackDescriptor::robust_match()` 已补充空描述子和形状不匹配保护，避免 OpenCV `batchDistance` 断言崩溃。

## 13. 推荐日常工作流

如果目标是稳定开发和验证，建议固定两套工作流。

### 13.1 稳定跑通工作流

1. 构建：

```bash
source /home/ros2/slam/slam_dev_env.sh
colcon build --packages-select ov_core ov_init ov_msckf
```

2. 启动：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config_mh01_launch.yaml \
  rviz_enable:=true
```

3. 播放 MH_01_easy：

```bash
ros2 bag play /home/ros2/slam_datasets/machine_hall/MH_01_easy/MH_01_easy \
   --clock 100.0 \
   --topics /imu0 /cam0/image_raw /cam1/image_raw
```

### 13.2 前端对照工作流

将配置切到：

- `config/euroc_mav/estimator_config_mh01_launch_orb.yaml`

然后重复同样的 launch 和 bag 播放流程，对比 KLT 与 ORB 的初始化速度、稳定性和轨迹输出。

### 13.3 单项目回归入口

如果目标是做 OpenVINS 仓库内部的日常开发回归，直接使用：

```bash
source /home/ros2/slam/slam_dev_env.sh
bash /home/ros2/slam/src/open_vins/scripts/run_regression_mh01.sh
```

该脚本会：

1. 固定使用当前工作区的 MH_01_easy 基线 bag。
2. 通过 `subscribe.launch.py` 启动 OpenVINS。
3. 打开 `save_total_state`，将状态文件落到 `/home/ros2/slam/vio_output/openvins/regressions/...`。
4. 把每次运行的日志与状态文件隔离到独立 run 目录，便于开发迭代前后直接对比。

ORB 前端回归可切换为：

```bash
bash /home/ros2/slam/src/open_vins/scripts/run_regression_mh01.sh \
   /home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config_mh01_launch_orb.yaml
```

## 14. 总结

可以把 OpenVINS 的使用顺序理解为：

1. 先看 `config` 目录确定传感器和数据集配置。
2. 再看 `VioManagerOptions.h` 确认 YAML 字段实际映射到哪里。
3. 通过 `subscribe.launch.py` 启动 ROS 2 入口。
4. 用 RViz 和控制台状态输出来判断是否真的进入估计。
5. 在初始化不过、没有 path、前端崩溃时，分别从初始化参数、namespace、前端跟踪分支三条线排查。

对于当前工作区，推荐优先使用 `estimator_config_mh01_launch.yaml` 作为基线，再用 `estimator_config_mh01_launch_orb.yaml` 做前端对照实验。