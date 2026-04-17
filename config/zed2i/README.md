# OpenVINS ZED2i Configuration

目录: `src/open_vins/config/zed2i`

文件:
- `estimator_config.yaml`: 当前设备对齐后的基础配置
- `estimator_config_stable.yaml`: 更保守的真机稳态配置，优先用于排查“飞掉”问题
- `estimator_config_motion_init.yaml`: 面向“起步时就在运动”的动态初始化配置
- `estimator_config_motion_timeoffset.yaml`: 面向“已经能初始化，但初始化后很快发散”的动态初始化 + 时间偏移在线估计配置
- `kalibr_imu_chain.yaml`: IMU 噪声参数（已对齐 runtime dump）
- `kalibr_imucam_chain.yaml`: 相机与 IMU 外参 + ZED2i 图像 topic

## 启动

先编译并加载环境：

```bash
source /home/ros2/slam/slam_dev_env.sh
colcon build --packages-select ov_msckf
```

优先直接指向源码树配置，避免 install 树旧文件干扰：

```bash
ros2 launch ov_msckf subscribe.launch.py \
	config_path:=/home/ros2/slam/src/open_vins/config/zed2i/estimator_config.yaml \
	rviz_enable:=true
```

如果你要先走更保守、更稳的配置：

```bash
ros2 launch ov_msckf subscribe.launch.py \
	config_path:=/home/ros2/slam/src/open_vins/config/zed2i/estimator_config_stable.yaml \
	rviz_enable:=true
```

如果启动时相机已经在运动，优先用动态初始化配置：

```bash
ros2 launch ov_msckf subscribe.launch.py \
	config_path:=/home/ros2/slam/src/open_vins/config/zed2i/estimator_config_motion_init.yaml \
	rviz_enable:=true
```

如果已经能初始化，但初始化后很快跑飞，优先试时间偏移在线估计版本：

```bash
ros2 launch ov_msckf subscribe.launch.py \
	config_path:=/home/ros2/slam/src/open_vins/config/zed2i/estimator_config_motion_timeoffset.yaml \
	rviz_enable:=true
```

`estimator_config_stable.yaml` 的差异是：
- 更长初始化窗口，减少刚启动时的误初始化
- 更保守的初始化视差阈值
- 更低的前端负载，减少 HD720 下的跟踪抖动与 CPU 峰值
- 默认开启图像降采样作为排障基线

`estimator_config_motion_init.yaml` 的差异是：
- 打开 `init_dyn_use: true`，允许在启动时已经运动的情况下初始化
- 将 `init_max_disparity` 降到 `1.5`，减少被误判成“静止/jerk 静态初始化”的概率
- 将 `init_dyn_min_deg` 放宽到 `5.0`，适配更接近平移主导的起步动作
- 保留固定外参/内参/时间偏移，先把初始化自由度压住

`estimator_config_motion_timeoffset.yaml` 的差异是：
- 在 `motion_init` 的基础上打开 `calib_cam_timeoffset: true`
- 目标是处理“初始化成功，但相机和 IMU 时序仍有小常值偏差，导致短时间内发散”的情况
- 仍然保持外参、内参固定，不把自由度一次放太多
- 略微放宽更新像素噪声到 `1.5`，先提高真机容忍度

## 录包排障

如果需要定位为什么会飞，先让 bridge 单独运行，再用下面的脚本做一次带状态输出的联合录制：

```bash
bash /home/ros2/slam/src/open_vins/scripts/run_zed2i_debug_capture.sh
```

默认行为：
- 使用 `estimator_config_stable.yaml`
- 连续录制 60 秒
- 保存 rosbag、OpenVINS log、状态文件、配置快照、runtime 标定导出

产物目录：
- `/home/ros2/slam/vio_output/openvins/zed2i_debug/<run_tag>/`

默认录制的话题：
- `/zed2i/imu`
- `/zed2i/left/image_raw`
- `/zed2i/right/image_raw`
- `/zed2i/left/camera_info_raw`
- `/zed2i/right/camera_info_raw`
- `/ov_msckf/poseimu`
- `/ov_msckf/odomimu`
- `/ov_msckf/pathimu`
- `/ov_msckf/points_msckf`
- `/ov_msckf/points_slam`
- `/tf`
- `/tf_static`

也可以指定配置和录制时长：

```bash
bash /home/ros2/slam/src/open_vins/scripts/run_zed2i_debug_capture.sh \
	/home/ros2/slam/src/open_vins/config/zed2i/estimator_config.yaml \
	120
```

如果要手动停止，把第二个参数设为 `0`，然后按 `Ctrl-C`：

```bash
bash /home/ros2/slam/src/open_vins/scripts/run_zed2i_debug_capture.sh \
	/home/ros2/slam/src/open_vins/config/zed2i/estimator_config_stable.yaml \
	0
```

如果 ZED bridge 推送压缩图像，请将 `kalibr_imucam_chain.yaml` 里面 rostopic 改为 `/zed2i/...`，并把对应订阅 transport 改为 `compressed`。

