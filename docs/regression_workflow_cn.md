# OpenVINS 单项目回归工作流

当前仓库已提供一个面向日常开发迭代的单项目回归入口。

## 基线入口

运行默认 EuRoC MH_01_easy 回归：

```bash
source /home/ros2/slam/slam_dev_env.sh
bash /home/ros2/slam/src/open_vins/scripts/run_regression_mh01.sh
```

默认基线：

- 配置：`config/euroc_mav/estimator_config_mh01_launch.yaml`
- 数据：`/home/ros2/slam_datasets/machine_hall/MH_01_easy/MH_01_easy`

每次运行会把产物隔离到：

- `/home/ros2/slam/vio_output/openvins/regressions/<config_name>/<run_tag>/`

每次运行包含：

- `openvins.log`：launch 和节点日志
- `state_estimate.txt`：估计状态输出
- `state_deviation.txt`：状态方差输出

## 切换 ORB 对照配置

```bash
source /home/ros2/slam/slam_dev_env.sh
bash /home/ros2/slam/src/open_vins/scripts/run_regression_mh01.sh \
  /home/ros2/slam/src/open_vins/config/euroc_mav/estimator_config_mh01_launch_orb.yaml
```

## 常用环境变量

- `REGRESSION_RUN_TAG=my_branch_head`
- `REGRESSION_BAG_TIMEOUT_SECONDS=0`：播放完整 bag
- `REGRESSION_START_DELAY_SECONDS=5`：慢机上增加启动等待
- `OPENVINS_REGRESSION_CLOCK_RATE=100.0`：调整 bag 播放时钟倍率

## 使用定位

这套脚本用于 OpenVINS 仓库内的日常回归，主要回答三类问题：

1. 当前分支是否还能稳定启动并进入估计。
2. KLT / ORB 前端切换后是否还能持续产出状态文件。
3. 初始化、前端或订阅改动后，日志和状态输出是否出现明显退化。