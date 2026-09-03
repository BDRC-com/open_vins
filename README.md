# 在Docker里运行OpenVINS ROS2 Humble 22.04

## 安装

**注意**：在继续之前需要先安装Docker！

```bash
git clone https://github.com/BDRC-com/open_vins.git
cd open_vins
git switch wan
export VERSION=ros2_22_04
docker build -t ov_$VERSION -f Dockerfile_$VERSION .
```

安装完成后将如下命令加入到bashrc里：

```bash
nano ~/.bashrc # 打开bashrc，添加如下命令

xhost + &> /dev/null
export DOCKER_CATKINWS=/home/username/workspace/catkin_ws_ov
export DOCKER_DATASETS=/home/username/datasets
alias ov_docker="docker run -it --net=host --gpus all \
    --env=\"NVIDIA_DRIVER_CAPABILITIES=all\" --env=\"DISPLAY\" \
    --env=\"QT_X11_NO_MITSHM=1\" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
    --mount type=bind,source=$DOCKER_CATKINWS,target=/catkin_ws \
    --mount type=bind,source=$DOCKER_DATASETS,target=/datasets $1"
    
source ~/.bashrc # 保存并退出后运行
```

随后进入Docker并编译：

```bash
ov_docker ov_ros2_22_04 bash # 进入docker
cd catkin_ws
colcon build --event-handlers console_cohesion+
source install/setup.bash
# 使用如下测试
ros2 run ov_eval plot_trajectories none src/open_vins/ov_data/sim/udel_gore.txt
ros2 run ov_msckf run_simulation src/open_vins/config/rpng_sim/estimator_config.yaml
```

## 运行OpenVINS

首先启动Docker：

```bash
ov_docker ov_ros2_22_04 bash
source catkin_ws/install/setup.bash
source /opt/ros/humble/setup.bash
```

成功编译后即可运行OpenVINS，在实时摄像头上运行：

```bash
# 需要启动无人机摄像头
ssh ros2@192.168.2.200
./start_camera.sh
echo $ROS_DOMAIN_ID # 应该是200
# 回到docker
export ROS_DOMAIN_ID=200 # 将其改为和无人机一样的值
ros2 topic list # 如果能看到/cam0/image_raw等，则说明正确
ros2 topic hz /cam0/image_raw # 如果没有25hz，则：
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # 无人机上也要改
# 启动OpenVINS
ros2 launch ov_msckf subscribe.launch.py \
config:=cyperstereo_c76_8mm_752x480 \
rviz_enable:=true
```

如果要在录制的包上运行OpenVINS，则需要启动另一个docker：

```bash
docker ps # 查看之前的容器ID
docker exec -it <容器ID> bash
source catkin_ws/install/setup.bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=200
ros2 bag play <需要播放的包>
```

修改了`estimator_config.yaml` 后要重新构建：

```bash
colcon build --packages-select ov_msckf --symlink-install
```

