# License_Plate_Number_Detect
An assignment of digital image processing

## Requirement

- OpenCV 4.5.1
- ROS2 Galactic

## Start

### 环境配置

参考[ROS2 Galactic Installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)配置环境

参考[Foxglove Studio Installation](https://foxglove.dev/docs/studio/installation)配置环境

### 拉取

```bash
mkdir -p ./ros_ws/src
cd ./ros_ws/src
git clone https://github.com/FaterYU/License_Plate_Number_Detect.git
```

### 编译

```bash
cd ./ros_ws
source /opt/ros/galactic/setup.zsh
colcon build --symlink-install
```

### 运行

```bash
source install/setup.zsh
ros2 launch license_detector_bringup license_detector_bringup.launch.py
```

### 可视化

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
