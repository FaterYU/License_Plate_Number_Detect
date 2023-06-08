# License_Plate_Number_Detect
An assignment of digital image processing

## Requirement

- OpenCV 4.5.1
- ROS2 Galactic

## 启动

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

## 节点

- [license_detector_bringup](/license_detector_bringup)
    启动文件
- [image_publisher](/image_publisher)
    静态图像发布节点
- [license_detector_interfaces](/license_detector_interfaces)
    定义识别节点的接口及用于Debug的消息
- [license_extract](/license_extract)
    车牌检测并分割字符节点，订阅静态图像，发布二值化处理后的车牌字符图像
- [char_detector](/char_detector)
    车牌字符识别节点，订阅二值化处理后的车牌字符图像，发布识别结果

[说明文档](./wiki/README.md)