# License_Plate_Number_Detect
An assignment of digital image processing

## Requirement

- OpenCV 4.5.1
- ROS2 Galactic

## Start

```bash
colcon build --symlink-install
source install/setup.zsh
ros2 launch license_detector license_detector.launch.py
```

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
