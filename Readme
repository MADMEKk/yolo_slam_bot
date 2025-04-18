# YOLOBot - ROS2 Robot with YOLOv8 Object Detection

A ROS2-based robot implementation that combines navigation, SLAM, and real-time object detection using YOLOv8.

## Project Structure

```
.
├── build/                  # Build artifacts
├── install/                # ROS2 installation files
├── log/                    # ROS2 log files
├── models/                 # YOLOv8 model files
└── src/                    # Source code
    ├── yolobot_control/   # Robot control package
    ├── yolobot_description/ # Robot URDF and meshes
    ├── yolobot_gazebo/    # Gazebo simulation files
    ├── yolobot_recognition/ # YOLOv8 object detection
    ├── yolobot_slam/      # SLAM implementation
    └── yolov8_msgs/       # Custom ROS2 messages
```

## Prerequisites

- ROS2 (Humble or later)
- Gazebo
- Python 3.8+
- YOLOv8
- OpenCV

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd yolobot
```

2. Install dependencies:
```bash
sudo apt install ros-humble-gazebo-ros
pip install ultralytics opencv-python
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

1. Launch the robot in Gazebo:
```bash
ros2 launch yolobot_gazebo yolobot_launch.py
```

2. Start object detection:
```bash
ros2 launch yolobot_recognition yolo_recognition.launch.py
```

3. Launch SLAM:
```bash
ros2 launch yolobot_slam slam_launch.py
```

## Features

- Real-time object detection using YOLOv8
- Autonomous navigation
- SLAM (Simultaneous Localization and Mapping)
- Gazebo simulation support
- Custom ROS2 messages for object detection results

## License

[License details here]

## Contributing

[Contribution guidelines here]