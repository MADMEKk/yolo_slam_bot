# YoloBot SLAM Module

This package implements SLAM (Simultaneous Localization and Mapping) functionality for the YoloBot using SLAM Toolbox. It enables the robot to create 2D maps of its environment using LiDAR sensor data while simultaneously tracking its position within the map.

## Features

- **2D SLAM Implementation**: Complete implementation of SLAM Toolbox for 2D mapping
- **LiDAR Integration**: A 360° LiDAR sensor for accurate environmental scanning
- **Map Management**: Tools for creating, saving, and manipulating maps
- **Visualization**: RViz configuration for viewing the map and robot position
- **Easy Control**: Keyboard teleop for intuitive robot control during mapping
- **Integrated Launch**: Combined launch files to easily start the entire system

## Components

This module includes the following components:

1. **LiDAR Sensor**: Added to the YoloBot URDF for environment scanning
2. **SLAM Toolbox Configuration**: Tuned parameters for optimal mapping
3. **Launch Files**: For starting SLAM independently or with the full system
4. **RViz Configuration**: Preset configuration for visualizing SLAM results
5. **Keyboard Teleop**: Simple utility for controlling the robot during mapping
6. **Documentation**: Comprehensive guidance on using the SLAM system

## Prerequisites

- ROS2 Humble or later
- slam_toolbox package
- Gazebo simulator
- RViz2
- Python 3.8+

## Installation

### Install SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

### Build the Package

```bash
# From the workspace root
cd /path/to/yolobot
colcon build --packages-select yolobot_slam
source install/setup.bash
```

## Usage Guide

### Starting SLAM with YoloBot

Launch the complete SLAM system along with the simulated robot:

```bash
source ~/ros2_ws/ros2_env/bin/activate  # If using a virtual environment
cd ~/memoir/slam_yolo/yolobot
source install/setup.bash
ros2 launch yolobot_slam yolobot_with_slam.launch.py
```

This will start:
- Gazebo simulation environment
- YoloBot with LiDAR sensor
- SLAM Toolbox for mapping
- RViz for visualization
- YOLO object detection (if available)

### Starting SLAM with Maze Environment

For better SLAM testing, you can use the maze world environment:

```bash
ros2 launch yolobot_slam yolobot_maze_slam.launch.py
```

The maze environment is specially designed for SLAM and includes:
- Complex maze structure with walls forming multiple paths and rooms
- LiDAR-friendly environment for better scan matching
- Various small objects distributed throughout the maze for YOLO detection
- Clear visualization of the mapping process and loop closure detection

The maze world is ideal for:
- Testing SLAM in a structured environment
- Evaluating loop closure capabilities
- Benchmarking mapping performance
- Training and demonstration purposes

### Running SLAM Toolbox Alone

If you want to run only the SLAM Toolbox node (e.g., to use with a real robot or a separately launched simulation):

```bash
ros2 launch yolobot_slam slam_toolbox.launch.py
```

### Controlling the Robot for Mapping

#### Using Keyboard Teleop

The keyboard teleop provides an intuitive way to control the robot during mapping:
```bash
source ~/ros2_ws/ros2_env/bin/activate && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/yolobot/cmd_vel
```

```bash
ros2 launch yolobot_slam teleop.launch.py
```

Keyboard controls:
- `w`: Move forward
- `x`: Move backward
- `a`: Rotate left
- `d`: Rotate right
- `q`/`e`/`z`/`c`: Move diagonally
- `s`: Emergency stop

#### Using ROS2 Topic

Alternatively, you can control the robot by publishing to the cmd_vel topic:

```bash
ros2 topic pub /yolobot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Map Management

#### Saving a Map

After mapping, save your map to file:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

This creates two files:
- `my_map.pgm`: The map image
- `my_map.yaml`: Map metadata

#### Serializing a Pose Graph

For more advanced map saving (preserving the full graph structure):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'my_graph'}"
```

#### Loading a Map Later

To load a previously saved map for localization:

1. Edit the config file to change mode from `mapping` to `localization`
2. Specify the map file:
   ```yaml
   map_file_name: /path/to/my_graph
   ```

## Visualizing LiDAR Data

The package includes an enhanced LiDAR visualization tool that provides a more detailed view of the LiDAR scans than the standard RViz display.

### Running the LiDAR Visualizer

```bash
ros2 launch yolobot_slam lidar_visualizer.launch.py
```

This will start a node that:
1. Subscribes to the `/yolobot/scan` topic
2. Creates visual representations of:
   - The current scan points (in red)
   - The laser rays (in green)
   - A history of previous scan points (in blue)

### Viewing the Visualization in RViz

After launching the visualizer, add these topics in RViz:
1. Add a MarkerArray display for `/lidar_visualization`
2. Add a MarkerArray display for `/lidar_history`

The visualizer provides:
- Real-time visualization of current LiDAR readings
- Visual rays showing the LiDAR beams
- Historical data display showing the scan trace as the robot moves
- Different colors for distinguishing current vs. historical data

This visualization is particularly helpful for:
- Understanding how the LiDAR is perceiving the environment
- Debugging sensor issues
- Visualizing the robot's path through the scan history
- Teaching and demonstrations of SLAM concepts

### Visualizing LiDAR Rays in Gazebo

You can also visualize the LiDAR rays directly in Gazebo:

1. Edit the robot's URDF file:
   ```bash
   nano ~/memoir/slam_yolo/yolobot/src/yolobot_description/robot/yolobot.urdf
   ```

2. Find the LiDAR sensor configuration (around line 400) and change `<visualize>false</visualize>` to `<visualize>true</visualize>`:
   ```xml
   <gazebo reference="lidar_link">
     <material>Gazebo/Black</material>
     <sensor type="ray" name="lidar">
       <pose>0 0 0 0 0 0</pose>
       <visualize>true</visualize>  <!-- Change this from false to true -->
       <!-- rest of the LiDAR configuration -->
     </sensor>
   </gazebo>
   ```

3. Rebuild and relaunch:
   ```bash
   cd ~/memoir/slam_yolo/yolobot
   colcon build --packages-select yolobot_description
   source install/setup.bash
   ros2 launch yolobot_slam yolobot_maze_slam.launch.py
   ```

4. Now you should see the LiDAR rays in Gazebo, which is helpful for understanding how the LiDAR interacts with the environment.

## SLAM Parameters

The SLAM behavior can be customized through the configuration file:

```
config/mapper_params_online_async.yaml
```

Key parameters:

- `mode`: Choose between `mapping` or `localization`
- `resolution`: Map resolution in meters
- `max_laser_range`: Maximum range to use from LiDAR scans
- `map_update_interval`: How frequently to update the map (seconds)
- `use_scan_matching`: Enable/disable scan matching
- `loop_search_maximum_distance`: Maximum distance for loop closure

## SLAM Toolbox RViz Plugin

The RViz configuration includes the SLAM Toolbox plugin which offers additional functionality:

- Interactive map manipulation
- Manual loop closure
- Grid manipulation
- Graph visualization

## Troubleshooting

### Common Issues

- **SLAM Toolbox plugin loading error in RViz**:
  - Make sure the plugin class name is correct (`slam_toolbox::SlamToolboxPlugin`)
  - Verify you have the latest version of slam_toolbox installed

- **LiDAR data not visible**:
  - Check that the LiDAR topic is being published:
    ```bash
    ros2 topic echo /yolobot/scan
    ```
  - Verify the LiDAR is properly configured in the URDF

- **Map not updating properly**:
  - Try adjusting the scan matching parameters
  - Move the robot slowly to get better scan data
  - Ensure the odometry is working correctly

- **Robot control issues**:
  - Verify the teleop is publishing to the correct cmd_vel topic
  - Check for any conflicts with other controllers

## References

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Navigation Stack](https://navigation.ros.org/)
- [LiDAR Sensor Integration Guide](https://gazebosim.org/tutorials) 