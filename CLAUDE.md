# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS 2 workspace containing a Simple 2D LiDAR Odometry package that implements sensor fusion odometry estimation. The system combines LiDAR data (for translation) with IMU data (for rotation) using the Generalized Iterative Closest Point (GICP) algorithm. The package is named `lidar_odometry` and is built for ROS 2 Humble.

## Project Structure

- `src/Simple-2D-LiDAR-Odometry/` - Main package source code
  - `src/` - C++ implementation files
    - `lidar_odometry_node.cpp` - Main ROS 2 node implementation
    - `lidar_odometry.cpp` - Core odometry algorithm implementation
  - `include/lidar_odometry/` - Header files
    - `lidar_odometry.hpp` - Main odometry class definition
    - `utils.hpp` - Utility functions and data structures
  - `CMakeLists.txt` - CMake build configuration
  - `package.xml` - ROS 2 package manifest

## Key Dependencies

- ROS 2 Humble
- Eigen3 (linear algebra)
- PCL (Point Cloud Library) 
- laser_geometry (laser scan projection)
- tf2 (transform handling)

## Build Commands

Build the entire workspace:
```bash
colcon build
```

Build only the lidar_odometry package:
```bash
colcon build --packages-select lidar_odometry
```

Source the workspace after building:
```bash
source install/setup.bash
```

## Running the Node

Execute the LiDAR odometry node:
```bash
ros2 run lidar_odometry lidar_odometry_node
```

## Core Architecture

The system implements sensor fusion with two main components:

1. **LidarOdometryNode** (`lidar_odometry_node.cpp`): ROS 2 node that handles:
   - LaserScan message subscription (for translation estimation)
   - IMU message subscription (for rotation estimation)
   - Parameter management (GICP configuration)
   - Sensor data fusion coordination
   - Odometry publishing
   - TF frame broadcasting (odom → base_link)
   - Static transform broadcasting (base_link → ldlidar_base)

2. **LidarOdometry** (`lidar_odometry.cpp`): Core sensor fusion algorithm that:
   - Processes point cloud data from laser scans (extracts translation only)
   - Processes IMU data for orientation (uses quaternion directly)
   - Applies GICP registration between consecutive scans
   - Fuses LiDAR translation with IMU rotation
   - Maintains pose estimation in SE(3)
   - Tracks robot state (pose and velocity)

## Key Data Structures

- **State**: Robot state containing pose (Eigen::Isometry3d) and velocity
- **ScanData**: Laser scan data with timestamp and PCL point cloud
- **ImuData**: IMU data with timestamp, angular velocity, orientation quaternion, and linear acceleration
- Point clouds are converted from LaserScan → PointCloud2 → PCL format
- IMU data is converted from ROS sensor_msgs/Imu to Eigen structures

## Configuration Parameters

The node accepts these parameters:
- `max_correspondence_distance` (default: 1.0)
- `transformation_epsilon` (default: 0.005) 
- `maximum_iterations` (default: 30)
- `scan_topic_name` (default: "ldlidar_node/scan")
- `imu_topic_name` (default: "/imu/mpu6050")
- `odom_topic_name` (default: "odom")

## Sensor Fusion Approach

- **LiDAR**: Provides translation estimation (X, Y movement) through GICP point cloud alignment
- **IMU**: Provides rotation estimation (orientation quaternion) for accurate heading
- **Fusion**: Combines LiDAR translation with IMU orientation for complete 6DOF pose
- **Benefits**: Higher accuracy rotation from IMU, drift-free translation from LiDAR

## Testing

The package includes ament linting tests. Run tests with:
```bash
colcon test --packages-select lidar_odometry
```

## Frame Structure

- `odom` → `base_link` (dynamic, from odometry)
- `base_link` → `ldlidar_base` (static, identity transform)