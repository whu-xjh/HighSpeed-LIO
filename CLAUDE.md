# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS workspace containing **highspeed_lio**, a high-speed LiDAR-Inertial Odometry system for multi-LiDAR SLAM applications. The system is optimized for high-speed scenarios (>5 m/s) and supports external IMU integration with high-frequency (250Hz) propagation.

## Build System

This is a ROS Catkin workspace using standard catkin build system:

```bash
# Build the workspace (from HighSpeed-LIO-main directory)
cd /home/xjh/Doc/highspeed-lio/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Key Dependencies

- **ROS**: roscpp, rospy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, tf, pcl_ros, vikit_common, vikit_ros
- **External Libraries**: PCL (>=1.8), Eigen3 (>=3.3.4), OpenCV, Sophus, Boost
- **Optional**: mimalloc (memory allocator), OpenMP (multithreading), LAStools
- **Architecture**: Optimized for both ARM and x86 with native optimizations

## Core Architecture

### Main Components
- **LIVMapper** (`src/LIVMapper.cpp`): Main LiDAR-IMU fusion node handling multi-LiDAR data
- **VoxelMapManager** (`src/voxel_map.cpp`): Voxel-based mapping with efficient neighbor search algorithms
- **IMU_Processing** (`src/IMU_Processing.cpp`): IMU processing, bias estimation, and external IMU integration
- **Preprocess** (`src/preprocess.cpp`): Point cloud preprocessing and filtering
- **Multi-LiDAR Handler** (`src/livox_multi_lidar.cpp`): Multi-LiDAR data fusion and synchronization
- **VIO Module** (`src/vio.cpp`): Visual-Inertial Odometry for camera integration (optional)

### Key Features
- **External IMU Support**: Seamless switching between internal and external IMU with configurable initialization periods
- **High-Frequency Propagation**: 250Hz IMU propagation for high-speed applications 
- **Voxel-Based Mapping**: Efficient octree voxel structure with LRU caching
- **Multi-LiDAR Fusion**: Heterogeneous LiDAR sensor support with automatic calibration
- **No Visual Dependencies**: Pure LIO system (LiDAR-Inertial Odometry only)

## Configuration

Configuration is managed through YAML files in `config/`:
- `livox_multi_lidar.yaml`: Multi-LiDAR setup and external IMU parameters

Key configuration sections:
- `external_imu/`: External IMU enable/disable and topic configuration
- `imu/`: IMU covariance and bias parameters  
- `lio/`: LiDAR odometry parameters (voxel size, iterations)
- `uav/`: High-speed application settings (imu_rate_odom, gravity_align)

## Usage

### Multi-LiDAR Mapping
```bash
# Terminal 1: Launch mapping node
roslaunch highspeed_lio mapping_livox_multi_lidar.launch

# Terminal 2: Play rosbag data
rosbag play your_multi_lidar.bag

# Terminal 3: Optional - Run external IMU test
python3 scripts/test_external_imu.py
```

### Available Launch Files
- `mapping_livox_multi_lidar.launch`: Multi-LiDAR setup with external IMU support
- `mapping_avia.launch`: Livox Avia LiDAR configuration
- `mapping_mid360.launch`: Livox Mid360 configuration
- `mapping_hesaixt32_hilti22.launch`: Hesai XT32 + Hilti dataset setup
- `mapping_kitti.launch`: KITTI dataset configuration

### Data Topics
- **LiDAR**: `/livox/multi_lidar` (merged multi-LiDAR data)
- **IMU**: `/livox/imu_192_168_1_159` (default internal IMU)
- **External IMU**: `/novatel/oem7/odom` (external IMU odometry)
- **Camera**: `/alphasense/cam0/image_raw` (optional, used in VIO mode)
- **Odometry**: `/aft_mapped_to_init` (fused odometry output)

## Code Structure

### Header Organization
- `include/`: Core header files
  - `LIVMapper.h`: Main mapper class with external IMU integration
  - `voxel_map.h`: Voxel octree structure and LRU cache management
  - `IMU_Processing.h`: IMU processing with external IMU support
  - `common_lib.h`: Common data structures and utilities
  - `utils/`: Mathematical utilities (SO3, types, color)

### Source Files
- `src/main.cpp`: Entry point
- `src/LIVMapper.cpp`: Main fusion logic
- `src/voxel_map.cpp`: Voxel mapping implementation
- `src/IMU_Processing.cpp`: IMU processing and bias estimation
- `src/preprocess.cpp`: Point cloud preprocessing
- `src/livox_multi_lidar.cpp`: Multi-LiDAR data handling

### Launch Files
- `launch/mapping_livox_multi_lidar.launch`: Main mapping launch file

## Development Notes

- The system uses C++17 standard with architecture-specific optimizations
- External IMU integration supports covariance-based fusion for optimal pose estimation
- Time synchronization handles configurable offsets between sensors
- Voxel map uses efficient LRU caching to manage memory usage
- Supports both pure LIO mode and VIO mode when camera data is available
- Multi-threading support with OpenMP for performance optimization
- Memory optimization with optional mimalloc integration

## Testing

The project includes a test script for external IMU functionality:
```bash
# Test external IMU integration
python3 scripts/test_external_imu.py
```

This script publishes simulated IMU data to validate the external IMU fusion pipeline.

