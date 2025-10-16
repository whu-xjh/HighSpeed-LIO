# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FAST-LIVO2 is a fast, direct LiDAR-Inertial-Visual odometry system for real-time 3D reconstruction and robotic localization. The system tightly couples LiDAR, IMU, and visual data using EKF-based fusion with a 19-dimensional state vector.

## Build Commands

### Standard Build
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Clean Build
```bash
cd ~/catkin_ws
catkin_clean
catkin_make
```

### Build Specific Package
```bash
cd ~/catkin_ws
catkin_make fast_livo
```

### Build from Project Root
```bash
cd ~/catkin_ws/src/FAST-LIVO2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Architecture-Specific Notes
- **ARM (32-bit)**: Uses `-O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math`
- **ARM64 (RK3588, Jetson)**: Uses `-O3 -mcpu=native -mtune=native -ffast-math`
- **x86-64**: Uses `-O2 -march=native` (conservative for stability)
- Multithreading enabled with OpenMP when available
- Maximum cores detected automatically, configurable via `MP_PROC_NUM`

## Running the System

### Basic Usage
```bash
# Terminal 1: Launch mapping node
roslaunch fast_livo mapping_avia.launch

# Terminal 2: Play rosbag
rosbag play YOUR_DOWNLOADED.bag
```

### Available Launch Files
- `mapping_avia.launch` - Livox AVIA LiDAR
- `mapping_avia_marslvig.launch` - Livox AVIA with MarsLVIG camera setup
- `mapping_mid360.launch` - Livox Mid360 LiDAR
- `mapping_hap.launch` - Hesai PandarXT32 + Hilti mapping
- `mapping_livox_multi_lidar.launch` - Multiple Livox LiDARs
- `mapping_ouster_ntu.launch` - Ouster LiDAR
- `mapping_hesaixt32_hilti22.launch` - Hesai XT32 + Hilti setup

## Architecture

### Core Components

**Main Processing Pipeline:**
- `LIVMapper.cpp/h` - Main fusion node handling LiDAR-Inertial-Visual odometry (~1.9K lines)
- `voxel_map.cpp/h` - LiDAR point cloud management and voxel-based mapping
- `IMU_Processing.cpp/h` - IMU preintegration and bias estimation with Kalman filtering
- `preprocess.cpp/h` - LiDAR point cloud preprocessing and deskewing with motion compensation

**Visual Processing:**
- `vio.cpp` - Visual-inertial odometry with direct image alignment
- `frame.cpp/h` - Visual frame management and feature tracking
- `visual_point.cpp/h` - Visual feature point representation and management

**Multi-LiDAR Support:**
- `livox_multi_lidar.cpp` - Multi-LiDAR data fusion and synchronization for heterogeneous sensors
- `imu_filter.cpp` - IMU data filtering and preprocessing

**Utilities and Math:**
- `utils/so3_math.h` - SO(3) Lie group operations and rotations
- `utils/types.h` - Custom data types and Eigen aliases
- `utils/color.h` - Point cloud coloring utilities
- `common_lib.h` - Common includes and system-wide constants

### Key Data Flow

1. **Sensor Input**: LiDAR points, IMU measurements, and camera images
2. **Preprocessing**: Point cloud deskewing, IMU bias estimation
3. **State Estimation**: EKF-based fusion with 19-dimensional state vector
4. **Mapping**: Voxel-based point cloud management with incremental updates
5. **Visual Alignment**: Direct image alignment for pose refinement

### Important Implementation Details

**High-Frequency IMU Propagation (`imu_rate_odom` parameter):**
- When `uav/imu_rate_odom: true`, enables 250Hz IMU propagation for high-speed scenarios
- Publishes high-frequency odometry on `/LIVO2/imu_propagate` topic
- Reduces position error accumulation by 625x compared to LiDAR-rate updates
- Critical for UAV applications and high-speed ground vehicles (>5 m/s)

**Neighbor Voxel Search Algorithm:**
- Implements adaptive neighbor voxel selection for robust point cloud registration
- Uses `voxel_center_ ± quater_length_` for boundary detection with distance-based skipping
- Skips 1 or 2 voxels based on point position relative to voxel boundaries for efficient search
- Located in `voxel_map.cpp` within `BuildResidualListOMP` function around lines 1050-1080

**Motion Distortion Compensation:**
- Forward propagation in `IMU_Processing.cpp:300` undistorts point clouds using IMU integration
- Each point is transformed to common reference frame using corresponding IMU state
- Essential for high-speed scenarios where motion occurs during LiDAR scan

**Ground Voxel Detection Optimization:**
- `hasAdjacentGroundVoxel` function uses precomputed lookup table for 8-neighbor horizontal offsets
- Elevation axis is determined at initialization and reused for efficient neighbor calculation
- Eliminates runtime loop calculations for neighbor position computation

**External IMU Support:**
- System supports external IMU data via `ExternalIMUData` structure defined in `common_lib.h`
- External IMU measurements buffered with configurable time offset and buffer size
- Integrated through `external_imu_buffer` with `external_imu_enable` flag
- Configurable via YAML: `external_imu_topic`, `external_imu/enable`, `external_imu/time_offset`
- Transformation matrices (`external_imu_T`, `external_imu_R`) for coordinate alignment
- Located in `LIVMapper.h:46-61` with callback `odom_cbk()` for nav_msgs/Odometry input

### Dependencies

**Required ROS Packages:**
- `roscpp`, `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `tf`, `pcl_ros`, `cv_bridge`, `image_transport`
- `vikit_common`, `vikit_ros` (from specific fork: `https://github.com/xuankuzcr/rpg_vikit.git`)

**External Libraries:**
- PCL (>=1.8) - Point cloud processing
- Eigen3 (>=3.3.4) - Linear algebra
- OpenCV (>=4.2) - Computer vision
- Sophus - Lie group operations (non-templated/double-only version)
- Boost - Thread management
- LAStools - Point cloud compression (specific paths: `/usr/local/include/LASlib`, `/usr/local/lib/liblaszip.so.8`)

**External IMU Integration:**
- System processes external IMU data through nav_msgs/Odometry messages
- Supports time synchronization via configurable `external_imu_time_offset` parameter
- Buffer management with configurable size (default 1000 measurements)
- Coordinate transformation between external IMU frame and system frame
- Configurable via YAML parameters under `external_imu/*` and `common/external_imu_topic`

### Key Configuration Files

**Sensor Configurations** in `config/`:
- `avia.yaml` - Livox AVIA configuration
- `mid360.yaml` - Livox Mid360 configuration
- `hap.yaml` - Hesai + Hilti configuration
- `livox_multi_lidar.yaml` - Multi-LiDAR setup
- `camera_*.yaml` - Camera intrinsic/extrinsic parameters

**Important Parameters:**
- `uav/imu_rate_odom: false` - Enable high-frequency IMU propagation
- `uav/gravity_align_en: false` - Gravity alignment enable
- `common/extrinsic_T` - LiDAR to IMU translation
- `common/extrinsic_R` - LiDAR to IMU rotation
- `common/external_imu_topic` - External IMU odometry topic
- `external_imu/enable: false` - Enable external IMU data processing
- `external_imu/time_offset: 0.0` - Time offset for external IMU synchronization
- `external_imu/external_T` - External IMU to system translation
- `external_imu/external_R` - External IMU to system rotation

## Build System Details

### Architecture-Specific Optimizations
- **ARM (32-bit)**: `-O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math`
- **ARM64 (RK3588, Jetson)**: `-O3 -mcpu=native -mtune=native -ffast-math`
- **x86-64**: `-O2 -march=native` (conservative for stability)
- Automatic detection via CMAKE_SYSTEM_PROCESSOR

### Multithreading Configuration
- OpenMP support enabled when found
- Maximum 4 cores by default (configurable via MP_PROC_NUM)
- Processor count detection via `include(ProcessorCount)`

### Memory Optimization
- Conditional mimalloc support when available
- Release build type by default for performance
- Debug builds use `-O0 -g` flags

## Development Notes

### Code Style and Architecture
- Uses C++17 standard with architecture-specific optimizations
- Multithreading with OpenMP (configurable via MP_PROC_NUM)
- Memory optimization with conditional mimalloc support
- Separate libraries for modularity: `vio`, `lio`, `pre`, `imu_proc`, `laser_mapping`

### Key Data Structures
- `M3D` / `V3D` - 3x3 matrices and 3D vectors (Eigen)
- `PointCloudXYZI` - LiDAR point clouds with intensity
- `PointCloudXYZINormal` - Point clouds with surface normals
- `ImuData` - IMU measurements with timestamps
- `ImageFrame` - Visual frames with features and pose
- `VOXEL_LOCATION` - Discrete 3D voxel coordinates in grid-based map
- `VOXEL_COLUMN_LOCATION` - 2D horizontal coordinates for column-based voxel management
- `pointWithVar` - Point with variance information (108 bytes, exceeds cache line)
- `StatesGroup` - 19-dimensional state vector for EKF
- `ExternalIMUData` - External IMU measurement data structure
- `LazSaveTask` - Asynchronous point cloud saving task structure
- `MeasureGroup` - Sensor measurement grouping for LiDAR-IMU-Visual synchronization
- `LidarMeasureGroup` - Complete LiDAR frame data with associated measurements
- `VoxelMapConfig` - Configuration parameters for voxel mapping system
- `PointToPlane` - Point-to-plane residual for ICP optimization

### Important Constants
- `DIM_STATE`: 19-dimensional state vector for EKF
- `G_m_s2`: Gravity constant (9.81 m/s²)
- `VOXELMAP_HASH_P`: Hash prime for voxel location hashing (116101)
- `VOXELMAP_MAX_N`: Maximum hash value for voxel locations (10000000000)
- `ROOT_DIR`: Project root directory defined at compile time
- `INIT_COV`: Initial covariance value (0.01)
- `SIZE_LARGE`: Large buffer size (500)
- `SIZE_SMALL`: Small buffer size (100)
- Build type defaults to Release for performance

## Common Issues

### Build Issues
- Ensure all dependencies are installed (PCL, Eigen, OpenCV, Sophus, vikit)
- Use specific vikit fork: `https://github.com/xuankuzcr/rpg_vikit.git`
- Verify LAStools library paths for point cloud compression
- Check ARM/x86 architecture detection in CMakeLists.txt

### Runtime Issues
- Verify sensor configuration files match hardware
- Check timestamp synchronization between sensors
- For high-speed scenarios, enable `imu_rate_odom: true`
- Ensure sufficient memory for voxel map operations
- Monitor CPU usage - system is computationally intensive
- Elevation axis configuration (`elevation_axis_` parameter) affects ground voxel detection performance
- Hash function performance may degrade with highly clustered voxel distributions
- External IMU time synchronization critical for multi-sensor fusion accuracy
- LAStools library paths must match system installation for point cloud compression
- External IMU extrinsic parameters (`external_imu/external_T`, `external_imu/external_R`) must be properly calibrated
- Buffer overflow can occur if `external_imu_buffer_size` is too small for high-frequency external IMU data