# 外置IMU集成功能说明

## 概述

FAST-LIVO2现在支持外置IMU数据融合功能，可以将内置IMU和外置IMU数据进行加权融合，提高定位精度和鲁棒性。

## 功能特点

1. **双IMU数据融合**：支持内置IMU和外置IMU数据的实时融合
2. **加权融合算法**：根据配置的权重参数进行数据融合
3. **时间同步**：自动处理两个IMU数据源的时间对齐
4. **配置灵活**：可以通过配置文件调整融合参数

## 配置参数

在配置文件（如`config/livox_multi_lidar.yaml`）中添加以下参数：

```yaml
# 外置IMU配置
external_imu:
  enable: true                    # 是否启用外置IMU
  weight: 0.5                     # 外置IMU权重(0-1)，0表示完全使用内置IMU，1表示完全使用外置IMU
  time_offset: 0.0                # 外置IMU时间偏移（秒）
```

### 话题配置

确保以下话题配置正确：

```yaml
common:
  imu_topic: "/livox/imu_192_168_1_159"        # 内置IMU话题
  external_imu_topic: "/novatel/oem7/odom"      # 外置IMU话题
```

## 使用方法

### 1. 基本使用

1. 确保外置IMU数据正在发布到指定话题
2. 在配置文件中启用外置IMU并设置合适的权重
3. 启动FAST-LIVO2：

```bash
roslaunch fast_livo2 mapping_livox_multi_lidar.launch
```

### 2. 测试验证

可以使用提供的测试脚本验证功能：

```bash
# 给脚本添加执行权限
chmod +x scripts/test_external_imu.py

# 运行测试脚本
python3 scripts/test_external_imu.py
```

### 3. 参数调优

根据实际应用场景调整融合权重：

- **高精度外置IMU**：可以设置较高的权重（0.7-0.9）
- **普通精度外置IMU**：建议使用中等权重（0.4-0.6）
- **仅作为备份**：设置较低权重（0.1-0.3）

## 技术细节

### 融合算法

系统使用加权平均算法进行IMU数据融合：

```
融合值 = (1 - weight) × 内置IMU值 + weight × 外置IMU值
```

### 时间同步

系统会自动查找时间戳最接近的外置IMU数据进行融合，时间差阈值默认为10ms。

### 数据处理流程

1. 订阅内置IMU和外置IMU数据
2. 根据时间戳进行数据对齐
3. 应用加权融合算法
4. 将融合后的数据传递给IMU处理模块

## 注意事项

1. **数据质量**：确保外置IMU数据质量良好，噪声过大会影响融合效果
2. **时间同步**：两个IMU数据源的时间戳应尽可能同步
3. **坐标系**：确保两个IMU数据在相同的坐标系下
4. **权重设置**：根据实际传感器精度合理设置权重参数

## 故障排除

### 问题1：外置IMU数据未融合

- 检查`external_imu/enable`是否设置为`true`
- 确认外置IMU话题是否正确发布
- 查看日志中是否有相关错误信息

### 问题2：融合效果不佳

- 调整`external_imu/weight`参数
- 检查两个IMU数据的时间戳差异
- 验证外置IMU数据质量

### 问题3：系统运行不稳定

- 降低外置IMU权重
- 检查外置IMU数据频率是否过高
- 确认系统资源是否充足

## 示例配置

### 高精度场景

```yaml
external_imu:
  enable: true
  weight: 0.8      # 主要依赖外置IMU
  time_offset: 0.0
```

### 普通场景

```yaml
external_imu:
  enable: true
  weight: 0.5      # 平衡两个IMU数据
  time_offset: 0.0
```

### 备份场景

```yaml
external_imu:
  enable: true
  weight: 0.2      # 主要使用内置IMU，外置IMU作为备份
  time_offset: 0.0