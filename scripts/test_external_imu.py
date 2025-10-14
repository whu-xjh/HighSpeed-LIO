#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试外置IMU集成功能
"""

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import time

class ExternalIMUTester:
    def __init__(self):
        rospy.init_node('external_imu_tester', anonymous=True)
        
        # 发布内部IMU数据
        self.internal_imu_pub = rospy.Publisher('/livox/imu_192_168_1_159', Imu, queue_size=10)
        
        # 发布外置IMU数据
        self.external_imu_pub = rospy.Publisher('/novatel/oem7/odom', Imu, queue_size=10)
        
        # 订阅融合后的IMU数据（通过查看FAST-LIVO2的日志来验证）
        self.fused_imu_sub = rospy.Subscriber('/aft_mapped_to_init', nav_msgs.Odometry, self.odom_callback)
        
        self.start_time = time.time()
        self.count = 0
        
    def odom_callback(self, msg):
        """回调函数，用于验证融合后的位姿数据"""
        if self.count % 100 == 0:
            rospy.loginfo(f"Received Odometry: Position [{msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f}]")
        self.count += 1
        
    def publish_test_data(self):
        """发布测试用的IMU数据"""
        rate = rospy.Rate(100)  # 100Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = time.time() - self.start_time
            
            # 创建内部IMU数据
            internal_imu = Imu()
            internal_imu.header.stamp = current_time
            internal_imu.header.frame_id = "livox_imu"
            
            # 模拟简单的运动（正弦波动）
            internal_imu.angular_velocity.x = 0.1 * np.sin(elapsed_time)
            internal_imu.angular_velocity.y = 0.1 * np.cos(elapsed_time)
            internal_imu.angular_velocity.z = 0.05 * np.sin(2 * elapsed_time)
            
            internal_imu.linear_acceleration.x = 0.2 * np.sin(elapsed_time)
            internal_imu.linear_acceleration.y = 0.2 * np.cos(elapsed_time)
            internal_imu.linear_acceleration.z = 9.8 + 0.1 * np.sin(elapsed_time)
            
            # 创建外置IMU数据（添加一些噪声）
            external_imu = Imu()
            external_imu.header.stamp = current_time
            external_imu.header.frame_id = "novatel_imu"
            
            # 外置IMU数据略有不同（模拟更高精度的传感器）
            external_imu.angular_velocity.x = 0.08 * np.sin(elapsed_time) + 0.01 * np.random.randn()
            external_imu.angular_velocity.y = 0.12 * np.cos(elapsed_time) + 0.01 * np.random.randn()
            external_imu.angular_velocity.z = 0.04 * np.sin(2 * elapsed_time) + 0.01 * np.random.randn()
            
            external_imu.linear_acceleration.x = 0.18 * np.sin(elapsed_time) + 0.02 * np.random.randn()
            external_imu.linear_acceleration.y = 0.22 * np.cos(elapsed_time) + 0.02 * np.random.randn()
            external_imu.linear_acceleration.z = 9.81 + 0.05 * np.sin(elapsed_time) + 0.02 * np.random.randn()
            
            # 发布数据
            self.internal_imu_pub.publish(internal_imu)
            self.external_imu_pub.publish(external_imu)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = ExternalIMUTester()
        rospy.loginfo("External IMU Tester Started. Publishing test data...")
        tester.publish_test_data()
    except rospy.ROSInterruptException:
        pass