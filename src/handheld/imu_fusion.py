#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
import tf
import math

class IMUFusionNode:
    def __init__(self):
        rospy.init_node('imu_fusion_node')
        
        # IMU message publisher
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        
        # Accelerometer and gyroscope subscribers
        self.accel_sub = rospy.Subscriber('/camera/accel/sample', Imu, self.accel_callback)
        self.gyro_sub = rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback)
        
        # Initialize variables to store data
        self.accel_data = None
        self.gyro_data = None
        
    def accel_callback(self, accel_msg):
        self.accel_data = accel_msg
        self.process_data()

    def gyro_callback(self, gyro_msg):
        self.gyro_data = gyro_msg
        self.process_data()

    def process_data(self):
        if self.accel_data is not None and self.gyro_data is not None:
            # Create a new IMU message to fuse accelerometer and gyroscope data
            imu_msg = Imu()
            imu_msg.header = self.accel_data.header

            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Copy accelerometer data
            imu_msg.linear_acceleration = self.accel_data.linear_acceleration
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # Copy gyroscope data
            imu_msg.angular_velocity = self.gyro_data.angular_velocity
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # Publish the fused IMU message
            self.imu_pub.publish(imu_msg)
            
            # Reset data
            self.accel_data = None
            self.gyro_data = None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    imu_fusion_node = IMUFusionNode()
    imu_fusion_node.run()
