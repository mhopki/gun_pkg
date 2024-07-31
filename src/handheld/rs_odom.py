#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist, TwistStamped, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import math
import tf.transformations as tf
import numpy as np

class IMUOdomCalculator:
    def __init__(self):
        rospy.init_node('imu_odom_node')
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.prev_time = None
        self.orientation = [1.0, 0.0, 0.0, 0.0]  # Initial orientation (identity quaternion)
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.linear_velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]

    def imu_callback(self, imu_msg):
        current_time = rospy.Time.now()
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).to_sec()

        # Extract angular velocity from IMU data
        self.angular_velocity = [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z]

        # Extract linear acceleration from IMU data
        self.linear_acceleration = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z]

        # Update orientation based on angular velocity
        angular_velocity_magnitude = np.linalg.norm(self.angular_velocity)
        if angular_velocity_magnitude > 0.001:
            delta_orientation = tf.quaternion_about_axis(angular_velocity_magnitude * dt, self.angular_velocity)
            self.orientation = tf.quaternion_multiply(self.orientation, delta_orientation)

        #thresholding
        acceleration_threshold = 1.0

        """
        self.linear_acceleration = [
            self.linear_acceleration[0] if abs(self.linear_acceleration[0]) > acceleration_threshold else 0.0,
            self.linear_acceleration[1] if abs(self.linear_acceleration[1]) > acceleration_threshold else 0.0,
            self.linear_acceleration[2] if abs(self.linear_acceleration[2]) > acceleration_threshold else 0.0
        ]"""

        # Define the acceleration vector
        linear_acceleration_v = [0.0, -9.81, 0.0]  # -9.81 m/s^2 in the y-axis (gravity)
        #linear_acceleration_v = [self.linear_acceleration[0], self.linear_acceleration[1], self.linear_acceleration[2]]  # -9.81 m/s^2 in the y-axis (gravity)

        # Perform the coordinate transformation using the camera's orientation
        # You may need to adjust this depending on how the orientation is represented in your message
        # This code assumes the orientation quaternion represents a rotation from the camera frame to the world frame
        from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate, quaternion_from_euler

        # Convert the orientation quaternion to a rotation matrix
        orientation_list = [self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # Construct a rotation quaternion that represents the orientation of the camera
        camera_quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Get the conjugate of the camera quaternion (represents the reverse rotation)
        camera_quaternion_conjugate = quaternion_conjugate(self.orientation)

        print(self.orientation)
        lin_acc_quat = [self.linear_acceleration[0], self.linear_acceleration[1], self.linear_acceleration[2], 0.0]
        #lin_acc_quat = [0.0, -9.81, 0.0, 0.0]
        print("m: ", quaternion_multiply(lin_acc_quat, camera_quaternion_conjugate))

        # Rotate the linear acceleration vector to align it with the camera's orientation
        rotated_acceleration = quaternion_multiply(self.orientation, quaternion_multiply(lin_acc_quat, camera_quaternion_conjugate))

        print(self.linear_acceleration)
        print(" vs ")
        print(rotated_acceleration)

        #threshold2
        fin_acc = [
            (rotated_acceleration[0] + linear_acceleration_v[0]) if abs((rotated_acceleration[0] + linear_acceleration_v[0])) > acceleration_threshold else 0.0,
            (rotated_acceleration[1] + linear_acceleration_v[1]) if abs((rotated_acceleration[1] + linear_acceleration_v[1])) > acceleration_threshold else 0.0,
            (rotated_acceleration[2] + linear_acceleration_v[2]) if abs((rotated_acceleration[2] + linear_acceleration_v[2])) > acceleration_threshold else 0.0
        ]

        print("fin vs: ")
        print(fin_acc)


        # Calculate linear velocity based on linear acceleration
        self.linear_velocity = [self.linear_velocity[0] + (fin_acc[0]) * dt,
                                self.linear_velocity[1] + (fin_acc[1]) * dt,
                                self.linear_velocity[2] + (fin_acc[2]) * dt]

        # Update position based on linear velocity
        self.position = [self.position[0] + self.linear_velocity[0] * dt,
                        self.position[1] + self.linear_velocity[1] * dt,
                        self.position[2] + self.linear_velocity[2] * dt]

        # Create an odom message
        odom_msg = Odometry()
        #print(odom_msg)
        odom_msg.header = Header()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = current_time
        #odom_msg.twist.linear = Twist()
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        #print("new: ", odom_msg)
        odom_msg.twist.twist.linear.x = self.linear_velocity[0]
        odom_msg.twist.twist.linear.y = self.linear_velocity[1]
        odom_msg.twist.twist.linear.z = self.linear_velocity[2]
        #odom_msg.twist.angular = Twist()
        odom_msg.twist.twist.angular.x = self.angular_velocity[0]
        odom_msg.twist.twist.angular.y = self.angular_velocity[1]
        odom_msg.twist.twist.angular.z = self.angular_velocity[2]

        # Publish the odom message
        self.pub_odom.publish(odom_msg)

        self.prev_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    odom_calculator = IMUOdomCalculator()
    odom_calculator.run()
