#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry

class ImuToPositionConverter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('imu_to_position_converter')

        # Initialize variables
        self.last_time = rospy.Time.now()
        self.linear_velocity = Vector3()
        self.position = Vector3()

        # Create ROS subscribers
        #rospy.Subscriber('/camera/gyro/sample', Odometry, self.imu_callback)
        rospy.Subscriber('/camera/accel/sample', Odometry, self.imu_callback)

        # Create ROS transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

    def imu_callback(self, data):
        # Get current time
        current_time = data.header.stamp

        # Calculate time difference
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate linear velocity
        self.linear_velocity.x += data.linear_acceleration.x * dt
        self.linear_velocity.y += data.linear_acceleration.y * dt
        self.linear_velocity.z += data.linear_acceleration.z * dt

        # Calculate position
        self.position.x += self.linear_velocity.x * dt
        self.position.y += self.linear_velocity.y * dt
        self.position.z += self.linear_velocity.z * dt

        # Create ROS transform message
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'camera_link'
        t.transform.translation = self.position
        t.transform.rotation = Quaternion()

        # Broadcast ROS transform message
        self.br.sendTransform(t)

if __name__ == '__main__':
    try:
        ImuToPositionConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
