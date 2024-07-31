#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_msgs.msg
import math
from message_filters import TimeSynchronizer, Subscriber

last_time = rospy.Time()
linear_velocity = Vector3()
position = Vector3()

def callback(accel_data, gyro_data):
    
    # Get current time
    print("OKAY:")
    print(accel_data)
    print(gyro_data)
    current_time = data.header.stamp

    global last_time 
    # Calculate time difference
    dt = (current_time - last_time).to_sec()
    #print("DT", dt)
    #print("linac",data.linear_acceleration)
    last_time = current_time

    global linear_velocity
    # Calculate linear velocity
    linear_velocity.x += data.linear_acceleration.x * dt
    linear_velocity.y += data.linear_acceleration.y * dt
    linear_velocity.z += data.linear_acceleration.z * dt

    #print("linv",linear_velocity)
    # Calculate position
    global position
    position.x += linear_velocity.x * dt
    position.y += linear_velocity.y * dt
    position.z += linear_velocity.z * dt
    #print("pos",position)

    # Create ROS transform message
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = 'odom'
    t.child_frame_id = 'camera_link'
    t.transform.translation = position
    #print("TRANSFROM:")
    #print(t)

    # Broadcast ROS transform message
    br.sendTransform(t)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('imu_to_position_converter')

    # Initialize variables
    last_time = rospy.Time.now()
    linear_velocity = Vector3()
    position = Vector3()

    # Create ROS transform broadcasterimport tf2_ros
    br = tf2_ros.TransformBroadcaster()

    # Create ROS subscribers
    accel_sub = rospy.Subscriber('/camera/accel/sample', Imu)
    gyro_sub = rospy.Subscriber('/camera/gyro/sample', Imu)

    ts = TimeSynchronizer([accel_sub, gyro_sub], 10)
    ts.registerCallback(callback)
    
    rospy.spin()
