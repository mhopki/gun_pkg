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
from tf.transformations import quaternion_from_euler

last_time = rospy.Time()
linear_velocity = Vector3()
position = Vector3()
rotation = Quaternion()

def accel_callback(data):
    
    # Get current time
    #print("OKAY:")
    #print(data)
    current_time = data.header.stamp

    global last_time 
    # Calculate time difference
    dt = (current_time - last_time).to_sec()
    #print("DT", dt)
    #print("linac",data.linear_acceleration)
    last_time = current_time

    dt = 0.02

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

def gyro_callback(data):
    
    # Get current time
    #print("OKAY:")
    #print(data)
    current_time = data.header.stamp

    global last_time 
    # Calculate time difference
    dt = (current_time - last_time).to_sec()
    #print("DT", dt)
    #print("linac",data.linear_acceleration)
    last_time = current_time

    dt = 0.02

    # Get the angular velocity values
    x_ang_vel = data.angular_velocity.x
    y_ang_vel = data.angular_velocity.y
    z_ang_vel = data.angular_velocity.z

    # Calculate the rotation angles
    roll = x_ang_vel * dt
    pitch = y_ang_vel * dt
    yaw = z_ang_vel * dt

    # Convert the rotation angles to a quaternion
    q = quaternion_from_euler(roll, pitch, yaw)

    # Create a ROS message with the quaternion data
    quat_msg = Quaternion()
    quat_msg.x = q[0]
    quat_msg.y = q[1]
    quat_msg.z = q[2]
    quat_msg.w = q[3]

    print(quat_msg)
    # Publish the quaternion data on the /rotation topic
    #pub.publish(quat_msg)

    # Create ROS transform message
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = 'odom'
    t.child_frame_id = 'camera_link'
    t.transform.rotation = quat_msg
    #print("TRANSFROM:")
    #print(t)

    # Broadcast ROS transform message
    br.sendTransform(t)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('imu_to_position_converter')

    rate = rospy.Rate(50)

    # Initialize variables
    last_time = rospy.Time.now()
    linear_velocity = Vector3()
    position = Vector3()
    rotation = Quaternion()

    # Create ROS transform broadcasterimport tf2_ros
    br = tf2_ros.TransformBroadcaster()

    # Create ROS subscribers
    accel_sub = rospy.Subscriber('/camera/accel/sample', Imu, accel_callback)
    gyro_sub = rospy.Subscriber('/camera/gyro/sample', Imu, gyro_callback)

    #pub = rospy.Publisher('/combined_data', Imu, queue_size=1000)
    
    while not rospy.is_shutdown():
        rate.sleep()
