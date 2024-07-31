#!/usr/bin/env python

import rospy
from message_filters import Subscriber, TimeSynchronizer
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros
import tf2_msgs.msg
import math

def callback(data1, data2):
    print("cb")
    # combine the data from the two topics
    combined_data = "Float data: " + str(data1.linear_acceleration) + " Int data: " + str(data2.angular_velocity)
    # publish the combined data as a string
    print(combined_data)
    pub.publish(combined_data)

if __name__ == '__main__':
    rospy.init_node('message_filter_example')

    print("start")

    # subscribe to the two topics using message_filters.Subscriber
    sub1 = Subscriber('/camera/accel/sample', Imu)
    sub2 = Subscriber('/camera/gyro/sample', Imu)

    # synchronize the two topics using message_filters.TimeSynchronizer
    ts = TimeSynchronizer([sub1, sub2], 10000000)
    ts.registerCallback(callback)

    # create a publisher for the combined data
    pub = rospy.Publisher('/combined_data', String, queue_size=1000)

    rospy.spin()
