#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np


points_sonar = None
points_camera = None

def callback_p(data):
    # Publish the new point cloud message
    global points_camera
    points_camera = data

    pub.publish(data)
    

def callback_s(data):
    global points_sonar
    points_sonar = data

    pub.publish(data)

def merge_point_clouds(pc_1, pc_2):

    # Merge the point clouds
    points1 = list(pc2.read_points(pc_1))
    points2 = list(pc2.read_points(pc_2))
    merged_points = points2

    # Populate the merged point cloud message
    new_cloud = pc2.create_cloud(pc_2.header, pc_2.fields, merged_points)

    return pc_2

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('sonar_cloud_builder')

    # Create a subscriber for the point cloud topic
    sub1 = rospy.Subscriber('sonar/points', PointCloud2, callback_s)
    sub2 = rospy.Subscriber('camera/depth_registered/points', PointCloud2, callback_p)

    # Create a publisher for the new point cloud topic
    pub = rospy.Publisher('points_depth_sonar', PointCloud2, queue_size=1000)
    pub2 = rospy.Publisher('points_depth_sonar', PointCloud2, queue_size=1000)

    # Spin the node
    rospy.spin()