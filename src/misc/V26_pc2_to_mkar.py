#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import struct

def pointcloud_to_marker_array(pointcloud_msg):
    marker_array = MarkerArray()

    for i, point in enumerate(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)):
        x, y, z, rgb = point
        global id_count

        marker = Marker()
        marker.header.frame_id = pointcloud_msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pointcloud_markers"
        marker.id = id_count
        #print(id_count)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = marker.scale.y = marker.scale.z = 0.001  # Adjust the scale as needed

        # Extract RGB values from the 32-bit integer color
        r = (rgb >> 16) & 0xFF
        g = (rgb >> 8) & 0xFF
        b = rgb & 0xFF

        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0
        marker.color.a = 1.0  # Alpha channel

        id_count += 1

        marker_array.markers.append(marker)

    return marker_array

def pointcloud_callback(pointcloud_msg):
    marker_array = pointcloud_to_marker_array(pointcloud_msg)
    #print("pub")
    marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        rospy.init_node('pointcloud_to_marker_array')
        pointcloud_sub = rospy.Subscriber('/dragonfly26/tof/reproject', PointCloud2, pointcloud_callback)
        marker_pub = rospy.Publisher('/dragonfly26/tof/voxl_marker', MarkerArray, queue_size=10)

        global id_count
        id_count = 0
        rate = rospy.Rate(2)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            # Process any other tasks here
            #print("wait")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
