#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from octomap_msgs.msg import Octomap
from octomap_msgs.msg import ColorOcTree

def pack_color(r, g, b):
    # Pack RGB values into a single 32-bit integer
    return (r << 16) | (g << 8) | b

def pointcloud_to_color_octomap(pc_msg):
    # Parse the point cloud data
    pc_data = pc2.read_points(pc_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    
    # Create an Octomap message
    octomap_msg = Octomap()
    octomap_msg.header = pc_msg.header  # Use the same header as the PointCloud2

    for point in pc_data:
        x, y, z, rgb = point
        r = (rgb >> 16) & 0xFF
        g = (rgb >> 8) & 0xFF
        b = rgb & 0xFF

        # Pack RGB values into a single 32-bit integer
        packed_color = pack_color(r, g, b)

        # Create a ColorOcTree message for each point
        color_octree_msg = ColorOcTree()
        color_octree_msg.id = 1  # Set a unique ID
        color_octree_msg.data = packed_color  # Assuming that the color information is stored here

        # Append the ColorOcTree message to the Octomap
        octomap_msg.data.append(color_octree_msg)

    return octomap_msg

def pointcloud_callback(pc_msg):
    octomap_msg = pointcloud_to_color_octomap(pc_msg)
    octomap_pub.publish(octomap_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_color_octomap_converter')

    # Substitute '/your/pointcloud/topic' with your actual point cloud topic
    pc_sub = rospy.Subscriber('/dragonfly26/tof/reproject', pc2.PointCloud2, pointcloud_callback)

    # Substitute '/color_octomap' with your desired Octomap topic
    octomap_pub = rospy.Publisher('/color_octomap', Octomap, queue_size=10)

    rospy.spin()
