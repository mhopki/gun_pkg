#!/usr/bin/env python3

import rospy
from octomap_msgs.msg import Octomap
from octomap_msgs.msg import FullMap
import numpy as np

def octomap_callback_1(msg):
    # Assuming 'msg' is of type Octomap
    # Access the octomap data and modify probabilities as needed
    # For illustration purposes, let's set all probabilities to 0.75
    modified_data = np.full(len(msg.data), 0.75)

    # Create a new Octomap message with the modified data
    modified_octomap_1 = Octomap(header=msg.header, id=msg.id, resolution=msg.resolution, data=modified_data)

    # Publish the modified Octomap
    pub_1.publish(modified_octomap_1)

def octomap_callback_2(msg):
    # Assuming 'msg' is of type Octomap
    # Access the octomap data and modify probabilities as needed
    # For illustration purposes, let's set all probabilities to 0.25
    modified_data = np.full(len(msg.data), 0.25)

    # Create a new Octomap message with the modified data
    modified_octomap_2 = Octomap(header=msg.header, id=msg.id, resolution=msg.resolution, data=modified_data)

    # Publish the modified Octomap
    pub_2.publish(modified_octomap_2)

def fuse_octomaps(octomap_1, octomap_2):
    # Fuse the probabilities of two OctoMaps
    fused_data = (octomap_1.data + octomap_2.data) / 2.0

    # Create a new Octomap message with the fused data
    fused_octomap = Octomap(header=octomap_1.header, id=octomap_1.id, resolution=octomap_1.resolution, data=fused_data)

    # Publish the fused Octomap
    pub_fused.publish(fused_octomap)

if __name__ == '__main__':
    rospy.init_node('octomap_fusion', anonymous=True)

    # Subscribe to the original OctoMap topics
    rospy.Subscriber('/your/original/octomap/topic_1', Octomap, octomap_callback_1)
    rospy.Subscriber('/your/original/octomap/topic_2', Octomap, octomap_callback_2)

    # Publishers for the modified OctoMaps and fused OctoMap
    pub_1 = rospy.Publisher('/your/modified/octomap/topic_1', Octomap, queue_size=10)
    pub_2 = rospy.Publisher('/your/modified/octomap/topic_2', Octomap, queue_size=10)
    pub_fused = rospy.Publisher('/your/fused/octomap/topic', Octomap, queue_size=10)

    # Fuse the probabilities whenever either of the callbacks is called
    rospy.Subscriber('/your/modified/octomap/topic_1', Octomap, lambda msg: fuse_octomaps(msg, Octomap()))  # Replace with actual topic
    rospy.Subscriber('/your/modified/octomap/topic_2', Octomap, lambda msg: fuse_octomaps(Octomap(), msg))  # Replace with actual topic

    rospy.spin()
