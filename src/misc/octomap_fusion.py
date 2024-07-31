#!/usr/bin/env python3

import rospy
from octomap_msgs.msg import Octomap
#from octomap_msgs.msg import FullMap
import numpy as np
#import octomap

octomap1 = None
octomap2 = None

def octomap_callback_1(msg):
    # Assuming 'msg' is of type Octomap
    # Access the octomap data and modify probabilities as needed
    # For illustration purposes, let's set all probabilities to 0.75
    modified_data = msg.data#np.full(len(msg.data), 0.75)

    # Create a new Octomap message with the modified data
    modified_octomap_1 = Octomap()
    modified_octomap_1.header = msg.header
    modified_octomap_1.data = modified_data


    global octomap1 
    octomap1 = modified_octomap_1

    # Publish the modified Octomap
    pub_1.publish(modified_octomap_1)

def octomap_callback_2(msg):
    # Assuming 'msg' is of type Octomap
    # Access the octomap data and modify probabilities as needed
    # For illustration purposes, let's set all probabilities to 0.25
    modified_data = msg.data#np.full(len(msg.data), 0.25)

    # Create a new Octomap message with the modified data
    modified_octomap_2 = Octomap()
    modified_octomap_2.header = msg.header
    modified_octomap_2.data = modified_data

    global octomap2
    octomap2 = modified_octomap_2

    # Publish the modified Octomap
    pub_2.publish(modified_octomap_2)

def fuse_octomaps(octomap_1, octomap_2):
    # Assuming octomap1 and octomap2 are already populated
    global octomap1
    global octomap2

    octomap_1 = octomap1
    octomap_2 = octomap2

    # Assuming that octomap1 and octomap2 have the same dimensions
    if octomap1 != None and octomap2 != None:
        fused_data = []

        for i in range(len(octomap1.data)):
            # Combine occupancy probabilities from octomap1 and octomap2
            # Example: Simple average (you might want to use a different strategy)
            print(octomap1.data)
            occupancy_prob = (octomap1.data[i] + octomap2.data[i]) / 2.0
            fused_data.append(occupancy_prob)

        # Create a new OctoMap message with the fused data
        fused_octomap = Octomap()
        fused_octomap.header = octomap1.header  # Use the header from one of the maps
        fused_octomap.data = fused_data

        #return fused_octomap

        # Publish the fused Octomap
        pub_fused.publish(fused_octomap)

if __name__ == '__main__':
    rospy.init_node('octomap_fusion', anonymous=True)

    # Subscribe to the original OctoMap topics
    rospy.Subscriber('/dragonfly26/octomap_full', Octomap, octomap_callback_1)
    rospy.Subscriber('/dragonfly26_sonar/octomap_full', Octomap, octomap_callback_2)

    # Publishers for the modified OctoMaps and fused OctoMap
    pub_1 = rospy.Publisher('/octomap_f1', Octomap, queue_size=10)
    pub_2 = rospy.Publisher('/octomap_f2', Octomap, queue_size=10)
    pub_fused = rospy.Publisher('/octomap_final', Octomap, queue_size=10)

    # Fuse the probabilities whenever either of the callbacks is called
    rospy.Subscriber('/octomap_f1', Octomap, lambda msg: fuse_octomaps(msg, Octomap()))  # Replace with actual topic
    rospy.Subscriber('/octomap_f2', Octomap, lambda msg: fuse_octomaps(Octomap(), msg))  # Replace with actual topic

    rospy.spin()
