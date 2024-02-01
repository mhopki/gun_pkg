#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def get_robot_pose():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("none")
            continue

        # Create a PoseStamped message to represent the robot pose in the map frame
        pose = PoseStamped()
        pose.header.frame_id = "/odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        # Extract the x, y, z, and theta values from the pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        theta = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]

        # Print the robot's localization information
        print("Robot Position: x = {:.2f}, y = {:.2f}, z = {:.2f}".format(x, y, z))
        print("Robot Orientation: theta = {:.2f} degrees".format(theta * 180.0 / 3.14159))

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('localization_node')
    get_robot_pose()
