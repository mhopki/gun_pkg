#! /usr/bin/env python3
import rospy
import numpy as np
import random

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal

from kr_python_interface.mav_interface import KrMavInterface

from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from vicon.msg import Subject

class ViconEye:
  def __init__(self):
    rospy.init_node('vicon_eye', anonymous=True)

    rospy.Subscriber('/vicon/dragonfly26/odom', Odometry, self.odom_callback)
    rospy.Subscriber('/vicon/dragonfly26', Subject, self.odom_callback_2)
    self.command_pub = rospy.Publisher('/quad_coms', Int32, queue_size=10)

    self.stopped = 0
    self.vicon_odom = None

    # Create a Rate object to control the loop frequency
    self.rate = rospy.Rate(30)  # 10 Hz

    # Main loop
    self.run()

  def run(self):
    while not rospy.is_shutdown():
      if (self.vicon_odom is not None):
        print(self.vicon_odom.pose.pose.position.y)
        if (self.vicon_odom.pose.pose.position.y >= 4.0 and self.stopped < 10):
          print("send stop command VICON")
          self.command_pub.publish(Int32(4))
          #self.command_callback(Int32(4))
          self.stopped += 1
      self.rate.sleep()

  def odom_callback(self, odom):
    self.vicon_odom = odom

  def odom_callback_2(self, odom):
    odom_var = Odometry()
    odom_var.pose.pose.position = odom.position
    self.vicon_odom = odom_var


if __name__ == '__main__':
  try :
    control = ViconEye()
  except rospy.ROSInterruptException :
    pass

#stop_control, vicon_eye, vicon_launch, snav_quad, tmux_voxl