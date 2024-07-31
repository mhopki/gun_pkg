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

class FlightControl:
  def __init__(self):
    rospy.init_node('mav_example', anonymous=True)

    # Creating MAV objects
    self.mav_namespace = 'dragonfly26'#quadrotor
    self.mav_id = 1
    self.mav_obj = KrMavInterface('dragonfly26', '')

    rospy.Subscriber('/quad_coms', Int32, self.command_callback)
    rospy.Subscriber('/vicon/dragonfly26/odom', Odometry, self.odom_callback)
    self.command_pub = rospy.Publisher('/quad_coms', Int32, queue_size=10)

    self.stopped = False
    self.vicon_odom = None

    # Create a Rate object to control the loop frequency
    self.rate = rospy.Rate(30)  # 10 Hz

    # Main loop
    self.run()

  def run(self):

    while not rospy.is_shutdown():
      """
      if (self.vicon_odom is not None):
        if (self.vicon_odom.pose.pose.position.y >= 5.0 and self.stopped is False):
          print("send stop command")
          #self.command_pub.publish(4)
          self.command_callback(Int32(4))
          self.stopped = True"""
      self.rate.sleep()

  def command_callback(self, command):
    #middle two red lines is vicon y=6.0
    #red corner by the egdge of vicon is y=10.o
    #start black corner is y=2.0
    if command.data == 4:
      print("EHover")#self.mav_obj.get_odom())
      stop_pose = self.mav_obj.get_odom().pose.pose.position
      stop_pose_off = self.mav_obj.get_odom().twist.twist.linear
      des_vel = 0.0#((stop_pose_off.x + stop_pose_off.y + stop_pose_off.z) / 3) / 2
      #des_acc = ((stop_pose_off.x + stop_pose_off.y + stop_pose_off.z) / 3) / 2.5
      des_acc = min(2.0, ((stop_pose_off.x + stop_pose_off.y + stop_pose_off.z) / 3) / 1.35)
      #self.mav_obj.traj_tracker_client.cancel_all_goals()
      #self.mav_obj.hover()
      #self.mav_obj.traj_tracker_client.cancel_all_goals()
      #self.mav_obj.line_tracker_client.cancel_all_goals()
      #self.mav_obj.circle_tracker_client.cancel_all_goals()
      #self.mav_obj.hover()
      #self.mav_obj.send_wp_block(stop_pose.x + stop_pose_off.x/100, stop_pose.y + stop_pose_off.y/100, stop_pose.z + stop_pose_off.z/100, 0.0, vel=des_vel, acc=des_acc)
      self.mav_obj.ehover()

  def odom_callback(self, odom):
    self.vicon_odom = odom


if __name__ == '__main__':
  try :
    control = FlightControl()
  except rospy.ROSInterruptException :
    pass
