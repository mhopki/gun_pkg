#! /usr/bin/env python
#from __future__ import print_function

import rospy
import numpy as np
import random
import math
import sys

import tf
import sensor_msgs
import std_msgs
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_msgs.msg import Header
#from std_msgs.msg import Image
import locale
from locale import atof

import serial
#from serial import *

def callback_pointcloud(data):
	x = 1

def talker():
	sonar_msg = Range()
	#ros::Publisher pub_sonar("height", &sonar_msg);
	ser = serial.Serial('/dev/ttyUSB0', 57600) #9600)
	#pub = rospy.Publisher('sonar0', String, queue_size=1)
	pub = rospy.Publisher('sonar_topic', Range, queue_size=1)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(15)
	sonar_msg.header.stamp = rospy.Time.now()
	sonar_msg.radiation_type = 0
	sonar_msg.header.frame_id = "/map" #frameid
	sonar_msg.field_of_view = 0.25#1.8; #based on datasheet for MB1222
	sonar_msg.min_range = 0.3;
	sonar_msg.max_range = 5.0;
	sonar_msg.range = 0;
	while not rospy.is_shutdown():
   		data= ser.read()
   		print(data)
   		#data = ser.readline() # I have "hi" coming from the arduino as a test run over the serial port
   		data_str = str(data)
   		true_data = data_str[3:7]
   		tt2 = "0"
   		if (data_str[2] == "R"):
   			sonar_msg.range = float(true_data)# / 1000.0
   			tt2 = str(sonar_msg.range)
   		#rrr =  atof(data);

   		#image_msg = rospy.Subscriber('/camera/color/image_raw', Image, callback_pointcloud)

   		rospy.loginfo(data_str + " - " + true_data + " - " + tt2)
   		#pub.publish(hello_str)
   		pub.publish(sonar_msg)
   		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass