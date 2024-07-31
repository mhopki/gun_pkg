#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

# Global variables to store data
zz_data, yy_data = [], []

# Callback function to process incoming data
def data_callback(msg):
    global zz_data, yy_data
    yy_data.append(msg.pose.pose.position.y)  # Assuming yy is a field in the message
    zz_data.append(rospy.Time.now().to_sec())  # Using current time as zz

# Function to plot the data
def plot_data():
    plt.plot(zz_data, yy_data)
    plt.xlabel('Time (sec)')  # Assuming zz represents time
    plt.ylabel('yy')
    plt.title('Plot of yy vs Time')
    plt.show()  # Display the plot

def main():
    # Initialize ROS node and subscribers
    rospy.init_node('plotter_node')
    rospy.Subscriber('/dragonfly26/vio/odometry', Odometry, data_callback)  # Replace 'your_topic_name' with the actual topic name

    # Keep the program running until shutdown
    rospy.spin()

    # Plot the data when the node shuts down
    plot_data()

if __name__ == "__main__":
    main()
