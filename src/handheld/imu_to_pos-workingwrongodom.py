#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf
import tf2_ros
import tf2_msgs.msg
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import numpy as np

over_rate = 100

class SensorFusion:
    def __init__(self):
        # Initialize node
        #rospy.set_param('use_sim_time', True)
        rospy.init_node('sensor_fusion', anonymous=True)

        global over_rate

        # Initialize variables
        self.accel = [0, 0, 0] # initialize accelerometer data
        self.gyro = [0, 0, 0] # initialize gyroscope data
        self.dt = 1/over_rate #0.01 # initialize time step
        self.angle = [0, 0, 0] # initialize orientation angles
        self.velocity = [0, 0, 0]
        self.position = [0, 0, 0]

        # Set up subscribers
        rospy.Subscriber('/camera/accel/sample', Imu, self.accel_callback)
        rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback)

        # create publisher for orientation
        self.pose_pub = rospy.Publisher("/imu/pose", Odometry, queue_size=10)

        # Set up broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        #self.br = tf.TransformBroadcaster()

    def accel_callback(self, msg):
        # Get accelerometer data
        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z

        self.sensor_fusion()

    def gyro_callback(self, msg):
        # Get gyroscope data
        self.gyro[0] = msg.angular_velocity.x
        self.gyro[1] = msg.angular_velocity.y
        self.gyro[2] = msg.angular_velocity.z

        self.sensor_fusion()

    def sensor_fusion(self):
        # Implement sensor fusion algorithm here to calculate orientation
        # ...
        # calculate orientation using complementary filter
        self.angle[0] = 0.98 * (self.angle[0] + self.gyro[0] * self.dt) + 0.02 * self.accel[0]
        self.angle[1] = 0.98 * (self.angle[1] + self.gyro[1] * self.dt) + 0.02 * self.accel[1]
        self.angle[2] = 0.98 * (self.angle[2] + self.gyro[2] * self.dt) + 0.02 * self.accel[2]

        # publish orientation
        orientation_msg = Imu()
        orientation_msg.header.stamp = rospy.Time.now()
        orientation_msg.orientation.x = self.angle[0]
        orientation_msg.orientation.y = self.angle[1]
        orientation_msg.orientation.z = self.angle[2]
        orientation_msg.orientation.w = 1.0
        orientation_out = geometry_msgs.msg.Quaternion(self.angle[0], self.angle[1], self.angle[2], 1.0)
        orientation_out = Quaternion(self.angle[0], self.angle[1], self.angle[2], 1.0)
        orientation_array = np.array([orientation_out.x, orientation_out.y, orientation_out.z, orientation_out.w])
        orientation_array = np.divide(orientation_array, np.linalg.norm(orientation_array))
        orientation_out.x = orientation_array[0]
        orientation_out.y = orientation_array[1]
        orientation_out.z = orientation_array[2]
        orientation_out.w = orientation_array[3]

        # integrate acceleration to get velocity
        self.velocity[0] += self.accel[0] * self.dt
        self.velocity[1] += self.accel[1] * self.dt
        self.velocity[2] += self.accel[2] * self.dt

        # integrate velocity to get position
        self.position[0] += self.velocity[0] * self.dt
        self.position[1] += self.velocity[1] * self.dt
        self.position[2] += self.velocity[2] * self.dt

        #publish pose
        pose_msg = Odometry()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = self.position[0]
        pose_msg.pose.pose.position.y = self.position[1]
        pose_msg.pose.pose.position.z = self.position[2]
        pose_msg.pose.pose.orientation.x = orientation_out.x
        pose_msg.pose.pose.orientation.y = orientation_out.y
        pose_msg.pose.pose.orientation.z = orientation_out.z
        pose_msg.pose.pose.orientation.w = orientation_out.w
        self.pose_pub.publish(pose_msg)

        # Broadcast transform
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = pose_msg.header.stamp
        transform_stamped.header.frame_id = "odom"
        transform_stamped.child_frame_id = "camera_link"
        transform_stamped.transform.translation = pose_msg.pose.pose.position
        transform_stamped.transform.rotation = orientation_out

        print(transform_stamped.header.stamp)
        

        # publish the transform message
        self.br.sendTransform(transform_stamped)

    def run(self):
        # Set node rate
        rate = rospy.Rate(over_rate) # 100 Hz

        # Spin node
        while not rospy.is_shutdown():
            self.sensor_fusion()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SensorFusion()
        node.run()
    except rospy.ROSInterruptException:
        pass
