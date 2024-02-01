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
from tf.transformations import quaternion_from_matrix
#from ekf import ExtendedKalmanFilter
#from robot_localization.srv import ExtendedKalmanFilter
#from robot_localization.extended_kalman_filter import ExtendedKalmanFilter
from scipy.linalg import inv
from scipy.spatial.transform import Rotation as R
import pyquaternion as pqt


over_rate = 100

class EKF:
    def __init__(self, Q, R):
        # state transition matrix
        self.F = np.eye(4)
        # observation matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0]])
        # state covariance matrix
        self.P = np.eye(4)
        # process noise covariance matrix
        self.Q = Q
        # measurement noise covariance matrix
        self.R = R
        # state vector (quaternion)
        self.x = np.array([1, 0, 0, 0])
        # initialize timestamp
        self.last_update_time = rospy.Time.now()

    def predict(self, dt, omega):
        # compute state transition matrix
        q1, q2, q3, q4 = self.x
        wx, wy, wz = omega
        F = np.array([[1, -dt*wx/2, -dt*wy/2, -dt*wz/2],
                      [dt*wx/2, 1, dt*wz/2, -dt*wy/2],
                      [dt*wy/2, -dt*wz/2, 1, dt*wx/2],
                      [dt*wz/2, dt*wy/2, -dt*wx/2, 1]])
        # update state transition matrix
        self.F = F
        # predict state
        self.x = np.dot(F, self.x)
        # compute process noise covariance matrix
        Qd = self.Q * dt
        # update state covariance matrix
        self.P = np.dot(np.dot(F, self.P), F.T) + Qd

    def update(self, accel):
        # compute innovation
        y = accel - np.dot(self.H, self.x)
        # compute innovation covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        # compute Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), inv(S))
        # update state
        self.x = self.x + np.dot(K, y)
        # update state covariance matrix
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

    def quaternion_from_matrix(self, R):
        q = np.empty((4, ))
        t = np.trace(R)
        if t > 0:
            s = np.sqrt(t + 1) * 2
            q[0] = 0.25 * s
            q[1] = (R[2, 1] - R[1, 2]) / s
            q[2] = (R[0, 2] - R[2, 0]) / s
            q[3] = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2

    def quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        R = np.array([[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3)],
                      [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
                      [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)]])
        return R

    def get_rotation_matrix(self):
        return self.quaternion_to_rotation_matrix(self.x)


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

        Q = np.diag([0.05, 0.05, 0.05, 0.1])
        R = np.diag([0.5, 0.5, 0.5])
        self.ekf = EKF(Q,R)
        self.prev_time = None

        self.br = tf2_ros.TransformBroadcaster()

        self.pose_pub = rospy.Publisher("/imu/pose", Odometry, queue_size=10)

        # Set up subscribers
        rospy.Subscriber('/camera/accel/sample', Imu, self.accel_callback)
        rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback)

        # create publisher for orientation
        
        

        # Set up broadcaster
        
        #self.br = tf.TransformBroadcaster()

    def accel_callback(self, msg):
        # Get accelerometer data
        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z

        self.sensor_fusion(msg.header.stamp )

    def gyro_callback(self, msg):
        # Get gyroscope data
        self.gyro[0] = msg.angular_velocity.x
        self.gyro[1] = msg.angular_velocity.y
        self.gyro[2] = msg.angular_velocity.z

        self.sensor_fusion(msg.header.stamp )

    def sensor_fusion(self, time_s):
        # Implement sensor fusion algorithm here to calculate orientation
        # ...
        # calculate orientation using complementary filter
        if self.prev_time is None:
            self.prev_time = time_s
            return

        dt = (time_s - self.prev_time).to_sec()
        accel = np.array(self.accel)
        gyro = np.array(self.gyro)
        self.ekf.predict(dt, gyro)
        self.ekf.update(accel)

        translation = self.ekf.x[:3]
        #bigT = self.ekf.x[:3, :3]
        #print(translation)
        #print("vs")
        #print(bigT)
        rotation_matrix = self.ekf.get_rotation_matrix()
        quaternion = Quaternion(self.ekf.x[1], self.ekf.x[3], self.ekf.x[0], self.ekf.x[2]+315.0)
        orientation_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        orientation_array = np.divide(orientation_array, np.linalg.norm(orientation_array))

        # calculate quaternion derivative
        #q = np.array([self.ekf.x[0], self.ekf.x[1], self.ekf.x[2], self.ekf.x[3]])
        #q_gyro = np.array([0, self.gyro[0], self.gyro[1], self.gyro[2]])
        #q_dot = 0.5 * q * q_gyro
        #q_accel = np.array([0, self.accel[0], self.accel[1], self.accel[2]])
        #q_dot = q_dot + q_accel * 0.5
        #print(pqt.Quaternion(*q))
        #print(*q)
        #print(pqt.Quaternion(0, *gyro))
        #print(*pqt.Quaternion(0, *gyro))

        # update quaternion state vector
        #q_dot = np.array[q_dot[0], 0, 0, 0]
        #print(q,q_dot,dt)
        #q = (q + q_dot * dt) / np.linalg.norm(q + q_dot * dt)
        #q = pqt.Quaternion(q)
        #print(q)

        # calculate rotation matrix
        #Rot = np.eye(3)
        #Rot = R.from_quat(q).as_matrix()

        #quaternion = Quaternion(q[1], q[3], q[0], q[2])

        quaternion.x = orientation_array[0]
        quaternion.y = orientation_array[1]
        quaternion.z = orientation_array[2]
        quaternion.w = orientation_array[3]

        # Integrate linear acceleration to get velocity and position
        #self.velocity = [self.velocity[i] + self.accel[i] * self.dt for i in range(3)]
        #self.position = [self.position[i] + self.velocity[i] * self.dt + 0.5 * self.accel[i] * self.dt**2 for i in range(3)]
        #print(self.velocity)
        #print(self.position)
        #print(quaternion)

        # publish translation and rotation
        # ...
        # publish the transform message
        #publish pose
        pose_msg = Odometry()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0
        pose_msg.pose.pose.orientation.x = quaternion.x
        pose_msg.pose.pose.orientation.y = quaternion.y
        pose_msg.pose.pose.orientation.z = quaternion.z
        pose_msg.pose.pose.orientation.w = quaternion.w
        self.pose_pub.publish(pose_msg)

        # Broadcast transform
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = pose_msg.header.stamp
        transform_stamped.header.frame_id = "odom"
        transform_stamped.child_frame_id = "camera_link"
        transform_stamped.transform.translation = pose_msg.pose.pose.position
        transform_stamped.transform.rotation = pose_msg.pose.pose.orientation

        self.br.sendTransform(transform_stamped)

        self.prev_time = time_s
        

    def run(self):
        # Set node rate
        rate = rospy.Rate(50) # 100 Hz

        # Spin node
        while not rospy.is_shutdown():
            self.sensor_fusion(rospy.Time.now())
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SensorFusion()
        node.run()
    except rospy.ROSInterruptException:
        pass
