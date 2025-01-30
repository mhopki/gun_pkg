#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import struct
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
# import pyoctree
import rospy
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import BoundingBoxQuery
from octomap_msgs.srv import GetOctomap
from sensor_msgs.msg import Range
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry

from std_msgs.msg import Int32


global cam_type, cam_fish
cam_type = 1
cam_fish = 1

class DepthToPointCloudConverter:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud_converter', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for depth image and camera info
        rospy.Subscriber('/royale_cam_royale_camera/depth_image_0', Image, self.depth_image_callback, queue_size=1)
        rospy.Subscriber('/sonar_topic', Range, self.sonar_callback)
        rospy.Subscriber('/yolov7/yolov7', Detection2DArray, self.yolo_callback)
        rospy.Subscriber('/vicon/Starling2/odom', Odometry, self.odom_callback)

        # Publisher for PointCloud2
        self.pointcloud_pub = rospy.Publisher('/dragonfly26/tof/reproject', PointCloud2, queue_size=10)

        self.pointcloud_pub_sonar = rospy.Publisher('/dragonfly26/tof/reproject_sonar', PointCloud2, queue_size=10)

        # Publisher for Image
        self.image_pub = rospy.Publisher('/dragonfly26/tof/voxl_alter', Image, queue_size=1)

        # Publisher for Image
        self.marker_pub = rospy.Publisher('/dragonfly26/tof/voxl_marker', MarkerArray, queue_size=10)

        self.command_pub = rospy.Publisher('/quad_coms', Int32, queue_size=10)

        # Camera info variables
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.inv_fx = 0.0
        self.inv_fy = 0.0

        self.quad_odom = None

        self.windows = []
        self.sonar_depth = 1000

        self.is_glass = False

        self.yolo_data = None
        self.last_yolo = None
        self.yolo_depth = 1000
        self.last_yolo_depth = 1000
        self.yolo_streak = 0
        self.sonar_data = None
        self.last_sonar = None
        self.sonar_streak = 0

        self.rate_up = 0

        self.id_counter = 0

        self.probvals = []

        #self.octree = pyoctree.OcTree(0.05)

        # Create a Rate object to control the loop frequency
        self.rate = rospy.Rate(10)  # 10 Hz

        # Main loop
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            #print(self.sonar_depth, self.sonar_streak)
            #print(self.yolo_depth, self.yolo_streak)
            #print(self.yolo_data)

            stop_trigger = False
            current_time = rospy.Time.now().to_sec()

            if (self.sonar_data != None):
                if (abs(current_time - self.sonar_data[1]) > 0.35):
                    self.sonar_streak = 0
                else:
                    if (self.last_sonar != None and self.last_sonar != self.sonar_data):
                        if self.sonar_data[0] - self.last_sonar[0] > 0.2:
                            self.sonar_streak = 0
                        else:
                            self.sonar_streak += 1

                if self.sonar_streak >= 2:
                    if (self.sonar_depth < 1.5):
                        stop_trigger = False#True

            if (self.yolo_data != None):
                if (abs(current_time - self.yolo_data[5]) > 0.35):
                    self.yolo_streak = 0
                else:
                    #print("y pass time")
                    if (self.last_yolo != None and self.last_yolo != self.yolo_data):
                        #print("y pass none")
                        #print("y depth comp: ", self.yolo_depth, " vs ", self.last_yolo_depth)
                        if self.yolo_depth - self.last_yolo_depth > 0.3 or np.isnan(self.yolo_depth) or np.isnan(self.last_yolo_depth):
                            self.yolo_streak = 0
                        else:
                            #print("y pass")
                            self.yolo_streak += 1

                if self.yolo_streak >= 2:
                    if (self.yolo_depth < 1.5):
                        stop_trigger = True


            if (self.sonar_data != None and self.yolo_data != None):
                #print("pass none")
                #print(abs(self.sonar_data[1] - self.yolo_data[5]), " vs ", current_time - (self.sonar_data[1] + self.yolo_data[5])/2)
                if (abs(self.sonar_data[1] - self.yolo_data[5]) < 0.5 and (current_time - (self.sonar_data[1] + self.yolo_data[5])/2) < 0.35):
                    #print("pass time")
                    if (self.sonar_depth <= 5 and self.yolo_depth <= 5):
                        #print("pass depth")
                        depth_diff = abs(self.yolo_depth - self.sonar_depth)
                        if (self.sonar_depth - self.yolo_depth < 0.3 and (self.sonar_depth + self.yolo_depth)/2 < 1.5):
                            stop_trigger = True

            if (stop_trigger):
                print("STOP!!!")
                print("Position: ", self.quad_odom.pose.pose.position)
                print("Velocity: ", self.quad_odom.twist.twist.linear)
                print("Sonar Depth: ", self.sonar_depth, self.sonar_streak)
                print("Yolo Depth: " ,self.yolo_depth, self.yolo_streak)
                self.command_pub.publish(Int32(4))

            #self.yolo_depth = 1000
            self.last_yolo = self.yolo_data
            self.last_yolo_depth = self.yolo_depth
            self.last_sonar = self.sonar_data
            self.rate.sleep()

    def depth_image_callback(self, depth_msg):
        #print("RECRECREC")
        # Convert depth image to numpy array
        #depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        #depth_image = cv2.medianBlur(depth_image, 5)

        #Project extra shapes in a mask size
        image_size = depth_image.shape
        # Get the center coordinates
        center = (image_size[1] // 2, image_size[0] // 2) #(x,y)

        # Create a meshgrid of coordinates
        y, x = np.ogrid[:image_size[0], :image_size[1]]

        y_off = 0
        radius = 50#25
        depth_value = 580
        # Create a mask for the circle
        ######mask = (x - center[0]) ** 2 + (y - center[1] - y_off) ** 2 <= radius ** 2

        ######pointcloud_msg_ori, points_list_ori  = self.generate_pointcloud(depth_image, depth_msg.header)

        # Insert points from the first point cloud
        """for point in pointcloud_msg_ori:
            self.octree.set_value(*point, 1.0)  # Set probability of occupancy (e.g., 1.0 for occupied)"""

        # Set the depth value for the circle
        #depth_image[mask] = 0#depth_value
        #depth_image[:,:] = 0

        if self.yolo_data != None:
            if abs(self.yolo_data[5] - rospy.Time.now().to_sec()) < 0.5:#0.3
                #print(self.yolo_data)
                mask3 = np.logical_and(
                    np.abs(x - (self.yolo_data[0]-(self.yolo_data[2]/2) + self.yolo_data[2])) <= self.yolo_data[2],
                    np.abs(y - (self.yolo_data[1]-(self.yolo_data[3]/2) + self.yolo_data[3])) <= self.yolo_data[3]
                )
                depth_mean_yolo = depth_image[mask3]
                valid_depth_yolo = depth_mean_yolo > 0
                depth_mean_yolo = np.mean(depth_mean_yolo[valid_depth_yolo])
                self.yolo_depth = depth_mean_yolo # / 1000
                #print("yolo depth: ", depth_mean_yolo)
                depth_mean = depth_mean_yolo

                offset_x = -center[0] + self.yolo_data[0]
                offset_y = -center[1] + self.yolo_data[1]

    def camera_info_callback(self, camera_info_msg):
        # Extract camera parameters
        self.fx = camera_info_msg.K[0]  # focal length in x direction
        self.fy = camera_info_msg.K[4]  # focal length in y direction
        self.cx = camera_info_msg.K[2]  # principal point in x direction
        self.cy = camera_info_msg.K[5]  # principal point in y direction
        self.inv_fx = 1.0 / self.fx
        self.inv_fy = 1.0 / self.fy

    def sonar_callback(self, data):
        # Extract camera parameters
        #print("son call", data.range)
        self.sonar_depth = data.range * 10
        self.sonar_data = [data.range * 10, rospy.Time.now().to_sec()]

    def yolo_callback(self, value):
        #global yolo_bbox, yolo_results
        max_size = 0
        #print("RECYOLOYOLO")
        max_ind = 0
        for i in range(len(value.detections)):
            det = value.detections[i]
            det_size = det.bbox.size_x * det.bbox.size_y
            if det_size > max_size:
                max_ind = i
                max_size = det_size

        yolo_bbox = value.detections[max_ind].bbox
        yolo_results = value.detections[max_ind].results[0]
        yolo_header = value.header
        self.yolo_data = [yolo_bbox.center.x, yolo_bbox.center.y, yolo_bbox.size_x, yolo_bbox.size_y, yolo_results.score, rospy.Time.now().to_sec()]

    def odom_callback(self, data):
        # Extract camera parameters
        #print("son call", data.range)
        self.quad_odom = data
        #self.sonar_data = [data.range, rospy.Time.now().to_sec()]


if __name__ == '__main__':
    try:
        converter = DepthToPointCloudConverter()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
