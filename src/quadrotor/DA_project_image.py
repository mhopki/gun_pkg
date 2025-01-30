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


global cam_type, cam_fish
cam_type = 1
cam_fish = 1

class DepthToPointCloudConverter:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud_converter', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for depth image and camera info
        rospy.Subscriber('/camera/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/dragonfly26/tof/voxl_depth_image_raw', Image, self.depth_header_callback)
        rospy.Subscriber('/dragonfly26/tof/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/windows', Float32MultiArray, self.window_callback)
        rospy.Subscriber('/is_glass', Bool, self.glass_callback)
        rospy.Subscriber('/sonar_topic', Range, self.sonar_callback)
        rospy.Subscriber('/yolo_out', Float32MultiArray, self.yolo_callback)

        # Publisher for PointCloud2
        self.pointcloud_pub = rospy.Publisher('/dragonfly26/tof/reproject', PointCloud2, queue_size=10)

        self.pointcloud_pub_sonar = rospy.Publisher('/dragonfly26/tof/reproject_sonar', PointCloud2, queue_size=10)

        # Publisher for Image
        self.image_pub = rospy.Publisher('/dragonfly26/tof/voxl_alter', Image, queue_size=1)

        # Publisher for Image
        self.marker_pub = rospy.Publisher('/dragonfly26/tof/voxl_marker', MarkerArray, queue_size=10)

        # Camera info variables
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.inv_fx = 0.0
        self.inv_fy = 0.0

        self.windows = []
        self.sonar_depth = 0

        self.is_glass = False

        self.yolo_data = None

        self.depth_header = None

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
            self.rate.sleep()

    def depth_header_callback(self, depth_msg):
        self.depth_header = depth_msg.header

    def depth_image_callback(self, depth_msg):
        print("depth")
        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="mono8")
        #cv2.imshow('image',depth_image)
        #cv2.waitKey(0)

        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.2) #0.03 # Adjust alpha as needed, makes distance gap bigger
        #depth_image = cv2.GaussianBlur(depth_image, (5, 5), 0) #((33, 33), 0)
        #depth_image = cv2.medianBlur(depth_image, 5)

        #SAVE IMAGE TO FILE
        """
        save_path = '/home/malakhi/Pictures/CNN/Bagfiles/1'
        # Save the image as JPEG
        timestamp_str = rospy.Time.now().to_sec()  # Convert timestamp to seconds
        image_filename = f"{save_path}/captured_image_{timestamp_str:.6f}.jpg"
        cv2.imwrite(image_filename, depth_image)
        rospy.loginfo("Image saved as %s", image_filename)
        print("saved: ", image_filename)
        """

        image_out = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono8")

        pheader = depth_msg.header
        if self.depth_header != None:
            pheader = self.depth_header

        # Generate PointCloud2 message
        pointcloud_msg, points_list  = self.generate_pointcloud(depth_image, pheader)
        #pointcloud_msg = self.generate_pointcloud(depth_alter, depth_msg.header)

        #print(len(pointcloud_msg))
        #print(len([pointcloud_msg, depth_msg.header]))
        #marker_msg = self.generate_colored_markers(points_list, depth_msg.header)

        # Publish the PointCloud2 message
        #self.pointcloud_pub.publish(pointcloud_msg_ori)
        self.pointcloud_pub_sonar.publish(pointcloud_msg)
        self.image_pub.publish(image_out)
        if False:
            if self.rate_up == 20:
                print("publish")
                #self.marker_pub.publish(MarkerArray(markers=marker_msg))
                self.rate_up = 0
            else:
                print(self.rate_up)
                self.rate_up += 1

        #####rows, cols = depth_image.shape
        """for i in range(rows):
            for j in range(cols):
                if pointcloud_msg_ori[i,j] != pointcloud_msg[i,j]:
                    point = self.octree[i,j]
                    current_probability = self.octree.get_value(*point)
                    new_probability = 0.5#update_probability(current_probability, sensor_model(point))
                    self.octree.set_value(*point, new_probability)"""


        """for point in point_cloud2:
            current_probability = octree.get_value(*point)
            new_probability = update_probability(current_probability, sensor_model(point))
            octree.set_value(*point, new_probability)"""

        """octomap_msg = Octomap()
        octomap_msg.header.stamp = rospy.Time.now()
        octomap_msg.header.frame_id = "your_frame_id"  # Set your desired frame ID

        # Serialize the Octree and assign to the Octomap message
        octomap_msg.data = self.octree.to_binary()

        octomap_pub = rospy.Publisher('/sonar_octo', Octomap, queue_size=1)

        # Publish the Octomap message
        octomap_pub.publish(octomap_msg)"""

    def camera_info_callback(self, camera_info_msg):
        # Extract camera parameters
        #K: [112.03800201416016, 0.0, 114.78600311279297, 0.0, 112.03800201416016, 85.03160095214844, 0.0, 0.0, 1.0]
        self.fx = 1112 #15000 #112.03800201416016 #camera_info_msg.K[0]  # focal length in x direction
        self.fy = 1112 #15000 #112.03800201416016 #camera_info_msg.K[4]  # focal length in y direction
        self.cx = 14 #960 #114.78600311279297 #camera_info_msg.K[2]  # principal point in x direction
        self.cy = 5 #540 #85.03160095214844 #camera_info_msg.K[5]  # principal point in y direction
        self.inv_fx = 1.0 / self.fx
        self.inv_fy = 1.0 / self.fy

    def window_callback(self, data):
        # Extract camera parameters
        self.windows = data.data

    def sonar_callback(self, data):
        # Extract camera parameters
        #print("son call", data.range)
        self.sonar_depth = data.range

    def glass_callback(self, data):
        # Extract camera parameters
        self.is_glass = data.data

    def yolo_callback(self, data):
        # Extract camera parameters
        self.yolo_data = data.data

    def generate_pointcloud(self, depth_image, header):
        # Create PointCloud2 message
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            #PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),  # Color as UINT32
        ]

        #print(header)
        points = self.convert_depth_to_points(depth_image, header)
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        return pointcloud_msg, points

    def convert_depth_to_points(self, depth_image, header):
        #print(depth_image.shape)
        cut_depth_image = depth_image#[:,:,0]
        rows, cols = cut_depth_image.shape

        # Create arrays for u and v coordinates
        u_coords, v_coords = np.meshgrid(range(cols), range(rows), indexing='ij')

        # Transpose u_coords for correct broadcasting
        u_coords = u_coords.T
        v_coords = v_coords.T

        # Convert depth to meters
        depth_values = cut_depth_image / 1000.0#1000.0

        # Mask for valid depth values
        valid_depth_mask = depth_values > 0.0

        # Calculate x, y, z coordinates
        x = (u_coords - self.cx) * depth_values * (self.sonar_depth * 10) * self.inv_fx
        y = (v_coords - self.cy) * depth_values * (self.sonar_depth * 10) * self.inv_fy
        z = depth_values

        # Create RGB array
        #r = 59
        #g = 0
        #b = 255
        #rgb = (r << 16) | (g << 8) | b

        # Mask out invalid points
        x = np.where(valid_depth_mask, x, 0.0)
        y = np.where(valid_depth_mask, y, 0.0)
        z = np.where(valid_depth_mask, z, 0.0)

        # Stack x, y, z, and rgb to create the points array
        #points = np.stack((x, y, z, np.full_like(x, rgb)), axis=-1)
        points = np.stack((x, y, z), axis=-1)

        points = points.astype(np.float32)

        # Flatten the array and remove invalid points
        points = points[valid_depth_mask]

        #print(points.shape)
        #print(points[0])
        #print(points[1])

        return points

    def generate_colored_markers(self, points, header):
        marker_array = []
        self.id_counter = 0
        #print("AYOOOO")

        for point in points:
            marker = Marker()
            marker.header.frame_id = header.frame_id  # Replace with your frame_id
            #marker.header.stamp = header.stamp
            marker.ns = "cp"
            marker.id = self.id_counter
            marker.type = Marker.CUBE
            #marker.action = Marker.ADD
            #print(point)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point[:3]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05  # Adjust the scale as needed
            marker.color.r = ((point[3] >> 16) & 0xFF) / 255.0
            marker.color.g = ((point[3] >> 8) & 0xFF) / 255.0
            marker.color.b = (point[3] & 0xFF) / 255.0
            marker.color.a = 1.0  # Alpha channel
            marker_array.append(marker)

            self.id_counter += 1

        # Publish the MarkerArray
        return marker_array

if __name__ == '__main__':
    try:
        converter = DepthToPointCloudConverter()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
