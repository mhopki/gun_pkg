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
import ros_numpy

global cam_type, cam_fish
cam_type = 1
cam_fish = 1

class DepthToPointCloudConverter:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud_converter', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for depth image and camera info
        rospy.Subscriber('/royale_cam_royale_camera/depth_image_0', Image, self.depth_image_callback, queue_size=1) #'/dragonfly26/tof/voxl_depth_image_raw'
        rospy.Subscriber('/royale_cam_royale_camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1) #'/dragonfly26/tof/camera_info'
        rospy.Subscriber('/windows', Float32MultiArray, self.window_callback, queue_size=1)
        rospy.Subscriber('/is_glass', Bool, self.glass_callback, queue_size=1)
        rospy.Subscriber('/sonar_topic', Range, self.sonar_callback, queue_size=1)
        rospy.Subscriber('/yolo_out', Float32MultiArray, self.yolo_callback, queue_size=1)

        # Publisher for PointCloud2
        self.pointcloud_pub = rospy.Publisher('/dragonfly26/tof/reproject', PointCloud2, queue_size=1)

        self.pointcloud_pub_sonar = rospy.Publisher('/dragonfly26/tof/reproject_sonar', PointCloud2, queue_size=1)

        # Publisher for Image
        self.image_pub = rospy.Publisher('/dragonfly26/tof/voxl_alter', Image, queue_size=1)

        # Publisher for Image
        self.marker_pub = rospy.Publisher('/dragonfly26/tof/voxl_marker', MarkerArray, queue_size=1)

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

    def depth_image_callback(self, depth_msg):

        # Convert depth image to numpy array
        #depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        image_32FC1 = depth_image
        image_normalized = cv2.normalize(image_32FC1, None, 0, 1, cv2.NORM_MINMAX)
        image_scaled = image_normalized * 255
        image_mono8 = np.clip(image_scaled, 0, 255).astype(np.uint8)
        depth_image = image_mono8
         
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
        
        #depth_image = cv2.convertScaleAbs(depth_image, alpha=5) #0.03 # Adjust alpha as needed, makes distance gap bigger
        #depth_image = cv2.GaussianBlur(depth_image, (5, 5), 0) #((33, 33), 0)
        depth_image = cv2.medianBlur(depth_image, 5)

        #Project extra shapes in a mask size
        image_size = depth_image.shape
        # Get the center coordinates
        center = (image_size[1] // 2, image_size[0] // 2) #(x,y)

        # Create a meshgrid of coordinates
        y, x = np.ogrid[:image_size[0], :image_size[1]]

        y_off = 0
        radius = 50#25
        depth_value = 580

        altered = 0
        # Create a mask for the circle
        ######mask = (x - center[0]) ** 2 + (y - center[1] - y_off) ** 2 <= radius ** 2

        ######pointcloud_msg_ori, points_list_ori  = self.generate_pointcloud(depth_image, depth_msg.header)

        # Insert points from the first point cloud
        """for point in pointcloud_msg_ori:
            self.octree.set_value(*point, 1.0)  # Set probability of occupancy (e.g., 1.0 for occupied)"""

        # Set the depth value for the circle
        #depth_image[mask] = 0#depth_value
        #depth_image[:,:] = 0
        is_conf = False
        self.is_glass = False
        if (self.yolo_data != None):
            if (len(self.yolo_data) > 0):
                ###print("confidence: ", self.yolo_data[4])
                if self.yolo_data[4] >= 0.55:#0.65:
                    is_conf = True
        if (self.is_glass or is_conf): # or self.yolo_data != None):
            """windows_l = self.windows
            windows = []
            i = 0
            while len(windows_l) > 0:
                windows.append(windows_l[0:4])
                windows_l = windows_l[4:]
                i += 1"""

            windows = [self.windows[i:i+4] for i in range(0, len(self.windows), 4)]

            """for box in windows:
                print(box, i)
                x_size = box[2] // 2
                y_size = box[3] // 2
                mask2 = np.logical_and(
                    np.abs(x - (box[0] + (box[2] // 2)) ) <= x_size,
                    np.abs(y - (box[1] + (box[3] // 2)) ) <= y_size
                )
                i -= 1
                #print(mask2)
                depth_mean = depth_image > 0
                depth_mean = depth_image[depth_mean]
                depth_mean = np.mean(depth_mean) * 1.2
                depth_mean = self.sonar_depth * 1000
                #print(depth_mean)
                #print("DEPTH_MEAN: ", depth_mean)

                for ii in range(len(mask2)):
                    for jj in range(len(mask2[ii])):
                        if depth_image[ii][jj] < 1:#for cam1 type
                            depth_image[ii][jj] = depth_mean# * ((ii+jj)/(len(mask2) + len(mask2[ii])))"""

            for box in windows:
                ###print(box)
                x_size = box[2] // 2
                y_size = box[3] // 2
                mask2 = np.logical_and(
                    np.abs(x - (box[0] + x_size)) <= x_size,
                    np.abs(y - (box[1] + y_size)) <= y_size
                )
                mask3 = None

                cols = image_size[1]
                rows = image_size[0]

                # Simplified computation of depth_mean
                valid_depth = depth_image > 0
                depth_mean = 0
                #depth_mean = np.mean(depth_image[valid_depth]) * 1.1#1.2
                if (self.is_glass):
                    xxx = 0
                    #depth_mean = self.sonar_depth * 1100  # Uncomment if you want to use sonar depth
                #depth_mean = (self.sonar_depth * 1100) * .75 + (np.mean(depth_image[valid_depth]) * 1.1) *.25
                #depth_image[mask2] = depth_mean
                ###print("sonar depth: ", depth_mean)
                #depth_mean = -1
                offset_x = 0
                offset_y = 0

                if len(self.yolo_data) > 0:
                    ###print(self.yolo_data)
                    mask3 = np.logical_and(
                        np.abs(x - (self.yolo_data[0]-(self.yolo_data[2]/2) + self.yolo_data[2])) <= self.yolo_data[2],
                        np.abs(y - (self.yolo_data[1]-(self.yolo_data[3]/2) + self.yolo_data[3])) <= self.yolo_data[3]
                    )
                    depth_mean_yolo = depth_image[mask3]
                    valid_depth_yolo = depth_mean_yolo > 0
                    depth_mean_yolo = np.mean(depth_mean_yolo[valid_depth_yolo])
                    ####print("yolo depth: ", depth_mean_yolo)
                    depth_mean = depth_mean_yolo

                    offset_x = -center[0] + self.yolo_data[0]
                    offset_y = -center[1] + self.yolo_data[1]

                    if mask3 is not None:
                        mask_inside = np.logical_and(mask3, mask2)
                        is_mask3_inside_mask2 = np.all(mask3 == mask_inside)
                        
                        if is_mask3_inside_mask2:
                            xxx = 0 #print("mask3 is completely inside mask2")
                            print("depth_mean: ", depth_mean)
                        else:
                            depth_mean = -10

                if (depth_mean > -10 and depth_mean < 65):
                    cols2, rows2 = mask2.shape

                    offset_angle = ((-offset_y)/112) * 90

                    tilt_angle_radians = np.radians(offset_angle)

                    # Calculate half-widths of the window along the tilted axis
                    half_width_left = (rows2 / 2) * 4 * np.sin(tilt_angle_radians)
                    half_width_right = (rows2 / 2) * 4 * np.sin(tilt_angle_radians)

                    # Calculate the depths at the left and right corners
                    depth_left = depth_mean #depth_mean - (half_width_left)
                    depth_right = depth_mean #depth_mean + (half_width_right)

                
                    # Define minimum and maximum depths
                    min_depth = depth_left #depth_mean - 1000 # Replace with your desired minimum depth
                    max_depth = depth_right #depth_mean + 1000 # Replace with your desired maximum depth
                    #print("offset_angle: ", offset_angle, offset_x, offset_y)
                    #print("depth: ", depth_left, depth_right, depth_mean, tilt_angle_radians, np.degrees(tilt_angle_radians), half_width_left, half_width_right)

                    half_w = rows/2
                    depth_correct = depth_mean #(depth_mean) * np.sin(tilt_angle_radians)
                    depth_l = depth_mean #np.sqrt(depth_mean**2 + half_w**2) - (depth_correct)
                    depth_r = depth_mean #np.sqrt(depth_mean**2 + half_w**2) + (depth_correct)
                    min_depth = depth_l
                    max_depth = depth_r
                    #print("NEW: ", half_w, depth_correct, depth_l, depth_r, depth_mean)

                    # Create a linear gradient across the x-axis
                    depth_gradient = np.linspace(min_depth, max_depth, cols2)[:, np.newaxis]

                    # Create a 2D array with the linear gradient repeated for each row
                    depth_mask = np.tile(depth_gradient, (1, rows2))
                    print(depth_mask.shape)
                    #depth_mask = depth_gradient

                    #depth_mask = depth_mask.flatten()[:len(depth_image[mask2].flatten())]
                    #print(len(depth_mask.flatten()), len(mask2.flatten()))

                    # Use boolean indexing for efficient assignment
                    depth_image[mask2] = depth_mask[mask2]

                    altered = 1

        self.yolo_data = None
        
        y_off = 0
        radius = 240#95#25
        depth_value = 580
        # Create a mask for the circle
        maskOuter = (x - center[0]) ** 2 + (y - center[1] - y_off) ** 2 >= radius ** 2
        #depth_image[maskOuter] = 0
        
        image_out = ros_numpy.image.numpy_to_image(depth_image, "mono8") #np.frombuffer(depth_image.data, dtype=np.float32).reshape(depth_image.height, depth_image.width)
        #image_out = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        image_out_norm = ros_numpy.image.numpy_to_image(image_mono8, "mono8")

        # Generate PointCloud2 message
        ###pointcloud_msg, points_list  = self.generate_pointcloud(depth_image, depth_msg.header)
        #pointcloud_msg = self.generate_pointcloud(depth_alter, depth_msg.header)

        #print(len(pointcloud_msg))
        #print(len([pointcloud_msg, depth_msg.header]))
        #marker_msg = self.generate_colored_markers(points_list, depth_msg.header)

        # Publish the PointCloud2 message
        #self.pointcloud_pub.publish(pointcloud_msg_ori)
        ##self.pointcloud_pub_sonar.publish(pointcloud_msg)
        image_out.header = depth_msg.header
        if (True):
            self.image_pub.publish(image_out)
        else:
            self.image_pub.publish(image_out_norm)
        if False:
            if self.rate_up == 20:
                ###print("publish")
                #self.marker_pub.publish(MarkerArray(markers=marker_msg))
                self.rate_up = 0
            else:
                ###print(self.rate_up)
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
        self.fx = camera_info_msg.K[0]  # focal length in x direction
        self.fy = camera_info_msg.K[4]  # focal length in y direction
        self.cx = camera_info_msg.K[2]  # principal point in x direction
        self.cy = camera_info_msg.K[5]  # principal point in y direction
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
        rows, cols = depth_image.shape

        # Create arrays for u and v coordinates
        u_coords, v_coords = np.meshgrid(range(cols), range(rows), indexing='ij')

        # Transpose u_coords for correct broadcasting
        u_coords = u_coords.T
        v_coords = v_coords.T

        # Convert depth to meters
        depth_values = depth_image / 1000.0

        # Mask for valid depth values
        valid_depth_mask = depth_values > 0.0

        # Calculate x, y, z coordinates
        x = (u_coords - self.cx) * depth_values * self.inv_fx
        y = (v_coords - self.cy) * depth_values * self.inv_fy
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
