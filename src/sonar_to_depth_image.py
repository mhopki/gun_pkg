#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray

global cam_type, cam_fish
cam_type = 1
cam_fish = 1

def sonar_callback(value):
    global sonar_value
    sonar_value = value.range

def depth_callback(value):
    global depth_img_comp
    depth_img_comp = value

def yolo_callback(value):
    global yolo_bbox, yolo_results
    yolo_bbox = value.detections[0].bbox
    yolo_results = value.detections[0].results[0]

def create_depth_image(radius, depth_value, image_size=(480, 640)):
    # Create an empty depth image
    depth_image = np.ones(image_size, dtype=np.float32) * 255

    # Get the center coordinates
    center = (image_size[1] // 2, image_size[0] // 2)

    # Create a meshgrid of coordinates
    y, x = np.ogrid[:image_size[0], :image_size[1]]

    y_off = 0
    # Create a mask for the circle
    global cam_type
    if cam_type == 0:
        y_off = 100
    mask = (x - center[0]) ** 2 + (y - center[1] - y_off) ** 2 <= radius ** 2

    # Set the depth value for the circle
    depth_image[mask] = depth_value

    return depth_image, mask

def publish_depth_image():
    # Initialize ROS node
    rospy.init_node('depth_image_publisher', anonymous=True)
    sub_sonar = rospy.Subscriber('/sonar_topic', Range, sonar_callback, queue_size=1)
    global cam_type

    if cam_type == 0:
        sub_depth = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback, queue_size=1)
    elif cam_type == 1:
        sub_depth = rospy.Subscriber('/dragonfly26/tof/voxl_depth_image_raw', Image, depth_callback, queue_size=1)

    sub_yolo = rospy.Subscriber('/yolov7/yolov7', Detection2DArray, yolo_callback, queue_size=1)

    global depth_img_comp
    depth_img_comp = None
    # Set the radius and depth value
    global sonar_value
    sonar_value = 10
    depth_value = 0
    radius = 0#*20
    global yolo_bbox, yolo_results
    yolo_bbox = None
    yolo_results = None

    # Create a depth image
    depth_image,_ = create_depth_image(radius, depth_value)

    # Convert the NumPy array to a ROS Image message
    bridge = CvBridge()
    depth_image_ros = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
    if cam_type == 0:
        i_h = 480 #depth_image_ros.height
        i_w = 640 #depth_image_ros.width
    elif cam_type == 1:
        i_h = 172 #depth_image_ros.height
        i_w = 224 #depth_image_ros.width
    h_w = (i_h, i_w)
    ###print("og_hw: ", h_w)

    # Create a publisher for the depth image
    pub = rospy.Publisher('/sonar_depth_image', Image, queue_size=1)
    pub2 = rospy.Publisher('/comp_image', Image, queue_size=1)
    pub3 = rospy.Publisher('/is_glass', Bool, queue_size=1)
    pub4 = rospy.Publisher('/yolo_out', Float32MultiArray, queue_size=1)

    # Publish the depth image
    rate = rospy.Rate(10)  # 1 Hz
    while not rospy.is_shutdown():
        """
        depth_value = sonar_value * 1.5 # GOES WHITE AT 10
        radius = sonar_value*40 + 20
        if sonar_value >= 5:
            radius = 0
        ###print(radius, depth_value, sonar_value)
        # Create a depth image
        #####print("h_w: ", h_w)
        depth_image,mask = create_depth_image(radius, depth_value, h_w)
        # Convert the NumPy array to a ROS Image message
        #bridge = CvBridge()
        depth_image_ros = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        pub.publish(depth_image_ros)

        if depth_img_comp is not None:
            mid_img = bridge.imgmsg_to_cv2(depth_img_comp, desired_encoding="passthrough")
            if cam_type == 0:
                img_comp = np.ones_like(mid_img) * np.inf#np.inf
            elif cam_type == 1:
                img_comp = np.ones_like(mid_img) * 255#np.inf

            img_comp[mask] = mid_img[mask]

            if cam_type == 0:
                img_comp = cv2.convertScaleAbs(img_comp, alpha=0.0052)
            elif cam_type == 1:
                img_comp = cv2.convertScaleAbs(img_comp, alpha=0.99) #0.03 # Adjust alpha as needed, makes distance gap bigger
                img_comp = cv2.GaussianBlur(img_comp, (15, 15), 0) #((33, 33), 0)

            _, img_comp = cv2.threshold(img_comp, 20, 255, cv2.THRESH_BINARY)

            if cam_type == 1:
                img_comp = cv2.bitwise_not(img_comp)

            d_avg = np.mean(img_comp[mask])
            d_max = max(img_comp[mask])
            d_min = min(img_comp[mask])
            d_ratio = .05
            if cam_type == 1:
                d_ratio = .5#.25

            is_glass = False
            if d_max > d_min and d_avg > d_max*d_ratio and sonar_value <= 0.9:#1.4:
                ###print("GLASS")
                cv2.putText(img_comp, "GLASS", (i_w//2, i_h//2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) 
                cv2.putText(img_comp, "GLASS", (i_w//2 - 2, i_h//2 - 10 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1) 
                is_glass = True

            img_out = bridge.cv2_to_imgmsg(img_comp, encoding="passthrough")
            pub2.publish(img_out)

            bool_msg = Bool()
            bool_msg.data = is_glass 
            pub3.publish(bool_msg)
            """
        if (True):
            yolo_out = Float32MultiArray()
            if (yolo_bbox != None and yolo_results != None):
                yolo_data = [yolo_bbox.center.x, yolo_bbox.center.y, yolo_bbox.size_x, yolo_bbox.size_y, yolo_results.score]
                yolo_out.data = yolo_data
            pub4.publish(yolo_out)
        #rate.sleep()

if __name__ == '__main__':
    try:
        publish_depth_image()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
