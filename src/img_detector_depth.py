#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

global cam_type, cam_fish
cam_type = 1
cam_fish = 1

def image_callback(msg):
    bridge = CvBridge()
    global cam_type

    # Convert the depth image to a numpy array
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #new = np.ones_like(depth_image) * 277
    #0 = depth-passive, 1 = depth-active
    #cam_type = 1
    #cam_fish = 1

    """center = (depth_image.shape[1] // 2, depth_image.shape[0] // 2)
    print(center, depth_image.shape[1], depth_image.shape[0])
    radius = 15
    depth_value = (255, 0, 0) # Depth in meters

    new = cv2.circle(new, center, radius, depth_value, thickness=-1)
    #depth_image = cv2.circle(depth_image, center, radius, depth_value, thickness=-1)

    """
    # Create a depth mask for the circle
    y, x = np.ogrid[:depth_image.shape[0], :depth_image.shape[1]]
    """
    new = depth_image.copy()

    radius = 15
    mask = (x - depth_image.shape[1] // 2) ** 2 + (y - depth_image.shape[0] // 2) ** 2 < radius ** 2

    new[mask] = 277

    new_image = bridge.cv2_to_imgmsg(new, encoding="passthrough")

    #print(depth_image)
    depth_image_pub.publish(new_image)"""

    # Create a grayscale image (single-channel)
    #Realsensed435i alpha=0.0052, size = 640,480
    if cam_type == 0:
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.0052)
    elif cam_type == 1:
        depth_gray = depth_image
        # Assuming 'image_32FC1' is your input image in 32FC1 format
        image_32FC1 = depth_gray.copy()#np.random.rand(100, 100).astype(np.float32)  # Example image

        # Normalize to range [0, 1] (if not already normalized)
        image_normalized = cv2.normalize(image_32FC1, None, 0, 1, cv2.NORM_MINMAX)

        # Scale to range [0, 255]
        image_scaled = image_normalized * 255

        # Convert to 8-bit unsigned integer
        image_mono8 = np.clip(image_scaled, 0, 255).astype(np.uint8)
        depth_gray = image_mono8
        #depth_gray = np.uint8(depth_gray)
        depth_gray = cv2.convertScaleAbs(depth_gray, alpha=0.99)
        ###depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.99) #0.03 # Adjust alpha as needed, makes distance gap bigger
        depth_gray = cv2.GaussianBlur(depth_gray, (15, 15), 0) #((33, 33), 0)
        #depth_gray = cv2.medianBlur(depth_gray, 31) #61
        #depth_gray = cv2.bitwise_not(depth_gray)

    if cam_fish == 1:
        #TOF FISHEYE
        radius = 240#95
        mask = (x - depth_image.shape[1] // 2) ** 2 + (y - depth_image.shape[0] // 2) ** 2 >= radius ** 2
        dp = 0 if cam_type == 0 else 255
        depth_gray[mask] = dp

    # Create a color image (bgr8) from the grayscale image
    depth_color = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

    # Threshold the grayscale image to isolate dark areas (assuming black squares)
    _, thresholded = cv2.threshold(depth_gray, 20, 255, cv2.THRESH_BINARY)
    #_, thresholded = cv2.threshold(depth_gray, 0, 255, cv2.THRESH_BINARY)

    # Publish the thresholded image
    if cam_type == 1:
        thresholded = cv2.bitwise_not(thresholded)
    thresholded_msg = bridge.cv2_to_imgmsg(thresholded, encoding="passthrough") #"mono8"
    thresholded_pub.publish(thresholded_msg)

    #"""

    # Find contours in the thresholded image
    #_, contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sum_box = []
    # Check if any contours represent a square
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.01 * cv2.arcLength(contour, True) #0.04
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # If the polygon has four vertices, it's likely a square
        if len(approx) >= 0:
            # Draw a bounding rectangle around the square on the color image
            x, y, w, h = cv2.boundingRect(approx)
            #w = 40x40 in 640x480
            if w > depth_image.shape[1]*0.0625 and h > depth_image.shape[0]*0.083:#change this to make glass detection wide view #normal /1
                #cv2.rectangle(depth_color, (x, y), (x + w, y + h), (0, 0, 255), 1)
                #cv2.putText(depth_color, "Window", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                sum_box.append([x,y,w,h])
                ##print("Window found", w, h)
                #break
    
    cascade = 1
    win_tol = depth_image.shape[1]*0.0625/20 #normal: depth_image.shape[1]*0.0625/2 #big_window tolerance 20 in 640x480
    max_windows = []
    origin_box = sum_box.copy()
    while cascade > 0 and len(sum_box) > 1:
        temp = []
        temp_r = []
        cascade = 0
        
        #merge overlapping windows
        for i, box_i in enumerate(sum_box):
            for j, box_j in enumerate(sum_box):
                if i != j:
                    if (
                        (box_i[0] > box_j[0] - win_tol and box_i[0] <= box_j[0] + box_j[2] + win_tol) and
                        (box_i[1] > box_j[1] - win_tol and box_i[1] <= box_j[1] + box_j[3] + win_tol)
                    ):
                        if box_j[0] > box_i[0]:
                            t_x, t_y, t_w, t_h = box_i[0], box_i[1], abs(box_i[0] - (box_j[0] + box_j[2])), abs(box_i[1] - (box_j[1] + box_j[3]))
                        else:
                            t_x, t_y, t_w, t_h = box_j[0], box_j[1], abs(box_i[0] + box_i[2] - box_j[0]), abs(box_i[1] + box_i[3] - box_j[1])

                        if [t_x, t_y, t_w, t_h] not in temp:
                            temp.append((t_x, t_y, t_w, t_h))
                            if box_i not in temp_r:
                                temp_r.append(box_i)
                            if box_j not in temp_r:
                                temp_r.append(box_j)
                            cascade = 1

        #add max_sized windows and remove smaller windows
        if len(temp) > 0:

            if set(temp) == set(temp_r):
                ##print("BREAK")
                cascade = 0
            for box in temp:
                sum_box.append(box)
                if box not in max_windows:
                    max_windows.append(box)
            for box in temp_r:
                sum_box.remove(box)
                if box in max_windows:
                    max_windows.remove(box)
            ##print("NG: ", sum_box)
            temp = []
            temp_r = []
        
        #publish windows
        if cascade == 0 or len(sum_box) <= 1:

            box_out = Float32MultiArray()
            box_list = []
            for box in sum_box:
                cv2.rectangle(depth_color, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 1)
                cv2.putText(depth_color, "Window", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1) #MAX  
                #print(box) 
                #box_list.append(box[0])
                #box_list.append(box[1])
                #box_list.append(box[2])
                #box_list.append(box[3])
                box_list.extend(box)
                
            box_out.data = box_list
            window_pub.publish(box_out)         
    else:
        if len(sum_box) == 1:
            box_out = Float32MultiArray()
            box_list = []
            for box in sum_box:
                cv2.rectangle(depth_color, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 1)
                cv2.putText(depth_color, "Window", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)#MAX
                #print(box)
                #box_list.append(box[0])
                #box_list.append(box[1])
                #box_list.append(box[2])
                #box_list.append(box[3])
                box_list.extend(box)
                
            box_out.data = box_list
            window_pub.publish(box_out)  
        else:
            ##print("No Window")
            x = 0

    #"""
    # Publish the depth_gray image

    depth_gray_msg = bridge.cv2_to_imgmsg(depth_gray, encoding="mono8") #"mono8"
    depth_gray_pub.publish(depth_gray_msg)

    if (False):
        #SAVE IMAGE TO FILE
        #"""
        save_path = '/home/malakhi/Pictures/CNN/Bagfiles/3'
        # Save the image as JPEG
        timestamp_str = rospy.Time.now().to_sec()  # Convert timestamp to seconds
        image_filename = f"{save_path}/captured_image_{timestamp_str:.6f}.jpg"
        cv2.imwrite(image_filename, depth_gray)
        rospy.loginfo("Image saved as %s", image_filename)
        print("saved: ", image_filename)
        #"""

    # Publish the depth_color image
    depth_color_msg = bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")#bgr8
    depth_color_pub.publish(depth_color_msg)

    # Display the annotated color image
    cv2.imshow("Square Detection", depth_color)
    cv2.waitKey(3)

def main():
    rospy.init_node('square_detection_node')
    
    # Create publishers for depth_gray, depth_color, and thresholded images
    global depth_gray_pub, depth_color_pub, thresholded_pub, depth_image_pub, window_pub
    global cam_type
    depth_gray_pub = rospy.Publisher('depth_gray_image', Image, queue_size=1)
    depth_color_pub = rospy.Publisher('depth_color_image', Image, queue_size=1)
    thresholded_pub = rospy.Publisher('thresholded_image', Image, queue_size=1)
    depth_image_pub = rospy.Publisher('depth_image_pub', Image, queue_size=1)
    window_pub = rospy.Publisher('windows', Float32MultiArray, queue_size=10)

    if cam_type == 0:
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, image_callback)
    elif cam_type == 1:
        rospy.Subscriber('/royale_cam_royale_camera/depth_image_0', Image, image_callback)#'/dragonfly26/tof/voxl_depth_image_raw'
    #rospy.spin()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
