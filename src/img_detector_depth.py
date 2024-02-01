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
    new = np.ones_like(depth_image) * 277
    #0 = depth-passive, 1 = depth-active
    #cam_type = 1
    #cam_fish = 1

    center = (depth_image.shape[1] // 2, depth_image.shape[0] // 2)
    print(center, depth_image.shape[1], depth_image.shape[0])
    radius = 15
    depth_value = (255, 0, 0) # Depth in meters

    new = cv2.circle(new, center, radius, depth_value, thickness=-1)
    #depth_image = cv2.circle(depth_image, center, radius, depth_value, thickness=-1)

    new_image = bridge.cv2_to_imgmsg(new, encoding="passthrough")

    #print(depth_image)
    depth_image_pub.publish(new_image)

    # Create a grayscale image (single-channel)
    #Realsensed435i alpha=0.0052, size = 640,480
    if cam_type == 0:
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.0052)
    elif cam_type == 1:
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.99) #0.03 # Adjust alpha as needed, makes distance gap bigger
        depth_gray = cv2.GaussianBlur(depth_gray, (15, 15), 0) #((33, 33), 0)
        #depth_gray = cv2.medianBlur(depth_gray, 31) #61
        #depth_gray = cv2.bitwise_not(depth_gray)

    if cam_fish == 1:
        #TOF FISHEYE
        # Create a meshgrid of coordinates
        y, x = np.ogrid[:depth_image.shape[0], :depth_image.shape[1]]

        # Create a mask for the circle
        radius = 95
        mask = (x - center[0]) ** 2 + (y - center[1]) ** 2 >= radius ** 2
        if cam_type == 0:
            dp = 0
        elif cam_type == 1:
            dp = 255

        # Set the depth value for the circle
        depth_gray[mask] = dp

    # Create a color image (bgr8) from the grayscale image
    depth_color = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

    # Threshold the grayscale image to isolate dark areas (assuming black squares)
    _, thresholded = cv2.threshold(depth_gray, 20, 255, cv2.THRESH_BINARY)
    #_, thresholded = cv2.threshold(depth_gray, 0, 255, cv2.THRESH_BINARY)

    # Publish the thresholded image
    if cam_type == 1:
        thresholded = cv2.bitwise_not(thresholded)
    thresholded_msg = bridge.cv2_to_imgmsg(thresholded, encoding="mono8")
    thresholded_pub.publish(thresholded_msg)

    # Find contours in the thresholded image
    _, contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            if w > depth_image.shape[1]*0.0625 and h > depth_image.shape[0]*0.083:#change this to make glass detection wide view
                #cv2.rectangle(depth_color, (x, y), (x + w, y + h), (0, 0, 255), 1)
                #cv2.putText(depth_color, "Window", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                sum_box.append([x,y,w,h])
                ##print("Window found", w, h)
                #break
    cascade = 1
    win_tol = depth_image.shape[1]*0.0625/2 #big_window tolerance 20 in 640x480
    max_windows = []
    origin_box = sum_box.copy()
    while cascade > 0 and len(sum_box) > 1:
        temp = []
        temp_r = []
        cascade = 0
        ##print("inloop")
        for i in range(0,len(sum_box)):
            ##print("i_scroll", i, len(sum_box))
            for j in range(0,len(sum_box)):
                ##print("j_scroll", j, len(sum_box))
                if (sum_box[i] != sum_box[j]):
                    i_x = sum_box[i][0]
                    i_y = sum_box[i][1]
                    i_w = sum_box[i][2]
                    i_h = sum_box[i][3]
                    ii_x = sum_box[j][0]
                    ii_y = sum_box[j][1]
                    ii_w = sum_box[j][2]
                    ii_h = sum_box[j][3]
                    """if (i_x > ii_x and i_x < ii_x+ii_w) and (i_y > ii_y and i_y < ii_y+ii_h): #i is in j
                        print("i in j")
                        t_x = ii_x
                        t_y = ii_y
                        t_w = i_w + ii_w - abs(ii_x+ii_w - i_x)
                        t_h = i_h + ii_h - abs(ii_y+ii_h - i_y)
                        if [t_x,t_y,t_w,t_h] not in temp:
                            cv2.rectangle(depth_color, (t_x, t_y), (t_x + t_w, t_y + t_h), (0, 255, 0), 1)
                            cv2.putText(depth_color, "Big Window", (t_x, t_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            print("Big Window found", t_w, t_h)
                            temp.append([t_x,t_y,t_w,t_h])
                            if sum_box[i] not in temp_r:
                                temp_r.append(sum_box[i])
                            if sum_box[j] not in temp_r:
                                temp_r.append(sum_box[j])
                            cascade = 1
                    elif (ii_x > i_x and ii_x < i_x+i_w) and (ii_y > i_y and ii_y < i_y+i_h): #j is in i
                        print("j in i")
                        t_x = i_x
                        t_y = i_y
                        t_w = ii_w + i_w - abs(i_x+i_w - ii_x)
                        t_h = ii_h + i_h - abs(i_y+i_h - ii_y)
                        if [t_x,t_y,t_w,t_h] not in temp:
                            cv2.rectangle(depth_color, (t_x, t_y), (t_x + t_w, t_y + t_h), (0, 255, 0), 1)
                            cv2.putText(depth_color, "Big Window", (t_x, t_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            print("Big Window found", t_w, t_h)
                            temp.append([t_x,t_y,t_w,t_h])
                            if sum_box[i] not in temp_r:
                                temp_r.append(sum_box[i])
                            if sum_box[j] not in temp_r:
                                temp_r.append(sum_box[j])
                            cascade = 1"""
                    if (i_x > ii_x - win_tol and i_x <= ii_x+ii_w + win_tol) and (i_y > ii_y - win_tol and i_y <= ii_y+ii_h + win_tol): #i is prox of j left
                        ##print("tol_find")
                        ##print("fusion of: ", sum_box[i], " + ", sum_box[j])
                        if ii_x > i_x:
                            t_x = i_x
                        else:
                            t_x = ii_x

                        if ii_y > i_y:
                            t_y = i_y
                        else:
                            t_y = ii_y

                        if ii_x+ii_w > i_x+i_w:
                            t_w = abs(t_x - (ii_x+ii_w))
                        else:
                            t_w = abs(t_x - (i_x+i_w))

                        if ii_y+ii_h > i_y+i_h:
                            t_h = abs(t_y - (ii_y+ii_h))
                        else:
                            t_h = abs(t_y - (i_y+i_h))

                        ##print("BWdim: ", t_x, t_y, t_w, t_h)
                        if [t_x,t_y,t_w,t_h] not in temp:
                            #cv2.rectangle(depth_color, (t_x, t_y), (t_x + t_w, t_y + t_h), (0, 255, 0), 1)
                            #cv2.putText(depth_color, "Big Window", (t_x, t_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            ##print("Big Window found", t_w, t_h)
                            temp.append([t_x,t_y,t_w,t_h])
                            if sum_box[i] not in temp_r:
                                temp_r.append(sum_box[i])
                            if sum_box[j] not in temp_r:
                                temp_r.append(sum_box[j])
                            cascade = 1
                    """elif (i_x >= ii_x and i_x < ii_x+ii_w + win_tol) and (i_y >= ii_y and i_y < ii_y+ii_h + win_tol): #i is prox of j right
                        print("tol_find")
                        if ii_x > i_x:
                            t_x = i_x
                        else:
                            t_x = ii_x

                        if ii_y > i_y:
                            t_y = i_y
                        else:
                            t_y = ii_y

                        if ii_x+ii_w > i_x+i_w:
                            t_w = abs(t_x - ii_x+ii_w)
                        else:
                            t_w = abs(t_x - i_x+i_w)

                        if ii_y+ii_h > i_y+i_h:
                            t_h = abs(t_y - ii_y+ii_h)
                        else:
                            t_h = abs(t_y - i_y+i_h)

                        if [t_x,t_y,t_w,t_h] not in temp:
                            cv2.rectangle(depth_color, (t_x, t_y), (t_x + t_w, t_y + t_h), (0, 255, 0), 1)
                            cv2.putText(depth_color, "Big Window", (t_x, t_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            print("Big Window found", t_w, t_h)
                            temp.append([t_x,t_y,t_w,t_h])
                            if sum_box[i] not in temp_r:
                                temp_r.append(sum_box[i])
                            if sum_box[j] not in temp_r:
                                temp_r.append(sum_box[j])
                            cascade = 1"""



        if len(temp) > 0:
            ##print("tempset")
            ##print("OG: ", sum_box)
            ##print("ADD: ", temp)
            ##print("REM: ", temp_r)
            if sorted(temp) == sorted(temp_r):
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
        
        if cascade == 0 or len(sum_box) <= 1:
            #max_windows = sum_box.copy()
            #for box in origin_box:
            #    if box in max_windows:
            #        max_windows.remove(box)
            #print()
            box_out = Float32MultiArray()
            box_list = []
            for box in sum_box:
                cv2.rectangle(depth_color, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 1)
                cv2.putText(depth_color, "Window", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1) #MAX  
                print(box) 
                box_list.append(box[0])
                box_list.append(box[1])
                box_list.append(box[2])
                box_list.append(box[3])
                
            box_out.data = box_list
            window_pub.publish(box_out)         
    else:
        if len(sum_box) == 1:
            box_out = Float32MultiArray()
            box_list = []
            for box in sum_box:
                cv2.rectangle(depth_color, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 1)
                cv2.putText(depth_color, "Window", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)#MAX
                print(box)
                box_list.append(box[0])
                box_list.append(box[1])
                box_list.append(box[2])
                box_list.append(box[3])
                
            box_out.data = box_list
            window_pub.publish(box_out)  
        else:
            ##print("No Window")
            x = 0

    # Publish the depth_gray image

    depth_gray_msg = bridge.cv2_to_imgmsg(depth_gray, encoding="mono8")
    depth_gray_pub.publish(depth_gray_msg)

    # Publish the depth_color image
    depth_color_msg = bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
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
        rospy.Subscriber('/dragonfly26/tof/voxl_depth_image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
