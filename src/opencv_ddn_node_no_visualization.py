#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

# Constants.
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45
CONFIDENCE_THRESHOLD = 0.45

# Text parameters.
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
THICKNESS = 1

# Colors
BLACK  = (0,0,0)
BLUE   = (255,178,50)
YELLOW = (0,255,255)
RED = (0,0,255)

# Global variables
classes = None
net = None

# Initialize the CvBridge
bridge = CvBridge()

def draw_label(input_image, label, left, top):
    """Draw text onto image at location."""
    # Get text size.
    text_size = cv2.getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS)
    dim, baseline = text_size[0], text_size[1]
    # Use text size to create a BLACK rectangle. 
    cv2.rectangle(input_image, (left, top), (left + dim[0], top + dim[1] + baseline), BLACK, cv2.FILLED)
    # Display text inside the rectangle.
    cv2.putText(input_image, label, (left, top + dim[1]), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS, cv2.LINE_AA)

def pre_process(input_image):
    # Create a 4D blob from a frame.
    blob = cv2.dnn.blobFromImage(input_image, 1/255, (INPUT_WIDTH, INPUT_HEIGHT), [0,0,0], 1, crop=False)
    # Sets the input to the network.
    net.setInput(blob)
    # Runs the forward pass to get output of the output layers.
    output_layers = net.getUnconnectedOutLayersNames()
    outputs = net.forward(output_layers)
    return outputs

def post_process(input_image, outputs):
    # Lists to hold respective values while unwrapping.
    class_ids = []
    confidences = []
    boxes = []
    
    # Create Detection2DArray for ROS.
    detection_array = Detection2DArray()

    # Rows.
    rows = outputs[0].shape[1]
    image_height, image_width = input_image.shape[:2]

    # Resizing factor.
    x_factor = image_width / INPUT_WIDTH
    y_factor =  image_height / INPUT_HEIGHT

    # Iterate through 25200 detections.
    for r in range(rows):
        row = outputs[0][0][r]
        confidence = row[4]

        # Discard bad detections and continue.
        if confidence >= CONFIDENCE_THRESHOLD:
            classes_scores = row[5:]

            # Get the index of max class score.
            class_id = np.argmax(classes_scores)

            # Continue if the class score is above threshold.
            if (classes_scores[class_id] > SCORE_THRESHOLD):
                confidences.append(confidence)
                class_ids.append(class_id)

                cx, cy, w, h = row[0], row[1], row[2], row[3]
                left = int((cx - w/2) * x_factor)
                top = int((cy - h/2) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)

                box = np.array([left, top, width, height])
                boxes.append(box)

                # Create a Detection2D object for each box
                detection = Detection2D()
                detection.bbox.center.x = left + width / 2
                detection.bbox.center.y = top + height / 2
                detection.bbox.size_x = width
                detection.bbox.size_y = height

                # Add class information to the detection
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = class_id
                hypothesis.score = float(confidence)
                detection.results.append(hypothesis)

                # Append detection to the array
                detection_array.detections.append(detection)

    # Perform non-maximum suppression to eliminate redundant overlapping boxes with lower confidences.
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    for i in indices:
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        #cv2.rectangle(input_image, (left, top), (left + width, top + height), BLUE, 3*THICKNESS)
        #label = "{}:{:.2f}".format(classes[class_ids[i]], confidences[i])
        #draw_label(input_image, label, left, top)

    # Set header for Detection2DArray
    detection_array.header.stamp = rospy.Time.now()

    return input_image, detection_array

def image_callback(ros_image):
    global net, classes

    # Convert the ROS image message to an OpenCV image
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Failed to convert image: %s" % e)
        return

    # Process image.
    detections = pre_process(frame)
    img, detection_array = post_process(frame.copy(), detections)

    # Publish the processed image
    output_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    #image_pub.publish(output_msg)

    # Publish the Detection2DArray
    detection_pub.publish(detection_array)

    # Optionally, print performance profile
    t, _ = net.getPerfProfile()
    rospy.loginfo(f"Detections processed in {t * 1000.0 / cv2.getTickFrequency()} ms")

if __name__ == '__main__':
    rospy.init_node('object_detection_node')

    # Load class names.
    classesFile = rospy.get_param('~classes_file', "coco.names")
    modelWeights = rospy.get_param('~model_weights', "best.onnx")

    # Read class names from file.
    with open(classesFile, 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')

    # Load the network
    net = cv2.dnn.readNet(modelWeights)

    # Subscribers and publishers
    image_sub = rospy.Subscriber('/depth_gray_image', Image, image_callback)
    image_pub = rospy.Publisher('/yolov7/yolov7/visualization', Image, queue_size=1)
    detection_pub = rospy.Publisher('/yolov7/yolov7', Detection2DArray, queue_size=1)

    # Spin to keep the node running
    rospy.spin()
