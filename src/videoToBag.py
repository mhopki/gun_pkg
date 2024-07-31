import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

TOPIC = 'camera/image_raw'

def CreateVideoBag(videopath, bagname):
    '''Creates a bag file with a video file'''
    true_bagname = videopath[:-4]+".bag"
    bag = rosbag.Bag(true_bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print("Warning: can't get FPS. Assuming 24.")
        prop_fps = 24
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.Time.from_sec(time.time())#rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "hal3_tof_camera" #"camera"
        bag.write(TOPIC, image, stamp)
        #rospy.sleep(0.015)
    cap.release()
    bag.close()


if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateVideoBag(*sys.argv[1:])
    else:
        print( "Usage: video2bag videofilename bagfilename", len(sys.argv), *sys.argv[1:])