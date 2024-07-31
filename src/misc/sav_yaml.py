#!/usr/bin/env python

import rospy
import roslib
import yaml
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from PIL import Image
from cv_bridge import CvBridge
import math


bridge = CvBridge()

def save_map_yaml(map_data, file_path):
    """
    Saves the map data as a YAML file
    """
    # Convert the map message to a dictionary
    """map_dict = {'info': {'map_load_time': str(rospy.Time.now()), 
                         'resolution': map_msg.info.resolution, 
                         'width': map_msg.info.width, 
                         'height': map_msg.info.height, 
                         'origin': {'position': {'x': map_msg.info.origin.position.x,
                                                 'y': map_msg.info.origin.position.y,
                                                 'z': map_msg.info.origin.position.z},
                                    'orientation': {'x': map_msg.info.origin.orientation.x,
                                                    'y': map_msg.info.origin.orientation.y,
                                                    'z': map_msg.info.origin.orientation.z,
                                                    'w': map_msg.info.origin.orientation.w}}},
                'data': map_msg.data}

    # Save the map dictionary to a YAML file
    with open('/path/to/map.yaml', 'w') as f:
        yaml.dump(map_dict, f)"""

    map_yaml = {}
    map_yaml['image'] = file_path
    map_yaml['resolution'] = map_data.info.resolution
    map_yaml['image_height'] = map_data.info.height
    map_yaml['image_width'] = map_data.info.width
    map_yaml['origin_p'] = [map_data.info.origin.position.x,
                          map_data.info.origin.position.y,
                          map_data.info.origin.position.z]
    map_yaml['origin_o'] = [map_data.info.origin.orientation.x,
                          map_data.info.origin.orientation.y,
                          map_data.info.origin.orientation.z,
                          map_data.info.origin.orientation.w]
    map_yaml['negate'] = 0
    map_yaml['occupied_thresh'] = 0.65
    map_yaml['free_thresh'] = 0.196

    print("adding file")
    with open(file_path + '.yaml', 'w') as yaml_file:
        yaml.dump(map_yaml, yaml_file, default_flow_style=False)

def save_map_to_png(map_topic):
    map_msg = rospy.wait_for_message(map_topic, OccupancyGrid)
    map_data = list(map_msg.data)
    map_width = map_msg.info.width
    map_height = map_msg.info.height

    if map_topic == '/projected_map':
        print("projected")
        desired_height = desired_width = 4000
        offx = 2000-(map_width//2)+5+math.floor(map_msg.info.origin.position.x)#-50+10
        offy = 2000-(map_height//2)-22+math.floor(map_msg.info.origin.position.y)#-50-10
        map_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        # Pad the grid data to the desired size using numpy
        padded_map_data = np.zeros((desired_height, desired_width), dtype=np.int8)
        padded_map_data[offy:map_data.shape[0]+offy, offx:map_data.shape[1]+offx] = 255-map_data
        # Convert the padded grid data to a cv2 image
        map_img = cv2.normalize(padded_map_data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        map_img = cv2.convertScaleAbs(map_img)
        map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)
        # Save the image as a png file
        cv2.imwrite(file_path + '.png', map_img)
        return

    map_img = [255 - int(round(x * 255. / 100.)) for x in map_data]
    print(np.array(map_img).shape)
    map_img = np.array(map_img).reshape((map_height, map_width))
    map_img = cv2.convertScaleAbs(map_img)
    map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)


    cv2.imwrite(file_path + '.png', map_img)

if __name__ == '__main__':
    rospy.init_node('map_saver')
    map_list = ['/map', '/projected_map']
    for map_topic in map_list:
        map_data = rospy.wait_for_message(map_topic, OccupancyGrid)
        file_path = '/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_' + map_topic[1:]
        print("saving: ", map_topic[1:])
        save_map_yaml(map_data, file_path)
        print("png")
        save_map_to_png(map_topic)
