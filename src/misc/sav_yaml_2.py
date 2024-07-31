#!/usr/bin/env python

import rospy
import roslib
import yaml
from nav_msgs.msg import OccupancyGrid

def save_map_yaml(map_data, file_path):
    """
    Saves the map data as a YAML file
    """
    map_yaml = {}
    map_yaml['image'] = file_path
    map_yaml['resolution'] = map_data.info.resolution
    map_yaml['image_height'] = map_data.info.height
    map_yaml['image_width'] = map_data.info.width
    map_yaml['origin'] = [map_data.info.origin.position.x,
                          map_data.info.origin.position.y,
                          map_data.info.origin.position.z]
    map_yaml['negate'] = 0
    map_yaml['occupied_thresh'] = 0.65
    map_yaml['free_thresh'] = 0.196

    with open(file_path + '.yaml', 'w') as yaml_file:
        yaml.dump(map_yaml, yaml_file, default_flow_style=False)

if __name__ == '__main__':
    rospy.init_node('map_saver')
    map_topic = '/projected_map'
    map_data = rospy.wait_for_message('/map', OccupancyGrid)
    file_path = '/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_' + map_topic[1:]
    save_map_yaml(map_data, file_path)
