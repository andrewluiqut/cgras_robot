# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import operator, yaml, os, math, random, copy
import rospy

class WorkAreaModel():
    def __init__(self):
        station_config_file = rospy.get_param(f'/cgras/workarea', 
            default=os.path.join(os.path.dirname(__file__), '../../../config/workarea.yaml'))
        with open(station_config_file, 'r') as f:
            self.station_config = yaml.safe_load(f)
        print(self.station_config)
        self.orientation = self.station_config['orientation'] # rpy

    def get_work_pose_as_list(self, tile_x, tile_y, cell_x, cell_y):
        x = (tile_x * self.station_config['num_stops'][0] + cell_x) * self.station_config['stop_step'][0] + self.station_config['origin_position'][0]
        y = (tile_y * self.station_config['num_stops'][1] + cell_y) * self.station_config['stop_step'][1] + self.station_config['origin_position'][1]
        z = self.station_config['work_level']
        return [x, y, z] + self.orientation

    def get_hover_pose_as_list(self, tile_x, tile_y, cell_x, cell_y):
        x = (tile_x * self.station_config['num_stops'][0] + cell_x) * self.station_config['stop_step'][0] + self.station_config['origin_position'][0]
        y = (tile_y * self.station_config['num_stops'][1] + cell_y) * self.station_config['stop_step'][1] + self.station_config['origin_position'][1]
        z = self.station_config['hover_level']
        return [x, y, z] + self.orientation
    
    def get_work_level(self):
        return self.station_config['work_level']
    
    def get_hover_level(self):
        return self.station_config['hover_level']

if __name__ == '__main__':
    scangrid_model = WorkAreaModel()
    pose_list = scangrid_model.get_work_pose_as_list(0, 0, 0, 2)
    print(pose_list)
