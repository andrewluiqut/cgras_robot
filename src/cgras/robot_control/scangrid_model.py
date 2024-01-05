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

class ScanGrid():
    def __init__(self, name):
        station_config_file = rospy.get_param(f'/cgras/objects_config', 
            default=os.path.join(os.path.dirname(__file__), '../../../config/stations.yaml'))
        with open(station_config_file, 'r') as f:
            self.station_config = yaml.safe_load(f)
    
    
