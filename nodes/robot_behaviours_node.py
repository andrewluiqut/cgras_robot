#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# general modules
import os, json, random, signal, copy, glob, collections, sys, time, yaml, argparse, logging
import rospy, actionlib
# project modules
from cgras.robot_control.robot_behaviours import RobotBehaviorsManager, RobotStates

# -- callback function for shutdown
def cb_shutdown():
    sys.exit(0)
def stop(*args, **kwargs):
    sys.exit(0)
    
# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_robot_behaviour_node', anonymous=False)
    signal.signal(signal.SIGINT, stop)
    rospy.on_shutdown(cb_shutdown)
    # log level
    # logger = logging.getLogger('rosout')
    # logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[logging.INFO])
    try:
        robot_behavior_manager = RobotBehaviorsManager()
        rospy.loginfo('robot behaviour node is running')
        robot_behavior_manager.spin(period_ms=500)
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)