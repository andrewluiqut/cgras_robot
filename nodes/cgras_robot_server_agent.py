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
import os, json, random, copy, glob, collections, sys, time, yaml
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import rospy, actionlib
import message_filters
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool, Int8, Int32
# project modules
from cgras_robot.msg import CalibrateAction, MoveAction, ResetAction
from cgras_robot.msg import RobotCommandAction, RobotCommandGoal

class RobotDemoAgent():

    def __init__(self):
        # --- Actions --- #
        # create demo action servers for every action
        # create action server: Calibrate.action
        self.action_server_calibrate = actionlib.SimpleActionServer('/cgras/robot/calibrate',
                            CalibrateAction, execute_cb=self.calibrate_received_goal, auto_start=False)
        self.action_server_calibrate.start()
        # create action server: Move.action
        self.action_server_move= actionlib.SimpleActionServer('/cgras/robot/move', 
                            MoveAction, execute_cb=self.move_received_goal, auto_start=False)
        self.action_server_move.start()
        # create action server: Reset.action
        self.action_server_reset = actionlib.SimpleActionServer('/cgras/robot/reset', 
                            ResetAction, execute_cb=self.reset_received_goal, auto_start=False)
        self.action_server_reset.start()
        # starts the timer at the end
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_timer)

    # -- callback function for shutdown
    def cb_shutdown(self):
        rospy.loginfo('the ros node is being shutdown')
        # sys.exit(0)

    # -- callback for waking up an IDLE robot agent
    def cb_timer(self, event):
        pass

    def cb_calibrate_done(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            print(f'success: the location apriltag found {result.data}')

        elif status == GoalStatus.ABORTED:
            print(f'error: the error is {result.data}')   

    def cb_move_done(status, result):   
        if status == GoalStatus.SUCCEEDED:
            print(f'success: moved to the target')
        elif status == GoalStatus.ABORTED:
            print(f'error: the error is {result.data}')   

    def cb_reset_done(status, result): 
        if status == GoalStatus.SUCCEEDED:
            print(f'success: moved to the desired pose')
        elif status == GoalStatus.ABORTED:
            print(f'error: the error is {result.data}')   

    # -- callback for receiving the goal of action: Calibrate
    def calibrate_received_goal(self, goal):
        self.action_client = actionlib.SimpleActionClient('/cgras/robot/do', RobotCommandAction)
        self.action_client.wait_for_server()
        goal = RobotCommandGoal(target=[RobotCommandGoal.ACTION_CALIBRATE])
        self.action_client.send_goal(goal, done_cb=self.cb_calibrate_done)
        finish_before_timeout = self.action_client.wait_for_result(rospy.Duration(20.0))
        if finish_before_timeout:
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                res = self.action_client.get_result()
                result = cgras_robot.msg.CalibrateResult(res.result)
                self.action_server_calibrate.set_succeeded(result)
            else:
                self.action_server_calibrate.set_aborted()
        else:
            self.action_server_calibrate.set_aborted()            
        
    # -- callback for receiving the goal of action: Move
    def move_received_goal(self, goal):
        self.action_client = actionlib.SimpleActionClient('/cgras/robot/do', RobotCommandAction)
        self.action_client.wait_for_server()
        goal = RobotCommandGoal(target=[RobotCommandGoal.ACTION_MOVE_CELL, goal.tile_x, goal.tile_y, goal.grid_x, goal.grid_y])
        self.action_client.send_goal(goal, done_cb=self.cb_calibrate_done)
        finish_before_timeout = self.action_client.wait_for_result(rospy.Duration(20.0))
        if finish_before_timeout:
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                res = self.action_client.get_result()
                self.action_server_calibrate.set_succeeded(result)
            else:
                self.action_server_calibrate.set_aborted()
        else:
            self.action_server_calibrate.set_aborted()       

    # -- callback for receiving the goal of action: Reset
    def reset_received_goal(self, goal):
        self.action_client = actionlib.SimpleActionClient('/cgras/robot/do', RobotCommandAction)
        self.action_client.wait_for_server()
        if goal.pose == 'stow':
            goal = RobotCommandGoal(target=[RobotCommandGoal.ACTION_MOVE_NAME, RobotCommandGoal.POSENAME_STOW])
        else:
            goal = RobotCommandGoal(target=[RobotCommandGoal.ACTION_MOVE_NAME, RobotCommandGoal.POSENAME_HOME])            
        self.action_client.send_goal(goal, done_cb=self.cb_calibrate_done)
        finish_before_timeout = self.action_client.wait_for_result(rospy.Duration(20.0))
        if finish_before_timeout:
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                self.action_server_calibrate.set_succeeded()
            else:
                self.action_server_calibrate.set_aborted()
        else:
            self.action_server_calibrate.set_aborted()   

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_robot_server_agent', anonymous=False)
    try:
        rospy.loginfo('robot demo agent is running')
        robot_agent = RobotDemoAgent()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)