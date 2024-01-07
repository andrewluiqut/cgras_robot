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
import os, json, random, signal, copy, glob, collections, sys, time, yaml, argparse
# sys.path.append('/home/qcr/cgras_moveit_ws/devel/lib/python3/dist-packages')
import rospy, actionlib
import message_filters
from std_msgs.msg import String
# project modules
import cgras_robot.msg
from cgras_robot.msg import RobotCommandAction, RobotCommandGoal
from cgras.robot_control.robot_behaviours import RobotBehaviorsManager
# py_trees
import py_trees
import py_trees.console as console
from py_trees import logging as log_tree
from py_trees.trees import BehaviourTree

class RobotControlAgent():

    def __init__(self):
        # - create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)
        # set constant
        self.TIMEOUT = 30.0 # seconds
        # log level
        log_tree.level = log_tree.Level.INFO
        # create published for robot states
        self.state_pub = rospy.Publisher('/cgras/robot/state', String, queue_size=1)
        # create action server
        self.action_server_robot = actionlib.SimpleActionServer('/cgras/robot/do', 
                            cgras_robot.msg.RobotCommandAction, execute_cb=self.received_goal, auto_start=False)
        self.action_server_robot.register_preempt_callback(self.received_preemption)
        self.action_server_robot.start()
        # starts the timer at the end
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_timer)
        # connect to the blackboard
        self.blackboard = py_trees.blackboard.Client(name="RobotCommandGoal")
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_params", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_result", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="status", access=py_trees.common.Access.READ)
        # create the robot behaviour manager
        self.robot_manager = RobotBehaviorsManager()
        self.robot_manager.spin(period_ms=500)
        
    # -- callback function for shutdown
    def cb_shutdown(self):
        rospy.loginfo('the ros node is being shutdown')

    def stop(self, *args, **kwargs):
        rospy.loginfo('the ros node is being stopped')
        sys.exit(0)

    # publish the state of the robot agent
    def _publish_state(self):
        msg:String = String()
        msg.data = 'ALIVE'
        self.state_pub.publish(msg)

    # -- callback for waking up an IDLE robot agent
    def cb_timer(self, event):
        self._publish_state()

    # -- callback for receiving the goal of action
    def received_goal(self, goal):
        rospy.loginfo(f'goal received: {goal}')
        result = cgras_robot.msg.RobotCommandResult()
        # feedback = cgras_robot.msg.RobotCommandFeedback()
        action = goal.target[0]
        params = [] if len(goal.target) <= 1 else goal.target[1:]

        self.blackboard.goal = action
        self.blackboard.goal_params = params
        self.blackboard.goal_result = None
        
        t = rospy.Time.now()
        while True:
            rospy.sleep(0.1)
            # rospy.loginfo(f'robot state: {self.blackboard.goal_result}')
            # rospy.loginfo(f'Status: {self.blackboard.status}')
            if self.blackboard.goal_result is not None:
                break
            if rospy.Time.now() - t > rospy.Duration(self.TIMEOUT):
                rospy.logerr(f"Timeout ({self.TIMEOUT} s) reached ... abort goal")
                result.data = 'TIMEOUT'
                self.action_server_robot.set_aborted(result)
                return
        rospy.sleep(2.0)
        if self.action_server_robot.is_preempt_requested():
            result.data = 'ABORTED'
            self.action_server_robot.set_aborted(result)
        elif self.blackboard.status == py_trees.common.Status.FAILURE:
            result.data = self.blackboard.goal_result
            self.action_server_robot.set_aborted(result)            
        else:
            result.data = self.blackboard.goal_result
            self.action_server_robot.set_succeeded(result)
    
    # -- callback for receiving preemption of the goal
    def received_preemption(self):
        rospy.loginfo(f'preemption received')
        result = cgras_robot.msg.RobotCommandResult()


# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_robot_node', anonymous=False)
    try:
        robot_agent = RobotControlAgent()
        rospy.loginfo('robot server agent is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)