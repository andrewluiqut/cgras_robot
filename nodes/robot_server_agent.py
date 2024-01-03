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
# sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
#sys.path.append('/home/qcr/cgras_moveit_ws/devel/lib/python3/dist-packages')
import rospy, actionlib
import message_filters
from std_msgs.msg import String
# project modules
import cgras_robot.msg
from cgras_robot.msg import RobotAction, RobotGoal
import cgras.robot_control.robot_behaviours as robot_behaviours
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
        self.state_pub = rospy.Publisher('/cgras/robot/state', String, queue_size=1)
        # create demo action servers for every action
        # create action server: Calibrate.action
        self.action_server_robot = actionlib.SimpleActionServer('/cgras/robot/do', 
                            cgras_robot.msg.RobotAction, execute_cb=self.received_goal, auto_start=False)
        self.action_server_robot.register_preempt_callback(self.received_preemption)
        self.action_server_robot.start()

        # starts the timer at the end
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_timer)
        
        log_tree.level = log_tree.Level.DEBUG
        
        # connect to blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        # starts the behavior tree
        bt:BehaviourTree = robot_behaviours.create_bt()
        # bt.tick_once()
        # args = self.command_line_argument_parser().parse_args()
        # py_trees.display.render_dot_tree(bt)
        # if args.interactive:
        #     unused_result = py_trees.console.read_single_keypress()
        # while True:
        #     try:
        #         rospy.logerr('tick')
        #         bt.tick_once()
        #         py_trees.display.print_ascii_tree(bt, show_status=True)
        #         if args.interactive:
        #             rospy.logerr('Press Enter')
        #             unused_result = py_trees.console.read_single_keypress()
        #         else:
        #             time.sleep(0.5)
        #     except KeyboardInterrupt:
        #         break
        bt.tick_tock(period_ms=500)


    # -- callback function for shutdown
    def cb_shutdown(self):
        rospy.loginfo('the ros node is being shutdown')

    def stop(self, *args, **kwargs):
        print('the ros node is being shutdown')
        # time.sleep(2)
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
        if goal.target == RobotGoal.CALIBRATE:
            self.blackboard.set('goal', 'calibrate')
        elif goal.target == RobotGoal.MOVE_TO_STOW:
            self.blackboard.set('goal', 'reset')
            self.blackboard.set('target', 'stow')
        elif goal.target == RobotGoal.MOVE_TO_HOME:
            self.blackboard.set('goal', 'reset')
            self.blackboard.set('target', 'home')
        elif goal.target == RobotGoal.MOVE_TO_CELL:
            self.blackboard.set('goal', 'move')
        
        action_feedback = cgras_robot.msg.RobotFeedback()
        result = cgras_robot.msg.RobotResult()
        if self.action_server_robot.is_preempt_requested():
            print('preempt requested')
            self.action_server_robot.set_aborted(result)
        else:
            self.action_server_robot.set_succeeded(result)
    
    # -- callback for receiving preemption of the goal
    def received_preemption(self):
        rospy.loginfo(f'preemption received')
        result = cgras_robot.msg.RobotResult()


# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_robot_demo_agent', anonymous=False)
    try:
        robot_agent = RobotControlAgent()
        rospy.loginfo('robot demo agent is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)