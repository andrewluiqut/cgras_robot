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
import rospy, actionlib
import message_filters
from std_msgs.msg import String
# project modules
import cgras_robot.msg 
from cgras_robot.msg import CalibrateAction, MoveAction, ResetAction, CalibrateGoal, MoveGoal, ResetGoal
import robot_behaviours
# py_trees
import py_trees
import py_trees.console as console
from py_trees import logging as log_tree


class RobotControlAgent():

    def __init__(self):
        # - create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        rospy.on_shutdown(self.cb_shutdown)
        self.state_pub = rospy.Publisher('/cgras/robot/state', String, queue_size=1)
        # create demo action servers for every action
        # create action server: Calibrate.action
        self.action_server_calibrate = actionlib.SimpleActionServer('/cgras/robot/calibrate', 
                            cgras_robot.msg.CalibrateAction, execute_cb=self.calibrate_received_goal, auto_start=False)
        self.action_server_calibrate.start()
        # create action server: Move.action
        self.action_server_move= actionlib.SimpleActionServer('/cgras/robot/move', 
                            cgras_robot.msg.MoveAction, execute_cb=self.move_received_goal, auto_start=False)
        self.action_server_move.start()
        # create action server: Reset.action
        self.action_server_reset = actionlib.SimpleActionServer('/cgras/robot/reset', 
                            cgras_robot.msg.ResetAction, execute_cb=self.reset_received_goal, auto_start=False)
        self.action_server_reset.start()
        # starts the timer at the end
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_timer)

        log_tree.level = log_tree.Level.DEBUG
        # connect to blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        # starts the behavior tree
        bt = robot_behaviours.create_bt()
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
        bt.tick_tock(0.5)

    def epilog(self):
        if py_trees.console.has_colours:
            return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
        else:
            return None

    def command_line_argument_parser(self):
        parser = argparse.ArgumentParser(description='robot server agent',
                                        epilog=self.epilog(),
                                        formatter_class=argparse.RawDescriptionHelpFormatter,
                                        )
        group = parser.add_mutually_exclusive_group()
        group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
        group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
        return parser

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

    # -- callback for receiving the goal of action: Calibrate
    def calibrate_received_goal(self, goal):
        rospy.loginfo(f'calibrate goal received')
        action_feedback = cgras_robot.msg.CalibrateFeedback()
        result = cgras_robot.msg.CalibrateResult()
        # if somekindoferror:
        #     self.action_server_calibrate.set_aborted(result)
        result.location = '1'  # the apriltag ID
        result.error = ''
        self.action_server_calibrate.set_succeeded(result)
        

    # -- callback for receiving the goal of action: Move
    def move_received_goal(self, goal):
        rospy.loginfo(f'move goal received: grid cell ({goal.grid_x} {goal.grid_y}) of tile ({goal.tile_x} {goal.tile_y})')
        action_feedback = cgras_robot.msg.MoveFeedback()
        result = cgras_robot.msg.MoveResult()
        self.blackboard.set('goal', 'move')
        rospy.loginfo(f'set blackboard: {self.blackboard.get("goal")}')
        # if somekindoferror:
        #     self.action_server_calibrate.set_aborted(result)        
        result.error = ''
        self.action_server_move.set_succeeded(result)


    # -- callback for receiving the goal of action: Reset
    def reset_received_goal(self, goal):
        rospy.loginfo(f'reset goal received: {goal.pose}') # example poses include 'stow', 'recovery_1', etc
        action_feedback = cgras_robot.msg.ResetFeedback()
        result = cgras_robot.msg.ResetResult()
        self.blackboard.set('goal', 'reset')
        rospy.loginfo(f'set blackboard: {self.blackboard.get("goal")}')
        # if somekindoferror:
        #     self.action_server_calibrate.set_aborted(result)
        result.error = ''
        self.action_server_reset.set_succeeded(result)   

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('cgras_robot_demo_agent', anonymous=False)
    try:
        robot_agent = RobotControlAgent()
        rospy.loginfo('robot demo agent is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)