#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, REF RAS, Research Infrastructure
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import sys, copy, threading, time
from enum import Enum
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionFeedback
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg

class RobotAgentStates(Enum):
    READY = 0
    BUSY = 1
    SUCCEEDED = 2
    ABORTED = 3

class MoveitRobotAgent():
    def __init__(self) -> None:
        # create lock for synchronization
        self.action_lock = threading.Lock()
        # -- initialize move_it variables 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.end_effector_link = self.move_group.get_end_effector_link()
        # -- subscribe to /move_group/feedback
        self.moveit_feedback_sub = rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, 
                                                    self.cb_move_group_feedback, queue_size=1)
        self.state:RobotAgentStates = RobotAgentStates.READY
        self.cached_feedback:MoveGroupActionFeedback = None 
    
    def info(self, print=False):
        string_list = [
        f'group name: {self.group_name}',
        f'pose reference frame: {self.move_group.get_pose_reference_frame()}',   
        f'end-effector:\nlinks: {self.end_effector_link}',
        f'pose: {self.move_group.get_current_pose(self.end_effector_link).pose}',
        f'roll, pitch, yaw: {self.move_group.get_current_rpy(self.end_effector_link)}',
        ]
        output = '\n'.join(string_list)
        if print:
            rospy.loginfo(output)
        return output
        
    def get_latest_feedback(self) -> MoveGroupActionFeedback:
        return self.cached_feedback
    
    def get_state(self, print=False) -> RobotAgentStates:
        if print:
            rospy.loginfo(self.state.name)
        return self.state   
    
    def reset(self) -> None:
         # -- set state
        self.state = RobotAgentStates.READY
        self.cached_feedback = None  
     
    def cb_move_group_feedback(self, msg:MoveGroupActionFeedback):
        # rospy.loginfo(f'feedback: {msg}')
        self.cached_feedback = msg
        if self.state == RobotAgentStates.BUSY:
            if msg.status.status in [GoalStatus.SUCCEEDED]:
                rospy.loginfo(f'Goal Ended: {msg.status.text}')
                self.state = RobotAgentStates.SUCCEEDED
            elif msg.status.status in [GoalStatus.ABORTED]:
                rospy.loginfo(f'Goal Ended: {msg.status.text}')
                self.state = RobotAgentStates.ABORTED                

    def abort_move(self, wait=True) -> bool:
        self.action_lock.acquire()
        try:
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            if wait:
                self.wait_while_busy()
        finally:
            self.action_lock.release()    

    def wait_while_busy(self):
        while True:
            if self.state not in [RobotAgentStates.BUSY]:
                return self.state
            rospy.sleep(0.05)

    def set_max_cartesian_speed(self, max_speed=None):
        if max_speed is None:
            self.move_group.clear_max_cartesian_link_speed()
        else:
            self.move_group.limit_max_cartesian_link_speed(max_speed, self.end_effector_link)
        
    def move_to_named_pose(self, named_pose, wait=True):
        self.action_lock.acquire()
        try:
            if self.state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_to_named_pose: not READY state')
                return
            self.move_group.set_named_target(named_pose)
            self.state = RobotAgentStates.BUSY            
            success = self.move_group.go(wait=wait)

            if not wait:
                return
        finally:
            self.action_lock.release()

    def move_displacement(self, x, y, z, wait=True):
        self.action_lock.acquire()
        try:
            if self.state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_displacement: not READY state')
                return
            obj_pose = self.move_group.get_current_pose(self.end_effector_link)
            obj_pose.pose.position.x += x
            obj_pose.pose.position.y += y
            obj_pose.pose.position.z += z
            self.move_group.set_pose_target(obj_pose)
            self.state = RobotAgentStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return
        finally:
            self.action_lock.release()
        

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("robot_control_agent", anonymous=True)
    try:
        robot_agent = MoveitRobotAgent()
        robot_agent.info()
        robot_agent.set_max_cartesian_speed(0.1)
        # robot_agent.move_to_named_pose('home', wait=False)
        # rospy.sleep(1.0)
        # robot_agent.abort_move()
        # final_state = robot_agent.wait_while_busy()
        # feedback = robot_agent.get_latest_feedback()
        # rospy.loginfo(f'Latest feedback: {feedback.status}')
        # rospy.loginfo(f'Goal Final: {final_state}')
        robot_agent.reset()
        robot_agent.move_displacement(1.0, 0, 0, wait=False)
        final_state = robot_agent.wait_while_busy()
        rospy.loginfo(f'Goal Final: {final_state}')        
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
