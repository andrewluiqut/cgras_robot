#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class SceneBuilder(object):
    def __init__(self):
        super(SceneBuilder, self).__init__()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def print_info(self):
        print(f'Planning frame: {self.move_group.get_planning_frame()}')
        print(f'End effector link: {self.move_group.get_end_effector_link()}')
        print(f'Group names: {self.robot.get_group_names()}')
        # print(f'Known objects: {self.robot.get_known_object_names()}')
        # print(f'Attached objects: {self.robot.get_attached_objects()}')
        print(f'Links: {self.robot.get_link_names()}')

    def print_current_state(self):
        print(f'Current state: {self.robot.get_current_state()}')

    def print_pose_of_links(self):
        for link in self.robot.get_link_names():
            print(f'Pose of {link}: {self.move_group.get_current_pose(link)}')

    def move_link_pose(self, link):
        print(f'Received pose move goal: {self.move_group.get_current_pose(link)}')
        obj_pose = self.move_group.get_current_pose(link)
        obj_pose.header.frame_id = 'world'
        obj_pose.pose.position.x += 0.05
        self.move_group.set_pose_target(obj_pose)
        success = self.move_group.go(wait=True)
        if success:
            print('Successful Pose Move Goal')
        else:
            print('Failed')
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose(link).pose
        print(f'COMPLETED: {current_pose}')


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    sb = SceneBuilder()
    # sb.print_info()
    # sb.print_pose_of_links()
    sb.move_link_pose(link='enclosure')