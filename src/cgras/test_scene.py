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
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        print('finished moving')

        current_pose = self.move_group.get_current_pose().pose
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory_publisher.publish(display_trajectory)

        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = "tool0"
        obj_pose.pose.orientation.w = 1.0
        obj_pose.pose.position.z = 0.6
 
    def print_info(self):
        print(f'Planning frame: {self.move_group.get_planning_frame()}')
        print(f'End effector link: {self.move_group.get_end_effector_link()}')
        print(f'Group names: {self.robot.get_group_names()}')
        print(f'Current state: {self.robot.get_current_state()}')
        # print(f'Known objects: {self.robot.get_known_object_names()}')
        # print(f'Attached objects: {self.robot.get_attached_objects()}')
        print(f'Links: {self.robot.get_link_names()}')



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    scene_builder = SceneBuilder()
    scene_builder.print_info()
    rospy.spin()