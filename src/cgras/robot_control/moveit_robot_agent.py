#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, REF RAS, Research Infrastructure
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import sys, copy, threading, time, signal, math
from enum import Enum
import rospy
import moveit_commander
import moveit_commander.conversions as conversions
from moveit_msgs.msg import MoveGroupActionFeedback, PlanningScene, ObjectColor 
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

class RobotAgentStates(Enum):
    READY = 0
    BUSY = 1
    SUCCEEDED = 2
    ABORTED = 3
class MoveitActionStates(Enum):
    IDLE = 0
    PLANNING = 1
    MONITOR = 2
    ERROR = 3
    COMPLETED = 4

class MoveitRobotAgent():
    def __init__(self) -> None:
        # create lock for synchronization
        self.action_lock = threading.Lock()
        # -- constants (should import from config)
        self.GROUP_NAME = 'manipulator'
        self.WORLD_REFERENCE_LINK = 'trolley'        
        # -- initialize move_it variables 
        self.robot = moveit_commander.RobotCommander()
        self.scene:moveit_commander.PlanningSceneInterface = moveit_commander.PlanningSceneInterface()
        self.move_group:moveit_commander.MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GROUP_NAME)
        self.end_effector_link = self.move_group.get_end_effector_link()

        # -- subscribe to /move_group/feedback
        self.moveit_feedback_sub = rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, 
                                                    self.cb_move_group_feedback, queue_size=1)
        self.agent_state:RobotAgentStates = RobotAgentStates.READY
        self.action_state:MoveitActionStates = MoveitActionStates.IDLE
        self.cached_feedback:MoveGroupActionFeedback = None
        # -- planning scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        # -- constraints
        self.reset_path_constraints()
    
    def info(self, print=False):
        string_list = [
        f'group name: {self.GROUP_NAME}',
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
    
    def get_agent_state(self, print=False) -> RobotAgentStates:
        if print:
            rospy.loginfo(self.agent_state.name)
        return self.agent_state      
    
    def cb_move_group_feedback(self, msg:MoveGroupActionFeedback):
        rospy.logdebug(f'feedback: {msg}')
        self.cached_feedback = msg
        if self.agent_state == RobotAgentStates.BUSY:
            if msg.status.status in [GoalStatus.SUCCEEDED]:
                rospy.loginfo(f'Goal Ended: {msg.status.text}')
                self.agent_state = RobotAgentStates.SUCCEEDED
            elif msg.status.status in [GoalStatus.ABORTED]:
                rospy.loginfo(f'Goal Ended: {msg.status.text}')
                self.agent_state = RobotAgentStates.ABORTED
            if msg.feedback.state == 'PLANNING':
                self.action_state = MoveitActionStates.PLANNING
            elif msg.feedback.state == 'MONITOR':
                self.action_state = MoveitActionStates.MONITOR
            elif msg.feedback.state == 'IDLE':
                if msg.status.text == 'TIMED_OUT':
                    self.action_state = MoveitActionStates.ERROR
                else:
                    self.action_state = MoveitActionStates.COMPLETED 

    # ----------------- Functions: query current status
    def current_joint_positions(self, print=False) -> dict:
        joint_values_dict = self.robot.get_current_variable_values()
        if print: rospy.loginfo(joint_values_dict)
        return joint_values_dict
    
    def current_joint_values(self, print=False) -> list:
        joint_values = self.move_group.get_current_joint_values()
        if print: rospy.loginfo(joint_values)
        return joint_values        
    
    def current_link_pose(self, link_name=None, print=False) -> PoseStamped:
        if link_name is None:
            link_name = self.end_effector_link
        current_pose = self.move_group.get_current_pose(link_name)
        if print: rospy.loginfo(current_pose)
        return current_pose
    
    def current_rpy(self, link_name=None, print=False):
        if link_name is None:
            link_name = self.end_effector_link
        current_rpy = self.move_group.get_current_rpy(link_name)
        if print: rospy.loginfo(current_rpy)
        return current_rpy
    
    def same_pose_with_tolerence(self, goal, actual, tolerance) -> bool:
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        Applicable for joint space.
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is PoseStamped:
            return self.same_pose_with_tolerence(goal.pose, actual.pose, tolerance)
        elif type(goal) is Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = conversions.pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = conversions.pose_to_list(goal)
            d = math.dist((x1, y1, z1), (x0, y0, z0)) # euclidean distance
            cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1) # angle between orientations
            return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0) 
        return True
    
    # ----------------- Functions: end-effector manipulation
    def reset(self) -> None:
        # -- set state
        self.agent_state = RobotAgentStates.READY
        self.action_state = MoveitActionStates.IDLE
        self.cached_feedback = None 
        
    def abort_move(self, wait=True) -> bool:
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if wait:
            self.wait_while_busy()
            self.reset()
        
    def wait_while_busy(self):
        while True:
            if self.agent_state not in [RobotAgentStates.BUSY]:
                return self.agent_state
            rospy.sleep(0.05)

    def set_max_cartesian_speed(self, max_speed=None):
        if max_speed is None:
            self.move_group.clear_max_cartesian_link_speed()
        else:
            self.move_group.limit_max_cartesian_link_speed(max_speed, self.end_effector_link)
        
    def move_to_named_pose(self, named_pose, wait=True):
        self.action_lock.acquire()
        try:
            if self.agent_state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_to_named_pose: not READY state')
                return
            self.move_group.set_named_target(named_pose)
            self.agent_state = RobotAgentStates.BUSY        
            success = self.move_group.go(wait=wait)
            if not wait:
                return
            rospy.loginfo(f'Success: {success}')           
            self.abort_move()
        finally:
            self.action_lock.release()

    def move_displacement(self, dx, dy, dz, wait=True):
        self.action_lock.acquire()
        try:
            if self.agent_state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_displacement: not READY state')
                return
            obj_pose = self.move_group.get_current_pose(self.end_effector_link)
            obj_pose.pose.position.x += dx
            obj_pose.pose.position.y += dy
            obj_pose.pose.position.z += dz
            obj_pose.pose.orientation.w = 0      # no rotation
            self.move_group.set_pose_target(obj_pose)
            self.agent_state = RobotAgentStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return
            rospy.loginfo(f'Success: {success}')
            self.abort_move()
        finally:
            self.action_lock.release()

    def move_to_position(self, x, y, z, wait=True):
        self.action_lock.acquire()
        try:
            if self.agent_state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_to_position: not READY state')
                return
            target_pose = self.move_group.get_current_pose(self.end_effector_link)
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            target_pose.pose.orientation.w = 0            
            self.move_group.set_pose_target(target_pose)
            self.agent_state = RobotAgentStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return
            rospy.loginfo(f'Success: {success}')
            self.abort_move()
        finally:
            self.action_lock.release()
            
    def rotate_to_orientation(self, roll, pitch, yaw, reference_link=None, wait=True):
        self.action_lock.acquire()
        try:
            if self.agent_state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent rotate_to_orientation: not READY state')
                return
            target_pose = self.move_group.get_current_pose(self.end_effector_link)
            xyz = [
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z]
            rpy = [roll, pitch, yaw]
            reference_link = reference_link if reference_link is not None else self.WORLD_REFERENCE_LINK
            target_pose = conversions.list_to_pose_stamped(xyz + rpy, reference_link)
            self.move_group.set_pose_target(target_pose)
            self.agent_state = RobotAgentStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return
            rospy.loginfo(f'Success: {success}')
            self.abort_move()
        finally:
            self.action_lock.release()
    
    def move_to_position_and_orientation(self, target_pose, reference_link=None, wait=True):
        # target_pose may be a list of 6 or 7 numbers (xyz+4q) or (xyz+rpy)
        self.action_lock.acquire()
        try:
            if self.agent_state != RobotAgentStates.READY:
                rospy.logerr('MoveitAgent move_to_position_and_orientation command: not READY state')
                return
            reference_link = reference_link if reference_link is not None else self.WORLD_REFERENCE_LINK
            if type(target_pose) is list: 
                target_pose = conversions.list_to_pose_stamped(target_pose, reference_link)
            self.move_group.set_pose_target(target_pose)
            self.agent_state = RobotAgentStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return
            rospy.loginfo(f'Success: {success}')
            self.abort_move()
        finally:
            self.action_lock.release()
    
    # --------------------- Functions: specify the workspace and objects
    def reset_world(self):
        self.scene.remove_attached_object(self.WORLD_REFERENCE_LINK)
        self.scene.remove_world_object()

    def attach_object(self, object_name, model_file, object_scale, reference_link, xyz, rpy):
        self.scene.remove_world_object(object_name)
        # xyz = [0.6, -0.85, 0]
        # rpy = [0, 0, math.pi/2]
        object_pose = conversions.list_to_pose_stamped(xyz + rpy, reference_link)
        self.scene.attach_mesh(
                reference_link, object_name, object_pose,
                model_file, object_scale
        )
    
    def set_workspace_walls(self, min_x, min_y, min_z, max_x, max_y, max_z):
        thickness = 0.005
        size_x = max_x - min_x
        size_y = max_y - min_y
        size_z = max_z - min_z
        cx = (max_x + min_x) / 2
        cy = (max_y + min_y) / 2
        cz = (max_z + min_z) / 2
        self._create_wall('ws_top', self.WORLD_REFERENCE_LINK, cx, cy, max_z, size_x, size_y, thickness)
        self._create_wall('ws_bottom', self.WORLD_REFERENCE_LINK, cx, cy, min_z, size_x, size_y, thickness)  
        self._create_wall('ws_side_1', self.WORLD_REFERENCE_LINK, cx, min_y, cz, size_x, thickness, size_z)
        self._create_wall('ws_side_2', self.WORLD_REFERENCE_LINK, cx, max_y, cz, size_x, thickness, size_z)  
        self._create_wall('ws_side_3', self.WORLD_REFERENCE_LINK, min_x, cy, cz, thickness, size_y, size_z)
        self._create_wall('ws_side_4', self.WORLD_REFERENCE_LINK, max_x, cy, cz, thickness, size_y, size_z) 
        rospy.sleep(0.2) # wait for the walls to be registered
        wall_names = ['ws_top', 'ws_bottom', 'ws_side_1', 'ws_side_2', 'ws_side_3', 'ws_side_4']
        # set colors of the wall
        p = PlanningScene()
        p.is_diff = True
        for wall in wall_names:
            color = self._set_object_color(wall, 0.6, 0, 1, 0.2)
            p.object_colors.append(color)
        self.scene_pub.publish(p)   

    def _create_wall(self, name, frame_id, x, y, z, size_x, size_y, size_z):
        self.scene.remove_world_object(name)
        box_pose = PoseStamped()
        box_pose.header.frame_id = frame_id
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        self.scene.add_box(name, box_pose, size=(size_x, size_y, size_z))

    def _set_object_color(self, name, r, g, b, a = 0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        return color
    # ------------- Functions: set path constraints
    
    # - using the current pose as the reference
    def add_path_orientation_constraint(self, name, x_axis, y_axis, z_axis):
        current_pose = self.move_group.get_current_pose(self.end_effector_link)
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = current_pose.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = current_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = x_axis
        orientation_constraint.absolute_y_axis_tolerance = y_axis
        orientation_constraint.absolute_z_axis_tolerance = z_axis
        orientation_constraint.weight = 1
        self.the_constraints.orientation_constraints.append(orientation_constraint)
        self.enable_path_constraints()
        return orientation_constraint

    # - using the current joint value as the reference
    def add_path_joint_constraint(self, joint_name, tol_above, tol_below, position=None):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.tolerance_above = tol_above
        constraint.tolerance_below = tol_below
        if position is None:
            the_joint:moveit_commander.robot.RobotCommander.Joint = self.robot.get_joint(joint_name)
            rospy.loginfo(the_joint)
            constraint.position = the_joint.value()
        else:
            constraint.position = position
        constraint.weight = 1
        self.the_constraints.joint_constraints.append(constraint)
        self.enable_path_constraints()
        return constraint

    def reset_path_constraints(self):
        self.disable_path_constraints()
        self.the_constraints = Constraints()
        self.the_constraints.name = 'the_robot_agent'
        
    def enable_path_constraints(self, the_constraints:Constraints=None):
        if the_constraints is not None:
            self.the_constraints = the_constraints
        self.move_group.set_path_constraints(self.the_constraints)
        
    def disable_path_constraints(self):
        self.move_group.set_path_constraints(None)
        
# -- callback function for shutdown
def cb_shutdown():
    sys.exit(0)

def stop(*args, **kwargs):
    sys.exit(0)

def attach_tank(robot_agent, attach_to_link='trolley'):
    object_name = 'the_tank'
    model_file = '/home/qcr/cgras_moveit_ws/src/cgras_scene/meshes/collision/SettlementSystemRight.stl'
    object_scale = [0.001, 0.001, 0.001]
    xyz = [0.6, -0.85, 0]
    rpy = [0, 0, math.pi/2]
    reference_link = attach_to_link
    robot_agent.attach_object(object_name, model_file, object_scale, reference_link, xyz, rpy)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("robot_control_agent", anonymous=True)
    signal.signal(signal.SIGINT, stop)
    # rospy.on_shutdown(cb_shutdown)
    try:
        robot_agent = MoveitRobotAgent()
        robot_agent.current_joint_positions(print=True)
        robot_agent.current_joint_values(print=True)
        robot_agent.info(print=True)
        robot_agent.set_max_cartesian_speed(0.1)
        robot_agent.reset_world()
        # robot_agent.set_workspace_walls(-1.2, -0.87, 0.8, 0.8, 0.5, 2.0)
        # attach_tank(robot_agent)
        robot_agent.reset_path_constraints()
        rospy.sleep(1.0)
        # robot_agent.add_path_joint_constraint('shoulder_pan_joint', 0.01, 0.01, -1.31)
        robot_agent.move_to_named_pose('home', wait=True)
        # rospy.sleep(1.0)
        # robot_agent.abort_move()
        # final_state = robot_agent.wait_while_busy()
        # feedback = robot_agent.get_latest_feedback()
        # rospy.loginfo(f'Latest feedback: {feedback.status}')
        # rospy.loginfo(f'Goal Final: {final_state}')
        robot_agent.reset()
        # robot_agent.move_displacement(0.3, 0, 0, wait=False)
        # robot_agent.rotate(math.pi - 0.8, 0, math.pi / 2, wait=False)
        # robot_agent.move_to_position(0.6, -0.55, 1.2, wait=True)
        #robot_agent.move_to_position_and_orientation([0.21, 0.5, 1.2, math.pi, 0, 0], reference_link='the_tank')
        final_state = robot_agent.wait_while_busy()
        rospy.loginfo(f'Goal Final: {final_state}')        
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
