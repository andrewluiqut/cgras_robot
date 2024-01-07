#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, REF RAS, Research Infrastructure
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from time import sleep
import operator, yaml, os, math, random, copy
from enum import Enum
import rospy
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree

# robot control module
from cgras.robot_control.moveit_robot_agent import MoveitRobotAgent, RobotAgentStates, MoveitActionStates
from cgras_robot.msg import RobotCommandAction, RobotCommandGoal
from cgras_robot.src.cgras.robot_control.workarea_model import WorkAreaModel

class RobotStates(Enum):
    INVALID = -1
    RESET = 0
    STANDBY = 1
    WORKING = 2
    FAULT = 3

class DoCalibrate(Behaviour):
    def __init__(self, name):
        super(DoCalibrate, self).__init__(name)
        # load config
        objects_config_file = rospy.get_param(f'/cgras/objects_config', 
            default=os.path.join(os.path.dirname(__file__), '../../../config/objects.yaml'))
        with open(objects_config_file, 'r') as f:
            self.objects_config = yaml.safe_load(f)
        # attach to blackboard
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key='goal_result', access=py_trees.common.Access.WRITE)

    def update(self):
        rospy.loginfo("Calibration started")
        for object in self.objects_config['attached_objects']:
            random_y = random.uniform(-0.1, 0.1)
            xyz = copy.deepcopy(object['pose']['xyz'])
            rpy = object['pose']['rpy']
            xyz[1] += random_y
            ROBOT_AGENT.attach_object(object['object_name'], object['model_file'], object['scale'], 
                                    object['attach_to'], xyz, rpy)
            id = object['id']
        self.blackboard.goal_result = id
        rospy.loginfo("Calibration finished")
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Do_Calibrate::terminate {self.name} to {new_status}")


class DoMovePose(Behaviour):
    def __init__(self, name, condition_fn=True, posename='home'):
        super(DoMovePose, self).__init__(name)
        self.condition_fn = condition_fn
        self.posename = posename
        
    def initialise(self):
        ROBOT_AGENT.abort_move()
        ROBOT_AGENT.reset()
        
    def update(self):
        if (type(self.condition_fn) == bool and not self.condition_fn) or \
            (hasattr(self.condition_fn, '__call__') and not self.condition_fn()):
                return Status.SUCCESS
        agent_state = ROBOT_AGENT.get_agent_state()
        rospy.loginfo(f'update: agent state: {agent_state.name} behaviour status: {self.status.name}')
        if agent_state in [RobotAgentStates.READY]:
            ROBOT_AGENT.move_to_named_pose(self.posename, wait=False)
            rospy.loginfo(f'started move to named pose: {self.posename}')   
            return Status.RUNNING   
        elif agent_state == RobotAgentStates.BUSY:
            return Status.RUNNING
        elif agent_state == RobotAgentStates.SUCCEEDED:
            self.blackboard.goal_result = 'SUCCESS'
            return Status.SUCCESS
        elif agent_state == RobotAgentStates.ABORTED:
            self.blackboard.goal_result = 'FAILED'
            return Status.FAILURE
        
    def terminate(self, new_status):
        rospy.loginfo(f"DoMovePose: terminate {self.name} to {new_status}")

class DoMoveHoverPose(Behaviour):
    def __init__(self, name, condition_fn=True, work_pose=[0, 0, 0, 0]):
        super(DoMoveHoverPose, self).__init__(name)
        self.condition_fn = condition_fn
        self.work_pose = work_pose

    def initialise(self):
        ROBOT_AGENT.abort_move()
        ROBOT_AGENT.reset()

    def update(self):
        if (type(self.condition_fn) == bool and not self.condition_fn) or \
            (hasattr(self.condition_fn, '__call__') and not self.condition_fn()):
                return Status.SUCCESS
        agent_state = ROBOT_AGENT.get_agent_state()
        rospy.loginfo(f'update: agent state: {agent_state.name} behaviour status: {self.status.name}')
        if agent_state in [RobotAgentStates.READY]:
            if self.work_pose is None or len(self.work_pose) != 4:
                self.blackboard.goal_result = 'INVALID POSE SPEC'
                return Status.FAILURE
            target_pose_as_list = self.scangrid_model.get_work_pose_as_list(self.work_pose)
            ROBOT_AGENT.move_to_position_and_orientation(target_pose_as_list, wait=False)
            rospy.loginfo(f'started move to pose: {target_pose_as_list}')   
            return Status.RUNNING   
        elif agent_state == RobotAgentStates.BUSY:
            return Status.RUNNING
        elif agent_state == RobotAgentStates.SUCCEEDED:
            self.blackboard.goal_result = 'SUCCESS'
            return Status.SUCCESS
        elif agent_state == RobotAgentStates.ABORTED:
            self.blackboard.goal_result = 'FAILED'
            return Status.FAILURE

    def terminate(self, new_status):
        rospy.loginfo(f"DoMoveCell: terminate {self.name} to {new_status}")

class Movement(Enum):
    UP = 0
    DOWN = 1

class DoMoveUpDown(Behaviour):
    def __init__(self, name, condition_fn=True, direction=Movement.UP):
        super(DoMoveUpDown, self).__init__(name)
        self.condition_fn = condition_fn
        self.direction = direction

    def initialise(self):
        ROBOT_AGENT.abort_move()
        ROBOT_AGENT.reset()

    def update(self):
        if (type(self.condition_fn) == bool and not self.condition_fn) or \
            (hasattr(self.condition_fn, '__call__') and not self.condition_fn()):
                return Status.SUCCESS
        agent_state = ROBOT_AGENT.get_agent_state()
        rospy.loginfo(f'update: agent state: {agent_state.name} behaviour status: {self.status.name}')
        if agent_state in [RobotAgentStates.READY]:
            if self.direction not in [Movement.UP, Movement.DOWN]:
                self.blackboard.goal_result = 'INVALID POSE SPEC'
                return Status.FAILURE
            target_pose_as_list = self.scangrid_model.get_work_pose_as_list(self.work_pose)
            ROBOT_AGENT.move_to_position_and_orientation(target_pose_as_list, wait=False)
            rospy.loginfo(f'started move to pose: {target_pose_as_list}')   
            return Status.RUNNING   
        elif agent_state == RobotAgentStates.BUSY:
            return Status.RUNNING
        elif agent_state == RobotAgentStates.SUCCEEDED:
            self.blackboard.goal_result = 'SUCCESS'
            return Status.SUCCESS
        elif agent_state == RobotAgentStates.ABORTED:
            self.blackboard.goal_result = 'FAILED'
            return Status.FAILURE

    def terminate(self, new_status):
        rospy.loginfo(f"DoMoveCell: terminate {self.name} to {new_status}")


# -- Global Variables
ROBOT_AGENT = MoveitRobotAgent()
WORKAREA_MODEL = WorkAreaModel()

# -- Definition of the main behavior tree
class RobotBehaviorsManager:
    def __init__(self):
        # - setup the robot agent
        self._setup_robot_agent()
        # - setup the behavior tree
        self.bt = self.create_bt()
        # - setup the blackboard
        self.the_blackboard = py_trees.blackboard.Blackboard()
        self.the_blackboard.set('state', RobotStates.STANDBY)
        self.the_blackboard.set('status', py_trees.common.Status.INVALID)

    def _setup_robot_agent(self):
        # - TODO: should initialize with config file
        ROBOT_AGENT.set_max_cartesian_speed(0.1)
        ROBOT_AGENT.reset_world()
        ROBOT_AGENT.set_workspace_walls(-1.2, -0.97, 0.8, 0.8, 0.6, 2.0)
        ROBOT_AGENT.reset_path_constraints()
        # - TODO: add constraints for the path planner

    def spin(self, period_ms=10):
        self.bt.tick_tock(period_ms=period_ms)
        
    def get_robot_state(self):
        return self.the_blackboard.get('state') 

    # --- create the behaviour tree and its branches
    def create_bt_do_work(self) -> Composite:
        check_calibrate_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_calibrate_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value=RobotCommandGoal.ACTION_CALIBRATE, operator=operator.eq
            ),
        )
        calibrate_branch = py_trees.decorators.Timeout(
            duration=2,
            name="calibrate_branch_timeout",
            child=py_trees.composites.Sequence(
                "calibrate_branch",
                memory=False,
                children=[
                    check_calibrate_node,
                    py_trees.behaviours.SetBlackboardVariable(
                        name='set_state', variable_name='state', variable_value=RobotStates.WORKING, overwrite=True),
                    py_trees.decorators.FailureIsSuccess(
                        name='calibrate_decorator', 
                        child=py_trees.decorators.StatusToBlackboard(
                            name='status_recorder', child=DoCalibrate("do_calibrate"),
                            variable_name='status')
                    ),
                    py_trees.behaviours.UnsetBlackboardVariable(
                        name='unset calibrate', key='goal'),
                    py_trees.behaviours.SetBlackboardVariable(
                        name='set_state', variable_name='state', variable_value=RobotStates.STANDBY, overwrite=True),                    
                ],
            ),
        )
        check_move_posename_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_move_posename_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value=RobotCommandGoal.ACTION_MOVE_NAME, operator=operator.eq
            ),
        )
        move_posename_branch = py_trees.decorators.Timeout(
            duration=60,
            name="move_posename_branch_timeout",
            child=py_trees.composites.Sequence(
                "move_posename_branch",
                memory=True,
                children=[
                    check_move_posename_node,
                    py_trees.behaviours.SetBlackboardVariable(
                        name='set_state', variable_name='state', variable_value=RobotStates.WORKING, overwrite=True),
                    py_trees.decorators.FailureIsSuccess(
                        name='move_posename_decorator', 
                        child=py_trees.decorators.StatusToBlackboard(
                            name='status_recorder', child=DoMovePoseName("do_move_pose"),
                            variable_name='status')
                    ),
                    py_trees.behaviours.UnsetBlackboardVariable(
                        name='unset move posename', key='goal'),
                    py_trees.behaviours.SetBlackboardVariable(
                        name='set_state', variable_name='state', variable_value=RobotStates.STANDBY, overwrite=True),                    
                ],
            ),
        )
        check_move_cell_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_move_cell_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value=RobotCommandGoal.ACTION_MOVE_CELL, operator=operator.eq
            ),
        )
        move_cell_branch = py_trees.decorators.Timeout(
            duration=60,
            name="move_cell_branch_timeout",
            child=py_trees.composites.Sequence(
                "move_cell_branch",
                memory=False,
                children=[
                    check_move_cell_node,
                    DoMoveCell("do_move"),
                ],
            ),
        )
        do_work_branch = py_trees.composites.Selector(
            "do_work_selector_branch",
            memory=False,
            children=[
                calibrate_branch,
                move_posename_branch,
                move_cell_branch,
            ],
        )
        return do_work_branch

    def create_bt(self) -> BehaviourTree:
        do_work_branch = self.create_bt_do_work()
        root_selector = py_trees.composites.Selector(
            "do_work_selector_branch",
            memory=False,
            children=[
                do_work_branch,
            ],
        )
        root_tree = BehaviourTree(root_selector)
        return root_tree


