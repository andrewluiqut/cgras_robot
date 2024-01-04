from time import sleep
import operator
import rospy
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree

# robot control module
from cgras.robot_control.moveit_robot_agent import MoveitRobotAgent, RobotAgentStates, MoveitActionStates


class Do_Calibrate(Behaviour):
    def __init__(self, name):
        super(Do_Calibrate, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Do_Calibrate::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Do_Calibrate::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Do_Calibrate::update {self.name}")
        rospy.logdebug("started calibrate")
        sleep(5)
        rospy.logdebug("finished reset")
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Do_Calibrate::terminate {self.name} to {new_status}")


class DoMovePoseName(Behaviour):
    def __init__(self, name):
        super(DoMovePoseName, self).__init__(name)

    def setup(self):
        self.logger.debug(f"DoMovePoseName::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"DoMovePoseName::initialise {self.name}")

    def update(self):
        self.logger.debug(f"DoMovePoseName::update {self.name}")
        rospy.logdebug("started move to named pose")
        sleep(3)
        rospy.logdebug("finished move to named pose")
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"DoMovePoseName::terminate {self.name} to {new_status}")


class DoMoveCell(Behaviour):
    def __init__(self, name):
        super(DoMoveCell, self).__init__(name)

    def setup(self):
        self.logger.debug(f"DoMoveCell::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"DoMoveCell::initialise {self.name}")

    def update(self):
        self.logger.debug(f"DoMoveCell::update {self.name}")
        rospy.logdebug("started move")
        sleep(3)
        rospy.logdebug("finished move")
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"DoMoveCell::terminate {self.name} to {new_status}")


class RobotBehaviorsManager:
    def __init__(self):
        # - setup robot control
        self.robot_agent = MoveitRobotAgent()
        self.robot_agent.set_max_cartesian_speed(0.1)
        self.robot_agent.reset_world()
        self.robot_agent.set_workspace_walls(-1.2, -0.87, 0.8, 0.8, 0.5, 2.0)
        self.robot_agent.reset_path_constraints()
        # - setup the behavior tree
        self.bt = self.create_bt()
        # - setup the blackboard
        self.the_blackboard = py_trees.blackboard.Blackboard()

    def spin(self):
        self.bt.tick_tock(period_ms=500)

    def write_on_blackboard(self, variable_name, value):
        self.the_blackboard.set(variable_name, value)

    # --- create the behaviour tree and its branches
    def create_bt_do_work(self) -> Composite:
        check_calibrate_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_calibrate_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value="calibrate", operator=operator.eq
            ),
        )
        calibrate_branch = py_trees.decorators.Timeout(
            duration=2,
            name="calibrate_branch_timeout",
            child=py_trees.composites.Sequence(
                "move_branch",
                memory=False,
                children=[
                    check_calibrate_node,
                    Do_Calibrate("do_calibrate"),
                ],
            ),
        )
        check_move_posename_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_move_posename_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value="move_posename", operator=operator.eq
            ),
        )
        move_posename_branch = py_trees.decorators.Timeout(
            duration=60,
            name="move_posename_branch_timeout",
            child=py_trees.composites.Sequence(
                "move_posename_branch",
                memory=False,
                children=[
                    check_move_posename_node,
                    DoMovePoseName("do_reset"),
                ],
            ),
        )
        check_move_cell_node = py_trees.behaviours.CheckBlackboardVariableValue(
            name="check_move_cell_goal",
            check=py_trees.common.ComparisonExpression(
                variable="goal", value="move_cell", operator=operator.eq
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
