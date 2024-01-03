

from time import sleep
import operator
import rospy
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree


class Do_Calibrate(Behaviour):
  def __init__(self, name):
    super(Do_Calibrate, self).__init__(name)

  def setup(self):
    self.logger.debug(f"Do_Calibrate::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Do_Calibrate::initialise {self.name}")

  def update(self):
    self.logger.debug(f"Do_Calibrate::update {self.name}")
    rospy.logdebug('started calibrate')
    sleep(5)
    rospy.logdebug('finished reset')
    return Status.SUCCESS
  def terminate(self, new_status):
    self.logger.debug(f"Do_Calibrate::terminate {self.name} to {new_status}")
    
class Do_Reset(Behaviour):
  def __init__(self, name):
    super(Do_Reset, self).__init__(name)

  def setup(self):
    self.logger.debug(f"Do_Reset::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Do_Reset::initialise {self.name}")

  def update(self):
    self.logger.debug(f"Do_Reset::update {self.name}")
    rospy.logdebug('started reset')
    sleep(3)
    rospy.logdebug('finished reset')    
    return Status.SUCCESS
  def terminate(self, new_status):
    self.logger.debug(f"Do_Reset::terminate {self.name} to {new_status}")   

class Do_Move(Behaviour):
  def __init__(self, name):
    super(Do_Move, self).__init__(name)

  def setup(self):
    self.logger.debug(f"Do_Move::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Do_Move::initialise {self.name}")

  def update(self):
    self.logger.debug(f"Do_Move::update {self.name}")
    rospy.logdebug('started move')
    sleep(3)
    rospy.logdebug('finished move')    
    return Status.SUCCESS
  def terminate(self, new_status):
    self.logger.debug(f"Do_Move::terminate {self.name} to {new_status}")  
# --- create the behaviour tree and its branches

def create_bt_do_work() -> Composite:
    check_calibrate_node = py_trees.behaviours.CheckBlackboardVariableValue(name='check_calibrate_goal', 
        check=py_trees.common.ComparisonExpression(
            variable='goal',
            value='calibrate',
            operator=operator.eq)
    )
    calibrate_branch = py_trees.decorators.Timeout(
        duration=2,
        name='calibrate_branch_timeout',
        child=py_trees.composites.Sequence('move_branch', memory=False, children=[
            check_calibrate_node,
            Do_Calibrate('do_calibrate'),
        ])
    )
    check_reset_node = py_trees.behaviours.CheckBlackboardVariableValue(name='check_reset_goal', 
        check=py_trees.common.ComparisonExpression(
            variable='goal',
            value='reset',
            operator=operator.eq)
    )
    reset_branch = py_trees.decorators.Timeout(
        duration=60,
        name='calibrate_branch_timeout',
        child=py_trees.composites.Sequence('reset_branch', memory=False, children=[
            check_reset_node,
            Do_Reset('do_reset'),
        ])
    )
    check_move_node = py_trees.behaviours.CheckBlackboardVariableValue(name='check_move_goal', 
        check=py_trees.common.ComparisonExpression(
            variable='goal',
            value='move',
            operator=operator.eq)
    )
    move_branch = py_trees.decorators.Timeout(
        duration=60,
        name='calibrate_branch_timeout',
        child=py_trees.composites.Sequence('move_branch', memory=False, children=[
            check_move_node,
            Do_Move('do_move'),
        ])
    )
    do_work_branch = py_trees.composites.Selector('do_work_selector_branch', memory=False, children=[
        calibrate_branch,
        reset_branch,
        move_branch,
    ])
    return do_work_branch

def create_bt() -> BehaviourTree:
    do_work_branch = create_bt_do_work()
    root_selector = py_trees.composites.Selector('do_work_selector_branch', memory=False, children=[
        do_work_branch,
    ])
    root_tree = BehaviourTree(root_selector)
    return root_tree

