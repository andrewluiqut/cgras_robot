from time import sleep
import operator
import rospy
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Parallel
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree

class ResetAction(Behaviour):
  def __init__(self, name):
    super(ResetAction, self).__init__(name)

  def setup(self):
    rospy.loginfo(f'ResetAction::setup {self.name}')

  def initialise(self):
    rospy.loginfo(f'ResetAction::initialise {self.name}')

  def update(self):
    rospy.loginfo(f'ResetAction::update {self.name}')
    sleep(1)
    return Status.SUCCESS
  
  def terminate(self, new_status):
    rospy.loginfo(f'ResetAction::terminate {self.name} to {new_status}')

class MoveAction(Behaviour):
  def __init__(self, name):
    super(MoveAction, self).__init__(name)

  def setup(self):
    rospy.loginfo(f'MoveAction::setup {self.name}')

  def initialise(self):
    rospy.loginfo(f'MoveAction::initialise {self.name}')

  def update(self):
    rospy.loginfo(f'MoveAction::update {self.name}')
    sleep(10)
    return Status.SUCCESS
  
  def terminate(self, new_status):
    rospy.loginfo(f'MoveAction::terminate {self.name} to {new_status}')

def create_bt():
    root_selector = py_trees.composites.Selector('root', memory=False)
    move_branch = py_trees.composites.Sequence('move_branch', memory=False)
    move_timeout_checker = py_trees.decorators.Timeout(
        duration=2,
        name='move_timeout_checker',
        child=py_trees.behaviours.CheckBlackboardVariableValue(name='check_move_goal', 
          check=py_trees.common.ComparisonExpression(
              variable="event_cancel_button",
              value=True,
              operator=operator.eq)
          )
    )
    move_branch.add_children([
        move_timeout_checker,
        MoveAction('do_move')
    ])
    root_selector.add_children([move_branch])
    root_tree = BehaviourTree(root_selector)
    return root_tree    
