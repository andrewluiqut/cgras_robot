#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees.decorators
import py_trees.display
import py_trees.behaviours
from py_trees import logging as log_tree

if __name__ == '__main__':
    log_tree.level = log_tree.Level.DEBUG
    root = py_trees.composites.Sequence(name="Life", memory=True)
    timeout = py_trees.decorators.Timeout(
        duration=5.0,
        name="Timeout",
        child=py_trees.behaviours.Success(name="Have a Beer!")
    )
    failure_is_success = py_trees.decorators.Inverter(
        name="Inverter",
        child=py_trees.behaviours.Failure(name="Busy?")
        )
    root.add_children([failure_is_success, timeout])
    py_trees.display.render_dot_tree(root)
    
    root = py_trees.trees.BehaviourTree(root=root)
    root.tick_tock(period_ms=500)