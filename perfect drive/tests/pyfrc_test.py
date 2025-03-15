"""
Basic test suite for robot code
"""

import pytest
from unittest.mock import MagicMock

def test_autonomous(robot):
    robot.autonomousInit()
    robot.autonomousPeriodic()
    assert True

def test_disabled(robot):
    robot.disabledInit()
    robot.disabledPeriodic()
    assert True

def test_operator_control(robot):
    robot.teleopInit()
    robot.teleopPeriodic()
    assert True

def test_practice(robot):
    # Run through all modes
    test_disabled(robot)
    test_autonomous(robot)
    test_operator_control(robot)
    assert True
