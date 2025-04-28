#!/usr/bin/env python3

import rospy
import unittest
import actionlib
from assignments.msg import stepAction, stepFeedback, stepResult, stepGoal
from actionlib_msgs.msg import GoalStatus

class FakeStepActionServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/step_action', stepAction, execute_cb=self.execute_cb, auto_start=False)
        self.success_behavior = []  # List of True/False to simulate success/failure for each goal
        self.server.start()
        self.current_goal_idx = 0

    def execute_cb(self, goal):
        rospy.loginfo(f"Fake server received goal: {goal.action} {goal.ingredient}")
        rospy.sleep(0.5)  # Simulate time to process

        # Choose the result based on the test scenario
        success = self.success_behavior[self.current_goal_idx]
        result = stepResult(success=success)
        self.server.set_succeeded(result)

        self.current_goal_idx += 1

class TestRecipeTracking(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_recipe_tracking_node', anonymous=True)
        self.server = FakeStepActionServer()

    def test_successful_execution(self):
        """
        Test sequence where all steps succeed.
        """
        self.server.success_behavior = [True, True, True]  # Example: 3 steps succeed

        rospy.sleep(5)  # Allow node to run for a bit to send goals

        # Assertions: We could check something on the server
        self.assertEqual(self.server.current_goal_idx, 3)  # We received 3 goals

    def test_failed_then_successful_execution(self):
        """
        Test where the first step fails, then succeeds on the second attempt.
        """
        self.server.success_behavior = [False, True, True]

        rospy.sleep(7)

        # Check that the node continued even after a failure
        self.assertGreaterEqual(self.server.current_goal_idx, 2)  # At least 2 goals sent

    def test_all_steps_fail(self):
        """
        Test sequence where all steps fail.
        """
        self.server.success_behavior = [False, False, False]  # Exemple: 3 steps failed

        rospy.sleep(5)

        # Check if the systems fails all the steps correctly
        self.assertEqual(self.server.current_goal_idx, 3)  # 3 goals received


if __name__ == '__main__':
    import rostest
    rostest.rosrun('recipe_tracking', 'recipe_tracking_test', TestRecipeTracking)
