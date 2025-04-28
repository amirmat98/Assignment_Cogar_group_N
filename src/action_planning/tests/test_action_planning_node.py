#!/usr/bin/env python3

import unittest
import rospy
import rostest

from std_srvs.srv import Empty
from assignments.msg import stepAction, stepActionFeedback, stepActionResult
import actionlib
from assignments.msg import stepAction, stepGoal, stepResult
from assignments.srv import Perception, PerceptionResponse
from assignments.srv import Speaker, SpeakerResponse
from actionlib_msgs.msg import GoalStatus
from threading import Thread
import random

from assignments.msg import stepActionResult
from std_msgs.msg import String


class TestActionPlanningNode(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_action_planning', anonymous=True)

        # Wait for the action server
        self.client = actionlib.SimpleActionClient('/step_action', stepAction)
        self.assertTrue(self.client.wait_for_server(timeout=rospy.Duration(10)))

        # Set up mock services
        self.mock_perception = rospy.Service('/perception', Perception, self.perception_callback)
        self.mock_speaker = rospy.Service('/speaker', Speaker, self.speaker_callback)

        self.perception_call_count = 0
        self.speaker_call_count = 0

        # Setup path_planning mock action server

        self.path_planning_server = actionlib.SimpleActionServer(
            '/path_planning', stepAction, execute_cb=self.path_planning_callback, auto_start=False)
        self.path_planning_server.start()

    def perception_callback(self, req):
        """
        Always return True for simplicity
        """
        self.perception_call_count += 1
        return PerceptionResponse(found= (random.random() < 0.9))

    def speaker_callback(self, req):
        self.speaker_call_count += 1
        return SpeakerResponse(success= (random.random() < 0.9))

    def path_planning_callback(self, goal):
        """
        Simulates a successful path planning result.
        """
        rospy.sleep(0.5)  # simulate delay
        result = stepResult(success= (random.random() < 0.9))
        self.path_planning_server.set_succeeded(result)

    def test_step_successful(self):
        """
        Test a successful step execution flow: perception finds object,
        path planning returns success, and the stepAction server completes.
        """
        goal = stepGoal(action="cutting", ingredient="carrots")
        self.client.send_goal(goal)
        self.client.wait_for_result(timeout=rospy.Duration(5))

        result = self.client.get_result()
        state = self.client.get_state()

        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual(self.perception_call_count, 1)
        self.assertEqual(self.speaker_call_count, 0)

    def tearDown(self):
        self.mock_perception.shutdown()
        self.mock_speaker.shutdown()


if __name__ == '__main__':
    rostest.rosrun('action_planning', 'test_action_planning_node', TestActionPlanningNode)
