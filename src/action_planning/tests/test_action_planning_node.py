#!/usr/bin/env python3
"""test_action_planning_node – rostest suite for :mod:`action_planning_node`
===========================================================================
This module contains *system‑level* (a.k.a. black‑box) tests that spin up an
:class:`~action_planning_node.ActionPlanningNode` instance together with mock
services and an action server in order to verify the end‑to‑end behaviour of
recipe‑step execution.

It is deliberately lightweight: no real robot, MoveIt! or cameras are
required.  Instead, the following components are faked at runtime:

* ``/perception``  – returns *found = True* 9 out of 10 times.
* ``/speaker``     – acknowledges the request 9 out of 10 times.
* ``/path_planning`` – action server that succeeds 9 out of 10 times.

Because of the probabilistic mocks, the default seed is fixed so the test
result is repeatable.  Adjust the *success_probability* class attribute if you
prefer deterministic *always‑success* or *always‑fail* scenarios.

Author
------
Amirmahdi Matin

"""

from __future__ import annotations

import random
import unittest
from threading import Thread

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from assignments.msg import stepAction, stepGoal, stepResult, stepFeedback
from assignments.srv import Perception, PerceptionResponse, Speaker, SpeakerResponse
from std_srvs.srv import Empty

__all__ = ["TestActionPlanningNode"]


class TestActionPlanningNode(unittest.TestCase):
    """Integration tests for :class:`action_planning_node.ActionPlanningNode`."""

    #: Probability in ``[0, 1]`` that a mock component responds with *success*.
    success_probability: float = 0.9

    # ------------------------------------------------------------------ lifecycle ---

    @classmethod
    def setUpClass(cls):  # noqa: D401 – imperative mood preferable
        """Initialise ROS node once for the whole test case."""
        rospy.init_node("test_action_planning", anonymous=True)
        random.seed(42)  # Make results reproducible

    def setUp(self):  # noqa: D401 – imperative mood preferable
        """Spin up mock services/action server and connect the client."""
        # -- Mock /perception -------------------------------------------------------
        self._mock_perception = rospy.Service(
            "/perception", Perception, self._perception_callback
        )
        self._perception_calls: int = 0

        # -- Mock /speaker ----------------------------------------------------------
        self._mock_speaker = rospy.Service(
            "/speaker", Speaker, self._speaker_callback
        )
        self._speaker_calls: int = 0

        # -- Mock /path_planning (action server) ------------------------------------
        self._path_planning_server = actionlib.SimpleActionServer(
            "/path_planning",
            stepAction,
            execute_cb=self._path_planning_callback,
            auto_start=False,
        )
        self._path_planning_server.start()

        # -- Action client under test ----------------------------------------------
        self._client = actionlib.SimpleActionClient("/step_action", stepAction)
        self.assertTrue(
            self._client.wait_for_server(rospy.Duration(15)),
            "Timeout waiting for /step_action server",
        )

    # ------------------------------------------------------------------ callbacks ----

    def _perception_callback(self, req: Perception.Request) -> PerceptionResponse:  # type: ignore[name-defined]
        """Return *found* with a configurable probability."""
        self._perception_calls += 1
        return PerceptionResponse(found=(random.random() < self.success_probability))

    def _speaker_callback(self, req: Speaker.Request) -> SpeakerResponse:  # type: ignore[name-defined]
        """Acknowledge speaker request with a configurable probability."""
        self._speaker_calls += 1
        return SpeakerResponse(success=(random.random() < self.success_probability))

    def _path_planning_callback(self, goal: stepGoal) -> None:  # noqa: D401
        """Simulate path planning and respond via ``set_succeeded``.

        A small delay is added to approximate computation time.
        """
        rospy.sleep(0.5)
        result = stepResult(success=(random.random() < self.success_probability))
        self._path_planning_server.set_succeeded(result)

    # ------------------------------------------------------------------ tests --------

    def test_single_step_success(self):
        """Verify happy‑path when all subsystems report success."""
        goal = stepGoal(action="cut", ingredient="carrot")
        self._client.send_goal(goal)
        finished = self._client.wait_for_result(timeout=rospy.Duration(10))
        self.assertTrue(finished, "Action did not finish in time")

        result = self._client.get_result()
        state = self._client.get_state()

        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual(self._perception_calls, 1)
        self.assertEqual(self._speaker_calls, 0)

    # ------------------------------------------------------------------ teardown -----

    def tearDown(self):  # noqa: D401 – imperative mood preferable
        """Shutdown mocks and ensure no dangling ROS interfaces."""
        self._mock_perception.shutdown()
        self._mock_speaker.shutdown()
        self._path_planning_server.shutdown()


# ---------------------------------------------------------------------------
# rostest entry‑point --------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Package name *action_planning* must match <package>/test/CMakeLists.txt
    rostest.rosrun("action_planning", "test_action_planning_node", TestActionPlanningNode)
