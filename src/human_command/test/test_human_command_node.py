#!/usr/bin/env python3
"""test_human_command_node – system test for *HumanCommandNode*
================================================================
Spins up a *dummy* ``/step_action`` server and publishes a voice command on
``/voice_command``.  Verifies that the command is propagated to the action
server and receives either **ACTIVE** or **SUCCEEDED** status within a
timeout.

Why system‑level?
-----------------
*HumanCommandNode* is largely *orchestration* logic; mocking every internal
method would provide little assurance.  Instead, this test boots the real
node (via ``roslaunch`` or test dependencies) and observes full ROS traffic.

Key components
~~~~~~~~~~~~~~
``DummyStepActionServer``
    Minimal :class:`actionlib.SimpleActionServer` that immediately succeeds.

Test flow
~~~~~~~~~
1. Initialise ROS node ``test_human_command_node``.
2. Start :pyclass:`DummyStepActionServer`.
3. Advertise publisher on ``/voice_command``.
4. Wait for connectivity (2 s).
5. Publish ``cut carrots``.
6. Assert we receive status ``ACTIVE`` (1) or ``SUCCEEDED`` (3) within 8 s.
"""

from __future__ import annotations

import unittest
from typing import Optional

import rospy
import actionlib
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from assignments.msg import stepAction, stepGoal, stepResult

__all__ = ["TestHumanCommandNode"]


# ---------------------------------------------------------------------------
# Dummy action server --------------------------------------------------------
# ---------------------------------------------------------------------------


class DummyStepActionServer:
    """A trivial :action:`stepAction` server that always succeeds."""

    def __init__(self) -> None:  # noqa: D401 – imperative mood preferable
        self._server = actionlib.SimpleActionServer(
            "/step_action", stepAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._server.start()
        rospy.loginfo("DummyStepActionServer up and running ✓")

    # .....................................................................

    def _execute_cb(self, goal: stepGoal) -> None:  # noqa: D401
        rospy.loginfo(f"Dummy server got goal: {goal.action} {goal.ingredient}")
        rospy.sleep(0.5)  # Simulate latency
        self._server.set_succeeded(stepResult(), "Goal auto‑succeeded")
        rospy.loginfo("Dummy server: goal set to SUCCEEDED")

    # .....................................................................

    def shutdown(self) -> None:
        """Stop the action server cleanly."""
        self._server.shutdown("Test complete")


# ---------------------------------------------------------------------------
# Test case ------------------------------------------------------------------
# ---------------------------------------------------------------------------


class TestHumanCommandNode(unittest.TestCase):
    """Integration test verifying HumanCommandNode → /step_action link."""

    _status: Optional[int] = None  # Last status code seen from /step_action

    # .............................................................. class‑level

    @classmethod
    def setUpClass(cls):  # noqa: D401
        rospy.init_node("test_human_command_node", anonymous=True)

        # Spin up dummy server
        cls._dummy_server = DummyStepActionServer()

        # Publisher for voice commands
        cls._voice_pub = rospy.Publisher("/voice_command", String, queue_size=10)

        # Subscriber for action status
        cls._status_sub = rospy.Subscriber(
            "/step_action/status", GoalStatusArray, cls._status_cb
        )

        # Give ROS time to wire connections
        rospy.sleep(2.0)

    # .............................................................. status‑cb

    @classmethod
    def _status_cb(cls, msg: GoalStatusArray) -> None:  # noqa: D401
        if msg.status_list:
            cls._status = msg.status_list[-1].status

    # .............................................................. test‑method

    def test_voice_command_triggers_goal(self):  # noqa: D401 – imperative mood
        """Publish *cut carrots* and expect ACTIVE or SUCCEEDED status."""
        rospy.loginfo("Publishing test voice command …")
        self._voice_pub.publish("cut carrots")

        # Wait up to 8 s for status update
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and TestHumanCommandNode._status is None:
            if rospy.Time.now() > deadline:
                break
            rospy.sleep(0.1)

        self.assertIsNotNone(
            TestHumanCommandNode._status, "No status received from /step_action"
        )
        self.assertIn(
            TestHumanCommandNode._status,
            (1, 3),  # 1=ACTIVE, 3=SUCCEEDED
            f"Unexpected goal status {TestHumanCommandNode._status}",
        )

    # .............................................................. teardown

    @classmethod
    def tearDownClass(cls):  # noqa: D401
        rospy.loginfo("Shutting down DummyStepActionServer …")
        cls._dummy_server.shutdown()
        rospy.sleep(0.5)  # Allow clean shutdown


# ---------------------------------------------------------------------------
# rostest entry‑point --------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import rostest

    rostest.rosrun("human_command", "test_human_command_node", TestHumanCommandNode)
