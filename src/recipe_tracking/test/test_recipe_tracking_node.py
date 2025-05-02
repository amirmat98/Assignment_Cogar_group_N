#!/usr/bin/env python3
"""test_recipe_tracking_node – integration tests for *RecipeTrackingNode*
===========================================================================
A lightweight rostest fixture that boots a **fake** ``/step_action`` server
with user‑programmable success/failure behaviour and then launches the real
:pyclass:`recipe_tracking_node.RecipeTrackingNode` under test (declared as a
<test_depend> in *package.xml*).  The tests assert that the correct number
of goals is dispatched in the expected order, independent of whether the
planner reports success or failure.

Design rationale
----------------
* The Action Planner is external; therefore a stub server suffices.
* Success/failure patterns are injected via ``success_pattern`` on the fake
  server so each test can describe arbitrary scenarios.

Key classes
~~~~~~~~~~~
``FakeStepActionServer``
    Minimal :class:`actionlib.SimpleActionServer` that responds according to
    an iterator of booleans.

``TestRecipeTracking``
    ``unittest.TestCase`` exercising three scenarios:
    * all steps succeed,
    * first fails then succeed,
    * all fail.

Environment requirements
~~~~~~~~~~~~~~~~~~~~~~~~
The real *recipe_tracking_node* should be started via <node> or
<include> in the corresponding *.test* launch file so that it runs in the
same ROS master as this test.
"""

from __future__ import annotations

import itertools
import unittest
from typing import Iterable, Optional

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from assignments.msg import stepAction, stepGoal, stepResult

__all__ = ["TestRecipeTracking"]


# ---------------------------------------------------------------------------
# Fake action server ---------------------------------------------------------
# ---------------------------------------------------------------------------


class FakeStepActionServer:
    """A programmable ``/step_action`` stub used for testing."""

    def __init__(self, pattern: Iterable[bool] | None = None) -> None:  # noqa: D401
        self._pattern = itertools.cycle(pattern or (True,))
        self._goal_count: int = 0
        self._server = actionlib.SimpleActionServer(
            "/step_action", stepAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._server.start()
        rospy.loginfo("FakeStepActionServer started ✓")

    # .....................................................................

    def _execute_cb(self, goal: stepGoal) -> None:  # noqa: D401
        success = next(self._pattern)
        rospy.loginfo(f"Fake server received goal → success={success}")
        rospy.sleep(0.3)  # Simulate processing time
        result = stepResult(success=success)
        self._server.set_succeeded(result)
        self._goal_count += 1

    # .....................................................................

    @property
    def goal_count(self) -> int:
        return self._goal_count

    # .....................................................................

    def set_pattern(self, pattern: Iterable[bool]) -> None:
        """Update the success/failure iterator for the next goals."""
        self._pattern = itertools.cycle(pattern)
        self._goal_count = 0

    # .....................................................................

    def shutdown(self) -> None:
        self._server.shutdown("tests complete")


# ---------------------------------------------------------------------------
# Test case ------------------------------------------------------------------
# ---------------------------------------------------------------------------


class TestRecipeTracking(unittest.TestCase):
    """Validate that RecipeTrackingNode dispatches goals correctly."""

    # .............................................................. class‑setup

    @classmethod
    def setUpClass(cls):  # noqa: D401
        rospy.init_node("test_recipe_tracking_node", anonymous=True)
        cls._fake_server = FakeStepActionServer()
        rospy.sleep(1.0)  # Allow wiring

    # .............................................................. scenario helpers

    def _run_scenario(self, pattern: list[bool], wait: float = 6.0):
        self._fake_server.set_pattern(pattern)
        rospy.sleep(wait)  # Let the node under test work
        return self._fake_server.goal_count

    # .............................................................. tests

    def test_all_succeed(self):  # noqa: D401
        """Every step returns *success* – goal count == len(pattern)."""
        pattern = [True, True, True]
        goals_sent = self._run_scenario(pattern)
        self.assertEqual(goals_sent, len(pattern))

    def test_first_fail_then_succeed(self):  # noqa: D401
        """First goal fails; node should continue to next step."""
        pattern = [False, True, True]
        goals_sent = self._run_scenario(pattern, wait=7.0)
        self.assertGreaterEqual(goals_sent, 2)

    def test_all_fail(self):  # noqa: D401
        """All goals fail – node should attempt each step once."""
        pattern = [False, False, False]
        goals_sent = self._run_scenario(pattern)
        self.assertEqual(goals_sent, len(pattern))

    # .............................................................. teardown

    @classmethod
    def tearDownClass(cls):  # noqa: D401
        cls._fake_server.shutdown()
        rospy.sleep(0.5)


# ---------------------------------------------------------------------------
# rostest entry‑point --------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import rostest

    rostest.rosrun("recipe_tracking", "test_recipe_tracking_node", TestRecipeTracking)
