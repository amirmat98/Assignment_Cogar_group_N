#!/usr/bin/env python3
"""path_planning_server – probabilistic mock for */path_planning* action
===========================================================================
This node emulates a motion‑planning backend (e.g. MoveIt!) for integration
and CI tests. It accepts :action:`assignments/stepAction` goals containing an
``action`` verb and an ``ingredient`` object, waits a configurable amount of
*planning time*, then responds with *SUCCEEDED* or *ABORTED* according to a
Bernoulli trial.

Parameters (ROS param server)
----------------------------
~planning_delay : float, *default* 2.0
    Seconds to sleep before responding, to mimic planning latency.
~success_prob : float, *default* 0.9
    Probability that the planning result is reported as *success*.

Topics & actions
~~~~~~~~~~~~~~~~
*Action server* **/path_planning** – type
:action:`assignments/stepAction <assignments.msg.stepAction>`

Typical usage::

    rosrun action_planning path_planning_server.py              # defaults
    rosrun action_planning path_planning_server.py _success_prob:=1.0  \
                                                 _planning_delay:=0.1
"""

from __future__ import annotations

import random
import rospy
import actionlib
from assignments.msg import stepAction, stepFeedback, stepResult

__all__ = ["main"]


# ---------------------------------------------------------------------------
# Action callback ------------------------------------------------------------
# ---------------------------------------------------------------------------


def _execute_cb(goal: stepAction.Goal, server: actionlib.SimpleActionServer):  # type: ignore[name-defined]
    """Handle one motion‑planning goal.

    Parameters
    ----------
    goal : assignments.msg.stepGoal
        Incoming goal containing *action* and *ingredient*.
    server : actionlib.SimpleActionServer
        The server instance used to publish feedback and results.
    """
    feedback = stepFeedback()
    result = stepResult()

    planning_delay: float = rospy.get_param("~planning_delay", 2.0)
    success_prob: float = rospy.get_param("~success_prob", 0.9)

    label = f"{goal.action} {goal.ingredient}"
    rospy.loginfo(f"[PathPlanner] ↓ goal received – {label}")

    feedback.status = "Planning path …"
    server.publish_feedback(feedback)

    # Simulate planning time ------------------------------------------------
    sleep_rate = rospy.Rate(10)  # 10 Hz loop allows pre‑emption checks
    ticks = int(planning_delay * 10)
    for _ in range(ticks):
        if server.is_preempt_requested():
            rospy.logwarn("[PathPlanner] goal pre‑empted by client")
            server.set_preempted()
            return
        sleep_rate.sleep()

    # Decide success/failure -----------------------------------------------
    success: bool = random.random() < success_prob
    if success:
        feedback.status = "Path planned successfully ✓"
        server.publish_feedback(feedback)
        result.success = True
        server.set_succeeded(result)
        rospy.loginfo("[PathPlanner] goal succeeded")
    else:
        feedback.status = "Path planning failed ✗"
        server.publish_feedback(feedback)
        result.success = False
        server.set_aborted(result)
        rospy.logwarn("[PathPlanner] goal aborted – planning failed")


# ---------------------------------------------------------------------------
# Node initialisation --------------------------------------------------------
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401 – imperative mood preferable
    """Start the mock path‑planning action server and spin forever."""
    rospy.init_node("path_planning_server")

    server = actionlib.SimpleActionServer(
        "/path_planning",
        stepAction,
        execute_cb=lambda goal: _execute_cb(goal, server),  # type: ignore[arg-type]
        auto_start=False,
    )
    server.start()

    planning_delay: float = rospy.get_param("~planning_delay", 2.0)
    success_prob: float = rospy.get_param("~success_prob", 0.9)
    rospy.loginfo(
        f"Mock /path_planning ready (planning_delay={planning_delay:.2f}s, "
        f"success_prob={success_prob:.2f})"
    )
    rospy.spin()


# ---------------------------------------------------------------------------
# Entry‑point ---------------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
