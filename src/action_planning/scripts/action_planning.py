#!/usr/bin/env python3
"""action_planning_node – Recipe‑execution orchestrator
========================================================
High‑level ROS node that coordinates every step of a cooking recipe by
linking *perception*, *path‑planning* and *user feedback* subsystems.

The node exposes one **action server** (``/step_action``). Each goal is a
single recipe *step* (action + ingredient). Typical flow::

   GUI / CLI ─┬──────────────────▶ /step_action (goal)
              │                    │
              │ 1. verify object  │
              │ 2. retry / speak  │
              │ 3. call planner   │
              │                    ▼
   Perception ◀────────────────────┘
                    ▲
                    │
          Path planning (MoveIt!)

The file was **fully refactored May 2025** to improve robustness and
readability.  Major changes include queue semantics (FIFO), configurable
parameters, cached service proxies, granular action outcomes and graceful
shutdown.

Sections
--------
* :class:`~ActionPlanningNode` – main ROS node implementation.
* :class:`~RecipeStep` – tiny dataclass representing a step.

Author
------
Amirmahdi Matin
"""

from __future__ import annotations

import rospy
import actionlib
from collections import deque
from dataclasses import dataclass
from typing import Deque, List

from assignments.srv import Speaker, Perception, PerceptionRequest
from assignments.msg import (
    stepAction,
    stepGoal,
    stepFeedback,
    stepResult,
)

__all__ = ["ActionPlanningNode", "RecipeStep"]

# ---------------------------------------------------------------------------
# Helper dataclass -----------------------------------------------------------
# ---------------------------------------------------------------------------


@dataclass
class RecipeStep:
    """
    A lightweight container for a single recipe instruction.

    Parameters
    ----------
    action : str
        The verb to perform (e.g. ``"pour"``, ``"mix"``).
    ingredient : str
        The physical item subject to the *action* (e.g. ``"milk"``).
    """

    action: str
    ingredient: str

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    @classmethod
    def from_goal(cls, goal: stepGoal) -> "RecipeStep":
        """Build a :class:`RecipeStep` from a :pyclass:`~assignments.msg.stepGoal`.

        Parameters
        ----------
        goal : assignments.msg.stepGoal
            The goal received on ``/step_action``.

        Returns
        -------
        RecipeStep
            Populated with *action* and *ingredient* from *goal*.
        """

        return cls(goal.action, goal.ingredient)


# ---------------------------------------------------------------------------
# Main node ------------------------------------------------------------------
# ---------------------------------------------------------------------------


class ActionPlanningNode:
    """ROS node that validates and executes recipe steps.

    Runtime parameters (ROS params)
    -------------------------------
    ~perception_timeout : float, *default* 2.0
        Seconds to wait for a response from the ``/perception`` service.
    ~planner_timeout : float, *default* 10.0
        Maximum seconds to wait for the path‑planning action to finish.
    ~retry_delay : float, *default* 2.0
        Pause before each perception retry when an object is missing.
    ~max_retries : int, *default* 1
        How many times to retry perception before aborting the current step.
    """

    # ------------------------------------------------------------------ construction

    def __init__(self) -> None:  # noqa: D401 – imperative mood preferable
        """Initialise the ROS node, connect servers/clients and set callbacks."""

        rospy.init_node("action_planning_node")

        # ------------------------------ parameters
        self.perception_timeout: float = rospy.get_param("~perception_timeout", 2.0)
        self.planner_timeout: float = rospy.get_param("~planner_timeout", 10.0)
        self.retry_delay: float = rospy.get_param("~retry_delay", 2.0)
        self.max_retries: int = rospy.get_param("~max_retries", 1)

        # ------------------------------ state
        self.pending: Deque[RecipeStep] = deque()
        self.done: List[RecipeStep] = []

        # ------------------------------ service proxies
        rospy.loginfo("Waiting for /perception and /speaker services …")
        rospy.wait_for_service("/perception", timeout=10)
        rospy.wait_for_service("/speaker", timeout=10)
        self._perception_srv = rospy.ServiceProxy("/perception", Perception)
        self._speaker_srv = rospy.ServiceProxy("/speaker", Speaker)
        rospy.loginfo("Services connected.")

        # ------------------------------ action client (path planner)
        self._path_planner = actionlib.SimpleActionClient("/path_planning", stepAction)
        rospy.loginfo("Waiting for /path_planning action server …")
        if not self._path_planner.wait_for_server(rospy.Duration(15)):
            rospy.logfatal("/path_planning action server not available – shutting down.")
            rospy.signal_shutdown("path planning action missing")
            return
        rospy.loginfo("/path_planning action server ready.")

        # ------------------------------ action server (this node)
        self._server = actionlib.SimpleActionServer(
            "/step_action",
            stepAction,
            execute_cb=self._on_step_goal,
            auto_start=False,
        )
        self._server.start()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("ActionPlanningNode initialised ✓")

    # ------------------------------------------------------------------ goal handler

    def _on_step_goal(self, goal: stepGoal) -> None:
        """Handle an incoming goal on ``/step_action``.

        Parameters
        ----------
        goal : assignments.msg.stepGoal
            The goal containing *action* and *ingredient* fields.
        """

        step = RecipeStep.from_goal(goal)
        feedback = stepFeedback()
        result = stepResult()

        # --------------------------- verify goal content
        if not step.action or not step.ingredient:
            result.success = False
            self._server.set_rejected(result, "Empty action or ingredient")
            return

        # --------------------------- enqueue and notify
        self.pending.append(step)
        rospy.loginfo(f"[Enqueue] {step.action} {step.ingredient}")
        feedback.status = f"Enqueued: {step.action} {step.ingredient}"
        self._server.publish_feedback(feedback)

        # --------------------------- process queue (may span multiple goals)
        while self.pending and not rospy.is_shutdown():
            current = self.pending[0]
            label = f"[{current.action} {current.ingredient}]"

            # ----------- perception --------------------------------------------------
            if not self._check_object_presence(current.ingredient):
                if not self._handle_missing_object(current.ingredient, feedback):
                    result.success = False
                    self._server.set_aborted(result, f"{label} missing and not resolved")
                    return

            # ----------- path‑planning ---------------------------------------------
            feedback.status = f"{label} → path planner"
            self._server.publish_feedback(feedback)

            if not self._execute_path_planner(current):
                result.success = False
                self._server.set_aborted(result, f"{label} path planning failed")
                return

            # ----------- mark complete ---------------------------------------------
            self.done.append(self.pending.popleft())
            feedback.status = f"Completed: {current.action} {current.ingredient}"
            self._server.publish_feedback(feedback)

        # --------------------------- all done ---------------------------------------
        result.success = True
        self._server.set_succeeded(result, "All queued steps completed")

    # ------------------------------------------------------------------ helpers

    def _check_object_presence(self, ingredient: str) -> bool:
        """Query the perception service for *ingredient*.

        Parameters
        ----------
        ingredient : str
            Name of the object to locate.

        Returns
        -------
        bool
            ``True`` if perception reports success, ``False`` otherwise.
        """
        try:
            req = PerceptionRequest(objects=[ingredient])  # type: ignore[arg-type]
            resp = self._perception_srv(req, timeout=self.perception_timeout)  # type: ignore[misc]
            return resp.found  # type: ignore[attr-defined]
        except rospy.ServiceException as exc:
            rospy.logwarn(f"Perception service error: {exc}")
            return False

    # ------------------------------------------------------------------ conflict‑handling

    def _handle_missing_object(self, ingredient: str, fb: stepFeedback) -> bool:
        """Announce a missing object and retry perception.

        Parameters
        ----------
        ingredient : str
            The missing ingredient.
        fb : assignments.msg.stepFeedback
            Feedback message to update clients.

        Returns
        -------
        bool
            ``True`` if the object eventually appears, ``False`` otherwise.
        """
        self._speak(f"Object {ingredient} not found. Please place it in view.")
        for attempt in range(1, self.max_retries + 1):
            fb.status = f"Retrying perception for {ingredient} (attempt {attempt})"
            self._server.publish_feedback
