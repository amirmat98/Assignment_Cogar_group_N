#!/usr/bin/env python3
"""human_command_node – interpreter/validator for *voice* cooking commands
===========================================================================
This node bridges a speech‑recognition frontend (publishing strings on
``/voice_command``) and the high‑level :action:`assignments/stepAction`
interface used by the *Action Planner*.

Responsibilities
----------------
* Subscribe to ``/voice_command`` for raw user commands.
* Parse each utterance into an **action** (verb) and **ingredient**.
* Validate the command against current robot feedback; accept or attempt
  *conflict resolution*.
* Forward accepted commands as goals to the ``/step_action`` server.
* Relay status to the user via the ``/speaker`` service.

Configuration (ROS params)
~~~~~~~~~~~~~~~~~~~~~~~~~~
~accept_prob : float, *default* 0.8
    Probability that *validate_command* considers a command **valid**.
~resolve_prob : float, *default* 0.5
    Probability that *resolve_conflict* will accept an initially invalid
    command.
~speaker_timeout : float, *default* 5.0
    Seconds to wait for the ``/speaker`` service before logging an error.
"""

from __future__ import annotations

import re
import random
from dataclasses import dataclass
from typing import Optional

import rospy
import actionlib
from std_msgs.msg import String

from assignments.msg import (
    stepAction,
    stepGoal,
    stepActionFeedback,
    stepFeedback,
    stepResult,
)
from assignments.srv import Speaker

__all__ = ["HumanCommandNode", "main"]

# ---------------------------------------------------------------------------
# Dataclass -----------------------------------------------------------------
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Command:
    """Structured representation of a voice command."""

    action: str
    ingredient: str

    def __str__(self) -> str:  # noqa: D401 – imperative mood preferable
        return f"{self.action} {self.ingredient}".strip()


# ---------------------------------------------------------------------------
# Main node ------------------------------------------------------------------
# ---------------------------------------------------------------------------


class HumanCommandNode:
    """ROS node that validates and forwards human cooking commands."""

    # ------------------------------------------------------------ construction

    def __init__(self) -> None:  # noqa: D401 – imperative mood preferable
        rospy.init_node("human_command_node")

        # Params --------------------------------------------------------------
        self._accept_prob: float = rospy.get_param("~accept_prob", 0.8)
        self._resolve_prob: float = rospy.get_param("~resolve_prob", 0.5)
        self._speaker_timeout: float = rospy.get_param("~speaker_timeout", 5.0)

        # Internal state ------------------------------------------------------
        self._current_feedback: Optional[str] = None
        self._expected_step: Optional[Command] = None

        # Subscribers ---------------------------------------------------------
        rospy.Subscriber("/step_action/feedback", stepActionFeedback, self._feedback_cb)
        rospy.Subscriber("/voice_command", String, self._voice_cb)

        # Action client -------------------------------------------------------
        self._client = actionlib.SimpleActionClient("/step_action", stepAction)
        rospy.loginfo("Waiting for /step_action server …")
        if not self._client.wait_for_server(rospy.Duration(15)):
            rospy.logfatal("/step_action server not available – shutting down")
            rospy.signal_shutdown("step_action absent")
            return
        rospy.loginfo("/step_action server connected ✓")
        rospy.loginfo("HumanCommandNode initialised; listening on /voice_command")

    # ---------------------------------------------------- subscriber callbacks

    def _feedback_cb(self, msg: stepActionFeedback) -> None:  # noqa: D401
        """Handle feedback from the Action Planner."""
        self._current_feedback = msg.status
        rospy.loginfo_throttle(2, f"Feedback: {self._current_feedback}")

    def _voice_cb(self, msg: String) -> None:  # noqa: D401
        """Handle an incoming voice command string."""
        raw = msg.data.strip()
        rospy.loginfo(f"Voice command: '{raw}'")

        command = self._interpret(raw)
        if command is None:
            self._notify_user("Command not understood.")
            rospy.logwarn("Interpretation failed – command discarded")
            return

        if self._validate(command):
            self._send_goal(command)
            self._expected_step = command
            self._notify_user(f"Command accepted: {command}")
        else:
            rospy.loginfo("Validation failed – trying conflict resolution …")
            if self._resolve_conflict(command):
                self._send_goal(command)
                self._expected_step = command
                self._notify_user(f"Conflict resolved – command accepted: {command}")
            else:
                self._notify_user(f"Command refused: {command}")
                rospy.logwarn("Conflict resolution failed – command refused")

    # ---------------------------------------------------------- core helpers

    @staticmethod
    def _interpret(text: str) -> Optional[Command]:
        """Parse *text* into an (action, ingredient) pair.

        This simplistic parser takes the first word as *verb* and the rest
        as *ingredient*.  If no verb is found, returns *None*.
        """
        if not text:
            return None
        parts = re.split(r"\s+", text, maxsplit=1)
        action = parts[0].lower()
        ingredient = parts[1].lower() if len(parts) > 1 else ""
        return Command(action, ingredient)

    # -----------------------------------------------------------------------

    def _validate(self, cmd: Command) -> bool:
        """Decide whether *cmd* is valid based on current feedback.

        The default implementation is probabilistic; override in subclasses
        for domain‑specific checks.
        """
        valid = random.random() < self._accept_prob
        rospy.loginfo(f"Validation for '{cmd}': {'✔' if valid else '✗'}")
        return valid

    def _resolve_conflict(self, cmd: Command) -> bool:
        """Attempt to resolve a validation conflict.

        Currently uses a Bernoulli trial controlled by *~resolve_prob*.
        """
        resolved = random.random() < self._resolve_prob
        rospy.loginfo(f"Conflict resolution for '{cmd}': {'ACCEPTED' if resolved else 'REJECTED'}")
        return resolved

    # -----------------------------------------------------------------------

    def _send_goal(self, cmd: Command) -> None:
        """Forward an accepted *cmd* to the Action Planner."""
        goal = stepGoal(action=cmd.action, ingredient=cmd.ingredient)
        self._client.send_goal(goal)
        rospy.loginfo(f"Goal dispatched: {cmd}")

    # -----------------------------------------------------------------------

    def _notify_user(self, text: str) -> None:
        """Publish *text* via the ``/speaker`` service (best‑effort)."""
        try:
            rospy.wait_for_service("/speaker", timeout=self._speaker_timeout)
            speaker = rospy.ServiceProxy("/speaker", Speaker)
            speaker(text)
            rospy.loginfo(f"User notified: {text}")
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logerr(f"/speaker service error: {exc}")


# ---------------------------------------------------------------------------
# Entry‑point ---------------------------------------------------------------
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401 – imperative mood preferable
    try:
        HumanCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
