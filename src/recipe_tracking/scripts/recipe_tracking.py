#!/usr/bin/env python3
"""recipe_tracking_node – execution‑history tracker for cooking recipes
========================================================================
Implements the *Recipe Tracking & Execution History* component.  The node
reads a recipe text file, validates its syntax, and sequentially sends
each step as an :action:`assignments/stepAction` goal to the Action
Planner.  The outcome of every goal updates an internal *check‑list*,
which forms a persistent execution trace.

Recipe file format
------------------
A UTF‑8 text file with one comma‑separated step per line::

    cutting,carrot
    pouring,water
    mixing,all
    end

Any additional field on a line marks the step as pre‑validated (not
recommended, reserved for future extensions).  Validation rules:

* File must terminate with ``end``.
* ``cutting``   – exactly one *ingredient*.
* ``pouring``   – exactly one *ingredient*.
* ``mixing``    – *ingredient* must be literal ``all``.

ROS interfaces
~~~~~~~~~~~~~~
Action client **/step_action** (type :action:`assignments/stepAction`) –
sends each step goal and waits for completion.

Parameters (ROS param server)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~recipe_txt : str, *default* «../../../assignments/recipes/recipe.txt»
    Path to the recipe file to execute.
~step_timeout : float, *default* 30.0
    Maximum seconds to wait for each step result before logging an error.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import actionlib
import rospy
from assignments.msg import stepAction, stepGoal, stepResult

__all__ = ["RecipeTrackingNode", "main"]


# ---------------------------------------------------------------------------
# Dataclasses ----------------------------------------------------------------
# ---------------------------------------------------------------------------


@dataclass
class RecipeStep:
    """Single instruction within a recipe."""

    action: str
    ingredient: str
    success: bool = False

    def to_goal(self) -> stepGoal:  # noqa: D401 – imperative mood preferable
        """Convert to :class:`assignments.msg.stepGoal`."""
        return stepGoal(action=self.action, ingredient=self.ingredient)

    def __str__(self) -> str:  # noqa: D401
        return f"{self.action} {self.ingredient}".strip()


# ---------------------------------------------------------------------------
# Parser & validator ---------------------------------------------------------
# ---------------------------------------------------------------------------


def _parse_recipe(path: Path) -> List[RecipeStep]:
    """Parse *path* into a list of :class:`RecipeStep`s and validate syntax.

    Raises
    ------
    ValueError
        If the recipe is syntactically invalid.
    """
    if not path.is_file():
        raise ValueError(f"Recipe file not found: {path}")

    steps: List[RecipeStep] = []
    with path.open("r", encoding="utf-8") as f:
        for ln, line in enumerate(f, start=1):
            parts = [p.strip() for p in line.split(",")]
            if not parts or not parts[0]:
                continue  # Skip empty lines
            action = parts[0].lower()
            ingredient = parts[1].lower() if len(parts) > 1 else ""
            pre_success = len(parts) > 2
            steps.append(RecipeStep(action, ingredient, pre_success))

    # ---------------- validation rules -----------------------------------
    if not steps or steps[-1].action != "end":
        raise ValueError("Recipe must terminate with 'end' step")

    for step in steps[:-1]:  # skip 'end'
        if step.action == "cutting":
            if not step.ingredient:
                raise ValueError("'cutting' requires exactly one ingredient")
        elif step.action == "pouring":
            if not step.ingredient:
                raise ValueError("'pouring' requires exactly one ingredient")
        elif step.action == "mixing":
            if step.ingredient != "all":
                raise ValueError("'mixing' ingredient must be 'all'")
        else:
            raise ValueError(f"Unknown action '{step.action}' on line {ln}")

    return steps


# ---------------------------------------------------------------------------
# Main node ------------------------------------------------------------------
# ---------------------------------------------------------------------------


class RecipeTrackingNode:  # noqa: D401 – imperative mood preferable
    """ROS node managing recipe execution history."""

    def __init__(self) -> None:
        rospy.init_node("recipe_tracking_node")

        # ---------------- parameters --------------------------------------
        default_path = Path(__file__).resolve().parents[2] / "assignments" / "recipes" / "recipe.txt"
        recipe_path = Path(rospy.get_param("~recipe_txt", str(default_path)))
        self._timeout = float(rospy.get_param("~step_timeout", 30.0))

        # ---------------- parse & validate recipe -------------------------
        try:
            self._steps: List[RecipeStep] = _parse_recipe(recipe_path)
        except ValueError as exc:
            rospy.logfatal(f"Recipe error: {exc}")
            rospy.signal_shutdown("invalid recipe")
            return
        rospy.loginfo(f"Loaded recipe with {len(self._steps)} steps from {recipe_path}")

        # ---------------- action client -----------------------------------
        self._client = actionlib.SimpleActionClient("/step_action", stepAction)
        rospy.loginfo("Waiting for /step_action action server …")
        if not self._client.wait_for_server(rospy.Duration(15)):
            rospy.logfatal("/step_action server not reachable – shutting down")
            rospy.signal_shutdown("action server missing")
            return

    # ------------------------------------------------------------------ run loop

    def run(self) -> None:
        """Iterate through the recipe, updating success flags."""
        for idx, step in enumerate(self._steps):
            if step.action == "end":
                rospy.loginfo("Recipe complete ✓")
                break
            if step.success:
                rospy.loginfo(f"Skipping pre‑validated step {idx}: {step}")
                continue

            rospy.loginfo(f"Executing step {idx}: {step}")
            self._client.send_goal(step.to_goal())
            if not self._client.wait_for_result(rospy.Duration(self._timeout)):
                rospy.logerr(f"Timeout waiting for result of step {idx}: {step}")
                continue  # keep going or abort? Here we continue.
            result: stepResult = self._client.get_result()
            if result.success:
                self._steps[idx].success = True
                rospy.loginfo(f"Step {idx} succeeded: {step}")
            else:
                rospy.logwarn(f"Step {idx} failed: {step}")

    # ---------------------------------------------------------------------- util

    def dump_checklist(self) -> None:
        """Print current recipe checklist to ROS log."""
        for i, step in enumerate(self._steps):
            status = "✔" if step.success else "✗"
            rospy.loginfo(f"[{status}] {i:02d} – {step}")


# ---------------------------------------------------------------------------
# Entry‑point ---------------------------------------------------------------
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401 – imperative mood preferable
    try:
        node = RecipeTrackingNode()
        node.run()
        node.dump_checklist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
