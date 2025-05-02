#!/usr/bin/env python3
"""perception_server – probabilistic mock for the */perception* service
=======================================================================
This lightweight node emulates an object‑detection service for
integration‑testing higher‑level nodes (e.g. *action_planning_node*).  It
returns *found = True* with a configurable probability so that test
scenarios can cover both *success* and *failure* branches without real
hardware or vision pipelines.

Usage
-----
Run::

    rosrun action_planning perception_server.py         # default 0.9 success
    
or override parameters::

    rosrun action_planning perception_server.py _success_prob:=0.5

Service
~~~~~~~
``/perception`` – type :srv:`assignments/Perception`

Parameters (ROS param server)
----------------------------
~success_prob : float, *default* 0.9
    Probability in ``[0, 1]`` that the service responds with
    ``PerceptionResponse(found=True)``.

-------------------------------------------------------
Author: Amirmahdi Matin
"""

from __future__ import annotations

import random
from typing import List

import rospy
from assignments.srv import Perception, PerceptionResponse  # type: ignore [import-not-found]

__all__ = ["main"]


# ---------------------------------------------------------------------------
# Callbacks ------------------------------------------------------------------
# ---------------------------------------------------------------------------


def _handle_perception(req: Perception.Request) -> PerceptionResponse:  # type: ignore[name-defined]
    """Service callback returning a probabilistic *found* flag.

    Parameters
    ----------
    req : assignments.srv.Perception.Request
        Contains a list of *objects* to be detected.

    Returns
    -------
    assignments.srv.PerceptionResponse
        ``found = True`` with probability *success_prob*.
    """
    rospy.loginfo(f"/perception request for objects: {list(req.objects)}")

    success_prob: float = rospy.get_param("~success_prob", 0.9)
    found: bool = random.random() < success_prob

    if found:
        rospy.loginfo("Mock perception: objects found ✓")
    else:
        rospy.logwarn("Mock perception: objects NOT found ✗")

    return PerceptionResponse(found=found)


# ---------------------------------------------------------------------------
# Node setup -----------------------------------------------------------------
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401 – imperative mood preferable
    """Initialise the mock perception service and spin forever."""
    rospy.init_node("perception_server")

    success_prob: float = rospy.get_param("~success_prob", 0.9)
    rospy.loginfo(f"Starting mock /perception (success_prob = {success_prob:.2f}) …")

    _service = rospy.Service("/perception", Perception, _handle_perception)
    rospy.loginfo("/perception service ready")
    rospy.spin()


# ---------------------------------------------------------------------------
# Entry‑point ---------------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
