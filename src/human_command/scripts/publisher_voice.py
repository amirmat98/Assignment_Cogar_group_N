#!/usr/bin/env python3
"""voice_command_publisher – periodic mock for the */voice_command* topic
===========================================================================
Publishes a predefined speech‑recognition string at a configurable rate to
exercise the :pymod:`human_command_node` pipeline during demos and CI.

Parameters (ROS param server)
----------------------------
~message : str, *default* "cut carrots"
    The text payload to publish on ``/voice_command``.
~rate_hz : float, *default* 0.1
    Publication frequency in Hz.  The default (0.1 Hz) sends one message
    every 10 seconds.
~latched : bool, *default* False
    Whether the publisher should latch the last message so future
    subscribers receive it immediately.

Example
~~~~~~~
Run with faster output and a different command::

    rosrun action_planning voice_command_publisher.py _message:="mix eggs" _rate_hz:=1.0
"""

from __future__ import annotations

import rospy
from std_msgs.msg import String

__all__ = ["main"]


# ---------------------------------------------------------------------------
# Publisher loop -------------------------------------------------------------
# ---------------------------------------------------------------------------


def _publish_loop() -> None:  # noqa: D401 – imperative mood preferable
    """Retrieve parameters, create publisher, and broadcast in a loop."""
    rospy.init_node("voice_command_publisher")

    message: str = rospy.get_param("~message", "cut carrots")
    rate_hz: float = rospy.get_param("~rate_hz", 0.1)
    latched: bool = rospy.get_param("~latched", False)

    pub = rospy.Publisher("/voice_command", String, queue_size=10, latch=latched)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(
        f"Publishing '{message}' on /voice_command at {rate_hz:.2f} Hz "
        f"(latched={latched})"
    )

    while not rospy.is_shutdown():
        pub.publish(message)
        rospy.logdebug(f"/voice_command → '{message}'")
        rate.sleep()


# ---------------------------------------------------------------------------
# Entry‑point ---------------------------------------------------------------
# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401 – imperative mood preferable
    try:
        _publish_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
