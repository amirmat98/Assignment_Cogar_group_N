#!/usr/bin/env python3

"""
.. module:: path_planning_server
   :platform: Unix
   :synopsis: Simulates a simple path planning server for executing recipe steps.

.. moduleauthor:: AmirMahdi Matin

This module defines a ROS node that acts as a path planning server:
- Receives movement goals through an action server.
- Simulates path planning with a random success rate.
- Publishes feedback and results to the action client.
"""

import rospy
import random
import actionlib
from assignments.msg import stepAction, stepFeedback, stepResult


class PathPlanner:
    """
    A ROS node that provides a simple path planning simulation service.
    
    This class sets up an action server at `/path_planning` to handle incoming movement goals
    and simulates planning success or failure.
    """
    def __init__(self):
        """
        Initializes the PathPlanner node by setting up the action server
        and preparing it to handle incoming path planning requests.
        """
        rospy.init_node('path_planning_server')
        self.server = actionlib.SimpleActionServer('/path_planning', stepAction, execute_cb=self.execute_callback, auto_start=False)
        self.server.start()
        rospy.loginfo("Path planning server started at /path_planning")

    def execute_callback(self, goal):
        """
        Callback function executed when a new goal is received by the action server.

        Simulates the path planning process:
        - Waits for 2 seconds to mimic planning time.
        - 90% chance to succeed; 10% chance to fail.

        :param goal: A stepGoal object containing action and ingredient details.
        :type goal: assignments.msg.stepGoal
        """
        feedback = stepFeedback()
        result = stepResult()

        rospy.loginfo(f"[PathPlanner] Received goal: {goal.action} {goal.ingredient}")

        feedback.status = "Planning path..."
        self.server.publish_feedback(feedback)

        rospy.sleep(2)  # Simulate planning delay

        # 90% success chance
        if random.random() < 0.9:
            feedback.status = "Path planned successfully."
            self.server.publish_feedback(feedback)
            result.success = True
            self.server.set_succeeded(result)
            rospy.loginfo("[PathPlanner] Planning succeeded.")
        else:
            feedback.status = "Path planning failed."
            self.server.publish_feedback(feedback)
            result.success = False
            self.server.set_aborted(result)
            rospy.logwarn("[PathPlanner] Planning failed.")


if __name__ == '__main__':
    try:
        PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
