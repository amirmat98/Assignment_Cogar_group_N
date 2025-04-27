#!/usr/bin/env python3

import rospy
import random
import actionlib
from assignments.msg import stepAction, stepFeedback, stepResult


class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planning_server')

        self.server = actionlib.SimpleActionServer(
            '/path_planning',
            stepAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Path planning server started at /path_planning")

    def execute_callback(self, goal):
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
