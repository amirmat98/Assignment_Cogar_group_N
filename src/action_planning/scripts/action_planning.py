#!/usr/bin/env python3

"""
.. module:: action_planning_node
   :platform: Unix
   :synopsis: Handles execution of recipe steps through object perception and path planning.

.. moduleauthor:: Paul Pham Dang

ROS node that coordinates execution of recipe steps:
- Receives steps via an action server
- Uses a perception service to check for required objects
- Sends valid steps to a path planning action server
- Uses a speaker service to handle and announce conflicts

"""

import rospy
import random
import actionlib
from std_msgs.msg import String
from assignments.srv import Speaker, Perception
from assignments.msg import stepAction, stepFeedback, stepResult, stepGoal


class ActionPlanningNode:
    """
    Class representing the action planning ROS node. Handles incoming steps,
    checks object availability, resolves conflicts, and sends commands to a path planning server.
    """

    def __init__(self):
        """
        Initializes the node, sets up action servers and clients, and initializes state variables.
        """
        rospy.init_node('action_planning_node')

        # Action server to receive steps
        self.server = actionlib.SimpleActionServer(
            '/step_action',
            stepAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()

        # Action client to send steps to the path planner
        self.path_planning_client = actionlib.SimpleActionClient('/path_planning', stepAction)
        rospy.loginfo("Waiting for /path_planning action server...")
        self.path_planning_client.wait_for_server()
        rospy.loginfo("/path_planning action server available.")

        # Stack of steps and completed steps
        self.stack_step = []
        self.steps_done = []

        rospy.loginfo("ActionPlanningNode initialized and ready.")

    def execute_callback(self, goal):
        """
        Callback triggered when a step is received on the /step_action action server.

        :param goal: stepGoal object containing the action and ingredient
        :type goal: assignments.msg.stepGoal
        """
        feedback = stepFeedback()
        result = stepResult()
        new_step = f"{goal.action} {goal.ingredient}"

        rospy.loginfo(f"[New Step] {new_step}")
        feedback.status = f"Received step: {new_step}"
        self.server.publish_feedback(feedback)

        # Push the new step to the stack
        self.stack_step.append(new_step)

        while self.stack_step:
            step = self.stack_step[-1]  # Peek at the top
            rospy.loginfo(f"[Stack Peek] {step}")

            try:
                action, ingredient = step.split(maxsplit=1)
            except ValueError:
                rospy.logerr("Invalid step format")
                result.success = False
                self.server.set_aborted(result, "Invalid step format")
                return

            # Check if the object is available using perception
            if not self.check_object_availability(ingredient, feedback):
                feedback.status = f"Object {ingredient} not available, resolving conflict..."
                self.server.publish_feedback(feedback)
                self.announce_conflict(ingredient)

                if not self.resolve_conflict(ingredient):
                    result.success = False
                    self.server.set_aborted(result, "Could not resolve conflict")
                    return
                continue  # Retry step

            # Send the goal to the path planner
            feedback.status = f"Sending step to /path_planning: {step}"
            self.server.publish_feedback(feedback)

            goal_msg = stepGoal()
            goal_msg.action = action
            goal_msg.ingredient = ingredient

            self.path_planning_client.send_goal(goal_msg)
            self.path_planning_client.wait_for_result()
            path_result = self.path_planning_client.get_result()

            if not path_result.success:
                rospy.logerr(f"Path planning failed for step: {step}")
                result.success = False
                self.server.set_aborted(result, "Path planning failed")
                return

            rospy.loginfo(f"Path planning succeeded for step: {step}")

            # Remove from stack and mark as done
            self.steps_done.append(self.stack_step.pop())
            feedback.status = f"Executed step: {step}"
            self.server.publish_feedback(feedback)
            rospy.sleep(0.5)

        result.success = True
        self.server.set_succeeded(result, "All steps completed")

    def check_object_availability(self, obj, feedback):
        """
        Checks if the required object is available using the /perception service.

        :param obj: Object to check
        :type obj: str
        :param feedback: stepFeedback to update the client
        :type feedback: assignments.msg.stepFeedback
        :return: True if object is found, False otherwise
        :rtype: bool
        """
        try:
            rospy.wait_for_service('/perception', timeout=5)
            perception = rospy.ServiceProxy('/perception', Perception)
            res = perception([obj])
            return res.found
        except Exception as e:
            rospy.logerr(f"Perception service error: {e}")
            feedback.status = f"Perception error for object: {obj}"
            self.server.publish_feedback(feedback)
            return False

    def announce_conflict(self, obj):
        """
        Uses the /speaker service to announce that an object is missing.

        :param obj: Name of the missing object
        :type obj: str
        """
        try:
            rospy.wait_for_service('/speaker', timeout=5)
            speaker = rospy.ServiceProxy('/speaker', Speaker)
            speaker(f"Object {obj} not found. Please check.")
        except Exception as e:
            rospy.logerr(f"Speaker service error: {e}")

    def resolve_conflict(self, obj):
        """
        Attempts to resolve a missing object conflict by retrying perception after a delay.

        :param obj: Object to check again
        :type obj: str
        :return: True if object becomes available, False otherwise
        :rtype: bool
        """
        rospy.loginfo(f"Attempting to resolve conflict for: {obj}")
        rospy.sleep(2)  # Simulate manual correction time

        try:
            rospy.wait_for_service('/perception', timeout=5)
            perception = rospy.ServiceProxy('/perception', Perception)
            res = perception([obj])
            return res.found
        except:
            return False


if __name__ == '__main__':
    try:
        ActionPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
