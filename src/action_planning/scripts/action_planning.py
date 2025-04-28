#!/usr/bin/env python3

"""
.. module:: action_planning_node
   :platform: Unix
   :synopsis: Coordinates recipe step execution by integrating object detection and path planning.

.. moduleauthor:: Amirmahdi Matin

This ROS node manages the execution flow of recipe steps by:
- Accepting steps through an action server
- Verifying object availability using a perception service
- Forwarding executable steps to a path planning action server
- Announcing missing objects and conflicts via a speaker service
"""
# Import necessary libraries

import rospy
import random
import actionlib
from std_msgs.msg import String
from assignments.srv import Speaker, Perception
from assignments.msg import stepAction, stepFeedback, stepResult, stepGoal


class ActionPlanningNode:
    """
    Defines the action planning ROS node, which manages incoming steps by checking object presence,
    resolving conflicts when necessary, and sending tasks to the path planning server for execution.
    """

    def __init__(self):
        """
        Sets up the ActionPlanningNode by initializing the ROS environment,
        configuring action servers and clients, and preparing internal data structures
        for tracking pending and completed steps.
        """
        # Initialize the ROS node
        rospy.init_node('action_planning_node')
        # Action server to receive steps
        self.server = actionlib.SimpleActionServer('/step_action', stepAction, execute_cb=self.execute_callback, auto_start=False)
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
        Callback function invoked when a new step is received through the /step_action action server.

        :param goal: A stepGoal message containing the specified action and associated ingredient.
        :type goal: assignments.msg.stepGoal
        """
        # Initialize feedback and result messages
        feedback = stepFeedback()
        result = stepResult()
        new_step = f"{goal.action} {goal.ingredient}"
        rospy.loginfo(f"[New step received] {new_step}")
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
        Verifies the availability of the specified object through the /perception service.

        :param obj: The name of the object to be verified.
        :type obj: str
        :param feedback: stepFeedback message used to update the client about the check status.
        :type feedback: assignments.msg.stepFeedback
        :return: True if the object is detected; otherwise, False.
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
        Utilizes the /speaker service to notify that a required object is not found.

        :param obj: The name of the missing object.
        :type obj: str
        """
        rospy.loginfo(f"Announcing conflict for object: {obj}")
        try:
            rospy.wait_for_service('/speaker', timeout=5)
            speaker = rospy.ServiceProxy('/speaker', Speaker)
            speaker(f"Object {obj} not found. Please check.")
        except Exception as e:
            rospy.logerr(f"Speaker service error: {e}")

    def resolve_conflict(self, obj):
        """
        Tries to resolve a missing object issue by reattempting object detection after a short delay.

        :param obj: The object to be rechecked.
        :type obj: str
        :return: True if the object is found after retrying; False otherwise.
        :rtype: bool
        """
        # Simulate manual correction time
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
