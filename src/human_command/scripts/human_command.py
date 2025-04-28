#!/usr/bin/env python3

"""
.. module:: human_command
   :platform: Unix
   :synopsis: Node for interpreting and validating human voice commands during recipe execution.

.. moduleauthor:: Amirmahdi Matin

This module implements a ROS node that listens to voice commands during a cooking task.
It processes and interprets incoming commands, checks them against the current cooking plan,
and determines whether they can be accepted, modified, or rejected.
Feedback is provided to the user through a speaker service, and updates are published to the next step topic.
"""


import rospy
from std_msgs.msg import String
from assignments.srv import Speaker
import random

class HumanCommandNode:
    """
    ROS node that processes human verbal commands and determines their validity
    according to the current cooking plan.
    
    Subscriptions:
        - /voice_command (std_msgs/String): Receives commands spoken by the user.
        - /step (std_msgs/String): Represents the most recent, non-validated step in the recipe.
        - /recipe_control (recipe_tracking/RecipeControl): Tracks the recipe's steps along with their validation statuses.
        - /plan (std_msgs/String): Provides the generated plan based on the latest step and image.

    Publications:
        - /next_step (std_msgs/String): Publishes the next step to execute, based on the processed decision.
    """
    def __init__(self):
        """
        Initializes the node and configures all subscribers and publishers.
        """
        # Initialize the ROS node
        rospy.init_node('human_command_node')
        # Subscribe to /voice_command (String messages)
        self.command_sub = rospy.Subscriber('/voice_command', String, self.command_callback)
        # Subscribe to /step (String messages) to get the current step
        self.step_sub = rospy.Subscriber('/step', String, self.step_callback)
        # Subscribe to /recipe_control (String messages) to get the status
        self.recipe_sub = rospy.Subscriber('/recipe', String, self.recipe_callback)
        # Subscribe to /plan (String messages) to get the current step
        self.plan_sub = rospy.Subscriber('/plan', String, self.plan_callback)
        # Publisher for /command_status topic
        self.next_step_pub = rospy.Publisher('/next_step', String, queue_size=10)
        self.current_step = None
        self.recipe_status = None
        self.plan = None
        rospy.loginfo("HumanCommandNode is up and subscribed to /voice_command, /plan, /recipe, and /step topics.")

    def command_callback(self, msg):
        """
        Handles incoming messages from the /voice_command topic.

        :param msg: String message containing the received voice command.
        :type msg: std_msgs.msg.String
        """
        
        rospy.loginfo(f"Received command: {msg.data}")
        command = msg.data.lower()
        if self.current_step is None:
            status_msg = "Command rejected: No current plan step available."
            self.notify_user(status_msg)
            rospy.loginfo(f"Published command status: {status_msg}")
            return
        # Simulate command interpretation
        interpreted_command = self.interpret_command(command)

        if interpreted_command and self.is_command_valid(interpreted_command): # command recognized and interpreted
            status_msg = f"Command accepted: {interpreted_command}"
            self.notify_user(status_msg)
            step_msg = interpreted_command

        elif self.conflict_resolution(command):
            self.modify_plan(interpreted_command)
            status_msg = f"Command accepted: {interpreted_command}"
            self.notify_user(status_msg)
            step_msg = interpreted_command
                
        else:
            status_msg = f"Command rejected due to conflict: {interpreted_command}"
            self.notify_user(status_msg)
            step_msg = self.current_step

        self.next_step_pub.publish(step_msg)
        rospy.loginfo(f"Published command status: {status_msg}")


    def step_callback(self, msg):
        """
        Handles incoming messages from the /step topic to update the current recipe step.

        :param msg: String message containing the latest unvalidated step.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received current step: {msg.data}")
        self.current_step = msg.data

    def recipe_callback(self, msg):
        """
        Processes messages from the /recipe topic to update the recipe status.

        :param msg: String message containing the current validation status of the recipe steps.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received recipe status: {msg.data}")
        self.recipe_status = msg.data

    def plan_callback(self, msg):
        """
        Handles incoming messages from the /plan topic to update the current execution plan.

        :param msg: String message containing the updated plan information.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received plan : {msg.data}")
        self.plan = msg.data

    def is_command_valid(self, command):
        """
        Check if the command is valid based on the current plan.
        """
        return random.random() > 0.5  # 50% chance of valid command


    def modify_plan(self, command):
        """
        Modify the plan based on the valid command.
        """
        rospy.loginfo(f"Plan modified based on command: {command}")


    def notify_user(self, message):
        """
        Notify the user using the speaker service.
        """
        try:
            rospy.wait_for_service('/speaker', timeout=5)
            speaker_service = rospy.ServiceProxy('/speaker', Speaker)
            response = speaker_service(message)
            rospy.loginfo(f"Speaker service response: {response.success}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call speaker service: {e}")



    def conflict_resolution(self):
        """
        Simulate conflict resolution using a random value.
        """
        return random.random() < 0.1  # 10% chance of conflict


    def interpret_command(self, command):
        """
        Simulate command interpretation.
        """
        return command


if __name__ == '__main__':
    try:
        HumanCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
