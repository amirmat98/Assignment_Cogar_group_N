#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from assignments.srv import Speaker
import random

class HumanCommandNode:
    """
    ROS node that listens to verbal commands and evaluates their validity
    based on the current cooking plan.
    
    Subscribes to:
        - /voice_command (std_msgs/String): Command from human.
        - /step (std_msgs/String): Step of the recipe input.
        
    Publishes to:
        - /next_step (std_msgs/String): Next step based on the decision made.
    """

    def __init__(self):
        """
        Initialize the Node, set up subscribers and publisher.
        """
        rospy.init_node('human_command_node')

        # Subscribe to /voice_command (String messages)
        self.command_sub = rospy.Subscriber('/voice_command', String, self.command_callback)

        # Subscribe to /step (String messages) to get the current step
        self.step_sub = rospy.Subscriber('/step', String, self.step_callback)

        # Publisher for /command_status topic
        self.next_step_pub = rospy.Publisher('/next_step', String, queue_size=10)

        # Store the latest plan step
        self.current_step = None

        rospy.loginfo("HumanCommandNode initialized and listening to /voice_command and /plan")


    def command_callback(self, msg):
        """
        Callback for /voice_command topic.
        
        :param msg: Incoming String message containing the vocal command.
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
        Callback for /plan topic to update the current step.

        :param msg: Incoming String message containing the current step.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received plan step: {msg.data}")
        self.current_step = msg.data


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
