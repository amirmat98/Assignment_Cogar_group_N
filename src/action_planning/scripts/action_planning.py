#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import random
from assignments.srv import Speaker

class ActionPlanningNode:
    """
    ROS node that listens to the step commands of the recipe and RGB images of the robot camera.
    It publishes the action planned.

    Subscribes to:
        - /step (std_msgs/String): Step of the recipe input.
        - /image_rgb (sensor_msgs/Image): RGB image input.

    Publishes to:
        - /plan (std_msgs/String): Generated plan based on the latest step and image.

    Attributes:
        latest_step (str): Latest received step command.
    """

    def __init__(self):
        """
        Initialize the Node, set up subscribers and publisher.
        """
        rospy.init_node('action_planning_node')

        # Subscribe to /step (String messages)
        self.step_sub = rospy.Subscriber('/step', String, self.step_callback)

        # Subscribe to /image_rgb (Image messages)
        self.image_sub = rospy.Subscriber('/image_rgb', Image, self.image_callback)

        # Publisher for /plan topic
        self.plan_pub = rospy.Publisher('/plan', String, queue_size=10)

        # Store the latest messages
        self.latest_step = None
        self.latest_image = None

        rospy.loginfo("PlanNode initialized and listening to /step and /image_rgb")

    def step_callback(self, msg):
        """
        Callback for /step topic.

        :param msg: Incoming String message containing the step command.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received step: {msg.data}")
        self.latest_step = msg.data
        self.try_publish_plan()

    def image_callback(self, msg):
        """
        Callback for /image_rgb topic.

        :param msg: Incoming Image message.
        :type msg: sensor_msgs.msg.Image
        """
        rospy.loginfo("Received image message")
        self.latest_image = msg

    def try_publish_plan(self):
        """
        Attempt to publish a plan if both a step and an image have been received.
        """
        if self.latest_step is not None and self.latest_image is not None:
            # Dummy plan generation logic
            plan_msg = String()
            
            # Generate a random number between 0 and 1
            random_value = random.random()

            if random_value > 0.1:
                plan_msg.data = (
                    "Plan to do : ... ")
            else:
                plan_msg.data = "Error: Plan is impossible to generate based on the current inputs..."
                # Log the error
                rospy.logerr("Plan generation failed")
                # use the speaker service to notify the user
                try:
                    rospy.wait_for_service('/speaker', timeout=5)
                    speaker_service = rospy.ServiceProxy('/speaker', Speaker)
                    speaker_service("Plan generation failed")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to call speaker service: {e}")
                
            # Publish the generated plan
            self.plan_pub.publish(plan_msg)
            rospy.loginfo(f"Published plan: {plan_msg.data}")

if __name__ == '__main__':
    try:
        ActionPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
