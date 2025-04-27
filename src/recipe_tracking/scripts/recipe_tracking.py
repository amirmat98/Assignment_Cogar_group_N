#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

class RecipeTrackingNode:
    """
    ROS node for recipe tracking and execution history.
    It listens to the state of validation of the current goal state.
    It publishes the current step of the recipe and the list of the steps of the recipe with their associated validation state (validated or not).

    Subscribes to:
        - /goal_state (std_msgs/Bool): Indicates whether the current goal step is validated or not.

    Publishes to:
        - /step (std_msgs/String): Step of the recipe. It is the latest non validated step.
        - /recipe_control (recipe_tracking/RecipeControl): list of the steps of the recipe with their associated validation status (validated or not).
    
    Attributes:
        - current_step (str): It is the latest step of the recipe which is not validated yet.
        - recipe_checklist (RecipeControl): list of the validation status of each step.

    """


    def __init__(self):
        """
        Initialize the Node, (set up subscriber and publishers).
        """
        rospy.init_node('recipe_tracking_node')
        
        # Initialize the cooking process
        #self.recipe_checklist = RecipeControl() # Need to initalize with the recipe at some point
        recipe_steps = rospy.get_param("recipe",["cut", "cut", "mix", "end"])
        self.recipe_checklist = { "steps": recipe_steps, "status": [False] * len (recipe_steps) }
        self.current_step = self.recipe_checklist["steps"][0]
        
        
        
        # Subscribe to /goal_state (Bool messages)
        self.goal_sub = rospy.Subscriber('/goal_state', Bool, self.goal_callback)
        
        # Publisher for /step topic
        self.step_pub = rospy.Publisher('/step', String, queue_size=10)
        
        # Publisher for /recipe_control topic
        #self.rec_control_pub = rospy.Publisher('/recipe_control', RecipeControl, queue_size=10)
        
        rospy.loginfo("TrackNode initialized and listening to /goal_state")


    def goal_callback(self, msg):
        """
        Callback for /goal_state topic. Updates the topics /step and /recipe_control according to the value of "/goal_state".
        
        :param msg: Validation or not of the current step of the recipe.
        :type msg: std_msgs.msg.Bool
        """
        rospy.loginfo(f"Received state of the step: {msg.data}")
        if msg.data:
            for i, (step_name, is_validated) in enumerate(zip(self.recipe_checklist["steps"], self.recipe_checklist["status"])):
                if step_name == self.current_step:
                    # Update the step history
                    self.recipe_checklist["status"][i] = True
                    rospy.loginfo(f"Step {self.current_step} is validated.")
                    
                    # Update the current step
                    if i==len(self.recipe_checklist["status"])-1:
                        self.current_step = "end"
                    else:
                        self.current_step = self.recipe_checklist["steps"][i+1]
                    rospy.loginfo(f"New goal step is {self.current_step}")
                    break
        
        # Publish the current step
        self.step_pub.publish(String(data=self.current_step))
        
        # Publish the step history
        #recipe_msg = RecipeControl()
        #recipe_msg.steps = self.recipe_checklist.steps
        #recipe_msg.status = self.recipe_checklist.status
        #self.rec_control_pub.publish(recipe_msg)
        


if __name__ == '__main__':
    try:
        RecipeTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
