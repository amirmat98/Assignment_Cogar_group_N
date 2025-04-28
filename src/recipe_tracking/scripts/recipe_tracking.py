#!/usr/bin/env python3

"""
.. module:: recipe_tracking
   :platform: Unix
   :synopsis: Node for managing and tracking recipe execution steps and validation status.

.. moduleauthor:: Amirmahdi Matin

This module implements a ROS node that tracks the progress of a cooking recipe.
It listens for the validation status of each recipe step, updates the list of completed steps,
and publishes the current step to be performed.

Features:
- Reads and parses a recipe from a text file.
- Validates the recipe structure and contents.
- Monitors validation of each step via the `/goal_state` topic.
- Publishes the current unvalidated step on the `/step` topic.
- (Optional) Publishes the full recipe validation status (currently commented out).

"""
import rospy # type: ignore
import random
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
    
    Attributes:
        current_step (str): It is the latest step of the recipe which is not validated yet.
        recipe_checklist (dictionary): list of the steps and their success status.

    """


    def __init__(self):
        """
        Initialize the Node, (set up subscriber and publishers).
        """
        rospy.init_node('recipe_tracking_node')
        
        # Initialize the cooking process
        self.recipe_txt = rospy.get_param("recipe_txt")
        self.recipe_checklist, self.current_step = self.initialize_recipe(self.recipe_txt)
        
        #Check the validity of the recipe
        if not self.is_a_valid_recipe(self.recipe_checklist) :
            rospy.logerr("Recipe is not valid.")
        else :
            rospy.loginfo("Recipe is initialized and valid.")
        	
            # Subscribe to /goal_state (Bool messages)
            self.goal_sub = rospy.Subscriber('/goal_state', Bool, self.goal_callback)
            
            # Publisher for /step topic
            self.step_pub = rospy.Publisher('/step', String, queue_size=10)
        
            # Publisher for /recipe_control topic
            #self.rec_control_pub = rospy.Publisher('/recipe_control', RecipeControl, queue_size=10)
		
            rospy.loginfo("TrackNode initialized and listening to /goal_state")

    def initialize_recipe(self, msg):
        recipe_checklist = {
                            "action": [],
                            "ingredient": [],
                            "success": []
                            }
        with open(msg,"r") as f:
            for step in f:
                action = None
                ingredient = None
                success = None
                step_msg = [s.strip() for s in step.strip().split(",")]
                if len(step_msg) > 0:
                    action = step_msg[0]
                if len(step_msg) > 1:
                    ingredient = step_msg[1]
                if len(step_msg) > 2: # notify if the recipe has too many parameters
                    success = True
                else:
                    success = False
                
                recipe_checklist["action"].append(action)
                recipe_checklist["ingredient"].append(ingredient)
                recipe_checklist["success"].append(success)
                
                current_step = {"action":recipe_checklist["action"][0], "ingredient":recipe_checklist["ingredient"][0], "success":recipe_checklist["success"][0]}

            return recipe_checklist, current_step
        
    def is_a_valid_recipe(self, recipe):
        """
        Check if the recipe is valid.
        
        :param recipe: recipe_checklist is the list of the steps to follow and their success status
        :type recipe: dictionary of (action (str), ingredient (str), success (str))
        """
        if recipe["action"][-1] != "end":
            rospy.logerr('Recipe does not end with "end" step.')
            return False
        for action, ingredient, success in zip(recipe["action"], recipe["ingredient"], recipe["success"]):
            if success != False:
                rospy.logerr("Too many parameters were written for one step.")
                return False
            if action == "pouring":
                if ingredient is None: # pouring has exactly one ingredient
                    rospy.logerr("Pouring action requires exactly one ingredient.")
                    return False
            elif action == "mixing": 
                if ingredient is not None: # cutting has no ingredient because it always mixes the same pot
                    rospy.logerr("Mixing action requires exactly no ingredient.")
                    return False
            elif action == "cutting":
                if ingredient is None: # cutting has exactly one ingredient
                    rospy.logerr("Cutting action requires exactly one ingredient.")
                    return False
            elif action!="end" : # unknown action (note that ingredients might be unknown)
                rospy.logerr("Recipe asking unknown action.")
                return False
        
        return True
        
        
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
