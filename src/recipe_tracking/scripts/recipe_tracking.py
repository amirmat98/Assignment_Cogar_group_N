#!/usr/bin/env python3

"""
.. module: recipe_tracking
   :platform: unix
   :synopsis: Python node to be described.

.. moduleauthor:: Bertille Beaujean

Python implementation of the architecture of the component "Recipe Tracking and Execution History", according to what is described in its behavioural diagrams.

"""

import rospy
import actionlib
from std_msgs.msg import String, Bool
from assignments.msg import stepAction, stepFeedback, stepResult, stepGoal
import os

class RecipeTrackingNode:
    """
    ROS node for recipe tracking and execution history.
    It implements the action client to send the goal step of the recipe to the Action Planner and get the success state of the step as result.

    Uses the action client \step_action.
    
    Attributes:
        current_step (str): It is the latest step of the recipe which is not validated yet.
        recipe_checklist (dictionary): list of the steps and their success status.

    """


    def __init__(self):
        """
        Initialize the Node, (set up action client).
        """
        rospy.init_node('recipe_tracking_node')
        
        # Get the current file's directory (where your script is located)
        current_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Build the path relative to current_dir
        default_recipe_path = os.path.abspath(os.path.join(current_dir, "../../assignments/recipes/recipe.txt"))
        
        # Initialize the cooking process
        self.recipe_txt = rospy.get_param("~recipe_txt", default_recipe_path)
        self.recipe_checklist, self.current_step = self.initialize_recipe(self.recipe_txt)
        
        #Check the validity of the recipe
        if not self.is_a_valid_recipe(self.recipe_checklist) :
            rospy.logerr("Recipe is not valid.")
        else :
            rospy.loginfo("Recipe is initialized and valid.")
            
            # Create the action client
            self.step_client = actionlib.SimpleActionClient('/step_action', stepAction)
            
            rospy.loginfo("Waiting for /step_action action server...")
            self.step_client.wait_for_server()
            rospy.loginfo("/step_action action server available.")
            
            rospy.loginfo("RecipeTrackingNode initialized and ready.")

    def initialize_recipe(self, msg):
        """
        Creates the checklist of steps from the .txt version of the recipe, then initializes the current step of the recipe.
        :param msg: recipe as a sequence of steps
        :type msg: .txt (ros parameter)
        """
        recipe_checklist = {
                            "action": [],
                            "ingredient": [],
                            "success": []
                            }
        with open(msg,"r") as f:
            for step in f:
                action, ingredient, success = None, None, None
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
        Check if the recipe is valid. A recipe is valid if it ends with the "end" step and if every other step is made of one action (cutting, pouring, mixing) and an ingredient.
        
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
                if ingredient != "all": # mixing has ingredient "all" because it mixes "all" in the cooking pot
                    rospy.logerr('Mixing action requires exactly "all" ingredient.')
                    return False
            elif action == "cutting":
                if ingredient is None: # cutting has exactly one ingredient
                    rospy.logerr("Cutting action requires exactly one ingredient.")
                    return False
            elif action!="end" : # unknown action (note that ingredients might be unknown)
                rospy.logerr(f"Recipe asking unknown action: {action}.")
                return False
        
        return True
        
    def step_tracking(self):
        """
        Tracks the cooking process with an action client to go from step to step in the recipe, send the current step it to the Action Planner and update the execution history when the Action Planner informs of the success of a step.
        """
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and self.current_step["action"] !="end" :
            # Send the current step to ActionPlannerNode
            goal = stepGoal(action=self.current_step["action"], ingredient=self.current_step["ingredient"])
            rospy.loginfo(f'Sending step: {self.current_step["action"]} {self.current_step["ingredient"]}')
            self.step_client.send_goal(goal)
            
            self.step_client.wait_for_result()
            result = self.step_client.get_result()
            
            if result is not None and result.success:
                # Update the execution history 
                step_idx = next(i for i, s in enumerate(self.recipe_checklist["success"]) if not s) # Looks for the 1st step of the recipe which success is False (=not changed, would also work if [False, True, Abort] possibilities)
                self.recipe_checklist["success"][step_idx] = True
                rospy.loginfo(f'Step "{self.current_step["action"]} {self.current_step["ingredient"]}" validated in checklist.')
                
                # Go to the next step
                self.current_step["action"] = self.recipe_checklist["action"][step_idx+1]
                self.current_step["ingredient"] = self.recipe_checklist["ingredient"][step_idx+1]
                rospy.loginfo("Processing to recipe's next step.")
            else:
                rospy.logwarn(f'Step "{self.current_step["action"]} {self.current_step["ingredient"]}" not successful. Waiting...')
            rate.sleep()
        rospy.loginfo("Recipe over.")


if __name__ == '__main__':
    try:
        node = RecipeTrackingNode()
        node.step_tracking()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
