.. Assignment_Cogar_Group_N documentation master file, created by
   sphinx-quickstart on Sun Apr 27 00:53:04 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Assignment_Cogar_Group_N's documentation!
====================================================

- AmirMahdi Matin, 5884715



Assignment: Topic 3
=====================================

The TIAGo robot is deployed in a domestic kitchen environment to assist elderly individuals with cooking-related tasks. The robotic system is equipped with a comprehensive set of sensory and communication tools:

- RGB‑D Camera  
- LiDAR  
- SONAR  
- Force Sensors  
- Microphones  
- Speakers  

System Capabilities
-------------------

**Ingredient Retrieval**  
Plan and execute obstacle-free, efficient trajectories using visual input and real-time mapping to locate and retrieve ingredients.

**Collaborative Cooking Execution**  
Execute structured cooking actions such as mixing, cutting, and pouring. The robot maintains a dynamic internal model of recipe progress and updates it during task execution.

**Intelligent Action Planning**  
Determine the next step based on current task state and execution history. Incorporate verbal input and resolve conflicts between planned actions and user instructions.

**Voice Command Interpretation**  
Continuously listen for verbal input. Accurately interpret spoken commands and decide whether to accept, override, or reject based on current context.

**Object Recognition and Localization**  
Identify and localize tools and ingredients within the workspace using the onboard vision system.

**Completion and User Notification**  
Confirm that all recipe steps are completed and notify the user via speech when the meal is ready.



.. toctree::
   :maxdepth: 2
   :caption: Contents:
   
   Architecture <architecture.rst>
   Behevioural <behavioural.rst>
   Test <test.rst>




