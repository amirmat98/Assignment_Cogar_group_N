======================
Architecture Diagram
======================

This page provides an overview of the software architecture, detailing each major component and its purpose within the TIAGo-based meal preparation system.

.. image:: ../../Diagrams/component_diagram.png
   :alt: Components diagram
   :align: center
   :width: 100%
   :height: 576px

Below we briefly examine every component, explaining its purpose and noting suitable design patterns that can lead to a clean, maintainable implementation.

The overall structure of the system is illustrated in the following component diagram.  
The robot carries several sensors and services—an RGB‑D camera, LiDAR, SONAR, force sensors, microphones, and speakers. The architecture lets these parts collaborate seamlessly so the robot can prepare meals.

The workflow starts with the **Recipe Tracking** and the **Human Command Monitoring**.  
* **Recipe Tracking** stores the internal representation of the recipe and logs the sequence of actions, guaranteeing each step is respected.  
* **Human Command Monitoring** captures spoken instructions via the microphones and checks whether they align with the current plan, resolving conflicts so the robot can obey the user without jeopardising the task.

Both subsystems forward information to the **Action Planning Subsystem**, which selects the next step according to the current state and past actions. The planner adapts dynamically to unexpected situations or fresh user commands.

Once a new action is chosen, the planner relies on the **Perception Subsystem** to carry it out. Perception uses the RGB‑D camera to build a real‑time map and recognise objects. These data are essential to the **Navigation Subsystem**, which computes safe, efficient paths using LiDAR for precision and SONAR for range sensing.

Finally, the **Robot Subsystem** executes the plan with force‑sensing arms and joints that manipulate tools and ingredients precisely. In concert, these subsystems allow the robot to move, perceive, act, and interact with people throughout the meal‑preparation process.

-------------------------------------------------------------
Design Patterns
-------------------------------------------------------------

.. list-table::
   :widths: 20 20 40
   :header-rows: 1

   * - Subsystem
     - Design Pattern(s)
     - Purpose
   * - Recipe Tracking
     - Singleton, Observer
     - Centralised recipe state
   * - Human Command
     - Command, Adapter, Strategy
     - Modular voice‑command processing
   * - Action Planning
     - Strategy, Mediator
     - Real‑time plan adaptation
   * - Perception
     - Adapter, Singleton
     - Uniform sensor interface
   * - Robot State
     - State, Mediator
     - Dynamic robot behaviour
   * - Navigation
     - Adapter
     - Reconcile LiDAR and SONAR data
   * - Motion / Gripper / Speaker
     - Singleton, Adapter
     - Hardware control interfaces


-----------------
Recipe tracking
-----------------

The **recipe tracking and execution history** subsystem bundles components and artefacts that monitor the active recipe and its progress. It communicates with **action planning** and **human command** through dedicated ports.  
Each artefact is updated by its component, and some artefacts in turn trigger internal behaviour.

Two ports remain unconnected internally; they act as entry and exit points for the error handler, which oversees recovery procedures.

.. rubric:: Implementation through patterns

Because the robot can handle only one recipe at a time, implement the **recipe** component as a **singleton**.

-----------------
Action planning
-----------------

Every 100 ms the planner runs a new cycle:

#. Read data from the various channels.  
#. Detect failures; if one occurs, signal an error and reboot.  
#. If the recipe has finished, announce completion; otherwise update the best next action.  
#. Dispatch commands to the robot and conclude the cycle.

As above, two ports are reserved for the error handler.

.. rubric:: Implementation through patterns

Implement the **update best action** component with the **strategy** pattern, selecting among multiple planning algorithms according to the current context.


-----------------
Human command
-----------------

The planner continuously listens for speech via the **Microphones** component, producing an *Audio Track* variable and deciding whether interaction occurred.  
* When no interaction is detected the loop ends.  
* Otherwise, the **Interpreter** parses the message.  
* Invalid sentences trigger a prompt to repeat.  
* A valid sentence is checked to determine whether it is the first interaction and whether ingredients have been shown.  
* Depending on the outcome the system requests a recipe, requests an ingredient list, and tries to connect to Wi‑Fi.  
* Failure to connect raises an error.  
* Success leads to an online recipe search or the suggestion of recipes based on available ingredients.

During execution the system accepts runtime voice commands, validates them, updates the recipe history, and responds verbally. The **High Level Action** component keeps the robot’s state in sync with the latest recipe info.

Again, two ports are delegated to the error handler.

.. rubric:: Implementation through patterns

* **Wi‑Fi connection** manager → **singleton** (only one network interface exists).  
* **Microphone to Interpreter** bridge → **adapter**.  
* Post‑interpretation decision flow → **strategy**.

-----------------
Perception
-----------------

The **perception** component includes a module for the **camera**, which collects visual information from the robot. Through the **object recognition** and **object tracking** components, it analyzes the images to recognize and track objects, exposing these functionalities to various components that require them, such as the **navigation**.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the error handler component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

Similarly, the **camera** component will be implemented using the **singleton** pattern, since, as previously explained, multiple instances of the same component cannot exist: the physical device is unique, and having multiple instances would lead to consistency issues.

Additionally, in order to communicate with the component responsible for **object recognition**, the **camera** component must use the **adapter** pattern to ensure data consistency during exchanges.

The **planner high level** component is responsible for translating the actions received from the **actino_planning** module into actual commands for the robot. It manages the **navigation** component, which, based on the known map and the task to be performed, determines the optimal path for the **motion** module to follow or the specific actions to be executed by the **gripper**.

Additionally, there is an artifact that keeps track of the **robot's current state** for example, states like cutting, cooking, grabbing, and so on. The robot's state dynamically updates according to the current situation.

Finally, there is a component that monitors the robot’s **battery level**. This component is responsible for managing the charge status and the error handler will send a vocal warning signal when the battery is running low.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the error handler component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

The **state** pattern is a behavioral design pattern that allows an object to change its behavior based on its internal state. It appears as if the object has changed its class. 

To effectively implement the **robot state** component, we can apply the **state** pattern. This is because the robot can be in various states, such as `noRecipe`, `cooking`, `cutting`, or `error`, each encapsulating its own transition logic. This approach helps avoid long chains of `if/else` statements. Moreover, since the robot’s behavior must change dynamically at runtime based on its current state, this pattern is particularly well suited for this purpose.

Since this component plays a key role by centralizing the management of hardware components responsible for movement, navigation, and the gripper, and because it dynamically alters the program's behavior at runtime, it could also be implemented using the **mediator** design pattern instead of the **state** pattern. The Mediator pattern is particularly suitable in cases like this, where a single component centralizes and manages complex interactions between multiple parts of the system.


-----------------
Navigation
-----------------

The **navigation** subsystem builds and maintains the environment map through connected sensors, persisting it via the **SLAM** artefact.

The **trajectory planning** component:

* Generates a path for the robot base or arm based on commands from the **planner**,
* Consults **perception** for obstacle data,
* Uses **motion** and **gripper** encoder feedback,
* Employs **LiDAR** and **SONAR** to maintain a 360‑degree view of the surroundings.

Two ports are dedicated to the error handler.

.. rubric:: Implementation through patterns

Connect **SLAM** to **LiDAR** and **SONAR** with **adapter** objects to reconcile differing data formats.

-----------------
Robot
-----------------

**Motion** controls navigation through wheel **encoders**, **controllers**, and **motors**. Controllers receive speed set‑points from **trajectory planning** inside **navigation**.

Two ports belong to the error handler.

.. rubric:: Implementation through patterns

* **Motor controllers** → **singleton**.  
* **Encoder to controller** link → **adapter**.

The **speaker** component handles verbal output, replaying messages generated by other modules.

.. rubric:: Implementation through patterns

* **Speaker** → **singleton**.

The **gripper** manages the end effector:

* **Encoder**, **controller**, **motor**, and **force sensor**,
* Closed‑loop grasping monitored via **perception**,
* Commands received from **trajectory planning**.

Two ports lead to the error handler.

.. rubric:: Implementation through patterns

* **Motor controller** → **singleton**.  
* **Encoder** and **force sensor** interfaces → **adapter**.





.. toctree::
    :maxdepth: 2

