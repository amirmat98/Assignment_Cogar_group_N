Planner high level
---------------------

.. image:: ../_static/planner.png
   :alt: Planner high level component
   :align: center
   :width: 100%
   :height: 576px
   :target: #

The **planner high level** component is responsible for translating the actions received from the :doc:`action planning<action_planning>` module into actual commands for the robot. It manages the :doc:`navigation<navigation>` component, which, based on the known map and the task to be performed, determines the optimal path for the :doc:`motion<motion>` module to follow or the specific actions to be executed by the :doc:`Gripper<gripper>`.

Additionally, there is an artifact that keeps track of the **robot's current state** for example, states like cutting, cooking, grabbing, and so on. The robot's state dynamically updates according to the current situation.

Finally, there is a component that monitors the robot’s **battery level**. This component is responsible for managing the charge status and the :doc:`error handler <error_handler>` will send a vocal warning signal when the battery is running low.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

The **state** pattern is a behavioral design pattern that allows an object to change its behavior based on its internal state. It appears as if the object has changed its class. 

To effectively implement the **robot state** component, we can apply the **state** pattern. This is because the robot can be in various states, such as `noRecipe`, `cooking`, `cutting`, or `error`, each encapsulating its own transition logic. This approach helps avoid long chains of `if/else` statements. Moreover, since the robot’s behavior must change dynamically at runtime based on its current state, this pattern is particularly well suited for this purpose.

Since this component plays a key role by centralizing the management of hardware components responsible for movement, navigation, and the gripper, and because it dynamically alters the program's behavior at runtime, it could also be implemented using the **mediator** design pattern instead of the **state** pattern. The Mediator pattern is particularly suitable in cases like this, where a single component centralizes and manages complex interactions between multiple parts of the system.