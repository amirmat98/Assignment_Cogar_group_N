Recipe tracking and execution history
--------------------------------------

.. image:: ../_static/recipe_tracking.png
   :alt: Recipe tracking component
   :align: center
   :width: 100%
   :height: 650px
   :target: #

The **recipe tracking and execution history** component is a subsystem that incorporates other components and artifacts which together are responsible for tracking the recipe to be cooked and its progress status through connections with the :doc:`action planning <action_planning>` e :doc:`human command <human_command>`. Within this subsystem, all components are connected to their corresponding artifacts, which represent tangible resources within the system. Each artifact is updated based on the activation of its component, and some artifacts are also used to trigger other internal components. 

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

The **recipe** component must be implemented using the **singleton** pattern, since the robot can cook only one recipe at a time. Therefore, multiple instances of a recipe cannot exist simultaneously, as that would imply the robot could cook multiple recipes at the same time.
