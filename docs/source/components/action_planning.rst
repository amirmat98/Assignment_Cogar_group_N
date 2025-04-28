Action planning based on cooking state and task history
---------------------------------------------------------

.. image:: ../_static/action_planning.png
   :alt: Action planning component
   :align: center
   :width: 100%
   :height: 700px
   :target: #

Every 100 ms the planner starts a new cycle by reading data from various channels and checking for unexpected conditions. If the recipe has failed, it signals an error and reboots the system; otherwise, it updates the best next action. If the recipe is finished, it notifies completion; if not, it updates ongoing actions, commands the robots, and ends the cycle.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

The **update best action** component can be implemented using the **strategy** design pattern. This pattern is a behavioral design pattern that allows selecting an algorithm at runtime. Rather than implementing a single fixed algorithm, the system can choose from a family of predefined strategies based on the context. In our case, depending on the input received by the component, it determines and applies the most appropriate action to execute.
