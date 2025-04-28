Navigation
--------------

.. image:: ../_static/navigation.png
   :alt: Navigation component
   :align: center
   :width: 100%
   :height: 576px
   :target: #

The **navigation** component is used to create and manage the environment map through the connected sensors, saving it via the artifact linked to the **SLAM** component.

The **trajectory planning** component is responsible for generating the path that the robot must follow based on the actions received from the :doc:`planner<planner>` module. To do this, it requires data from other modules, such as :doc:`perception<perception>` for obstacle detection and tracking, :doc:`motion<motion>` for wheel encoder data, and :doc:`gripper<gripper>` for gripper encoder feedback. If the planner's command involves moving the robot base, the trajectory planner will generate an optimal path and send velocity commands to the wheels. If, instead, the command involves moving the robotic arm, the component will generate a sequence of actions for the arm to reach the target goal.
| With the integration of **LIDAR** and **SONAR** sensors, the component can build and continuously update a 360-degree map of the environment.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

To connect the **SLAM** component to the two internal sensors, LIDAR and SONAR, it is necessary to implement them using the **adapter** pattern, in order to make the exchanged data compatible and allow SLAM to manage them properly.
