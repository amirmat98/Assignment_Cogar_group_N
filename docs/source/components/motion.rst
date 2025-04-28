Motion
--------------

.. image:: ../_static/motion.png
   :alt: Motion component
   :align: center
   :width: 100%
   :height: 550px
   :target: #

The **motion** component is used to manage, as the name suggests, the robot's navigation through the **encoders**, **controllers**, and **motors** of both wheels. The **controllers** components will receive the speed signal to be set for the wheels from the module **trajectory planning** within the :doc:`navigation<navigation>`.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

As previously mentioned for the :doc:`gripper<gripper>`, the **motor controllers** in the motion component will also be implemented using the **singleton** pattern, since multiple instances of the same controller are not allowed.

To connect the **encoders** to their respective **controllers**, the **adapter** pattern will be required to resolve the mismatch between the exchanged data.
