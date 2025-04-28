Gripper
--------------

.. image:: ../_static/gripper.png
   :alt: Gripper component
   :align: center
   :width: 100%
   :height: 576px
   :target: #

The **gripper** component manages the end part of the robotic arm. It includes the components necessary for movement, namely the **encoder**, **controller**, and **motor**, along with a **force sensor** connected to the controller.

This component uses the :doc:`perception<perception>` module to track the movement of the object and monitor its position within the gripper.
| Actions are issued to the controller through the **trajectory planning** component inside the :doc:`navigation<navigation>`.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

The **motor controller** is implemented using the **singleton** pattern, as there must be at most one instance per controller; otherwise, consistency issues could arise, since there is only one physical controller.

Furthermore, to connect the **encoder** to the **controller**, the **adapter** pattern must be used to handle the mismatch between their different data types. Similarly, the **force sensor**, which the controller uses to regulate the gripper's force, is also integrated using the **adapter** pattern, as it exchanges a different type of data compared to the controller. The use of the adapter allows the controller to properly handle the incoming data.
