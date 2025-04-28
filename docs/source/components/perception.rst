Perception
--------------

.. image:: ../_static/perception.png
   :alt: Perception component
   :align: center
   :width: 400px
   :height: 500px
   :target: #

The **perception** component includes a module for the **camera**, which collects visual information from the robot. Through the **object recognition** and **object tracking** components, it analyzes the images to recognize and track objects, exposing these functionalities to various components that require them, such as the :doc:`navigation<navigation>`.

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

Similarly, the **camera** component will be implemented using the **singleton** pattern, since, as previously explained, multiple instances of the same component cannot exist: the physical device is unique, and having multiple instances would lead to consistency issues.

Additionally, in order to communicate with the component responsible for **object recognition**, the **camera** component must use the **adapter** pattern to ensure data consistency during exchanges.
