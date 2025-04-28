======================
Components Diagram
======================

This page provides an overview of the software architecture, detailing each major component and its purpose within the TIAGo-based meal preparation system.

.. image:: _static/components.png
   :alt: Components diagram
   :align: center
   :width: 100%
   :height: 576px

Let's now take a brief look at each component to better understand their application, and analyze which design patterns could be adopted to achieve the best implementation for each of them.

.. toctree::
    :maxdepth: 2
    :hidden:

    Recipe tracking <components/recipe_tracking.rst>
    Action planning <components/action_planning.rst>
    Human command <components/human_command.rst>
    Planner <components/planner.rst>
    Speaker <components/speaker.rst>
    Error handler <components/error_handler.rst>
    Gripper <components/gripper.rst>
    Navigation <components/navigation.rst>
    Motion <components/motion.rst>
    Perception <components/perception.rst>


.. include::    components/recipe_tracking.rst
