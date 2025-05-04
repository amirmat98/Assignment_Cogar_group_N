Behavioural diagram
====================

This page captures the run‑time behaviour of the TIAGo meal‑preparation system. It shows how data objects travel through always‑on channels and how the main control loops react to user interaction, recipe progress, and unexpected conditions.


.. image:: ../../Diagrams/action_planning.png
   :alt: action_planning
   :align: center
   :width: 100%
   :height: 576px
   
.. image:: ../../Diagrams/recipe_tracking.png
   :alt: recipe_tracking
   :align: center
   :width: 100%
   :height: 1476px

.. image:: ../../Diagrams/human_command.png
   :alt: human_command
   :align: center
   :width: 100%
   :height: 1476px

The prose below mirrors the information in the diagrams so the document remains self‑contained even when images are unavailable.


Data Objects
------------

Four structured objects circulate through the planner; all are lists of *Action* records.

Recipe
~~~~~~
- **label** – symbolic name the high‑level executor understands.
- **order** – integer index in normal sequence.
- **mandatory** – `true` if the action cannot be skipped.
- **prerequisites** – list of `order` integers that must complete first.
- **tools** – list of tool identifiers.
- **ingredients** – list of ingredient identifiers.

The robot firmware already knows how to recognise each tool and ingredient ID.

Recipe History
~~~~~~~~~~~~~~
Inherits the fields of *Recipe* and adds:
- **executed** – `true` once the action finishes.
- **actual_order** – position in real execution sequence.

New Recipe History
~~~~~~~~~~~~~~~~~~
Extends *Recipe History* with:
- **added_by_user** – highlights tasks introduced by voice command.

On‑Execution Actions
~~~~~~~~~~~~~~~~~~~~
Extends *Recipe* with:
- **in_execution** – `true` while the action is active.
- **time_remaining_s** – estimated seconds until completion.
- **interruptible** – `true` if the action can be paused safely.

At robot start‑up all objects are `NULL` (empty lists).


Communication Channels (always on)
----------------------------------

- **A** – Robot State (initial value `NoRecipe`).
- **B** – Recipe.
- **C** – Recipe History.
- **D** – On‑Execution Actions.
- **E** – New Recipe History.
- **F** – Object Tracking facts.

Each channel is a publish–subscribe bus; producers push updates, and consumers receive them immediately without polling.


System Start‑up Scenarios
-------------------------

On boot the human can:

1. Ask for a **specific recipe**.
2. **Tell** ingredients by voice so the robot proposes dishes.
3. **Show** ingredients to the camera so the robot proposes dishes.

The chosen scenario determines how the *Human Command Monitoring* loop (below) proceeds.


Human Command Monitoring & Conflict Resolution
-------------------------------------------------

Loop runs continuously:

#. **Listen** – Microphones feed an audio stream.
#. **Detect speech** – If no speech, end iteration.
#. **Transcribe** – Convert speech to text.
#. **Interpret** – If the sentence is invalid → say *“Please repeat”* and end iteration.
#. **Evaluate context** – Merge the sentence with Robot State (A) and decide:
   * First interaction & Robot State =`NoRecipe`?
     * Ingredients shown? → trigger *Check Ingredients* in Perception.
     * Speech is recipe name? → build *Recipe Asked*.
     * Speech or vision produces ingredients list? → build *Ingredients List*.
#. **Connect Wi‑Fi** – Required for online search. If connection fails → raise *Wi‑Fi Error* and restart Wi‑Fi module.
#. **Search** – Depending on request:
   * **Search Proposals** – Find recipes for given ingredients.
   * **Search Recipe** – Retrieve full steps for a named dish.
   * Announce result via Speaker.
#. **Update Recipe** – When a recipe is found, push it to *Recipe* (B) and advance Robot State from `NoRecipe` to `awaiting_recipe`.

**Runtime voice commands** (pause, resume, skip step, etc.) follow the same path but reach *Command Validation*. Using Recipe (B), Recipe History (C), and On‑Execution Actions (D) the validator:

- Rejects invalid commands → **Speaker** says *“Sorry, I can’t”*.
- Accepts valid commands → updates New Recipe History (E) and confirms with *“OK”*.


Dummy Implementation
---------------------------------------------------

.. automodule:: human_command.scripts.human_command
   :members:


Recipe Tracking & Execution History Loop
---------------------------------------

Triggered when *Update Recipe* completes.

1. **Update Recipe** – One‑off initialisation from internet.
2. **Update Recipe History** – Reflects the baseline recipe in *Recipe History* (C).
3. **Wait** for *Update On‑Execution Actions* signal.
4. **Update On‑Execution Actions** – Copies the new best action into list (D) and raises *Update Recipe History* to log completion.
5. **Command Recipe History** – Runs in parallel when user inserts new tasks; merges them into *New Recipe History* (E).


Dummy Implementation
---------------------------------------------------

.. automodule:: recipe_tracking.scripts.recipe_tracking
   :members:


Action Planning Cycle
------------------------------

Every tick:

#. Read Recipe History (C), On‑Execution Actions (D), Object Tracking (F).
#. **Unexpected Condition Check** – Detect human interventions or environment changes that break assumptions.
   * If failure → raise *Recipe Fail* and instruct Error Handler to reboot.
#. **Update Best Action** – Fuse inputs from Robot State (A), Recipe (B), and New Recipe History (E) to choose the optimal next task.
#. **Finish tests** – If recipe finished → notify *High Level Action* and Robot State enters `completed`.
#. Else:
   * Signal *Update On‑Execution Actions* so the tracker logs the new step.
   * Signal *High Level Action* to execute the command and update Robot State.

Dummy Implementation
----------------------------------------------------

.. automodule:: action_planning.scripts.action_planning
   :members:


High‑Level Action Execution
---------------------------

Translates the *action label* from Recipe (B) into concrete manipulator or base motions, guided by the specified **tools** and **ingredients**.

Examples:

- `pick_up_pan` → move arm to pan pose (from Perception) and close gripper.
- `pour_milk` → align milk carton over pan and tilt until weight sensor threshold.

The module also sets Robot State (A) so other loops see the current context (e.g. `preparing`, `paused`).

