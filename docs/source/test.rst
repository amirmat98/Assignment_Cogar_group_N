============================
Key Performance Indicators
============================

This section defines objective, measurable KPIs that verify whether the core subsystems deliver the responsiveness and reliability required for safe, efficient meal preparation. For each metric we state *what it measures*, *how to compute it*, and the *target threshold* that indicates success.


Recipe Tracker KPIs
-------------------

**Step‑Validation Latency**
  Measures the delay between the moment the robot controller signals *Step Finished* and the moment the Recipe Tracker records that success.

  *Computation* — `latency = t_tracker_update − t_step_done` averaged over all steps; report mean ± standard deviation.

  *Target* — _mean < 1 s_ and _σ < 0.2 s_. Keeping latency below human reaction time prevents the planner from stalling and reduces accident risk during tasks such as cutting.

**Execution‑History Accuracy**
  Indicates how faithfully the Recipe Tracker logs completed actions.

  *Computation* — Compare *Recipe History* against ground‑truth labels gathered offline; derive **accuracy**, **false‑positive rate** (incorrectly marked done), and **false‑negative rate** (missed completion).

  *Target* — _accuracy ≥ 0.90_, *false‑positive < 0.05*, *false‑negative < 0.10*. False positives are more critical because they might skip essential cooking steps.


Action Planning KPIs
--------------------

**Step Success Rate**
  Percentage of planned steps executed without manual intervention.

  *Computation* — `success_rate = successful_steps / total_steps`.

  *Target* — > 90 %.

**Average Conflict‑Resolution Time**
  Time taken to detect a plan conflict, compute a workaround, and resume execution.

  *Computation* — Mean of `(t_resolved − t_conflict_detected)` for each incident.

  *Target* — < 3 s.

**Step Processing Time**
  Delay from receiving a new step to inserting it into the execution stack.

  *Computation* — Mean of `(t_step_integrated − t_step_received)`.

  *Target* — < 2 s.

**Planning Error Rate**
  Fraction of steps that fail definitively after retries.

  *Computation* — `errors / total_steps`.

  *Target* — < 10 %.

**User Satisfaction Score**
  Subjective rating provided by users after a cooking session (1–5 scale).

  *Computation* — Mean score across test sessions.

  *Target* — ≥ 4.0.

**Object‑Detection Accuracy**
  Measures Action Planning’s reliance on correct object IDs from Perception.

  *Computation* — `true_detections / total_objects` during planning‑relevant queries.

  *Target* — ≥ 95 %.


Human Command Monitoring KPIs
-----------------------------

**Command‑Response Time**
  Interval from the onset of a spoken command to the robot’s audible or behavioural acknowledgment.

  *Computation* — `(t_robot_response − t_voice_onset)` averaged over commands.

  *Target* — < 3 s. This maintains conversational flow.

**Conflict‑Resolution Accuracy**
  Correctly detected and resolved conflicts divided by total true conflicts.

  *Computation* — `true_positive_conflicts / (true_positive_conflicts + false_positive_conflicts)`.

  *Target* — ≥ 90 %.

**Command‑Recognition Rate**
  Ratio of recognised commands to total spoken commands.

  *Computation* — `valid_commands / total_commands`.

  *Target* — ≥ 90 %.


Measurement Notes
-----------------

* Timestamps **t_*** must come from a synchronised clock across subsystems.
* Log raw events to CSV/ROS bags; analyse offline with the provided Python notebooks.
* Report each KPI with **mean ± SD** and include 95 % confidence intervals for statistical significance.
