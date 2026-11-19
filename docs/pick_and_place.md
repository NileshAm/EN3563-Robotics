# GP7 pick-and-place implementation

## Run the simulation

From the workspace root:

```bash
./run.sh
```

The dashboard opens with the original pick-and-place waypoints. Select a row,
edit its pose and Keep / Grab / Release action, then click **Move to waypoint**.
Each click runs one waypoint; motion never starts automatically. Add, update,
remove, save, and load waypoints in the dashboard. See the
[dashboard guide](../src/control_dashboard/README.md) for the controls and units.
Click **Execute motion plan** to run the complete list in order, waiting for
each waypoint and its gripper action. Any failed step stops the remaining plan.
For a headless simulator with the dashboard add `gui:=false dashboard:=true`.
The launcher builds and runs with system ROS Jazzy in a clean environment,
preventing Conda ROS libraries from causing `fastcdr` symbol lookup errors.
Gazebo discovery uses loopback so VPN interfaces do not interrupt local feedback;
set `GZ_IP` explicitly if using Gazebo across machines.
Each launch has its own Gazebo transport partition, so an older simulator cannot
feed stale box poses into a new sequence. The server and GUI are supervised
directly and both stop when the launch exits.
RViz shows named link frames; Gazebo shows the boxes and physical parallel gripper.

Box A is 0.30 × 0.30 × 0.20 m, with its bottom-centre model origin at
**(0.5, 0.5, 0)**, following the user's requested change from the task sheet.
Its physical centre is at z=0.10 m. Box B is a 0.05 m cube initially centred at
(0.45, -0.20, 0.025) m, a reproducible reachable ground position.
The robot base remains at the world origin, with its existing 180° mounting yaw.

## Gripper and sequence

The parallel gripper has a blue motor housing, metal fingers, and dark gripping
pads. Two independent prismatic joints close symmetrically around the 50 mm
cube: the open collision-surface gap is 90 mm. The servo targets a 46 mm gap,
so a 50 mm cube blocks the jaws and develops sustained squeezing force.
Each finger has 22 mm travel, a 7.5 N force limit, and a damped position servo.
At the nominal cube width the servo provides about 4.5 N per finger.
The fingers extend 163 mm from the flange, leaving 7 mm ground clearance at
the pickup pose. The `tool_tip` frame at 120 mm marks the box's top surface.

`/gripper/close` (`std_msgs/Bool`) drives both fingers. The box is held entirely
by collision friction (coefficient 2.0 on the fingers and box), with no temporary
joint or pose constraint. Contact sensors confirm that both fingers touch Box B
for at least 100 ms before `/gripper/attached` reports true. This legacy topic
now indicates bilateral contact, not an attachment. A 150 ms grace period
filters brief contact interruptions; sustained loss or opening clears feedback.
Opening removes squeezing force and retracts the jaws;
the sequence waits 0.8 seconds before retreating. Joint-state feedback includes
both finger joints, so their motion is also visible in RViz.

The default dashboard waypoints preserve these Link6-origin coordinates in metres.
Attach runs on arrival at Descend; Release runs on arrival at Place. Verify is
part of the validation procedure, rather than an automatic dashboard step:

| Stage | X | Y | Z | Motion |
|---|---:|---:|---:|---|
| Approach | .45 | -.20 | .47 | Smooth joint motion, before grasp |
| Descend | .45 | -.20 | .17 | Cartesian linear |
| Attach | — | — | — | Wait for physical grasp acknowledgement |
| Lift | .45 | -.20 | .47 | Cartesian linear |
| Transfer | .50 | .50 | .47 | Cartesian linear |
| Place | .50 | .50 | .372 | Cartesian linear, 2 mm release clearance |
| Release | — | — | — | Wait for detached acknowledgement |
| Retreat | .50 | .50 | .47 | Cartesian linear |
| Verify | — | — | — | Allow settling, inspect actual box pose |

The target flange rotation is R_y(90°); local +X points vertically down. Holding
this rotation throughout carrying preserves the cube's original orientation.
The transfer clears Box A by 125 mm at the cube's bottom. Each move waits for
measured FK to reach the requested pose, rather than just finishing commands.
Missing feedback, unreachable poses, joint limits and lost grasp stop the current
waypoint. The jaws stay closed if a carrying motion fails. The default paths hold
the flange orientation; the dashboard also permits custom orientations.
The validation procedure checks measured Box B centre within 10 mm of (0.5, 0.5, 0.225),
rotation within 0.08 rad of its starting orientation, and a released grasp.
These are numerical simulation tolerances, not a claim of exact zero error.

## Coordinate frames and standard D–H model

All lengths below are millimetres and joint angles are degrees. The D–H frames
are mathematical frames; they differ from the CAD link frames shown by RViz.
Choose z_(i−1) along joint i, x_i along the common normal between consecutive
joint axes, and y_i = z_i × x_i. Frame 0 coincides with the world frame.
`robot_kinematics/dh.py` returns the explicit frame transforms for all six joints.

Use standard D–H: A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i),
where theta_i = q_i + offset_i and q_i uses the shared model convention.

| i | a_i | alpha_i | d_i | theta offset |
|---|---:|---:|---:|---:|
| 1 | 40 | -90 | 330 | 0 |
| 2 | 445 | 0 | 0 | -90 |
| 3 | 40 | -90 | 0 | 0 |
| 4 | 0 | 90 | 440 | 0 |
| 5 | 0 | 90 | 0 | 180 |
| 6 | 0 | 0 | 67.5 | 0 |

Each homogeneous transform is

```text
A_i = [ cosθ  -sinθ cosα   sinθ sinα   a cosθ ]
      [ sinθ   cosθ cosα  -cosθ sinα   a sinθ ]
      [  0        sinα        cosα       d   ]
      [  0          0           0        1   ]
```

World-to-Link6 is A_1 … A_6 C, with fixed frame conversion

```text
C = [0 1 0 0]
    [0 0 1 0]
    [1 0 0 0]
    [0 0 0 1]
```

The tool tip adds Tx(120) in Link6. The nominal D–H model uses exact right
angles. Runtime `H06` retains the CAD URDF's rounded RPY constants (e.g. 1.5708),
so the two chains differ by micrometres. A regression test compares their
positions and orientations over 30 seeded configurations (tolerances 0.03 mm
and 0.0001 rad). Dimensions come from the existing URDF, not new CAD measurements.

## Inverse kinematics and velocity planning

For desired pose (p_d, R_d), the numerical IK error is

```text
e(q) = [p_d − p(q); Log(R_d R(q)^T)]
J_e[:,i] ≈ (e(q + epsilon e_i) − e(q)) / epsilon
Δq = −(J_e^T W^2 J_e + (lambda^2 + mu^2) I)^−1
      (J_e^T W^2 e + mu^2 (q − q_seed))
```

W scales translation and rotation by their requested tolerances; lambda is
adaptive damping and mu attracts the solution toward measured starting joints.
Backtracking reduces the step when it increases the weighted error. Runtime
checks both pose convergence and URDF joint limits. See `Homogenous.py::IK`.

For normalized time u=t/T, use the quintic profile

```text
s(u) = 10u^3 − 15u^4 + 6u^5
s_dot = (30u^2 − 60u^3 + 30u^4)/T
s_ddot = (60u − 180u^2 + 120u^3)/T^2
p(t) = p_start + s(u) (p_end − p_start)
R(t) = R_start Exp(s(u) Log(R_start^T R_end))
```

During carrying R_start and R_end are equal (up to tracking error), so the
planned orientation stays constant. IK uses the previous sample as its next
seed. Joint commands run at 100 Hz; nominal limits are 45 deg/s and 45 deg/s².
Cartesian paths are slowed when sampled joint derivatives exceed these limits.
Joint speed and acceleration limits are 45 degrees/s and 45 degrees/s²,
including while carrying a box.
The six existing Gazebo PID controllers track the position samples with joint
feedback. This is not a general obstacle planner; the fixed scene uses explicit
vertical approach/lift waypoints and an elevated transfer.

## Validation and submission

With the earlier reduced carrying limits (15 degrees/s and 10 degrees/s²),
the friction-only implementation completed a separate headless Gazebo sequence:
approach, grasp, lift, transfer, place, release, and retreat. The final check
confirmed Box B upright on Box A at (0.5, 0.5, 0.225) m using the existing
10 mm position and 0.08 rad orientation tolerances. The motion-service
regression suite passed all nine tests.

Before the friction-only change, a real Gazebo run driven through the dashboard Move button completed approach,
grasp, lift, transfer, release and retreat. A separate check verified placement.
The completion log is saved in
`docs/pick_place_validation.log`. Kinematics, motion-service and monitor
regressions are also run; the nominal D–H model has a separate consistency test.

For the report, add the group member index numbers and screenshots of the
labelled frames and final placement. Record the visible Gazebo run for the
required screen-recorded demonstration. Those personal submission details and
the screen recording are not supplied by this implementation.

Implementation references: `robot_motion/pick_place.py`,
`robot_kinematics/Homogenous.py`, `robot_kinematics/velocityProfile.py`,
`robot_kinematics/dh.py`, and `my_robot_description/plugins/gripper.cpp`.
The grasp uses Gazebo collision friction and contact sensors. The earlier
validation log describes the previous assisted grasp implementation.
