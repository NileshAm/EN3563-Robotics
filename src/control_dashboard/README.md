# Robot waypoint dashboard

Start the workcell from the workspace root:

```bash
./run.sh
```

Gazebo, RViz, the motion controller, and the dashboard open together. **No robot
motion starts automatically.** The former `pick_and_place:=true` launch argument
is accepted for compatibility; it also waits for dashboard commands now.
Use `./run.sh dashboard:=false` to open the simulator without the dashboard,
or `./run.sh gui:=false dashboard:=true` for the dashboard with a headless simulator.

## Edit and move

1. Select a waypoint in the table. Its pose and gripper action appear in the editor.
2. Change the name, XYZ in **millimetres**, and fixed-axis roll/pitch/yaw in **degrees**.
   Coordinates describe **world → Link6**, not the end of the fingers.
3. Choose **MoveL** for a straight Cartesian path, or uncheck it for joint motion.
4. Set **On arrival** to **Keep**, **Grab**, or **Release**. Keep leaves the jaws in
   their current state; Grab closes them and waits for attachment; Release opens
   them and waits for detachment and 0.8 seconds of jaw retraction.
5. Click **Move to waypoint**. This saves the current editor values into the
   selected row, sets the target marker, plans, and moves. One click runs one
   waypoint. Select the next row and click Move when ready.

To run the full list, click **Execute motion plan**. This saves the selected
row's current edits and runs **all waypoints from the first row to the last**,
in table order. Each waypoint waits for measured arrival and its grab/release
acknowledgement before the next starts. The bar shows overall plan progress and
the status names the current waypoint. Editing and other motion buttons stay
locked until the plan completes or stops.

A rejected move, execution/gripper failure, or controller disconnection stops
the remaining plan. Closing the dashboard also stops scheduling later waypoints;
the currently accepted move still finishes. The plan does not run automatically
when the dashboard opens or a waypoint file is loaded.

**Add waypoint** appends the editor values; **Update** saves edits without moving;
**Remove** deletes the selected row. **Save / Load** store and restore waypoint
lists as JSON files. **Defaults** restores the original pick-and-place poses.
**Move home** sends a joint-space move to the original home pose and keeps the
current gripper state. Editing and further moves are disabled during an active move.

The initial defaults preserve the former automatic sequence:

| Waypoint | X (mm) | Y (mm) | Z (mm) | Motion | On arrival |
| --- | ---: | ---: | ---: | --- | --- |
| Approach | 450 | -200 | 470 | MoveJ | Keep |
| Descend and grab | 450 | -200 | 170 | MoveL | Grab |
| Lift | 450 | -200 | 470 | MoveL | Keep |
| Transfer | 500 | 500 | 470 | MoveL | Keep |
| Place and release | 500 | 500 | 372 | MoveL | Release |
| Retreat | 500 | 500 | 470 | MoveL | Keep |

All default orientations are roll=0°, pitch=90°, yaw=0° (Link6 +X points down).
The original separate attach/release stages now belong to the corresponding
arrival actions. The dashboard executes custom individual waypoints, so it does
not automatically verify that Box B was placed on Box A after the last point.

## Feedback and service

The GUI sends `/robot/move_to_pose` (`control_dashboard/srv/MoveToPose`) and displays
`/robot/move_progress` plus `/gripper/attached`. Planning acceptance is separate
from completion. The controller waits for measured FK within **5 mm and 0.03 rad**,
then applies the requested gripper action. 100% means both have completed.
Gripper actions require fresh feedback and time out after 10 seconds. A lost
grasp during carrying stops the move; failed carrying moves leave the jaws closed.
Closing the dashboard does not cancel an accepted waypoint.

Progress states: `idle`, `planning`, `moving`, `settling`, `grabbing`, `releasing`,
`succeeded`, `failed`. Positions in the service are metres; orientations use xyzw
quaternions. `gripper_action` accepts `keep` (or empty for compatibility), `grab`,
or `release`. Existing measured-joint, joint-limit, IK, and settling checks remain.
The planner has no general collision checking; choose clear approach/lift paths.

To open a replacement dashboard in another terminal without rebuilding:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run control_dashboard dashboard
```

Rebuild and restart both the motion controller and dashboard after interface
changes. `run.sh` does this in a clean system ROS Jazzy environment.
