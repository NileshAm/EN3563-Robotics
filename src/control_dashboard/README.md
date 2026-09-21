# Robot control dashboard

The Tk GUI sends `/robot/move_to_pose` service requests to `turnNode` and displays
`/robot/move_progress`. One move runs at a time. Services return an acceptance
response after planning; the progress topic provides subsequent completion or
failure. No motion is started automatically when `turnNode` starts.

Build in the system ROS Jazzy environment (not the Conda ROS environment):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to my_robot_description control_dashboard
source install/setup.bash
ros2 launch my_robot_description gazebo.launch.py
```

In a second sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run control_dashboard dashboard
```

Restart any previously running `turnNode` after building. The Gazebo launch
already starts it; do not run a second copy. The dashboard is launched separately
to keep the existing simulation launch unchanged.

Enter XYZ in **millimetres** and fixed-axis roll/pitch/yaw in **degrees**. The
target is the **Link6 origin in world coordinates**. The service transports the
position in metres and orientation as a quaternion. MoveL plans a straight
Cartesian path; unchecking it uses the existing joint-space trajectory generator.
The initial GUI values approximate the URDF's zero-joint pose. Positive rotations
and conventions are those already configured in `robot_kinematics`.

`turnNode` uses measured `/joint_states` as the starting angles and reads limits
from `/robot_description`. It rejects missing/stale feedback, invalid targets,
failed IK/linear paths, trajectories exceeding joint limits, and requests while
busy. Both feedback and description are provided by the existing Gazebo launch.

Progress states are `idle`, `planning`, `moving`, `settling`, `succeeded`, `failed`.
0–99% indicates how much of the planned command sequence has been sent, not the
measured percentage of travel. 100% requires measured FK to reach the target
within **5 mm and 0.03 rad**. After the final command it allows 10 seconds to
settle; missing feedback for 2 seconds fails the move. These deadlines use wall
time; the command timer follows ROS/simulation time. The existing planner has no
collision checking, and closing the dashboard does not cancel an accepted move.

The original `IK`, `s_curve`, conversions and joint publishers are reused.
Optional original plots can be enabled with `show_plot:=true` on `turnNode`.

Inspect the interface and progress:

```bash
ros2 interface show control_dashboard/srv/MoveToPose
ros2 topic echo /robot/move_progress
ros2 service call /robot/move_to_pose control_dashboard/srv/MoveToPose \
  "{target: {position: {x: 0.547498, y: 0.0, z: 0.815002}, orientation: {x: -0.70710678, y: 0.0, z: 0.0, w: 0.70710678}}, move_linear: true}"
```

The package uses CMake to generate its service/message interfaces and installs
one Python dashboard script. No web server or additional interface package is needed.
