# Shared GP7 kinematics and monitoring

Build from the workspace root in a system ROS Jazzy terminal:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_motion monitor my_robot_description
source install/setup.bash
```

All workspace Python packages can use:

```python
from robot_kinematics import H06, IK, generateH, s_curve
```

Declare `<depend>robot_kinematics</depend>` in each consuming ROS package's
`package.xml`. The installed Python package is discoverable after sourcing the
workspace; do not add `src` to `sys.path`.

`H06` and `IK` use model joint angles in **degrees**, translations in
**millimetres**, and the **world** reference frame. The endpoint is the **Link6
origin**, not an additional tool tip. The 180-degree world-to-base mounting and
joint transforms currently match `my_robot_description/urdf/robot.urdf`.
Keep those transforms consistent when changing the robot model.

## Joint directions and offsets

Edit `config/joint_conventions.yaml`, ordered Joint1 through Joint6:

```yaml
joint_directions: [1.0, -1.0, 1.0, 1.0, 1.0, 1.0]
joint_offsets_deg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

This example reverses Joint2 between ROS and the mathematical model. The rules are:

```text
model_degrees = direction * degrees(ROS_radians) + offset_degrees
ROS_radians   = radians((model_degrees - offset_degrees) / direction)
```

Motion applies the inverse rule to every outgoing command; monitoring applies
the forward rule to measured feedback. Both nodes load this same file on startup.
Restart both after editing it (rebuild first if not using symlink installation).
For a separate configuration use `--ros-args -p joint_conventions_file:=/absolute/path.yaml`
on both nodes. ROS parameters `joint_names`, `joint_directions`, and
`joint_offsets_deg` can also override the file; keep overrides identical in both nodes.

Defaults are all +1 and zero offsets because the current FK follows the URDF.
Only reverse a sign if the model and ROS conventions actually differ. ROS joint
rotation follows the right-hand rule around the URDF joint's **local** `<axis>`.
Changing the physical simulated axis requires updating the URDF and matching FK
transform together; a boundary conversion alone does not change robot_state_publisher's
TF calculations. Otherwise the monitor's computed pose will disagree with TF.
Offsets are calibration between conventions, not a world-frame translation.

The motion node still assumes startup at zero ROS joint positions; this change
does not add measured-state motion planning or joint-limit enforcement.

## Monitoring

The existing `monitor` package now contains a read-only feedback node:

```bash
ros2 run monitor joint_monitor
```

Both display and Gazebo launch files start it automatically; do not start another
copy when using these launch files. For Gazebo standalone use
`--ros-args -p use_sim_time:=true`.

It reads `/joint_states`, matches joints by name rather than array order, and
requires all six positions to be finite in the same message. It calculates FK
from **measured feedback**, not the requested trajectory. It publishes:

| Topic | Type | Meaning |
|---|---|---|
| `/monitor/joint_angles_deg` | `std_msgs/Float64MultiArray` | Measured ROS angles in degrees; layout labels give joint order |
| `/monitor/model_angles_deg` | `std_msgs/Float64MultiArray` | Angles after direction/offset conversion |
| `/monitor/end_effector_pose` | `geometry_msgs/PoseStamped` | World-to-Link6 XYZ in metres and quaternion xyzw; feedback timestamp |
| `/monitor/end_effector_rpy_deg` | `geometry_msgs/Vector3Stamped` | Fixed-axis roll, pitch, yaw in degrees (x, y, z); feedback timestamp |

RPY has Euler singularities; the quaternion is the canonical orientation.
The console logs joints, XYZ and RPY every second, and reports missing or stale
feedback. Invalid/incomplete messages do not publish a pose. Parameters:
`joint_states_topic` (default `/joint_states`), `log_period` (1 s),
`stale_timeout` (2 s), plus the shared convention parameters above.

```bash
ros2 topic echo /monitor/end_effector_pose --once
ros2 topic echo /monitor/joint_angles_deg --once
ros2 run tf2_ros tf2_echo world Link6
```

With the default mapping the computed pose should agree with TF at the same
joint state. Monitoring publishes no joint commands or TF transforms.
