# Gazebo workcell

`workcell.sdf` is the default world for `gazebo.launch.py`. It contains the
existing robot, a ground plane, a fixed low worktable, and three movable objects.
All assets are local; no Fuel downloads are required.

Launch it from the workspace after installing dependencies and building (see the
[workspace README](../../../README.md)):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_description gazebo.launch.py
```

The launch file passes `-r` to Gazebo, so physics starts running immediately.
Use Gazebo's play/pause control to change the simulation state.

## Reset to the saved poses

Use Gazebo's simulation reset control, or run this in a sourced terminal:

```bash
gz service -s /world/workcell/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: true, reset: {all: true}'
```

This resets the simulation and leaves it paused at its original configuration.
The robot is included in the world at startup, so it is restored along with the
objects. The launch file does not spawn a second robot.

## Change the initial layout

Edit the model-level `<pose>x y z roll pitch yaw</pose>` values in `workcell.sdf`.
Positions are in metres and angles are in radians. The table surface is at
`z=0.30`; object poses specify their centres.

| Model | Initial position (x, y, z) |
| --- | --- |
| my_robot | 0, 0, 0 (yaw: pi) |
| worktable | 0.65, 0, 0 |
| red_cube | 0.60, -0.25, 0.34 |
| green_cylinder | 0.60, 0, 0.36 |
| blue_sphere | 0.60, 0.25, 0.35 |

After editing, run `colcon build --packages-select my_robot_description` and
restart the launch. Moving objects in the GUI does not change the saved file.

## ROS 2 interfaces

The bridge configuration exposes these simulation interfaces:

- `/robot/joint1/command` through `/robot/joint6/command`
  (`std_msgs/msg/Float64`) send joint position targets to Gazebo.
- `/joint_states` (`sensor_msgs/msg/JointState`) reports the simulated joints.
- `/clock` (`rosgraph_msgs/msg/Clock`) provides simulation time.

The robot model is included directly by `workcell.sdf`; do not spawn another
copy when using `gazebo.launch.py`.
