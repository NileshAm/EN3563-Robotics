# Workcell

The robot is mounted at the origin. Box A has bottom centre (0.5, 0.5, 0) m,
size 0.30 × 0.30 × 0.20 m, and top surface z=0.20 m. Box B is a dynamic
0.05 m cube, initially centred at (0.45, -0.20, 0.025) m.

Run `ros2 launch my_robot_description gazebo.launch.py pick_and_place:=true`
after building and sourcing the workspace. Close and restart the launch to
reset the world, controller and one-shot sequence together; do not use Gazebo's
Reset All during an active grasp. See [full instructions](../../../docs/pick_and_place.md).
