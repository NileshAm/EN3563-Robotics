# EN3563 Robotics Workcell

A ROS 2 Jazzy workspace for visualizing and simulating a six-axis industrial
robot. It includes the robot model, an RViz configuration, a Gazebo workcell,
ROS–Gazebo topic bridges, measured-feedback pose control, and a dashboard with editable
upright pick-and-place waypoints.

## Packages

| Package | Purpose |
| --- | --- |
| `my_robot_description` | URDF, meshes, RViz configuration, Gazebo world, launch files, and topic bridge configuration |
| `robot_motion` | Python demonstration node that publishes position commands for joints 1–6 |

## Requirements

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Gazebo Harmonic with the ROS–Gazebo integration packages
- `colcon`, `rosdep`, and Python 3

ROS package dependencies are declared in each `package.xml` and should be
installed with `rosdep` for the system installation. The root `requirements.txt`
contains the PyPI development tools. ROS Python modules are installed from the
RoboStack Jazzy Conda channel when using the Conda environment; they are not
available as normal PyPI requirements.

## Installation

### System Python

```bash
# From the workspace root
source /opt/ros/jazzy/setup.bash

# Install Python development dependencies (optional for runtime use)
python3 -m pip install -r requirements.txt

# Install ROS dependencies declared by the packages
sudo rosdep init  # Run once per machine; skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and source the workspace
colcon build --symlink-install
source install/setup.bash
```

### Conda

The workspace is configured to use the Conda environment named `ros` for Python
editing and VS Code autocomplete. It uses Python 3.12 and ROS packages from the
RoboStack Jazzy channel. Gazebo and the complete runtime should still be
installed through the system ROS installation and `rosdep` as shown above.

```bash
# Create the environment and install Python-facing ROS packages (run once)
conda create --name ros \
  -c robostack-jazzy -c conda-forge --strict-channel-priority \
  python=3.12 pip \
  ros-jazzy-rclpy \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-ament-index-python \
  ros-jazzy-launch \
  ros-jazzy-launch-ros \
  -y

# Activate it and install the remaining development dependencies
conda activate ros
python -m pip install -r requirements.txt
```

If the `ros` environment already exists, update it with the packages required by
this project:

```bash
conda install --name ros \
  -c robostack-jazzy -c conda-forge --strict-channel-priority \
  pip ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs ros-jazzy-ament-index-python \
  ros-jazzy-launch ros-jazzy-launch-ros -y
conda run --name ros python -m pip install -r requirements.txt
```

VS Code is pinned to `/home/nilesh/miniconda3/envs/ros/bin/python` in
`.vscode/settings.json`. Reload the VS Code window after installation. If
autocomplete still shows missing imports, run **Python: Select Interpreter** and
choose the `ros` environment. Verify the environment with:

```bash
conda run --name ros python -c \
  "import rclpy, std_msgs, sensor_msgs, geometry_msgs, launch, launch_ros"
```

For each new terminal using system Python, source both ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Usage

### View the robot in RViz

```bash
ros2 launch my_robot_description display.launch.py
```

Use the Joint State Publisher window to move individual joints.

### Run the Gazebo workcell

```bash
./run.sh
```

This starts Gazebo, RViz, the waypoint dashboard, `robot_state_publisher`, and the ROS–Gazebo bridge.
The simulation starts running and publishes simulation time and joint states to
ROS 2. See [`src/my_robot_description/worlds/README.md`](src/my_robot_description/worlds/README.md)
for scene layout and reset instructions.

### Complete the pick-and-place task

```bash
./run.sh
```

`run.sh` builds and launches using system ROS Jazzy in a clean environment,
including when called from a Conda terminal. It forwards launch arguments;
add `gui:=false` for a headless simulation.

Box A is at **(0.5, 0.5, 0)** (bottom centre), as requested. The robot picks up
Box B from the ground and places it upright at the centre of Box A's top.
The dashboard loads the original poses by default. Select each waypoint and click
**Move to waypoint**, or edit/add poses and select Keep / Grab / Release on arrival.
No motion starts automatically. Save and load your custom lists as JSON.
Click **Execute motion plan** to run every waypoint in table order, waiting for
each move and gripper action to finish. A failed step stops the remaining plan.
The launch starts the motion node; do not start a second motion node alongside it.
See [run instructions and report notes](docs/pick_and_place.md) for the sequence,
D–H table, FK/IK equations, velocity planning, checks, and submission details.
The launch exposes `/robot/move_to_pose` for individual waypoint control through
the [dashboard](src/control_dashboard/README.md). The legacy `pick_and_place` flag
is accepted but no longer starts automatic motion.

## Useful checks

```bash
colcon test
colcon test-result --verbose
ros2 topic list
ros2 topic echo /joint_states
```

After changing a URDF, launch file, bridge configuration, or world file, rebuild
the affected package and source the workspace again:

```bash
colcon build --packages-select my_robot_description --symlink-install
source install/setup.bash
```

## Project layout

```text
robot_prod/
├── requirements.txt
└── src/
    ├── my_robot_description/
    │   ├── config/       # RViz and ROS–Gazebo bridge settings
    │   ├── launch/       # RViz-only and Gazebo launch files
    │   ├── meshes/       # Robot link meshes
    │   ├── urdf/         # Robot description
    │   └── worlds/       # Gazebo workcell
    └── robot_motion/     # ROS 2 Python motion package
```

## Troubleshooting

- **`fastcdr` undefined symbol / nodes exit with code 127:** system ROS and
  Conda ROS libraries have been mixed. Use `./run.sh` to
  rebuild and launch with only system ROS libraries.
- **Box B start-location error immediately after a fresh launch:** an older
  Gazebo server may still be publishing its finished scene. New launches use
  separate Gazebo partitions and supervise the server directly to prevent this.
- **Package not found:** source `/opt/ros/jazzy/setup.bash`, rebuild, then source
  `install/setup.bash` in the current terminal.
- **Robot meshes are missing:** launch from a terminal where the workspace setup
  file has been sourced.
- **The robot does not move:** confirm Gazebo is running and that the six
  `/robot/jointN/command` topics are listed by `ros2 topic list`.
- **`rosdep init` reports that it is already initialized:** continue with
  `rosdep update`; this is not an error.
