"""Convert between ROS radians and the model's joint-angle convention."""

from pathlib import Path

import numpy as np


class JointConvention:
    """Keep feedback and command conversion exactly inverse to each other."""

    def __init__(self, names, directions, offsets_deg):
        self.names = tuple(names)
        if (len(self.names) != 6 or len(set(self.names)) != 6
                or not all(isinstance(name, str) and name for name in self.names)):
            raise ValueError('joint_names must contain six unique nonempty names')
        self.directions = self._six(directions)
        self.offsets_deg = self._six(offsets_deg)
        if not np.isin(self.directions, [-1.0, 1.0]).all():
            raise ValueError('joint_directions must each be +1 or -1')

    @staticmethod
    def _six(values):
        values = np.asarray(values, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('Expected six finite joint values')
        return values

    def ros_to_model(self, radians):
        return self.directions * np.rad2deg(self._six(radians)) + self.offsets_deg

    def model_to_ros(self, degrees):
        return np.deg2rad((self._six(degrees) - self.offsets_deg) / self.directions)

    def ordered_positions(self, names, positions):
        """Require a complete single feedback sample; never assume array order."""
        if len(names) != len(positions) or len(names) != len(set(names)):
            raise ValueError('JointState names/positions are inconsistent or duplicated')
        by_name = dict(zip(names, positions))
        missing = [name for name in self.names if name not in by_name]
        if missing:
            raise ValueError('Missing joint positions: ' + ', '.join(missing))
        return self._six([by_name[name] for name in self.names])


def convention_from_node(node):
    """Load common defaults, with optional per-node ROS parameter overrides."""
    from ament_index_python.packages import get_package_share_directory
    import yaml

    default_path = Path(get_package_share_directory('robot_kinematics')) / 'config' / 'joint_conventions.yaml'
    path = node.declare_parameter('joint_conventions_file', str(default_path)).value
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    values = [node.declare_parameter(key, config[key]).value for key in (
        'joint_names', 'joint_directions', 'joint_offsets_deg')]
    return JointConvention(*values)
