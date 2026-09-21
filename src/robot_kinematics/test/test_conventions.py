import numpy as np
import pytest

from robot_kinematics.conventions import JointConvention


NAMES = [f'Joint{i}' for i in range(1, 7)]


def test_reversed_axes_and_offsets_round_trip():
    convention = JointConvention(NAMES, [1, -1, 1, -1, 1, -1], [0, 10, -20, 0, 5, 90])
    ros = np.deg2rad([10, 20, 30, 40, 50, 60])
    expected = [10, -10, 10, -40, 55, 30]
    np.testing.assert_allclose(convention.ros_to_model(ros), expected)
    np.testing.assert_allclose(convention.model_to_ros(expected), ros)


def test_feedback_is_ordered_by_name_and_allows_extra_joints():
    convention = JointConvention(NAMES, [1]*6, [0]*6)
    names = ['gripper'] + NAMES[::-1]
    np.testing.assert_equal(
        convention.ordered_positions(names, [99, 6, 5, 4, 3, 2, 1]), [1, 2, 3, 4, 5, 6])


@pytest.mark.parametrize('names,positions', [
    (NAMES[:-1], [0]*5),
    (NAMES, [0]*5),
    (NAMES[:-1] + ['Joint1'], [0]*6),
    (NAMES, [0, 0, 0, np.nan, 0, 0]),
    (NAMES, [0, 0, 0, np.inf, 0, 0]),
])
def test_invalid_feedback_is_rejected(names, positions):
    with pytest.raises(ValueError):
        JointConvention(NAMES, [1]*6, [0]*6).ordered_positions(names, positions)


@pytest.mark.parametrize('signs,offsets', [
    ([1, 0, 1, 1, 1, 1], [0]*6),
    ([1]*5, [0]*6),
    ([1]*6, [float('nan')]*6),
])
def test_invalid_conventions_are_rejected(signs, offsets):
    with pytest.raises(ValueError):
        JointConvention(NAMES, signs, offsets)
