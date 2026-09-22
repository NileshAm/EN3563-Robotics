"""Shared joint and Cartesian trajectory generation."""

from numpy import (
    array,
    arange,
    abs,
    max,
    maximum,
    sqrt,
    deg2rad,
    rad2deg,
    gradient,
    unwrap,
    eye,
)


def s_curve(
        start,
        end,
        v_max=6.28,
        a_max=6.28,
        isdegrees=False,
        dt=0.01,
        move_linear=False):
    """
    Return a time-sampled MoveJ or MoveL trajectory.

    Parameters:
        start : six start joint angles in degrees
        end : six end joint angles in degrees
        v_max : maximum velocity of a joint
        a_max : maximum acceleration of a joint
        isdegrees : whether v_max and a_max are supplied in degrees
        dt : time between trajectory samples
        move_linear : False selects MoveJ; True selects Cartesian MoveL

    MoveL uses ``H06`` and ``IK``, whose joint-angle unit is degrees. Its
    returned position, velocity, and acceleration are therefore degrees,
    degrees/second, and degrees/second squared respectively.
    """
    start = array(start, dtype=float)
    end = array(end, dtype=float)
    if start.shape != (6,) or end.shape != (6,):
        raise ValueError("start and end must each contain exactly 6 angles")
    if dt <= 0:
        raise ValueError("dt must be greater than zero")

    # The robot FK/IK uses degrees. Convert radian limits when necessary so
    # trajectory positions and their time derivatives remain unit-consistent.
    if not isdegrees:
        v_max = rad2deg(v_max)
        a_max = rad2deg(a_max)
    T = calculate_move_time(start, end, v_max, a_max)
    print(f"Max velocity : {round(v_max, 3)} deg/s")
    print(f"Max acceleration : {round(a_max, 3)} deg/s^2")
    print(f"Movement time : {round(T, 3)} s")
    return _s_curve(start, end, T, dt, move_linear)


def _s_curve(start, end, T, dt=0.01, move_linear=False):
    if move_linear:
        return _move_l_curve(start, end, T, dt)
    return _move_j_curve(start, end, T, dt)


def _profile_samples(T, dt):
    """Return time and quintic position/velocity/acceleration scale values."""
    if T <= 0:
        return [(0.0, 0.0, 0.0, 0.0)]

    samples = []
    for t in arange(0, T + dt, dt):
        t = min(float(t), float(T))
        tau = t / T
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / T**2
        samples.append((t, s, s_dot, s_ddot))
    return samples


def _move_j_curve(start, end, T, dt=0.01):
    start = array(start)
    end = array(end)

    dq = end - start
    trad = []

    for t, s, s_dot, s_ddot in _profile_samples(T, dt):
        q = start + dq * s
        q_dot = dq * s_dot
        q_ddot = dq * s_ddot

        trad.append({
            "time": t,
            "position": q,
            "velocity": q_dot,
            "acceleration": q_ddot
        })

    return trad


def _move_l_curve(start, end, T, dt=0.01):
    """Generate a straight tool-space path and solve its joint trajectory."""
    # Lazy imports keep the standalone MoveJ profile independent of the robot
    # model and avoid an import cycle during module initialisation.
    from scipy.spatial.transform import Rotation
    from .Homogenous import H06, IK

    start_transform = H06(*start)
    end_transform = H06(*end)
    start_xyz = start_transform[:3, 3]
    xyz_delta = end_transform[:3, 3] - start_xyz
    start_rotation = start_transform[:3, :3]
    relative_rotation_vector = Rotation.from_matrix(
        start_rotation.T @ end_transform[:3, :3]
    ).as_rotvec()

    samples = _profile_samples(T, dt)
    times = array([sample[0] for sample in samples])
    joint_positions = []
    previous_angles = array(start, dtype=float)

    for index, (_, s, _, _) in enumerate(samples):
        if index == 0:
            joint_angles = array(start, dtype=float)
        elif index == len(samples) - 1:
            joint_angles = array(end, dtype=float)
        else:
            target = eye(4)
            target[:3, 3] = start_xyz + xyz_delta * s
            target[:3, :3] = start_rotation @ Rotation.from_rotvec(
                relative_rotation_vector * s
            ).as_matrix()

            # Cartesian orientation changes can pass close to a wrist
            # singularity.  The endpoint solve is usually quick, but the
            # small intermediate rotations need more iterations to leave the
            # singular configuration while preserving the tool position.
            joint_angles, converged = IK(
                target,
                previous_angles,
                max_iterations=1000,
            )
            if not converged:
                raise RuntimeError(
                    "Target pose is reachable, but the straight-line path "
                    "is not reachable: inverse kinematics failed at "
                    f"t={times[index]:.4f} s (path fraction {s:.4f})"
                )

        joint_positions.append(joint_angles)
        previous_angles = joint_angles

    joint_positions = array(joint_positions)
    if len(joint_positions) == 1:
        joint_velocities = array([[0.0] * 6])
        joint_accelerations = array([[0.0] * 6])
    else:
        # Remove equivalent +/-360-degree jumps before differentiating.
        continuous_positions = rad2deg(
            unwrap(deg2rad(joint_positions), axis=0)
        )
        joint_positions = continuous_positions
        edge_order = 2 if len(joint_positions) >= 3 else 1
        joint_velocities = gradient(
            continuous_positions,
            times,
            axis=0,
            edge_order=edge_order,
        )
        joint_accelerations = gradient(
            joint_velocities,
            times,
            axis=0,
            edge_order=edge_order,
        )

        # The quintic path has exactly zero endpoint velocity/acceleration.
        joint_velocities[[0, -1]] = 0.0
        joint_accelerations[[0, -1]] = 0.0

    return [
        {
            "time": time,
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
        }
        for time, position, velocity, acceleration in zip(
            times,
            joint_positions,
            joint_velocities,
            joint_accelerations,
        )
    ]

def calculate_move_time(q_start, q_end, v_max, a_max):
    q_start = array(q_start)
    q_end = array(q_end)

    v_max = array(v_max)
    a_max = array(a_max)

    dq = abs(q_end - q_start)

    # Velocity constraint
    T_velocity = 1.875 * dq / v_max

    # Acceleration constraint
    T_acceleration = sqrt(
        5.7735 * dq / a_max
    )

    T = max(
        maximum(
            T_velocity,
            T_acceleration
        )
    )

    return T
