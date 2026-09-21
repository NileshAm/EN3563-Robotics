"""Shared GP7 forward/inverse kinematics: degrees, millimetres, world frame."""

from .utils import generateH
from numpy import array, linalg, zeros, eye, max as maxnp, abs as absnp, concatenate, isfinite
from scipy.spatial.transform import Rotation


def _joint_transform(translation_mm, rpy, axis, angle_degrees):
    """Build a URDF joint transform using millimetres and degrees."""
    origin = generateH(
        Rotation.from_euler("xyz", rpy).as_matrix(),
        translation_mm,
    )
    joint_rotation = generateH(
        Rotation.from_rotvec(
            array(axis, dtype=float) * angle_degrees,
            degrees=True,
        ).as_matrix(),
        [0.0, 0.0, 0.0],
    )
    return origin @ joint_rotation


def H01(angle):
    return _joint_transform(
        [0.0, 0.0, 190.1],
        [1.5708, 0.0, 1.5708],
        [0.0, 1.0, 0.0],
        angle,
    )


def H12(angle):
    return _joint_transform(
        [0.0, 139.9, -40.0],
        [1.5708, -1.5708, 0.0],
        [0.0, 1.0, 0.0],
        angle,
    )


def H23(angle):
    return _joint_transform(
        [0.0, 0.0, -445.0],
        [3.1416, 0.0, 3.1416],
        [0.0, 1.0, 0.0],
        angle,
    )


def H34(angle):
    return _joint_transform(
        [71.5, 0.0, 40.0],
        [1.5708, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        angle,
    )


def H45(angle):
    return _joint_transform(
        [368.5, 0.0, 0.0],
        [3.1416, 0.0, 1.5708],
        [0.0, 0.0, 1.0],
        angle,
    )


def H56(angle):
    return _joint_transform(
        [0.0, 67.5, 0.0],
        [0.0, 0.0, 1.5708],
        [1.0, 0.0, 0.0],
        angle,
    )


def H06(a1, a2, a3, a4, a5, a6):
    """Return the world-to-Link6 transform in millimetres."""
    world_to_base = generateH(
        [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 0.0],
    )
    return (
        world_to_base
        @ H01(a1)
        @ H12(a2)
        @ H23(a3)
        @ H34(a4)
        @ H45(a5)
        @ H56(a6)
    )


def translation_jacobian(joint_angles, step_degrees=1e-4):
    """Return the 3x6 translational Jacobian in mm/degree.

    ``H06`` uses degrees, so this Jacobian deliberately differentiates with
    respect to degrees. Multiplying it by joint velocities in degrees/second
    therefore produces an XYZ velocity in mm/second.
    """
    q = array(joint_angles, dtype=float)
    if q.shape != (6,):
        raise ValueError("joint_angles must contain exactly 6 values")
    if not isfinite(q).all():
        raise ValueError("joint_angles must contain finite numbers")
    if step_degrees <= 0:
        raise ValueError("step_degrees must be greater than zero")

    jacobian = zeros((3, 6))
    for joint in range(6):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[joint] += step_degrees
        q_minus[joint] -= step_degrees
        p_plus = H06(*q_plus)[:3, 3]
        p_minus = H06(*q_minus)[:3, 3]
        jacobian[:, joint] = (
            (p_plus - p_minus) / (2.0 * step_degrees)
        )

    return jacobian


def joint_to_cartesian_motion(
        joint_angles,
        joint_velocity,
        joint_acceleration,
        jacobian_step_degrees=1e-4,
        curvature_step_degrees=0.05):
    """Convert joint motion to end-effector XYZ velocity and acceleration.

    Inputs use degrees, degrees/second, and degrees/second squared. Returned
    vectors use mm/second and mm/second squared. The acceleration includes
    both ``J @ q_ddot`` and the velocity-dependent ``J_dot @ q_dot`` term.
    """
    q = array(joint_angles, dtype=float)
    q_dot = array(joint_velocity, dtype=float)
    q_ddot = array(joint_acceleration, dtype=float)

    for name, values in (
            ("joint_angles", q),
            ("joint_velocity", q_dot),
            ("joint_acceleration", q_ddot)):
        if values.shape != (6,):
            raise ValueError(f"{name} must contain exactly 6 values")
        if not isfinite(values).all():
            raise ValueError(f"{name} must contain finite numbers")
    if curvature_step_degrees <= 0:
        raise ValueError("curvature_step_degrees must be greater than zero")

    jacobian = translation_jacobian(q, jacobian_step_degrees)
    xyz_velocity = jacobian @ q_dot

    # J_dot @ q_dot is the second directional derivative of position in the
    # q_dot direction. Normalising the direction keeps the finite-difference
    # displacement small and independent of the actual joint speed.
    maximum_joint_speed = float(maxnp(absnp(q_dot)))
    if maximum_joint_speed <= 1e-12:
        velocity_dependent_acceleration = zeros((3,))
    else:
        direction = q_dot / maximum_joint_speed
        q_plus = q + direction * curvature_step_degrees
        q_minus = q - direction * curvature_step_degrees
        p = H06(*q)[:3, 3]
        p_plus = H06(*q_plus)[:3, 3]
        p_minus = H06(*q_minus)[:3, 3]
        curvature = (
            (p_plus - 2.0 * p + p_minus)
            / curvature_step_degrees**2
        )
        velocity_dependent_acceleration = (
            curvature * maximum_joint_speed**2
        )

    xyz_acceleration = (
        jacobian @ q_ddot + velocity_dependent_acceleration
    )
    return xyz_velocity, xyz_acceleration


def pose_error(q, target):
    current = H06(q[0], q[1], q[2], q[3], q[4], q[5])

    # Target position minus current position
    position_error = target[:3, 3] - current[:3, 3]

    # Rotation needed to reach the target orientation
    rotation_error = Rotation.from_matrix(
        target[:3, :3] @ current[:3, :3].T
    ).as_rotvec()

    return concatenate((position_error, rotation_error))


def IK(
        target,
        initial_angles,
        max_iterations=300,
        learning_rate=1.0,
        max_step_degrees=5.0,
        position_tolerance=0.1,
        rotation_tolerance=1e-3,
        damping=0.1,
        seed_weight=1.0):
    """Solve IK using damped least squares.

    Joint angles, finite-difference steps, and joint updates are in degrees.
    Position values and ``position_tolerance`` are in millimetres, while
    rotation errors and ``rotation_tolerance`` are in radians. ``seed_weight``
    selects the valid IK branch nearest to ``initial_angles``; use the robot's
    current joint state as ``initial_angles`` during motion planning.
    """
    if learning_rate <= 0:
        raise ValueError("learning_rate must be greater than zero")
    if max_step_degrees <= 0:
        raise ValueError("max_step_degrees must be greater than zero")
    if position_tolerance <= 0 or rotation_tolerance <= 0:
        raise ValueError("IK tolerances must be greater than zero")
    if damping <= 0:
        raise ValueError("damping must be greater than zero")
    if seed_weight < 0:
        raise ValueError("seed_weight cannot be negative")

    q = array(initial_angles, dtype=float).copy()
    preferred_q = q.copy()
    epsilon_degrees = 1e-4

    # Scaling by the requested tolerances prevents position errors in mm from
    # overwhelming rotation errors in radians in the least-squares solve.
    error_scale = array([
        1.0 / position_tolerance,
        1.0 / position_tolerance,
        1.0 / position_tolerance,
        1.0 / rotation_tolerance,
        1.0 / rotation_tolerance,
        1.0 / rotation_tolerance,
    ])
    current_damping = damping
    identity = eye(6)

    for _ in range(max_iterations):
        error = pose_error(q, target)

        # Stop when both configured pose tolerances are satisfied.
        if (linalg.norm(error[:3]) <= position_tolerance
                and linalg.norm(error[3:]) <= rotation_tolerance):
            return q, True

        scaled_error = error * error_scale

        # Numerical Jacobian of the ERROR:
        # measure how each joint changes the error.
        J = zeros((6, 6))
        for i in range(6):
            perturbed_q = q.copy()
            perturbed_q[i] += epsilon_degrees
            J[:, i] = (
                pose_error(perturbed_q, target) - error
            ) * error_scale / epsilon_degrees

        # Keep the solution on the branch nearest the supplied initial joint
        # state. Fade this secondary objective near the target so it cannot
        # hold the solver just outside the requested pose tolerance.
        scaled_error_norm = linalg.norm(scaled_error)
        effective_seed_weight = seed_weight * min(
            1.0,
            scaled_error_norm / 100.0,
        )
        seed_delta = q - preferred_q
        seed_weight_squared = effective_seed_weight**2

        # Damped least-squares correction. Expanding the augmented least-
        # squares system directly avoids allocating a 12-by-6 matrix every
        # iteration.
        dq = -linalg.solve(
            J.T @ J
            + (seed_weight_squared + current_damping**2) * identity,
            J.T @ scaled_error + seed_weight_squared * seed_delta,
        )

        # q is measured in degrees, so the update limit is also in degrees.
        dq *= min(
            1.0,
            max_step_degrees / max(maxnp(absnp(dq)), 1e-12),
        )

        # A Jacobian is only a local approximation. Backtracking prevents a
        # large-distance iteration from accepting a step that increases the
        # actual pose error.
        current_cost = (
            scaled_error @ scaled_error
            + seed_weight_squared * (seed_delta @ seed_delta)
        )
        step_scale = learning_rate
        accepted = False

        while step_scale >= 1e-4:
            candidate_q = q + step_scale * dq
            candidate_pose_error = pose_error(candidate_q, target) * error_scale
            candidate_seed_delta = candidate_q - preferred_q
            candidate_cost = (
                candidate_pose_error @ candidate_pose_error
                + seed_weight_squared
                * (candidate_seed_delta @ candidate_seed_delta)
            )

            if candidate_cost < current_cost:
                q = candidate_q
                current_damping = max(damping, current_damping * 0.5)
                accepted = True
                break

            step_scale *= 0.5

        if not accepted:
            # Near a singularity, a larger damping value produces a smaller,
            # safer correction on the next iteration.
            current_damping = min(current_damping * 10.0, 1e6)

    return q, False
