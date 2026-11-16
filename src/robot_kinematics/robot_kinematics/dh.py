"""Nominal standard-DH model (mm, degrees), including the flange frame."""
import numpy as np

# a_i, alpha_i, d_i, theta offset; theta_i = model joint angle + offset.
DH_TABLE = np.array([
    [40., -90., 330., 0.],
    [445., 0., 0., -90.],
    [40., -90., 0., 0.],
    [0., 90., 440., 0.],
    [0., 90., 0., 180.],
    [0., 0., 67.5, 0.],
])
DH_TO_FLANGE = np.array([[0., 1., 0., 0.], [0., 0., 1., 0.],
                         [1., 0., 0., 0.], [0., 0., 0., 1.]])


def dh_frames(angles):
    """World transforms for DH frames 0..6; z_(i-1) is joint i's axis."""
    angles = np.asarray(angles, dtype=float)
    if angles.shape != (6,) or not np.isfinite(angles).all():
        raise ValueError('Expected six finite angles in degrees')
    frames = [np.eye(4)]
    for q, (a, alpha, d, offset) in zip(angles, DH_TABLE):
        theta, alpha = np.deg2rad([q + offset, alpha])
        c, s, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        A = np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                      [0., sa, ca, d], [0., 0., 0., 1.]])
        frames.append(frames[-1] @ A)
    return frames


def dh_fk(angles):
    """Nominal world-to-Link6 pose; CAD rounded RPY introduces micron offsets."""
    return dh_frames(angles)[-1] @ DH_TO_FLANGE
