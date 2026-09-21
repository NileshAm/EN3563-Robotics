"""Plot a 3-D path with velocity or acceleration represented by colour."""

from __future__ import annotations

from typing import Iterable, Literal, Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colormaps
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection


ColourBy = Literal["velocity", "acceleration"]


def _prepare_points(points: Iterable[Sequence[float]]) -> np.ndarray:
    """Validate and return an array of XYZ positions."""
    positions = np.asarray(list(points), dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("points must be a list/array with shape (n, 3)")
    if len(positions) < 2:
        raise ValueError("at least two points are required")
    if not np.isfinite(positions).all():
        raise ValueError("points must contain only finite numbers")
    return positions


def _prepare_values(
    values: Sequence[float] | Iterable[Sequence[float]],
    point_count: int,
    name: str,
) -> np.ndarray:
    """Return scalar values, converting XYZ vectors to magnitudes if needed."""
    data = np.asarray(list(values), dtype=float)
    if data.ndim == 2 and data.shape[1] == 3:
        data = np.linalg.norm(data, axis=1)
    elif data.ndim != 1:
        raise ValueError(f"{name} must contain scalars or XYZ vectors")

    if len(data) != point_count:
        raise ValueError(f"{name} must contain one value for each point")
    if not np.isfinite(data).all():
        raise ValueError(f"{name} must contain only finite numbers")
    return data


def _set_equal_3d_axes(ax: Axes, points: np.ndarray) -> None:
    """Give X, Y, and Z the same visual scale."""
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    centre = (lower + upper) / 2.0
    radius = max(float(np.max(upper - lower)) / 2.0, 0.5)

    ax.set_xlim(centre[0] - radius, centre[0] + radius)
    ax.set_ylim(centre[1] - radius, centre[1] + radius)
    ax.set_zlim(centre[2] - radius, centre[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def plot_trajectory(
    points: Iterable[Sequence[float]],
    values: Sequence[float] | Iterable[Sequence[float]],
    *,
    color_by: ColourBy = "velocity",
    ax: Axes | None = None,
    cmap: str = "turbo",
    line_width: float = 4.0,
) -> tuple[Figure, Axes, LineCollection]:
    """Plot a list of XYZ points as a colour-mapped 3-D trajectory.

    Parameters
    ----------
    points:
        XYZ coordinates, for example ``[(0, 0, 0), (1, 2, 3)]``.
    values:
        One velocity or acceleration value per point. Values may be scalars
        or XYZ vectors; vectors are converted to magnitudes automatically.
    color_by:
        ``"velocity"`` colours the line by speed; ``"acceleration"`` colours
        it by acceleration magnitude.
    ax:
        Optional existing 3-D axes. A new figure is created when omitted.

    Returns
    -------
    (figure, axes, line_collection)
        Useful when the caller wants to further customise or save the plot.
    """
    if color_by not in ("velocity", "acceleration"):
        raise ValueError("color_by must be 'velocity' or 'acceleration'")

    positions = _prepare_points(points)
    values = _prepare_values(values, len(positions), color_by)

    if ax is None:
        figure = plt.figure(figsize=(9, 7))
        ax = figure.add_subplot(111, projection="3d")
    else:
        figure = ax.figure
        if getattr(ax, "name", None) != "3d":
            raise ValueError("ax must be a 3-D matplotlib axes")

    # One colour value per segment; use the mean of its two end points.
    segments = np.stack((positions[:-1], positions[1:]), axis=1)
    segment_values = (values[:-1] + values[1:]) / 2.0
    value_min = float(segment_values.min())
    value_max = float(segment_values.max())
    if np.isclose(value_min, value_max):
        # Normalize needs a non-zero range for a useful, stable colour mapping.
        padding = max(abs(value_min) * 0.01, 1e-12)
        norm = Normalize(value_min - padding, value_max + padding)
    else:
        norm = Normalize(value_min, value_max)

    line = Line3DCollection(
        segments,
        cmap=colormaps[cmap],
        norm=norm,
        linewidth=line_width,
    )
    line.set_array(segment_values)
    ax.add_collection3d(line)

    ax.scatter(*positions[0], color="limegreen", s=55, label="Start", zorder=3)
    ax.scatter(*positions[-1], color="crimson", s=55, label="End", zorder=3)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"3-D trajectory colored by {color_by}")
    ax.legend()
    ax.grid(True)
    _set_equal_3d_axes(ax, positions)

    label = "Velocity (distance / time)" if color_by == "velocity" else "Acceleration (distance / time²)"
    figure.colorbar(line, ax=ax, pad=0.1, shrink=0.75, label=label)
    return figure, ax, line


def plot_velocity_and_acceleration(
    points: Iterable[Sequence[float]],
    velocities: Sequence[float] | Iterable[Sequence[float]],
    accelerations: Sequence[float] | Iterable[Sequence[float]],
) -> Figure:
    """Show velocity and acceleration colour maps next to each other."""
    # Materialise generators because the same points are plotted twice.
    point_list = list(points)
    figure = plt.figure(figsize=(15, 6))
    velocity_ax = figure.add_subplot(121, projection="3d")
    acceleration_ax = figure.add_subplot(122, projection="3d")
    plot_trajectory(point_list, velocities, color_by="velocity", ax=velocity_ax)
    plot_trajectory(point_list, accelerations, color_by="acceleration", ax=acceleration_ax)
    figure.tight_layout()
    return figure


if __name__ == "__main__":
    # Replace these sample values with your measured XYZ points and times.
    sample_points = [
        (0, 0, 0),
        (1, 0.5, 0.2),
        (2, 1.5, 0.9),
        (4, 2.0, 1.7),
        (7, 2.2, 3.0),
        (8, 4.0, 4.5),
        (8.5, 7.0, 5.0),
    ]
    sample_velocities = [0.0, 1.2, 2.1, 3.8, 4.4, 3.0, 1.0]
    sample_accelerations = [1.5, 1.3, 0.8, 0.4, -0.5, -1.2, -1.8]

    plot_velocity_and_acceleration(
        sample_points,
        sample_velocities,
        sample_accelerations,
    )
    plt.show()
