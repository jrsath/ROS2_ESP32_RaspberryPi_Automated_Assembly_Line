# Geometry for the far-end proximity sensor and the control-panel position display.
import numpy as np


def range_occupied(distance, threshold):
    return bool(np.isfinite(distance) and 0.003 <= distance < threshold)


def belt_fraction(position, start, stop):
    direction = np.asarray(stop) - start
    return float(
        np.clip(
            np.dot(np.asarray(position) - start, direction)
            / np.dot(direction, direction),
            0.0,
            1.0,
        )
    )
