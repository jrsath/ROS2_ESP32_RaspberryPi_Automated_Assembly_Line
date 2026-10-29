# The far sensor must clear when arm 3 lifts the toy, not when inspection finishes.
import numpy as np
from group4_project2_al.conveyor import range_occupied, belt_fraction


def test_range_sensor_distinguishes_toy_and_empty_belt():
    assert range_occupied(0.054, 0.064)
    assert not range_occupied(0.070, 0.064)
    assert not range_occupied(float("inf"), 0.064)
    assert not range_occupied(float("nan"), 0.064)
    assert not range_occupied(0, 0.064)


def test_position_bar_uses_distance():
    start = np.array([-0.25, 0.21, 1.0779])
    stop = np.array([0.24, 0.21, 1.0779])
    assert belt_fraction(start, start, stop) == 0
    assert belt_fraction((start + stop) / 2, start, stop) == 0.5
    assert belt_fraction(stop, start, stop) == 1


def test_configured_beam_hits_base_wall_above_empty_belt():
    # Check the actual STL: a beam over the hollow centre misses the raised wall.
    from pathlib import Path
    import yaml
    from group4_project2_al.vision import triangles

    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / "config/production.yaml").read_text())
    sensor = config["conveyor_sensor"]
    origin = np.asarray(sensor["beam_offset"])
    mesh = triangles(root / "models/product/1V4_centered.stl")
    edge1, edge2 = mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0]
    direction = np.array([0.0, 0.0, -1.0])
    cross = np.cross(direction, edge2)
    determinant = np.einsum("ij,ij->i", edge1, cross)
    inverse = np.divide(
        1.0, determinant, out=np.zeros_like(determinant), where=abs(determinant) > 1e-12
    )
    offset = origin - mesh[:, 0]
    u = np.einsum("ij,ij->i", offset, cross) * inverse
    q = np.cross(offset, edge1)
    v = (q @ direction) * inverse
    distance = np.einsum("ij,ij->i", edge2, q) * inverse
    hits = (
        (abs(determinant) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (distance > 0)
    )
    assert hits.any()
    assert range_occupied(float(distance[hits].min()), sensor["trigger_range_m"])
    belt_range = config["conveyor_out"][2] + origin[2] - 0.070
    assert not range_occupied(belt_range, sensor["trigger_range_m"])
