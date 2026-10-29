import numpy as np
from group4_project2_al.collision_check import overlap


def test_separated_and_rotated_boxes():
    identity = np.eye(3)
    a = (np.zeros(3), identity, np.ones(3) * 0.01)
    assert overlap(a, (np.array([0.01, 0, 0]), identity, np.ones(3) * 0.01))
    assert not overlap(a, (np.array([0.03, 0, 0]), identity, np.ones(3) * 0.01))
