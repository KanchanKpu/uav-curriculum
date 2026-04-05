import numpy as np

from shared.rotation import rotation_matrix


def test_rotation():
    assert np.allclose(rotation_matrix(0.0, 0.0, 0.0), np.eye(3), atol=1e-12)
    R = rotation_matrix(0.1, 0.2, 0.3)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.allclose(np.linalg.det(R), 1.0, atol=1e-12)
    R2 = rotation_matrix(0, 0, np.pi / 2)
    assert np.allclose(R2, np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), atol=1e-12)
