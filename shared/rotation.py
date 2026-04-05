import numpy as np


def rotation_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Provides a body->inertial frame rotation matrix

    Parameters
    ----------
    phi : float
        roll angle [rad]
    theta : float
        pitch angle [rad]
    psi : float
        yaw angle [rad]

    Returns
    -------
    np.ndarray
        body to inertial rotation matrix

    """
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)

    c_theta = np.cos(theta)
    s_theta = np.sin(theta)

    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    R_roll = np.array([[1, 0, 0], [0, c_phi, -s_phi], [0, s_phi, c_phi]])
    R_pitch = np.array([[c_theta, 0, s_theta], [0, 1, 0], [-s_theta, 0, c_theta]])
    R_yaw = np.array([[c_psi, -s_psi, 0], [s_psi, c_psi, 0], [0, 0, 1]])

    return R_yaw @ R_pitch @ R_roll
