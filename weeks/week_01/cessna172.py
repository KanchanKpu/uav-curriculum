import hashlib
from collections import namedtuple
from dataclasses import dataclass, field, fields

import numpy as np


# Roskam Part 1 Table B1 (pg. 483) for stability and control derivatives
@dataclass
class CessnaLongitudinalConfig:
    V0: float  # ft/s
    m: float  # lbf
    I_yy: float  # slug-ft^2
    theta0: float  # degrees
    X_u: float  # 1/s
    X_w: float  # 1/s
    X_del_e: float  # ft/s^2/rad
    Z_u: float  # 1/s
    Z_w: float  # 1/s
    Z_q: float  # ft/s
    Z_w_dot: float  # unitless
    Z_del_e: float  # ft/s^2/rad
    M_u: float  # 1/ft/s
    M_w: float  # 1/ft/s
    M_q: float  # 1/s
    M_w_dot: float  # 1/ft/s^2
    M_del_e: float  # rad/s^2/rad
    monte_carlo_seed: int = 42  # rng seed
    vel_noise_std: float = 0.3  # ft/s
    gyro_noise_std: float = 0.005  # rad/s
    angle_noise_std: float = 0.000875  # rad
    angle_random_walk: float = 0.00175  # rad/s^0.5
    config_hash: str = field(init=False, repr=False, default="")  # SHA-256 hex digest

    def __post_init__(self):
        data_string = str(tuple(getattr(self, f.name) for f in fields(self) if f.init))
        self.config_hash = hashlib.sha256(data_string.encode("utf-8")).hexdigest()


LongStateMatrix = namedtuple("LongStateMatrix", ["A", "B"])


def build_state_space(config: CessnaLongitudinalConfig) -> LongStateMatrix:
    """
    Creates the Longitudinal A and B matrices used in a State Space Model:
    xdot = A\*x + B\*u, where A is 4x4, B is 4xN, and u is Nx1.

    x takes the form of [u, w, q, θ]'

    Parameters
    ----------
    config: CessnaLongitudinalConfig
        dataclass containing the longitudinal stability derivatives of Airplane A found
        in Roskam Part 1

    Returns
    -------
    A namedtuple with the following attributes:

        A : (4, 4) array
            State matrix that define the dynamics of the supplied aircraft model
        B : (4, N) array
            Control matrix that define how the model reacts to actuator input

    """

    Z_const = 1 - config.Z_w_dot  # Constant applied to all Z-terms (~1)
    g = 32.17  # acceleration caused by gravity, ft/s^2
    theta0_rad = np.deg2rad(config.theta0)  # Converting trim pitch from deg to rad
    A = np.array(
        [
            [config.X_u, config.X_w, 0, -g * np.cos(theta0_rad)],
            [
                config.Z_u / Z_const,
                config.Z_w / Z_const,
                (config.V0 + config.Z_q) / Z_const,
                -g * np.sin(theta0_rad),
            ],
            [
                config.M_u + config.M_w_dot * config.Z_u / Z_const,
                config.M_w + config.M_w_dot * config.Z_w / Z_const,
                config.M_q + config.M_w_dot * (config.V0 + config.Z_q) / Z_const,
                -g * np.sin(theta0_rad),
            ],
            [0, 0, 1, 0],
        ]
    )
    B = np.array(
        [
            [config.X_del_e],
            [config.Z_del_e / Z_const],
            [config.M_del_e + config.M_w_dot * config.Z_del_e / Z_const],
            [0],
        ]
    )

    return LongStateMatrix(A=A, B=B)
