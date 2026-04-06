import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

from weeks.week_01 import cessna172


def simulate_longitudinal(
    config: cessna172.CessnaLongitudinalConfig,
    x0,
    t_span,
    t_eval,
    elevator_cmd: float,
) -> pd.DataFrame:
    """
    Simulates the longitudinal dynamics of a given aircraft configuration for a specific
    initial condition and elevator hold input

    Parameters
    ----------
    config: CessnaLongitudinalConfig
        dataclass containing the longitudinal stability derivatives of Airplane A found
        in Roskam Part 1
    x0: array_like, shape \(4, )
        Initial state. Takes the form [u, w, q, theta]
    t_span: 2-member sequence
        Interval of integration (t0, tf). Solver starts at t0 and integrates until tf
    t_eval: array_like
        Times at which to store the computed solution. Must fall within *t_span*
    elevator_cmd: float
        Angle (rad) at which the elevator is held for the entire simulation. Positive is
        a downwards deflection

    Returns
    -------
    DataFrame object with the index set to *t_eval*. Takes the following structure:

        time_s | u_ft_s | w_ft_s | q_rad_s | theta_rad

    """

    A, B = cessna172.build_state_space(config)
    sol = solve_ivp(
        lambda t, x: A @ x + B @ np.array([elevator_cmd]),
        t_span=t_span,
        y0=x0,
        t_eval=t_eval,
        method="RK45",
    )
    df = pd.DataFrame(sol.y.T, columns=["u_ft_s", "w_ft_s", "q_rad_s", "theta_rad"])
    df["time_s"] = sol.t
    df.set_index("time_s", inplace=True)
    return df


def extract_modal_parameters(A) -> dict:
    """
    Determines the natural frequency \(w_n) and damping ratio \(zeta) of the phugoid and
    short period modes of a given state space model A.

    Parameters
    ----------
    A: \(4, 4) array
        Matrix for which the modal parameters will be computed

    Returns
    -------
    A dictionary containing the following key\:value pairs:
        -**phugoid_omega_n**:   phugoid mode natural frequency

        -**phugoid_zeta**:  phugoid mode damping ratio

        -**shortperiod_omega_n**:   short period natural frequency

        -**shortperiod_zeta**:  short period damping ratio

    """
    eigenvalues, _ = np.linalg.eig(A)
    w_n = abs(eigenvalues)
    zeta = -np.real(eigenvalues) / w_n
    idx_sp = np.argmax(w_n)
    idx_p = np.argmin(w_n)
    modal_parameters = {
        "phugoid_omega_n": w_n[idx_p],
        "phugoid_zeta": zeta[idx_p],
        "shortperiod_omega_n": w_n[idx_sp],
        "shortperiod_zeta": zeta[idx_sp],
    }

    return modal_parameters


def add_sensor_noise(
    data: pd.DataFrame,
    config: cessna172.CessnaLongitudinalConfig,
    rng: np.random.Generator,
) -> pd.DataFrame:
    """
    Creates a new set of data with Gaussian White Noise added to the given data entries.
    The nature of the white noise is determined by the noise parameters found in
    *config*

    Parameters
    ----------
    data: DataFrame
        Contains the longitudinal response of a given aircraft configuration. Takes the
        following structure:

            time_s | u_ft_s | w_ft_s | q_rad_s | theta_rad

    config: CessnaLongitudinalConfig
        dataclass containing the longitudinal stability derivatives of Airplane A found
        in Roskam Part 1
    rng: Generator
        numpy Generator object

    Returns
    -------
    **DataFrame object**
        Copy of *data* with gaussian white noise added to all entries

    """

    gyro_noise_std = config.gyro_noise_std
    vel_noise_std = config.vel_noise_std
    angle_random_walk = config.angle_random_walk
    angle_noise_std = config.angle_noise_std
    data_size = len(data)

    def generate_noise(std, loc=0):
        return rng.normal(loc=0, scale=std, size=data_size)

    data_new = data.copy()
    data_new.loc[:, "u_ft_s"] = data.u_ft_s + generate_noise(vel_noise_std)
    data_new.loc[:, "w_ft_s"] = data.w_ft_s + generate_noise(vel_noise_std)
    data_new.loc[:, "q_rad_s"] = data.q_rad_s + generate_noise(gyro_noise_std)
    data_new.loc[:, "theta_rad"] = data.theta_rad + generate_noise(
        std=angle_noise_std, loc=angle_random_walk * np.sqrt(data.index.values)
    )

    return data_new
