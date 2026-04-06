import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

from weeks.week_01 import cessna172


def simulate_longitudinal(
    config: cessna172.CessnaLongitudinalConfig,
    x0: np.ndarray,
    t_span,
    t_eval,
    elevator_cmd,
) -> pd.DataFrame:
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


def extract_modal_parameters(A):
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


def add_sensor_noise(df, config, rng: np.random.Generator):
    gyro_noise_std = config.gyro_noise_std
    vel_noise_std = config.vel_noise_std
    angle_random_walk = config.angle_random_walk
    angle_noise_std = config.angle_noise_std
    data_size = len(df)

    def generate_noise(std, loc=0):
        return rng.normal(loc=0, scale=std, size=data_size)

    df_new = df.copy()
    df_new.loc[:, "u_ft_s"] = df.u_ft_s + generate_noise(vel_noise_std)
    df_new.loc[:, "w_ft_s"] = df.w_ft_s + generate_noise(vel_noise_std)
    df_new.loc[:, "q_rad_s"] = df.q_rad_s + generate_noise(gyro_noise_std)
    df_new.loc[:, "theta_rad"] = df.theta_rad + generate_noise(
        std=angle_noise_std, loc=angle_random_walk * np.sqrt(df.index.values)
    )

    return df_new
