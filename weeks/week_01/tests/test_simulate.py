import json

import numpy as np

from weeks.week_01 import cessna172, simulation


def get_config():
    with open("weeks/week_01/data/airplane_a.json", mode="r") as f:
        plane_json = json.load(f)
    return cessna172.CessnaLongitudinalConfig(**plane_json)


def test_modal():
    config = get_config()
    A, _ = cessna172.build_state_space(config)
    modal = simulation.extract_modal_parameters(A)
    assert np.isclose(modal["shortperiod_omega_n"], 5.2707, rtol=0.01)
    assert np.isclose(modal["phugoid_omega_n"], 0.1711, rtol=0.01)


def test_simulate_response():
    config = get_config()
    t_eval = np.linspace(0, 20, 2001)
    df = simulation.simulate_longitudinal(
        config=config, x0=[0, 0, 0, 0], t_span=(0, 60), t_eval=t_eval, elevator_cmd=0.01
    )
    assert df.shape == (len(t_eval), 4)
    assert not np.allclose(df["theta_rad"].values, 0.0, atol=1e-8)


def test_sensor_noise():
    config = get_config()
    rng = np.random.default_rng(seed=config.monte_carlo_seed)
    rng2 = np.random.default_rng(seed=config.monte_carlo_seed)
    delta_e = 2
    df_clean = simulation.simulate_longitudinal(
        config=config,
        x0=[0, 0, 0, 0],
        t_span=(0, 120),
        t_eval=np.linspace(0, 120, 12001),
        elevator_cmd=np.deg2rad(delta_e),
    )
    df_clean_copy = df_clean.copy()
    df_noisy = simulation.add_sensor_noise(df=df_clean, config=config, rng=rng)
    df_noisy2 = simulation.add_sensor_noise(df=df_clean, config=config, rng=rng2)

    residual_std = (df_noisy["q_rad_s"] - df_clean["q_rad_s"]).std()
    assert np.allclose(residual_std, config.gyro_noise_std, rtol=0.20)
    assert np.allclose(
        df_noisy["u_ft_s"].values, df_noisy2["u_ft_s"].values, atol=1e-12
    )
    assert np.allclose(
        df_clean["theta_rad"].values, df_clean_copy["theta_rad"].values, atol=1e-12
    )
