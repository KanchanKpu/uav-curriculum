import json

import numpy as np

from weeks.week_01 import cessna172, simulation

with open("weeks/week_01/data/airplane_a.json", "r") as file:
    cessna_json = json.load(file)

plane_config = cessna172.CessnaLongitudinalConfig(**cessna_json)

rng = np.random.default_rng(seed=plane_config.monte_carlo_seed)
rng2 = np.random.default_rng(seed=plane_config.monte_carlo_seed)
delta_e = 2
response_clean = simulation.simulate_longitudinal(
    config=plane_config,
    x0=[0, 0, 0, 0],
    t_span=(0, 120),
    t_eval=np.linspace(0, 120, 12001),
    elevator_cmd=np.deg2rad(delta_e),
)
response_noisy = simulation.add_sensor_noise(
    df=response_clean, config=plane_config, rng=rng
)
with open("weeks/week_01/data/cessna_clean.csv", "w") as f:
    f.write(f"# Aircraft Config Hash:, {plane_config.config_hash}\n")
    f.write(f"# Elevator Deflection:, {delta_e}, deg\n")
with open("weeks/week_01/data/cessna_noisy.csv", "w") as f:
    f.write(f"# Aircraft Config Hash:, {plane_config.config_hash}\n")
    f.write(f"# Elevator Deflection:, {delta_e}, deg\n")
response_clean.to_csv("weeks/week_01/data/cessna_clean.csv", mode="a", index=False)
response_noisy.to_csv("weeks/week_01/data/cessna_noisy.csv", mode="a", index=False)

axes = response_clean.plot(subplots=True, grid=True, layout=(4, 1))
axes2 = response_noisy.plot(subplots=True, grid=True, layout=(4, 1))

A, B = cessna172.build_state_space(plane_config)
