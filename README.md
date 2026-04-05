# Semester Curriculum — Autonomous Fixed-Wing UAV: Design, Simulation, and Flight
### A Python-First Engineering Curriculum for Aerospace Undergraduates

---

## How to Use This Document

This outline is designed to be consumed by a senior aerospace engineer (human or AI) who will
populate each week with a concrete daily lesson plan. Each section below contains:

- **Theme and engineering objective** — the "why" that motivates the week's work
- **Prerequisite knowledge** — what the student must have completed before starting
- **Core concepts to teach** — the subject matter, with enough specificity to generate rigorous daily tasks
- **Python ecosystem** — the general-purpose scientific Python libraries introduced or exercised this week
- **Capstone deliverable** — the tangible artifact produced by end of week
- **Connection to the larger UAV goal** — how this week feeds directly into later weeks and the final flying vehicle
- **Recommended references** — textbooks and documentation for deeper study

The student has **8 hours per day** and works best with **small daily milestones** that
accumulate into a weekly capstone. Each day should contain: a concept lesson explaining the
mathematical foundation and its connection to the UAV goal, explicit MATLAB-to-Python
analogies where relevant, syntax warmup tasks introducing new library functions with brief
usage examples, and 2–4 milestones with quantitative test cases expressed as `assert`-level
numerical criteria with specific tolerances.

**Day 5 of every week is integration and validation day.** No new concepts are introduced on
Day 5. Its sole purpose is to wire together everything built on Days 1–4, make `pytest` pass
completely, and produce the week's capstone output. An LLM populating a week must schedule
Day 5 accordingly — it is never a concept day.

The student has a strong MATLAB/Simulink background and basic Python. All weeks should draw
explicit MATLAB-to-Python analogies where relevant, and should enforce good software
engineering habits (modules, version control, assertions, unit tests, logging) as a running
thread throughout every week.

### Library Philosophy

**All implementations use only general-purpose scientific Python libraries: `numpy`, `scipy`,
`pandas`, `matplotlib`, `plotly`, `python-control`, `pytest`, `json`, `struct`, `subprocess`,
`threading`, `socket`, `logging`, and `time`.** Specialized aerospace libraries (AeroSandbox,
filterpy, navpy, pymavlink wrappers, etc.) are intentionally excluded. The student implements
every domain-specific algorithm from first principles — vortex lattice solvers, Kalman
filters, coordinate transforms, propeller models, MAVLink framers. This is the difference
between an engineer who understands their tools and one who depends on them. Where an external
executable is invoked (AVL, XFOIL), the Python layer is written from scratch using
`subprocess` and custom file parsers — no wrapper libraries.

---

## Budget and Hardware Note

The hardware required for Phase 4 (Weeks 11–14) costs approximately **$300–400** for a
complete flying system. The major line items are a MAVLink-compatible flight controller
($25–90 depending on model), a companion computer such as a Raspberry Pi Zero 2W ($15),
a GPS module ($20–35), a budget RC transmitter and receiver ($25–40 used), motor/ESC/
propeller ($25–40), a 3S LiPo battery ($20–30), servos ($15–25), and miscellaneous wiring
and connectors ($20–30). The airframe can be built for near-zero cost from foam board (Dollar
Tree foam board at ~$1/sheet is the recommended platform — it is structurally appropriate for
a beginner, directly manufacturable from the Week 4–5 design, and repairable in the field).

**Hardware should be ordered at the end of Week 6**, after the propulsion system design is
complete and the motor, propeller, and battery specifications are known. This timing provides
a 4-week buffer before hardware is needed in Week 11 and ensures the student orders exactly
what the design requires rather than guessing. A procurement checklist is part of the Week 6
capstone deliverable.

**Flight regulatory compliance:** Join the Academy of Model Aeronautics (AMA) before any
outdoor flight. AMA membership (~$85/year) provides liability insurance and site access at
AMA-chartered fields, which is the appropriate and cost-effective path for a student flying
recreationally. Register the aircraft with the FAA (free for AMA members under the
club-based registration program). All outdoor flights must be conducted in accordance with AMA
safety codes and local site rules. These steps must be completed before Week 13 ground testing
begins — add this as a background task starting in Week 1.

**Flight controller note:** The curriculum refers generically to "a MAVLink-compatible flight
controller." The Holybro Pixhawk 4 Mini ($60–90) is the reference hardware for all MAVLink
and HIL sections, but any ArduPlane-compatible controller with a UART companion computer port
is acceptable. The Matek F405-Wing ($25–40) is a budget-viable alternative. All software is
written against the MAVLink protocol, not against any specific flight controller model.

---

## Semester Arc

The curriculum is organized into **four phases** that mirror the real engineering development
cycle for a UAV program:

| Phase | Weeks | Theme |
|---|---|---|
| **Phase 1 — Foundations** | 1–3 | Python fluency, flight mechanics, simulation, navigation |
| **Phase 2 — Design** | 4–6 | Aerodynamic analysis, structural sizing, propulsion |
| **Phase 3 — Autonomy** | 7–10 | Autopilot, guidance, mission planning, atmospheric disturbances |
| **Phase 4 — Integration & Flight** | 11–14 | Hardware interfaces, real-time systems, HIL testing, flight |

By the end of the semester the student will have: a parameterized 6-DOF flight simulator, a
from-scratch aerodynamic analysis pipeline, a full autopilot stack, and the software
infrastructure to support hardware-in-the-loop testing and a real flight attempt. Every
component will be code the student wrote and understands line by line.

---

---

# PHASE 1 — FOUNDATIONS
## Weeks 1–3: Python Fluency, Flight Mechanics, and Navigation

---

## Week 1 — Repository Setup, Software Engineering Foundations, and Python for Aerospace Controls

### Theme
Before writing a single line of simulation code, establish the GitHub repository, virtual
environment, and software engineering conventions that will be used for the entire semester.
Then transition from MATLAB to Python by rebuilding familiar aerospace concepts — kinematics,
ODE simulation, signal processing, and classical and modern control — using a Cessna 172
linearized longitudinal model as the vehicle. Day 1 is dedicated entirely to Python tooling
and NumPy warmup before any physics appears. Days 2–4 each add one technical layer to the
growing simulation. Day 5 is integration and validation day. The student leaves this week
with both a correctly structured codebase and a working simulation-and-control workbench they
understand at every line of code.

**Note for LLM population:** The linear Kalman filter has been deliberately moved out of Week
1 and into Day 1 of Week 3, where it serves as a direct conceptual setup for the EKF. Week 1
ends with LQR and the observer — which is still ambitious but coherent. Do not reintroduce
the Kalman filter into Week 1.

### Prerequisite Knowledge
- MATLAB matrix operations and ODE solvers (`ode45`, `lsim`, `eig`)
- Basic Python syntax: lists, loops, functions, `import` statements
- Undergraduate control theory through state-space representation
- A GitHub account and Git installed locally

### Repository Structure (scaffolded on Day 1)

The following top-level layout is created on Day 1 before any simulation code is written.
Every subsequent week adds a new directory to this scaffold; nothing is ever renamed or
moved.

```
uav-curriculum/                         ← GitHub repository root
│
├── .gitignore
├── .pre-commit-config.yaml
├── README.md
├── requirements.txt
├── requirements.in                     ← direct dependencies only, unpinned
│
├── weeks/
│   ├── week_01/
│   │   ├── __init__.py
│   │   ├── main.py
│   │   ├── tests/
│   │   ├── data/
│   │   └── figures/
│   └── ...                             ← week_02 through week_14 added each week
│
├── uav/                                ← integrated stack, assembled from Week 7 onward
│   ├── __init__.py
│   ├── config.py                       ← UAVConfig dataclass (student's aircraft)
│   ├── sim/
│   ├── nav/
│   ├── autopilot/
│   │   ├── inner_loop/
│   │   ├── outer_loop/
│   │   ├── guidance/
│   │   └── disturbances/
│   ├── hardware/
│   ├── hil/
│   └── mission/
│
├── shared/                             ← utilities used by more than one week
│   ├── __init__.py
│   ├── rotation.py                     ← rotation_matrix() promoted from week_01
│   ├── atmosphere.py                   ← isa_density() promoted from week_02
│   └── coordinate_frames.py           ← NED/ECEF/LLA promoted from week_03
│
└── docs/
    └── weekly_summaries/
        ├── week_01_summary.md
        └── ...
```

Each `weeks/week_NN/` directory is a self-contained Python package. It may import from
`shared/` but never from another `weeks/` directory. When a module is needed in a later week,
it is imported from `uav/` or `shared/` — not from `weeks/`. The `uav/` directory is the
single source of truth for the integrated stack.

### Core Concepts

**Day 1 — Python tooling and NumPy warmup (no aerospace content):** The first half of Day 1
is pure tooling: create the GitHub remote, clone locally, create a virtual environment with
`python -m venv`, install packages, pin to `requirements.txt` using `pip freeze --local`, and
make the first commit. Understand the difference between `requirements.in` (direct
dependencies, unpinned, human-maintained) and `requirements.txt` (full pinned lockfile,
generated). Configure pre-commit hooks (`black`, `isort`, `flake8`) and run `pre-commit
install`. Set up GitHub Actions CI: a single workflow file (`.github/workflows/ci.yml`) that
installs dependencies and runs `pytest` on every push to any `week-NN` branch. This is 15
lines of YAML and costs nothing for public repositories — from Day 1, every push is
automatically tested. The second half of Day 1 is a NumPy warmup with no aerospace content:
array creation, 0-indexing, the `@` vs `*` distinction (matrix multiply vs. element-wise —
the opposite of MATLAB's default), broadcasting rules, `np.linalg.eig`, and vectorized
operations vs. Python loops. The student writes 2–3 short warmup scripts that each produce
an `assert`-level result before any physics appears.

**NumPy arrays and broadcasting for aerospace:** Rotation matrices in the ZYX Euler
convention and numerical integration via `np.cumsum` are the first aerospace applications of
NumPy. Broadcasting, 0-indexing, and the performance difference between vectorized operations
and Python loops are enforced throughout the semester. Every physical constant carries a unit
comment on the same line: `rho = 1.225  # air density at sea level [kg/m³]`. Magic numbers
are an error.

**Pandas DataFrames for simulation data:** Simulation state histories are structured as
typed, labeled DataFrames from Day 2 onward — not raw NumPy arrays. The key concepts are
label-based vs. integer indexing, time-series resampling, rolling window statistics, and
sensor noise injection using `numpy.random.default_rng` with explicit seeds for
reproducibility. All stochastic simulations seed the RNG from a value stored in the
configuration object (default seed: 42). The CSV I/O pattern established here is used in
every subsequent week. Every simulation result is associated with the configuration that
produced it (see `config_hash` in Appendix A).

**Matplotlib and Plotly visualization:** Object-oriented Matplotlib (`fig, ax =
plt.subplots()`) is the static figure standard; Plotly with `make_subplots` and
`fig.write_html()` is the interactive dashboard standard. The `build_dashboard(df) →
go.Figure` function pattern introduced this week is reused without modification through Week
10.

**ODE simulation with `scipy.integrate.solve_ivp`:** `solve_ivp` is the Python equivalent of
MATLAB's `ode45` — the RK45 method, a callable derivative function, and `t_eval` for output
times. The Cessna 172 linearized longitudinal model (`ẋ = Ax + Bu`, four states: u, w, q, θ)
is the first physics-based system. Eigenanalysis of the A matrix recovers the short-period
and phugoid modal parameters, which the student verifies against published Cessna 172 values.

**Signal processing:** FFT via `numpy.fft`, Welch PSD via `scipy.signal.welch`, and
Butterworth filtering via `scipy.signal.butter` and `scipy.signal.filtfilt` are introduced
in the context of processing the noisy simulated sensor stream. The concepts of Nyquist
frequency, aliasing, and zero-phase filtering are connected directly to why the navigation
filter needs clean sensor inputs.

**Classical control with `python-control`:** `python-control` maps directly to MATLAB's
Control System Toolbox. PID design, Bode plots, gain and phase margins, and root locus are
covered using the longitudinal transfer function from elevator to pitch angle. The target
robustness criteria (gain margin ≥ 6 dB, phase margin ≥ 45°) are introduced here and applied
in every subsequent control design week.

**Modern control — LQR, observer, and separation principle:** LQR via `control.lqr`, pole
placement via `control.place`, and the Luenberger observer close the week's new content. The
separation principle — that observer and controller can be designed independently for LTI
systems — is the key theorem. Controllability and observability rank checks are the required
precondition. The combined LQR+observer architecture is the LQG baseline that Week 7 extends
to the full 6-DOF system.

### Python Ecosystem
`numpy`, `scipy` (`integrate`, `signal`, `linalg`), `pandas`, `matplotlib`, `plotly`,
`python-control`, `pytest`, `logging`

### Tooling (Day 1 only)
`git`, GitHub remote setup, GitHub Actions CI, `python -m venv`, `pip`, `requirements.txt`,
`requirements.in`, `pre-commit`

### Capstone Deliverable
`weeks/week_01/` — a modular Python package that simulates the Cessna 172 longitudinal
dynamics, runs PID and LQR+observer controllers side by side, and produces an interactive
Plotly dashboard comparing settling time, overshoot, peak actuator deflection, and
steady-state error for each controller. Running `python main.py` executes the full pipeline
end-to-end. All values in the dashboard are computed programmatically — no hand-entered
numbers. The `week-01` branch is merged to `main` only when all `pytest` tests pass and the
GitHub Actions CI workflow reports green.

### Connection to Larger Goal
Every tool introduced this week reappears in the UAV stack without exception. `solve_ivp`
powers the 6-DOF simulator in Week 2. LQR becomes the inner-loop attitude controller in Week
7. The `build_dashboard` pattern becomes the mission replay interface in Week 10. The
repository structure, Git conventions, and CI workflow established on Day 1 are used without
modification for the entire semester — later weeks simply add new directories to the scaffold
defined above.

### References
- Stevens, Lewis & Johnson, *Aircraft Simulation and Control* (3rd ed.) — Chapters 1–3
- Franklin, Powell & Emami-Naeini, *Feedback Control of Dynamic Systems* — Chapters 4–8
- Ogata, *Modern Control Engineering* — Chapters 5–8, 10–11
- Pro Git (free): https://git-scm.com/book/en/v2 — Chapters 1–3, Chapter 6
- GitHub Actions documentation: https://docs.github.com/en/actions
- `python-control` documentation: https://python-control.readthedocs.io

---

## Week 2 — Full Nonlinear 6-DOF Simulation and Flight Envelope Analysis

### Theme
Remove the linearization approximation from Week 1. Implement the full 6-DOF rigid body
equations of motion with nonlinear aerodynamic force and moment models, a trim solver, and a
numerical Jacobian linearizer. Discover how the aircraft's modal behavior — phugoid and
short-period frequencies, damping ratios, lateral-directional modes — changes across the
flight envelope. The Week 1 linear model is revealed to be the Jacobian of this nonlinear
system evaluated at one specific trim point. This simulator is the computational foundation
that all future phases build upon, and its parameterized configuration pattern is the template
used for the student's own UAV design starting in Week 4.

### Prerequisite Knowledge
- Week 1 complete and merged to `main`: `solve_ivp`, LQR, Plotly dashboards, Git workflow,
  module structure, CI passing
- Undergraduate aerodynamics: lift/drag polar, stability derivatives as partial derivatives
  of force and moment coefficients with respect to state variables
- Basic understanding of Euler angles and Newton-Euler equations of motion

### Core Concepts

**Full 6-DOF equations of motion:** The complete Newton-Euler EOM for a rigid body aircraft
has twelve states: body-axis translational velocities, body-axis angular rates, Euler angles,
and inertial position. Aerodynamic forces and moments are nonlinear functions of angle of
attack, sideslip, angular rates, and control deflections. The entire EOM is implemented as a
single callable function compatible with `solve_ivp`. The ISA atmospheric model — density,
pressure, and temperature as polynomial functions of altitude — is implemented from scratch
and called inside the EOM at every timestep, replacing the fixed-density assumption from Week
1.

**Aircraft configuration as a dataclass:** All aerodynamic coefficients, geometry, and
inertia parameters are stored in a `CessnaConfig` dataclass — nothing is hardcoded in
simulation functions. This parameterized pattern is the design decision that makes the
simulator reusable for any aircraft: swapping in a new configuration object is the only
change required to simulate a different airframe. This is the same pattern the student will
use for their own `UAVConfig` in Weeks 4–5. The dataclass includes a `config_hash` field
(SHA-256 of all field values) that is logged alongside every simulation result, making every
output traceable to the configuration that produced it.

**Trim solver using `scipy.optimize.fsolve`:** Finding the control inputs and angle of attack
that produce steady flight at a given airspeed and altitude is a nonlinear root-finding
problem. The trim solver wraps the EOM in a residual function and uses
`scipy.optimize.fsolve` to drive all state derivatives to zero. A trim sweep across the
flight envelope stores results in a Pandas DataFrame and produces the L/D polar — the
performance curve that determines the endurance-optimal cruise speed.

**Numerical Jacobian for linearization:** The A and B matrices at any trim point are computed
by finite-differencing the EOM with respect to each state and control input — the same
central difference technique used in numerical methods courses, now applied to a physical
system. This produces the full linearized model at any operating condition without symbolic
math. Verifying that the longitudinal subblock at cruise matches the Week 1 A matrix confirms
that the nonlinear model is implemented correctly.

**Modal analysis across the flight envelope:** Computing eigenvalues of the linearized A
matrix at each point in the trim sweep reveals how the phugoid and short-period poles migrate
with airspeed — a root locus-style migration plot. This is the first time the student sees
the aircraft's dynamics as a continuous function of its operating condition rather than a
fixed set of numbers, which motivates the gain scheduling introduced later in the week.

**Large-angle maneuver simulation:** Simulating maneuvers that violate the small-angle
assumption — steep banked turns, pull-ups, approach to stall — and comparing the nonlinear
simulation to the Week 1 linear model propagated from the same initial condition makes the
linearization error concrete and visible. The student identifies the operating region where
linear control design remains valid.

**Monte Carlo uncertainty propagation:** Aerodynamic coefficients are uncertain. Sampling
each coefficient as a Gaussian random variable (seeded from `CessnaConfig.monte_carlo_seed`)
and propagating the ensemble through the trim solver and eigenanalysis produces distributions
of modal parameters. Computing the fraction of samples that satisfy MIL-F-8785C flying
qualities requirements is the first quantitative robustness analysis in the curriculum. The
same Monte Carlo framework is applied to the student's own aircraft in Week 6 after the
aerodynamic analysis produces its coefficient uncertainties.

**Gain-scheduled LQR:** Designing LQR controllers at multiple trim points and implementing
linear interpolation of gain matrices across airspeed introduces the concept that a single
fixed-gain controller cannot be optimal across a varying flight envelope. Validating the
gain-scheduled controller on a maneuver that traverses multiple trim points is the capstone
control result of the week.

**Testing infrastructure with `pytest`:** Fixtures and `pytest.mark.parametrize` are
introduced to replace the MATLAB habit of copy-pasting test scripts. Type annotations and
`mypy` static checking are introduced alongside the `CessnaConfig` dataclass — a typed
configuration object is the natural first target for annotations. `mypy --strict` must report
zero errors on the Week 2 codebase and on every subsequent week that introduces new classes
or dataclasses.

### Python Ecosystem
`numpy`, `scipy` (`optimize`, `integrate`, `linalg`), `pandas`, `matplotlib`, `plotly`,
`python-control`, `pytest`, `mypy`, `dataclasses`

### Capstone Deliverable
`weeks/week_02/` — a fully parameterized nonlinear flight simulator with trim solver,
numerical linearizer, Monte Carlo uncertainty module, and gain-scheduled LQR controller.
Running `python main.py` produces an interactive Plotly flight envelope dashboard: trim sweep
performance curves, pole migration plot, Monte Carlo modal histograms with MIL-F-8785C
requirement lines, and gain-scheduled vs. fixed-gain controller comparison. All `pytest` tests
pass; `mypy --strict` reports no errors. The `week-02` branch is merged to `main` only when
the GitHub Actions CI workflow reports green.

### Connection to Larger Goal
This simulator is the most important artifact in the semester. Every future week either
extends it or uses it to validate a design decision. The `CessnaConfig` dataclass pattern is
reused directly as `UAVConfig` in Weeks 4–5 — plugging the student's own aerodynamic
coefficients into the same simulator requires only a new configuration object. The numerical
Jacobian function is reused in Week 7 to extract SISO plants for inner-loop controller
design. The Monte Carlo module is reused in Week 10 for full autopilot robustness validation.

### References
- Stevens, Lewis & Johnson, *Aircraft Simulation and Control* — Chapters 2–4
- Stengel, *Flight Dynamics* — Chapters 3–5
- Cook, *Flight Dynamics Principles* — Chapters 2–4
- MIL-F-8785C — Flying Qualities of Piloted Aircraft
- `pytest` fixtures and parametrize: https://docs.pytest.org/en/stable/how-to/fixtures.html
- `mypy` documentation: https://mypy.readthedocs.io

---

## Week 3 — Navigation, State Estimation, and Sensor Fusion

### Theme
A UAV that cannot localize itself cannot fly a mission. This week the student builds the
navigation stack from scratch: a linear Kalman filter as the conceptual foundation, coordinate
frame transforms, sensor models for GPS, IMU, pitot-static airspeed, barometer, and
magnetometer, and a 13-state Extended Kalman Filter (EKF) that fuses them into a coherent
position-velocity-attitude estimate. Every element — quaternion kinematics, the linearized
measurement Jacobian, the predict-update cycle — is implemented by hand using only NumPy and
SciPy. The student leaves this week with a navigation module that accepts raw sensor streams
and outputs a filtered navigation solution including wind velocity estimates.

### Prerequisite Knowledge
- Week 1: `solve_ivp`, Plotly dashboards, module structure
- Week 2: 6-DOF simulation as a clean source of ground-truth state data; ISA atmosphere;
  rotation matrices; `CessnaConfig` dataclass pattern

### Core Concepts

**Day 1 — Linear Kalman filter from scratch:** Before extending to the EKF, implement the
linear Kalman filter predict-update loop in plain NumPy. The student discretizes the Week 1
Cessna A matrix using `scipy.signal.cont2discrete`, tunes Q and R covariance matrices, and
verifies that the filter estimate converges to the true state on a 30-second simulation run.
This implementation is the direct conceptual foundation for the 13-state EKF introduced on
Day 3 — every line of code in the linear filter has an exact analogue in the EKF. Placing
this on Day 1 of Week 3 rather than in Week 1 ensures the student has seen two weeks of ODE
simulation and state-space dynamics before implementing a state estimator, and that the EKF
follows immediately rather than being separated by an entire week of 6-DOF material.

**Coordinate frames and conversion functions:** Inertial NED (North-East-Down), body, ECEF
(Earth-Centered Earth-Fixed), and LLA (latitude-longitude-altitude) frames. The student
implements all conversion functions from their mathematical definitions using only `numpy`:
LLA → ECEF via the WGS-84 ellipsoid equations, ECEF → NED via the rotation matrix that
depends on the reference point latitude and longitude, NED → body via the Euler rotation
matrix. These functions are exercised in every subsequent phase and must be implemented
correctly once.

**Magnetic declination:** The magnetometer measures magnetic north, not true north. The
heading computed from the magnetometer has a declination offset that varies by location and
can exceed 15–20° in some regions. Implement `magnetic_declination(lat, lon) → float` as a
lookup function using a small embedded table of World Magnetic Model coefficients (WMM2020,
available as a plain text file). Apply the declination correction inside the magnetometer
measurement function so that all EKF heading estimates are in true north. Failure to apply
this correction produces a systematic heading bias proportional to the local declination —
which translates directly into lateral navigation error. **This is not optional: a
magnetometer-fused EKF without declination correction is wrong by definition.**

**Quaternion algebra from scratch:** Euler angles suffer from gimbal lock near ±90° pitch —
a real failure mode in aggressive fixed-wing maneuvers. Quaternions represent attitude as a
unit 4-vector that avoids this singularity. The student implements: quaternion multiplication
(using the Hamilton product definition), normalization, inversion (conjugate for unit
quaternions), conversion to rotation matrix (via the Rodrigues formula), conversion from
Euler angles, and quaternion kinematics integration `q̇ = ½ · Ξ(q) · ω_body`. All in
`numpy`. `scipy.spatial.transform.Rotation` is used only to verify results against the
student's implementation.

**IMU sensor model:** Accelerometers measure specific force (total acceleration minus gravity
vector in body frame). Gyroscopes measure angular rate in body frame. Both are corrupted by:
additive white Gaussian noise, a slowly drifting bias (modeled as an integrated random walk),
and a scale factor error. Implement a sensor simulation function that takes true 6-DOF states
and returns noisy IMU measurements. This function is the bridge between the Week 2 simulator
and the EKF.

**GPS sensor model:** GPS provides NED position and velocity at 5–10 Hz with white Gaussian
noise (≈2–5 m position, ≈0.1 m/s velocity). Because GPS arrives at a much lower rate than
IMU, the fusion architecture must handle asynchronous, multi-rate updates: the EKF predicts
at IMU rate (100 Hz) and executes a measurement update only when a GPS reading is flagged as
available.

**Pitot-static airspeed sensor model:** The pitot tube measures differential stagnation
pressure; airspeed is recovered as `V = sqrt(2 · ΔP / ρ)` where ρ is the ISA density at
current altitude. Add Gaussian measurement noise and a blockage event flag (a boolean that
sets the sensor output to zero to simulate a blocked tube). This sensor is critical: stall
speed is airspeed-dependent, not groundspeed-dependent, and the TECS controller in Week 8
requires airspeed. When the pitot flag is blocked, the EKF falls back to a synthetic airspeed
estimate derived from GPS groundspeed and the current wind estimate. This fallback is
implemented in the EKF measurement update function: if `pitot_blocked`, skip the airspeed
update and use the synthetic estimate in its place.

**Barometer and magnetometer models:** The barometer gives altitude from the ISA pressure
model plus noise. The magnetometer measures the Earth's field vector in body frame, corrupted
by a hard-iron offset (constant vector, calibrated out) and soft-iron distortion (scaling),
and corrected for magnetic declination as described above. These provide independent altitude
and heading references that correct IMU drift between GPS updates.

**Extended Kalman Filter from scratch:** The EKF propagates the state mean through the
nonlinear dynamics and approximates the covariance propagation by linearizing (using the same
finite-difference Jacobian from Week 2) at the current estimate each timestep. State vector:
NED position (3), NED velocity (3), quaternion (4), gyro bias body (3), wind NED (2) = 15
states. The wind states make the EKF observable on GPS velocity alone: the difference between
GPS velocity (groundspeed) and IMU-integrated velocity (airspeed) constrains the wind
estimate. Implement as a class with `predict(imu_measurement, dt)` and
`update(sensor_measurement, H, R)` methods. The measurement update equation is standard
Kalman: `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`, `x̂ = x̂ + K·(z - h(x̂))`, `P = (I - K·H)·P`. Use
`numpy.linalg.solve` instead of `numpy.linalg.inv` for numerical stability.

**Wind estimation in the EKF:** The 2 wind states (north and east wind velocity) are
augmented into the EKF state vector. Their process model is a random walk (`ẇ = noise`),
reflecting that wind changes slowly relative to the IMU rate. The observation equation links
them to GPS velocity: `v_GPS_NED ≈ v_airspeed_NED + v_wind_NED`. With IMU-integrated
airspeed and GPS groundspeed as measurements, the wind states become observable. The wind
estimate is made available to the Week 9 guidance layer for wind-corrected path following,
and to Week 8's TECS controller for airspeed-to-throttle mapping correction.

**GPS dropout and dead reckoning:** Simulate a 30-second GPS outage mid-flight. The EKF
continues running in prediction-only mode, propagating state uncertainty forward using IMU
alone. Plot the 3σ position uncertainty ellipse derived from the diagonal of P and verify it
correctly captures the growing true position error. This is dead reckoning — the student sees
concretely why GPS loss causes position drift but not attitude drift (the gyros still
constrain attitude).

**Error-state EKF — conceptual treatment only:** The error-state (indirect) EKF estimates
only the deviation from the IMU-propagated estimate rather than the full state, making the
linearization assumption more accurate. This architecture is used inside PX4 and ArduPilot.
Introduce it as a conceptual contrast to the direct EKF — explain its structure and
motivation without a full implementation. Students who complete the direct EKF early may
attempt the error-state formulation as an extension using the Joan Solà reference.

### Python Ecosystem
`numpy`, `scipy` (`integrate`, `linalg`, `spatial.transform` for verification only),
`matplotlib`, `plotly`, `pytest`

### Day 2 Sanity-Check Milestone (motivation anchor)
Before implementing the full EKF, the student verifies quaternion integration alone: propagate
a known rotation sequence (e.g., 90°/s roll for 1 second) and assert that the final
quaternion converts to a rotation matrix matching the expected result within 0.01 rad. This
milestone produces a passing `pytest` test on Day 2, giving the student a concrete success
before the full EKF is assembled.

### Capstone Deliverable
`nav/` — a navigation module containing: coordinate frame conversion functions (all four
frames), magnetic declination lookup, quaternion algebra utilities (multiplication,
conversion, kinematics), sensor simulation (IMU, GPS, pitot-static with blockage flag,
barometer, magnetometer with declination correction), a 15-state EKF class with integrated
wind estimation, and a validation test harness. The test harness drives the Week 2 6-DOF
simulator, injects sensor noise, runs the EKF, and produces a Plotly dashboard showing true
vs. estimated position, velocity, attitude, and wind over a 5-minute flight with: a mid-flight
GPS dropout and growing 3σ uncertainty ellipse, a pitot blockage event with synthetic airspeed
fallback, and wind estimate convergence from zero initial guess to true wind within 60
seconds.

### Connection to Larger Goal
This module is the navigation layer of every real autopilot. The coordinate frame functions,
wind estimates, and pitot fallback logic written this week are used without modification in
every subsequent phase. When hardware is integrated in Week 11, the student will understand
what the flight controller's internal EKF is doing — essential for diagnosing divergence and
tuning process noise covariances.

### References
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
  (2nd ed.) — Chapters 3–5, 9–11
- Titterton & Weston, *Strapdown Inertial Navigation Technology* — Chapters 1–4
- Mahony, Hamel & Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal
  Group" — IEEE T-AC 2008
- Joan Solà, "A micro Lie theory for state estimation in robotics" — arXiv:1812.01537
  (error-state EKF — for extension only)
- NOAA World Magnetic Model: https://www.ngdc.noaa.gov/geomag/WMM/

---

---

# PHASE 2 — DESIGN
## Weeks 4–6: Aerodynamic Analysis, Structural Sizing, and Propulsion

---

## Week 4 — Aerodynamic Analysis: Panel Methods and Stability Derivative Computation

### Theme
Where do aerodynamic coefficients come from? Weeks 1–2 handed the student a configuration
file full of numbers. This week they learn to compute those numbers from geometry. The student
implements a simplified Vortex Lattice Method (VLM) solver in pure NumPy, validates it against
AVL output parsed via subprocess, and uses it to compute stability and control derivatives for
any wing-tail configuration. By the end of the week the student can generate a complete
aerodynamic model from nothing but geometry — and they understand exactly how every number was
produced.

**Partial integration milestone (end of Day 4):** Before Day 5, substitute the student's
VLM-computed lift curve slope into a modified `CessnaConfig` object and verify that the
Week 2 trim solver still converges with the new coefficient. This gives a simulator payoff
mid-Phase 2, before the student's own `UAVConfig` is fully assembled in Week 5.

### Prerequisite Knowledge
- Weeks 1–2: stability derivatives and their physical meaning; `numpy.linalg.solve`;
  dataclasses
- Undergraduate aerodynamics: thin airfoil theory, Prandtl lifting line, induced drag concept

### Core Concepts

**Airfoil geometry and thin airfoil theory:** An airfoil's aerodynamic behavior is governed
primarily by its camber line. Thin airfoil theory gives `C_lα = 2π /rad` and the zero-lift
angle `α_0` as a Fourier integral of the camber slope. Implement the thin airfoil integrals
numerically using `scipy.integrate.quad` and verify against tabulated NACA 4-series results.
This is the 2D building block for the 3D VLM.

**XFOIL automation via subprocess:** XFOIL is the standard viscous 2D airfoil analysis code.
The student drives XFOIL entirely through `subprocess.run`: write an input command sequence
as a string, pipe it to XFOIL's stdin, parse the polar output file with `numpy.loadtxt`. No
Python wrapper library. The student must read the XFOIL documentation to understand its
input format. This subprocess-plus-file-parser pattern is the same approach used to automate
any legacy Fortran/C engineering executable.

**Vortex Lattice Method (VLM) from scratch — planar rectangular wing:** For the initial VLM
implementation, restrict the geometry to a planar unswept rectangular wing (no dihedral, no
sweep, rectangular panels). This significantly reduces the Biot-Savart geometry complexity
while preserving all the conceptual content: AIC matrix construction, vortex strength solve,
and lift/drag integration. The pedagogical goal is understanding the method; AVL handles the
full 3D geometry for production-accuracy coefficient computation. Divide each lifting surface
into rectangular panels, place a horseshoe vortex on each panel, and solve for vortex
strengths Γ that satisfy zero normal velocity at each panel's control point. The aerodynamic
influence coefficient matrix A (shape: N_panels × N_panels) is computed via the Biot-Savart
law. The system `A · Γ = b` is solved with `numpy.linalg.solve`. Spanwise and chordwise
lift distributions are recovered from the Γ solution; total lift and induced drag are
computed by numerical integration over the span.

**AVL automation for validation and full derivative computation:** AVL computes the complete
set of stability and control derivatives far more accurately than the simplified VLM.
Automate AVL entirely via `subprocess`: write the geometry input file (`.avl` format) and
mass file from the aircraft dataclass, call the executable, capture stdout, and parse the
stability derivative table using string parsing and `pandas`. Validate: the student's VLM
lift curve slope should match AVL within 5% for a planar unswept wing.

**Planform parameters and their aerodynamic consequences:** Aspect ratio drives induced drag
efficiency (Oswald factor `e`). Taper ratio shapes the spanwise lift distribution toward
elliptical optimum. Sweep affects `C_nβ` (directional stability). Dihedral drives `C_lβ`
(roll stability). Implement a function that takes planform parameters as a dataclass, runs
AVL, and returns the drag polar and key stability derivatives. This becomes the aerodynamic
analysis callable for the Week 5 optimizer.

**Tail sizing for static stability and trim authority:** The horizontal tail must provide
`C_mα < 0` (static pitch stability) and enough elevator authority to trim across the flight
envelope. The tail volume coefficient `V_H = l_t · S_t / (c̄ · S)` is the primary sizing
parameter. Implement a tail sizing solver: given wing geometry and a required `C_mα` target,
use `scipy.optimize.brentq` to find the minimum tail area and moment arm satisfying the
stability constraint. Do the same for the vertical tail using `V_V` and the required `C_nβ`.

**Component drag buildup:** Zero-lift drag is the sum of friction drag from all wetted
surfaces. For each component (wing, fuselage, horizontal tail, vertical tail), compute
flat-plate skin friction coefficient `C_f` as a function of Reynolds number (Blasius formula
for laminar, Prandtl-Schlichting for turbulent), multiply by a component form factor for
thickness and interference effects, multiply by wetted area, and normalize by wing reference
area. Implement as a loop over a list of component geometry objects. Sum to get `C_D0`. This
value, combined with the VLM-computed induced drag factor `k`, gives the full drag polar used
in all performance calculations.

### Python Ecosystem
`numpy` (`linalg.solve` for VLM AIC system), `scipy` (`integrate.quad` for airfoil
integrals, `optimize.brentq` for tail sizing), `subprocess` (AVL and XFOIL automation),
`pandas` (output parsing), `matplotlib`, `plotly`

### Capstone Deliverable
`uav_design/aero/` — a module containing: the planar rectangular VLM solver, the XFOIL
subprocess driver and polar parser, the AVL subprocess driver and stability derivative
parser, the component drag buildup function, and the tail sizing solver. Accepts any
wing-tail geometry dataclass and returns a complete aerodynamic dataset. Final output: a
Plotly dashboard showing the lift curve, drag polar, L/D vs. CL, and a stability derivative
summary table. Day 4 milestone: VLM lift curve slope substituted into `CessnaConfig` and
trim solver convergence verified.

### Connection to Larger Goal
This module produces the aerodynamic coefficients that populate the student's own `UAVConfig`
dataclass, which feeds directly into the Week 2 6-DOF simulator. By the end of Week 4,
changing a wing parameter, re-running the analysis, and seeing the effect on trim, stability,
and performance takes under 60 seconds. This closes the geometry-to-simulation loop for the
first time.

### References
- Katz & Plotkin, *Low-Speed Aerodynamics* — Chapters 10–12
- Drela, *Flight Vehicle Aerodynamics* — Chapters 3–5
- Raymer, *Aircraft Design: A Conceptual Approach* — Chapters 4–6
- Anderson, *Introduction to Flight* — Chapters 5–7
- AVL User Primer: https://web.mit.edu/drela/Public/web/avl/avl_primer.txt

---

## Week 5 — Structural Sizing, Weight Estimation, and Design Optimization

### Theme
Aircraft design is fundamentally about managing weight. Every component must be light enough
to fly and strong enough not to break. This week the student builds structural sizing from
first principles (Euler-Bernoulli beam theory for the wing spar), a component weight buildup,
and a design optimizer that finds the geometry maximizing endurance subject to structural,
aerodynamic, and mission constraints using `scipy.optimize`. The output is a fully sized
aircraft whose every parameter was computed, not guessed.

**Scope note:** The optimization is intentionally scoped to a 2-variable parametric sweep
(span and root chord, holding taper ratio and spar height fraction at design-point values)
rather than a full 4-variable SLSQP problem. The full MDO formulation is described for
completeness and as a reference, but the implementation uses the 2-variable sweep for two
reasons: (1) the target airframe is a foam board build where manufacturing tolerances dwarf
optimizer precision, and (2) a 2D sweep produces a contour plot the student can read directly,
making the sensitivity of endurance to each design variable immediately visible without
debugging optimizer convergence.

### Prerequisite Knowledge
- Week 4: aerodynamic model as a callable function `aero(geometry) → coefficients`; drag
  polar; tail sizing
- Undergraduate structures: `σ = My/I`, cantilever beam bending, factor of safety

### Core Concepts

**V-n diagram and design load factor:** Compute the V-n diagram from scratch: the
aerodynamic limit curve `n(V) = C_Lmax · ½ρV² · S / W` and the structural limit `n =
n_ult`. The intersection defines corner speed `V_A`. For a small foam UAV the design
ultimate load factor is typically `n_ult = 4.5` with a 1.5 safety factor. The bending
moment at the wing root is `M_root = ∫_{0}^{b/2} L(y) · y dy` integrated numerically via
`scipy.integrate.trapezoid` over the spanwise lift distribution from the Week 4 VLM.

**Wing spar sizing via Euler-Bernoulli beam theory:** Model the spar as a cantilever with the
VLM lift distribution as a distributed load. For a rectangular box-beam spar of height h and
wall thickness t, the section modulus is `Z = I/c = (h² · t) / 3`. The stress constraint is
`M_root / Z ≤ σ_allowable / safety_factor`. Implement a function `size_spar(lift_distribution,
span, material) → (h, t, mass)` that finds the minimum-weight (h, t) pair satisfying the
stress constraint, using `scipy.optimize.minimize_scalar` on total spar mass.

**Material database:** Define a `materials.py` module — a plain Python dictionary of
materials (CFRP unidirectional, CFRP woven, 6061-T6 aluminum, fiberglass, balsa/epoxy). Each
entry stores density (kg/m³), tensile strength (Pa), modulus (Pa), and cost ($/kg) as a
`dataclass`. Implement `select_material(moment, geometry, objective)` as a brute-force search
over this dictionary, returning the minimum-weight material that satisfies the stress
constraint. Introduce the concept of specific strength `σ/ρ` and specific stiffness `E/ρ` as
the material selection metrics.

**Component weight buildup:** Implement Raymer's statistical regression equations as
individual functions — one per component: wing structure, fuselage shell, horizontal tail,
vertical tail, landing gear, motor mount, avionics bay, wiring harness. Each regression is a
power-law formula calibrated to historical aircraft data. Implement each as a Python function
with clear docstrings, unit comments on every argument, and an `assert` that the result is
physically plausible (e.g., wing weight < 30% of TOGW). Sum all components to get structural
weight.

**Weight-sizing fixed-point iteration:** Takeoff weight drives required thrust, which drives
battery capacity, which drives battery mass, which increases takeoff weight — a circular
dependency. Implement as a fixed-point iteration: initial TOGW guess → compute all dependent
masses → new TOGW → repeat until `|TOGW_new - TOGW_old| < 0.1 kg`. Plot convergence (TOGW
vs. iteration number). A design that diverges is infeasible — the student must relax a
constraint (larger wing, lighter payload) to achieve convergence.

**Aerodynamic coefficient uncertainty characterization:** After the weight iteration
converges, estimate the uncertainty of the student's own UAV aerodynamic coefficients using
the discrepancy between the Week 4 VLM and AVL results as a proxy for modeling error. Store
these as a `UAVUncertainty` dataclass with a standard deviation for each key coefficient
(CL_alpha, CD0, Cm_alpha, etc.). This dataclass feeds the Week 10 Monte Carlo runs — without
it, the Week 10 robustness analysis has no defensible uncertainty inputs.

**2-variable endurance trade study:** Sweep span (b) and root chord (c_root) over a
physically reasonable design space. At each (b, c_root) point: compute aspect ratio, run the
aerodynamic analysis, compute the drag polar, run the weight iteration, estimate Breguet
endurance. Store the full sweep in a Pandas DataFrame. Plot endurance as a `matplotlib
contourf` and `plotly go.Surface`. Identify the optimum point and the sensitivity of
endurance to each variable (gradient computed by central differences from the sweep data).
Add inequality constraints as overlay lines: stall speed `V_stall ≤ 15 m/s`, span `≤ 3.0 m`,
structural margin `≥ 0`. The feasible design space is the region satisfying all constraints.

**MDO reference formulation (documentation only):** Document the full 4-variable SLSQP
problem (span, aspect ratio, taper ratio, spar height fraction) with the SLSQP
implementation in a `docs/mdo_reference.md` file as a reference for students who want to
extend the optimizer. This is not implemented or tested — it is a design document.

### Python Ecosystem
`numpy`, `scipy` (`integrate.trapezoid`, `optimize.minimize_scalar`), `pandas`, `matplotlib`,
`plotly`

### Capstone Deliverable
`uav_design/structures/` — a module containing: V-n diagram computation, wing spar sizing
from lift distribution, material selection database and function, component weight buildup,
fixed-point weight iteration, and `UAVUncertainty` dataclass. Final output: a `UAVDesign`
dataclass with all geometry, mass properties, aerodynamic coefficients, and performance
metrics for the student's preliminary UAV design, plus a 2-variable endurance contour plot
with constraint boundary overlays and a sensitivity analysis bar chart.

### Connection to Larger Goal
The `UAVDesign` dataclass produced this week is the student's actual aircraft. Its geometry
feeds the Week 4 aerodynamic tool; its mass properties populate the `UAVConfig` for the Week
2 6-DOF simulator. By the end of Week 5 the student can simulate their own aircraft — not
the Cessna 172. The `UAVUncertainty` dataclass feeds the Week 10 Monte Carlo validation. This
is the transition from learning tools to using tools to build something real.

### References
- Raymer, *Aircraft Design: A Conceptual Approach* — Chapters 14–16
- Nicolai & Carichner, *Fundamentals of Aircraft and Airship Design* — Chapters 12–14
- Megson, *Aircraft Structures for Engineering Students* — Chapters 9–11
- Martins & Ning, *Engineering Design Optimization* — Chapters 4–7 (SLSQP reference)

---

## Week 6 — Propulsion System Design, Electric Powertrain Modeling, and Hardware Procurement

### Theme
Thrust comes from a propulsion system — for an electric UAV: motor, propeller, ESC, and
battery. This week the student models each component from its governing equations, solves for
the motor-propeller operating point at any flight condition, and integrates the complete
powertrain into the 6-DOF simulator. Every equation is implemented from scratch using only
NumPy and SciPy. The deliverable is a propulsion module that predicts shaft speed, thrust,
current draw, and battery state-of-charge as continuous functions of time. The week closes
with a hardware procurement checklist — the design is now frozen, and the student orders
exactly what the analysis specifies.

### Prerequisite Knowledge
- Week 5: takeoff weight, required cruise thrust, drag polar
- Basic electrical engineering: Ohm's law, power balance

### Core Concepts

**Propeller performance from tabulated data:** A propeller is characterized by its thrust
coefficient `C_T(J)` and power coefficient `C_P(J)` as functions of advance ratio `J = V /
(n · D)`. Load coefficient data from the UIUC Propeller Database (plain CSV files) with
`numpy.loadtxt`, store in Pandas DataFrames, and build smooth interpolating functions using
`scipy.interpolate.CubicSpline`. Thrust = `C_T · ρ · n² · D⁴`; shaft power absorbed = `C_P
· ρ · n³ · D⁵`. At any given airspeed and shaft speed these give thrust and the torque the
motor must supply.

**BLDC motor model from circuit equations:** The motor obeys: back-EMF = `K_v⁻¹ · ω`,
terminal current `I = (V_in - K_v⁻¹ · ω) / R_m`, output torque `Q = (I - I_0) / K_v`.
Implement as `motor_state(V_in, omega, config) → (I, Q)`. The shaft speed at a given
throttle and airspeed is found by solving the torque balance `Q_motor(ω) = Q_prop(ω, V)`
using `scipy.optimize.brentq` — the motor and propeller torque curves intersect at exactly
one stable equilibrium point for a given input voltage and airspeed.

**ESC model:** The ESC converts battery voltage and a throttle command (0–1) to a modulated
motor input voltage: `V_in = throttle · V_batt · η_ESC` where `η_ESC ≈ 0.95`.

**Battery electrochemical model:** Model the LiPo cell as a voltage source with internal
resistance: `V_terminal = OCV(SOC) - I · R_int`. The `OCV(SOC)` curve is represented as a
lookup table interpolated with `scipy.interpolate.CubicSpline`. State of charge integrates:
`SOC(t) = SOC(0) - ∫₀ᵗ I(τ) / Q_cap dτ`. Implement with `scipy.integrate.solve_ivp` in
event mode: add an event function that triggers when `V_terminal` drops to 3.3 V/cell
(low-voltage cutoff), terminating the integration.

**System integration — `PropulsionSystem` class (Day 5):** Encapsulate the motor-propeller-
ESC-battery chain as a class with a `step(V_airspeed, throttle, dt)` method that: computes
`V_batt` from SOC, computes `V_in` from ESC, solves the torque balance via `brentq` for ω,
returns `(thrust, current, omega, V_batt)`, and updates internal SOC. Day 5 is dedicated
entirely to plugging `PropulsionSystem.step()` into the Week 2 6-DOF EOM as a callable thrust
model, replacing the fixed-thrust approximation, and verifying that the trim solver converges
with realistic thrust values. No new propulsion concepts are introduced on Day 5.

**Throttle-to-thrust map:** At 5 airspeeds across the flight envelope, sweep throttle 0→1
and record `(thrust, current)` at each point. Store as a 2D NumPy array and build a smooth
interpolator using `scipy.interpolate.RegularGridInterpolator`. This map is the interface
between the autopilot airspeed controller (Week 8) and the propulsion system.

**Mission energy budget:** Drive the `PropulsionSystem` with a representative mission profile
(full-throttle climb, partial-throttle cruise, near-idle descent). Integrate current over
time using `scipy.integrate.trapezoid` to compute total energy consumed in Wh. The design is
viable only if the remaining SOC at mission end exceeds a 20% reserve.

**Propeller sizing optimization:** Sweep propeller diameter at constant pitch-to-diameter
ratio. Compute cruise propulsive efficiency `η_prop = T · V / P_shaft` at the cruise
operating point for each diameter. Plot `η_prop` vs. diameter. Identify the optimum diameter
subject to a tip Mach number constraint `V_tip = π · D · n < 0.7 · a`.

**Monte Carlo aerodynamic uncertainty integration:** Apply the `UAVUncertainty` dataclass
from Week 5 to the 6-DOF simulator. Draw 50 aerodynamic coefficient samples using
`numpy.random.default_rng(seed=UAVConfig.monte_carlo_seed)` and run the trim solver for each.
Store trim sweep distributions in a Pandas DataFrame. This validates that the propulsion
system produces adequate thrust across the aerodynamic uncertainty envelope, not just at the
nominal design point.

**Hardware procurement checklist (capstone deliverable component):** At the end of Week 6,
the motor, propeller, battery, ESC, flight controller, GPS, companion computer, RC system,
and servos are all specified by the design analysis. Generate a structured BOM as a Pandas
DataFrame: component, specification, quantity, estimated cost, supplier. Verify the total is
within the project budget. **Order all hardware at the end of Week 6.** This provides a
minimum 4-week lead time before hardware is needed in Week 11.

### Python Ecosystem
`numpy`, `scipy` (`interpolate`, `optimize.brentq`, `integrate`), `pandas`, `matplotlib`,
`plotly`

### Capstone Deliverable
`uav_design/propulsion/` — a module containing: `C_T/C_P` table loading and spline
interpolation, motor equations, ESC model, LiPo OCV curve with SOC integration via
`solve_ivp`, the `PropulsionSystem` class, the throttle-to-thrust map generator, and the
mission energy budget tool. Final output: a Plotly propulsion dashboard showing predicted
flight time, battery SOC vs. time, thrust vs. time, and the throttle-to-thrust map as a 3D
surface. The `PropulsionSystem.step()` method integrates correctly into the Week 2 6-DOF
simulator. Hardware BOM delivered as a Pandas DataFrame CSV and committed to the repository
under `docs/hardware_bom.csv`.

### Connection to Larger Goal
`PropulsionSystem.step()` replaces the fixed-thrust model in the Week 2 EOM — the 6-DOF
simulator now models propulsion physics throughout the flight envelope. The throttle-to-thrust
map feeds the airspeed controller in Week 8. The hardware BOM, ordered this week, arrives in
time for Week 11 integration.

### References
- UIUC Propeller Database: https://m-selig.ae.illinois.edu/props/propDB.html
- Gur & Rosen, "Optimizing Electric Propulsion Systems for UAVs" — Journal of Aircraft, 2009
- Drela, *QPROP User Guide*

---

---

# PHASE 3 — AUTONOMY
## Weeks 7–10: Autopilot, Guidance, Mission Planning, and Disturbance Rejection

---

## Week 7 — Autopilot Inner Loop: Rate Stabilization and Attitude Control

### Theme
The autopilot is a hierarchy of nested control loops. The innermost loops stabilize angular
rates (p, q, r); just outside them, attitude loops command angles (φ, θ, ψ). This week the
student designs and implements these inner loops for their own UAV using the Week 2 6-DOF
simulator configured with their Week 4–5 design. Stable inner loops are the prerequisite for
every outer-loop function — nothing else in the autonomy stack works until this week's
deliverable is solid.

### Prerequisite Knowledge
- Week 1: PID, LQR, Bode plots, gain/phase margin, `python-control`
- Week 2: 6-DOF simulator, numerical linearizer, gain-scheduled LQR
- Week 5: student's own `UAVDesign` → `UAVConfig`

**Note for LLM population:** Week 7 applies the gain-scheduling architecture established in
Week 2 to the student's own UAV configuration — not the Cessna 172. The student has already
implemented gain-scheduled LQR in Week 2; Week 7 does not re-teach the concept, it directs
the student to apply it to a new plant. No new control theory is introduced for gain
scheduling; the new content this week is the discrete-time PID implementation, actuator
dynamics, anti-windup, and asymmetric saturation.

### Core Concepts

**Control loop hierarchy and timescale separation:** UAV autopilots are nested loops with
strictly separated bandwidths. Rate loops (innermost) must have bandwidth 5–10× higher than
attitude loops; attitude loops must be 5–10× faster than velocity loops. This separation is
the mathematical condition under which cascaded loops are independently stable — each inner
loop approximates unity gain to the outer loop. The student should understand this as a
consequence of singular perturbation theory, not just a rule of thumb.

**Extracting SISO plants from the 6-DOF linearization:** Use the Week 2 numerical linearizer
at the cruise trim point of the student's UAV. For each axis, extract the relevant SISO
transfer function from control surface to body rate: `p/δ_a(s)`, `q/δ_e(s)`, `r/δ_r(s)`.
Use `control.ss2tf` to convert, then `control.bode` and `control.margin` to characterize each
plant's phase margin and gain margin in open-loop.

**Rate controllers as discrete-time PID:** Design PID controllers for p, q, and r. Tune in
the frequency domain: target gain margin ≥ 6 dB, phase margin ≥ 45°. Implement as
discrete-time difference equations — not as `control.tf` objects — so they can run in the
simulation time-stepping loop: `P = Kp · e`, `I_k = I_{k-1} + Ki · e · dt`, `D = Kd · (e -
e_prev) / dt`. The discrete derivative term requires low-pass filtering to suppress sensor
noise amplification.

**Attitude controllers as proportional-rate feedforward:** With rate controllers as the inner
loop, design proportional attitude controllers: `p_cmd = Kp_φ · (φ_cmd - φ)`, `q_cmd =
Kp_θ · (θ_cmd - θ)`. Choose outer-loop gains so the attitude loop bandwidth is 3–5× below
the rate loop bandwidth.

**Coordinated turn yaw damper:** A fixed-wing aircraft zeros sideslip β during banked turns
by coordinating the rudder with the roll rate. Implement `δ_r = -K_r · r + K_coord · p`
where the feedforward term uses roll rate to anticipate the yaw coupling. Verify by simulating
a 45° banked turn and confirming that `β < 0.5°` at steady state.

**Actuator dynamics and hard saturation:** Model servos as first-order low-pass systems
`δ̇ = (δ_cmd - δ) / τ_act` with `τ_act ≈ 0.05 s` (20 rad/s bandwidth). Add hard deflection
limits (e.g., ±25° elevator). Actuator dynamics add phase lag that can destabilize controllers
designed without them — re-check phase margins after adding the actuator model and retune if
necessary.

**Asymmetric actuator saturation:** Extend the actuator model to handle asymmetric deflection
limits — for example, an aileron mechanically limited to +20° up but only -15° down due to
hinge geometry. Implement `clip(δ_cmd, δ_min, δ_max)` where `δ_min` and `δ_max` are
per-actuator values stored in `UAVConfig`. Verify that the rate controller still converges
to the commanded value when the actuator limit is asymmetric (convergence may be slower on
one side — this is expected and must be quantified, not hidden). This addresses the real
failure mode where a partially jammed control surface limits authority on one side only.

**Anti-windup by back-calculation:** When the actuator saturates, the PID integrator must be
corrected to prevent windup. Implement back-calculation: `İ = e + (δ_sat - δ_cmd) / T_aw`
where `T_aw = √(1/(Ki · Kp))` is the anti-windup time constant. Demonstrate windup vs.
anti-windup on a large step command that saturates the elevator — the anti-windup response
must not exhibit integrator-driven overshoot.

**Gain scheduling across the flight envelope:** Compute linearizations and design separate
PID controllers at 5 airspeeds from stall to maximum, using the Week 2 gain-scheduling
architecture applied to the student's own `UAVConfig`. Implement gain interpolation using
`numpy.interp` with airspeed as the scheduling variable. Store gain tables as NumPy arrays in
`UAVConfig`.

### Python Ecosystem
`numpy`, `scipy` (`integrate`), `python-control`, `matplotlib`, `plotly`, `pytest`

### Capstone Deliverable
`autopilot/inner_loop/` — a module containing: rate and attitude controllers for all three
axes, actuator model with first-order dynamics, symmetric and asymmetric hard limits,
anti-windup logic, gain scheduling tables and `numpy.interp` interpolation, and a `pytest`
validation suite. The aircraft must track a commanded attitude sequence (level → 30° banked
turn → 20° pitch-up → level) with attitude errors < 2° at steady state and no sustained
actuator saturation. `mypy --strict` must report zero errors.

### Connection to Larger Goal
This module is the foundation layer of the autopilot stack. Every subsequent week adds one
more layer on top without modifying the inner loops. The inner-loop gains designed this week
remain fixed throughout the rest of the curriculum.

### References
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* — Chapters 6–7
- Stevens, Lewis & Johnson, *Aircraft Simulation and Control* — Chapter 6
- Åström & Hägglund, *PID Controllers: Theory, Design, and Tuning* — Chapter 5 (anti-windup)
- MIL-F-8785C — flying qualities requirements the inner loop must satisfy

---

## Week 8 — Autopilot Outer Loop: Altitude, Airspeed, and Heading Hold

### Theme
With stable inner loops in place, add the outer autopilot channels: altitude hold, airspeed
hold, and heading hold. These are the channels a transport pilot engages on the autopilot
panel. By the end of the week the simulated UAV holds a commanded altitude, airspeed, and
heading indefinitely against simple atmospheric perturbations — the behavioral definition of
a functioning autopilot.

**Turbulence note for LLM population:** Week 8 uses simplified additive white Gaussian noise
on the body-axis velocity components as the disturbance model for initial rejection testing.
The full Dryden colored turbulence model (shaping filters, PSD validation) is implemented in
Week 10 — it is not introduced here. Do not implement Dryden shaping filters in Week 8.

### Prerequisite Knowledge
- Week 7: inner-loop rate and attitude controllers (treated as ideal for outer-loop design)
- Week 6: `PropulsionSystem` class and throttle-to-thrust map
- Week 3: pitot-static airspeed sensor model and synthetic airspeed fallback

### Core Concepts

**Altitude hold — pitch-to-altitude channel:** Altitude error drives an elevator pitch command
via PI control: `θ_cmd = Kp_h · (h_cmd - h) + Ki_h · ∫(h_cmd - h) dt`. The integrator
eliminates steady-state altitude error in the presence of modeling uncertainty and constant
wind. The pitch command feeds the Week 7 attitude controller. Add an altitude rate feedforward
term `K_hdot · ḣ_cmd` to improve tracking during commanded climbs and descents. Tune using
`control.rlocus` on the altitude-to-pitch loop and verify gain and phase margins.

**Airspeed hold — throttle-to-airspeed channel:** A PI controller closes the loop on
airspeed: `throttle_cmd = Kp_V · (V_cmd - V) + Ki_V · ∫(V_cmd - V) dt`. Feed the throttle
command to the `PropulsionSystem.step()` method. The primary airspeed measurement is the
pitot-static sensor from Week 3; when the pitot blockage flag is active, the controller
automatically switches to the EKF synthetic airspeed estimate. Document this fallback clearly
in the function docstring. The well-known phugoid coupling between pitch and airspeed must be
identified in the simulation and its effect characterized.

**Total Energy Control System (TECS):** TECS resolves the altitude-airspeed coupling by
controlling two decoupled energy quantities. Total specific energy `E = ½V² + gh`; energy
distribution `D = gh - ½V²`. Throttle controls `Ė` (total energy rate); elevator controls
`Ḋ` (energy distribution rate). Implement two PI controllers — one on E, one on D — and show
that cross-coupling between altitude and airspeed is eliminated compared to independent
channels. Derive the error signals from first principles before writing any code. The wind
estimate from the Week 3 EKF is used to compute true airspeed from EKF velocity for the
energy calculation: `V_TAS = |v_NED - v_wind|` where `v_wind` is the EKF wind estimate.

**Heading hold — bank-to-heading channel:** `φ_cmd = Kp_ψ · (ψ_cmd - ψ)` with a bank angle
limit (±35°). Add a heading rate feedforward `K_ψdot · ψ̇_cmd` to reduce heading overshoot
on large heading changes. Verify that the coordinated-turn yaw damper from Week 7 keeps
`β < 1°` during a 90° heading change.

**Climb and descent phase management:** During large altitude step commands, the altitude PI
integrator would command an unreachable pitch angle. Implement pitch limit scheduling: in
climb, limit `θ_cmd` to the pitch for maximum climb rate (from trim sweep); in descent, limit
`θ_cmd` to the pitch for best glide. The autopilot degrades gracefully to maximum performance
rather than commanding an impossible attitude.

**Disturbance rejection with additive velocity noise:** Inject additive white Gaussian noise
onto body-axis velocity components (σ = 1.5 m/s) to represent atmospheric perturbations for
initial rejection testing. Run a 300-second cruise. Verify: altitude hold maintains altitude
within ±20 m, airspeed hold maintains airspeed within ±2 m/s. This is a simplified
disturbance model — the full Dryden model is implemented in Week 10 and will produce
different (larger) tracking errors at the same turbulence intensity.

**Channel interaction audit:** Longitudinal and lateral-directional channels interact in the
full 6-DOF model. A banked turn reduces vertical lift, causing altitude to drop; the altitude
hold corrects by pitching up, which changes airspeed. The student must identify and quantify
these interactions during a 360° orbit maneuver and document which are negligible and which
require explicit compensation.

### Python Ecosystem
`numpy`, `scipy`, `python-control`, `matplotlib`, `plotly`, `pytest`

### Capstone Deliverable
`autopilot/outer_loop/` — a module containing the TECS altitude/airspeed controller
(including pitot fallback to EKF synthetic airspeed and wind-corrected TAS computation),
heading hold controller, and channel coordination logic. Validated against the full 6-DOF
simulator with additive velocity noise: altitude within ±20 m and airspeed within ±2 m/s
during 300-second turbulent cruise.

### Connection to Larger Goal
These outer-loop channels are the interface the guidance layer (Week 9) commands. The guidance
layer commands `h_cmd`, `V_cmd`, and `ψ_cmd` and trusts the autopilot to execute them. The
pitot fallback implemented here is a real safety feature that will be verified in HIL testing
in Week 12.

### References
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* — Chapters 8–9
- Lambregts, "Integrated System Design for Flight and Propulsion Control Using Total Energy
  Principles" — AIAA 1983-2561

---

## Week 9 — Guidance, Waypoint Navigation, and Mission State Machine

### Theme
The guidance layer sits above the autopilot and commands where to go: follow this path,
capture this waypoint, execute this loiter orbit. This week the student implements a complete
guidance stack — path-following geometry, a waypoint sequencer, a Dubins path planner, a
finite-state machine for autopilot modes, and mode-transition integrator management — using
only `numpy` and Python's built-in `json` module. By the end of the week the simulated UAV
flies a multi-waypoint mission autonomously from takeoff through approach.

### Prerequisite Knowledge
- Weeks 7–8: full inner and outer autopilot channels
- Week 3: coordinate frame transforms (NED, LLA, ECEF) and wind estimate from EKF
- Week 2: 6-DOF simulator

### Core Concepts

**Waypoint representation and coordinate transforms:** Mission waypoints are specified in
geodetic coordinates (LLA). The guidance layer converts these to NED offsets from a local
origin using the Week 3 coordinate transform functions. All path geometry is computed in NED
using `numpy` vector operations. Missions are loaded from JSON files using Python's built-in
`json` module.

**Cross-track and along-track error geometry:** For a leg between two waypoints, define: unit
vector along the leg `û_leg`, cross-track error `e_ct` (signed perpendicular distance,
computed via the 2D cross product of the position error vector and `û_leg`), and along-track
distance `d_at` (dot product of position error with `û_leg`). The heading command is `ψ_cmd =
χ_leg + K_ct · e_ct + ψ_wind_correction` where `ψ_wind_correction` uses the EKF wind estimate
to crab into the wind, maintaining the desired ground track. This wind-corrected L1 guidance
law prevents the systematic downwind drift that occurs without wind compensation.

**Pure pursuit (carrot-chasing) guidance law:** Place a look-ahead point L meters ahead of
the aircraft along the desired track. Compute the bearing from the aircraft to the look-ahead
point; command that bearing as the heading. Implement both cross-track and pure-pursuit
guidance and compare cross-track error and heading oscillation on a curved path.

**Dubins path planning:** When transitioning between leg bearings, a naive heading jump is
kinematically infeasible for a fixed-wing aircraft. A Dubins path finds the minimum-length
path between two poses using arc-line-arc segments, respecting the minimum turning radius
`R_min = V² / (g · tan φ_max)`. Implement the Dubins path geometry from its mathematical
definition: enumerate the 6 path types (RSR, LSL, RSL, LSR, RLR, LRL), compute arc and
straight-segment lengths using `numpy` trigonometry, select the shortest feasible path.

**Waypoint sequencer and capture logic:** Declare a waypoint reached when the along-track
distance drops below the capture radius (100–200 m). On capture, advance the active waypoint
index and compute new leg geometry.

**Loiter (orbit) guidance:** The aircraft circles a fixed ground point at a commanded radius
R and altitude. At each timestep, compute the vector from aircraft to orbit center, the
desired tangential heading (perpendicular to the radius vector, in the direction of orbit),
and the orbit deviation `R - r_actual`. Command `ψ_cmd = ψ_tangential + K_orbit · (R -
r_actual)`.

**Autopilot finite-state machine in plain Python:** Define autopilot modes as a class-based
FSM with no external library. States: INITIALIZING, CLIMB, CRUISE, DESCENT, LOITER,
APPROACH. Each state is a class with `enter(nav_state)`, `update(nav_state, dt) →
AutopilotCommands`, and `check_transitions(nav_state) → next_state_or_None` methods.

**Mode transition integrator management:** When the FSM transitions between autopilot modes,
the integral states in the Week 7–8 PID controllers contain accumulated values appropriate
for the previous mode but not the new one. Naive mode switching causes a step transient in
actuator command that can destabilize the aircraft at the transition. Implement integrator
pre-loading: at each FSM state transition, set each integrator to the value that produces
zero output change at the transition instant. This is computed as `I_new = (δ_current -
Kp · e_new) / Ki` where `δ_current` is the actuator command just before the transition and
`e_new` is the error in the new mode. Verify by simulating the CLIMB→CRUISE transition and
asserting that actuator deflection is continuous (no step) at the transition time to within
±0.5°.

**Mission JSON format and validation:** Define a mission file format: a JSON object with
`takeoff`, a `waypoints` list (each with `lat`, `lon`, `alt`, `airspeed`, `action`), and
optional `loiter` parameters. Write a `validate_mission(mission, uav_config)` function that
checks: all altitudes within ceiling, all airspeeds above stall speed, each waypoint is
reachable from the previous within battery endurance. Raise descriptive exceptions for any
violation.

**Full mission simulation:** Run a 5-waypoint mission with CLIMB, CRUISE, LOITER, DESCENT,
and APPROACH phases. Validate: all waypoints captured within capture radius, cross-track
error < 200 m throughout, altitude hold within ±20 m, FSM never simultaneously in two states,
actuator commands continuous at all mode transitions.

### Python Ecosystem
`numpy`, `scipy`, `json`, `dataclasses`, `matplotlib`, `plotly`, `pytest`

### Capstone Deliverable
`autopilot/guidance/` — cross-track controller (wind-corrected), pure pursuit guidance,
Dubins path planner, waypoint sequencer, loiter guidance, FSM autopilot mode manager, and
integrator pre-loading at mode transitions. `mission/` — JSON mission format, parser, and
validator. Validated on a 5-waypoint mission with cross-track error < 200 m, all waypoints
captured, and actuator command continuity verified at each mode transition. A Plotly ground
track plot overlaid on a Scattermapbox shows the simulated flight path and waypoints.

### Connection to Larger Goal
This guidance layer transforms the autopilot into a mission-capable UAV. The wind-corrected
path following and mode-transition management implemented here are the features most likely
to determine mission success in real flight.

### References
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* — Chapters 10–12
- Dubins, "On Curves of Minimal Length with a Constraint on Average Curvature" — American
  Journal of Mathematics, 1957
- Park, Deyst & How, "A New Nonlinear Guidance Logic for Trajectory Tracking" — AIAA 2004-4900

---

## Week 10 — Atmospheric Disturbances, Robustness Validation, and Mission Replay Dashboard

### Theme
A UAV that works only in still air is a simulator artifact. This week the student adds fully
implemented atmospheric disturbances — Dryden colored turbulence (shaping filters with PSD
validation), wind shear, and discrete gusts — evaluates autopilot robustness across the Monte
Carlo aerodynamic uncertainty envelope from Weeks 5–6, connects the Week 3 EKF navigation to
the autopilot (replacing ground-truth states with estimated states), and builds the mission
replay Dash application. The week closes Phase 3 with a complete, validated,
disturbance-tested autonomous flight simulation.

### Prerequisite Knowledge
- Weeks 7–9: full autopilot and guidance stack
- Week 5: `UAVUncertainty` dataclass with aerodynamic coefficient standard deviations
- Week 6: Monte Carlo framework applied to propulsion uncertainty
- Week 3: 13-state EKF navigation module with wind estimation

### Core Concepts

**Dryden turbulence model as digital shaping filters:** The Dryden model produces velocity
perturbations `(u_g, v_g, w_g)` with PSDs matching MIL-HDBK-1797. Each component's PSD
corresponds to a first- or second-order transfer function driven by white noise: `H_u(s) =
σ_u · √(2L_u/πV) / (1 + L_u·s/V)` for the longitudinal component. Implement: (1) represent
each shaping filter as a `scipy.signal.lti` object constructed from the transfer function
coefficients, (2) discretize using `scipy.signal.cont2discrete` with the ZOH method, (3)
drive the discrete filter at each timestep with `numpy.random.default_rng(seed=UAVConfig.
monte_carlo_seed)` white noise samples. Validate by computing the generated signal's PSD via
`scipy.signal.welch` and confirming it matches the Dryden PSD within 3 dB across 0.01–10
rad/s. Turbulence velocities enter the EOM as additive perturbations to the body-axis
velocity components before angle-of-attack and sideslip computation. This is the first time
colored turbulence appears in the curriculum — Week 8 used white noise disturbances as a
simpler placeholder.

**Wind shear model:** Use the logarithmic wind profile `V_wind(h) = V_ref · ln(h/h_0) /
ln(h_ref/h_0)` to model boundary-layer wind shear. Implement as a static function of
altitude in `disturbances/wind.py`. Add to the turbulence velocity as the deterministic
component of the wind field.

**1-cosine discrete gust:** The FAA/MIL-SPEC isolated gust is `u_g(t) = (U_ds/2) · (1 -
cos(πV·t / H_g))` over gust gradient distance `H_g`. Implement as a time-limited
perturbation injected at a specified simulation time. Verify that altitude hold recovers
within ±20 m after a 10 m/s discrete gust.

**EKF navigation integration:** Replace ground-truth autopilot state access with EKF
estimates from Week 3. The autopilot now sees: EKF-estimated position, velocity, attitude,
and wind — not simulation truth. Run the full simulation with GPS at 5 Hz, IMU at 100 Hz,
and verify that EKF estimation error does not cause autopilot instability. This is the first
systems-level integration test: navigation → autopilot → simulator → sensors → navigation.

**Monte Carlo robustness validation:** Draw 50 aerodynamic coefficient samples from the
`UAVUncertainty` dataclass (Week 5) using `numpy.random.default_rng(seed=UAVConfig.
monte_carlo_seed)`. Run a complete 5-waypoint mission for each sample. Collect: waypoint
capture rate, maximum cross-track error, altitude deviation RMS. Plot as `matplotlib`
histograms and compute the 95th-percentile values. The design passes robustness validation if
95% of Monte Carlo cases satisfy all performance requirements.

**Plotly Dash mission replay application:** Build a Dash app in `replay/dash_app.py` that
plays back a logged mission CSV (time, 12 states, control inputs, autopilot mode, active
waypoint, EKF estimates). Layout: a `dcc.Scattermapbox` ground track on an OpenStreetMap
base layer with active waypoint highlighted; four linked strip charts (altitude, airspeed,
heading, bank angle) with a synchronized vertical cursor line; an autopilot mode display
updated from the log. Use `dcc.Interval` triggering at 100 ms to advance the replay.
Pre-process all data into NumPy arrays at app startup — never re-read the CSV in a callback.
Each callback must execute in < 100 ms for smooth playback.

**Final validation suite:** Run three complete missions: (1) still air, nominal aerodynamics;
(2) light Dryden turbulence; (3) heavy turbulence with a Monte Carlo aerodynamic draw.
Produce a `go.Table` comparing all five performance metrics (waypoint capture rate, max
cross-track error, max altitude deviation, max airspeed deviation, mission time) across all
three cases. All 40+ `pytest` tests must pass in < 3 minutes.

### Python Ecosystem
`numpy`, `scipy` (`signal`, `integrate`, `interpolate`), `pandas`, `matplotlib`, `plotly`
(including `plotly.dash`), `pytest`

### Capstone Deliverable
`autopilot/disturbances/` — Dryden shaping filters (with PSD validation), wind shear,
discrete gust. `replay/` — Plotly Dash mission replay application with ground track map,
linked strip charts, and mode display. Final validation report: Monte Carlo performance
histograms, three-scenario performance summary table, autopilot mode Gantt chart using
`go.Bar`.

### Connection to Larger Goal
Phase 3 is complete. The student has a fully validated, disturbance-tested, robustness-
analyzed autonomous flight simulation of their own designed aircraft. Phase 4 takes this
software and connects it to physical hardware — the simulation becomes the reference model
for hardware-in-the-loop testing and, ultimately, real flight.

### References
- MIL-HDBK-1797 — Flying Qualities of Piloted Aircraft (Dryden turbulence specification)
- Etkin & Reid, *Dynamics of Flight: Stability and Control* — Chapter 13
- Plotly Dash documentation: https://dash.plotly.com

---

---

# PHASE 4 — INTEGRATION & FLIGHT
## Weeks 11–14: Hardware, Real-Time Systems, Testing, and Flight

---

## Week 11 — Hardware Interface and Embedded Communication Protocols

### Theme
Software running in simulation must eventually talk to hardware. This week the student learns
to communicate with a flight controller, GPS, IMU, RC receiver, ESC, and servo using standard
embedded protocols — MAVLink, PWM, UART, I2C — from Python on a companion computer
(Raspberry Pi Zero 2W or equivalent). Every protocol layer is implemented from scratch using
Python's `struct`, `pyserial`, and `smbus2` — no wrapper libraries. By the end of the week
Python is sending attitude commands to the flight controller and reading back IMU data over a
live serial link.

**Scope and timing note:** This week covers seven distinct deliverables for five days, and
hardware debugging is inherently unpredictable in duration. If hardware arrives late or a
protocol implementation takes longer than expected, Day 5 (normally integration and validation
day) may expand to two days. The hardware abstraction layer (dependency injection pattern) is
explicitly designated as the Day 5 deliverable — it is the last piece assembled, after all
individual protocol drivers are working.

**Day 1 quick-win milestone:** Connect to the flight controller, receive HEARTBEAT messages
(MAVLink message ID 0), and print the message count to the terminal. This milestone is
achievable in 20–30 minutes once the physical connection is confirmed, and it proves the
serial link is working before any parsing code is written. No MAVLink parsing logic is
implemented on Day 1 — only raw byte reception and start-byte detection.

### Prerequisite Knowledge
- Weeks 1–10: complete simulation and autopilot stack
- Basic electronics: voltage levels, connector types, wiring
- Comfort with a Linux terminal (file system, processes, permissions)
- Hardware ordered and received from Week 6 BOM

### Core Concepts

**Companion computer architecture:** The student's Python autopilot runs on a small SBC
mounted on the aircraft. The SBC sends guidance commands and receives telemetry from the
flight controller via MAVLink over UART. The flight controller handles low-level sensor
fusion and servo output at 400 Hz. The companion computer runs high-level guidance and
mission logic at 10–50 Hz. This division of labor — fast inner loop on dedicated hardware,
slow outer loop on general-purpose CPU — is the architecture used in virtually all research
and commercial UAVs. Python is not suitable for the 400 Hz rate loop — it runs on dedicated
flight controller firmware, not in Python.

**MAVLink protocol from raw bytes:** MAVLink is a lightweight binary message protocol. The
student implements MAVLink parsing and construction entirely using Python's `struct.pack` /
`struct.unpack`. MAVLink v2 frame structure: start byte (0xFD), payload length (1 byte),
flags (2 bytes), sequence (1 byte), system/component IDs (2 bytes), message ID (3 bytes),
payload (variable), checksum (2 bytes, CRC-16-MCRF4XX). Implement the CRC from its
polynomial definition. Parse three message types manually from raw bytes before using any
higher-level abstraction. Key messages: `HEARTBEAT` (ID 0), `ATTITUDE` (ID 30),
`GLOBAL_POSITION_INT` (ID 33), `SET_ATTITUDE_TARGET` (ID 82), `HIL_SENSOR` (ID 107).

**UART byte framer as a state machine:** Open a serial port with `pyserial`. Implement a
state-machine byte framer that searches the byte stream for the MAVLink start byte, reads the
length field, reads the exact payload, validates the checksum, and dispatches the parsed
message.

**PWM signal generation for servos and ESC:** Servos and ESCs expect a 50 Hz PWM signal with
pulse width 1000–2000 µs. On a Raspberry Pi, generate PWM by writing directly to the
hardware PWM kernel interface at `/sys/class/pwm/` via Python's `open()` and `write()` — no
GPIO library beyond what accesses the sysfs interface.

**I2C sensor readout without abstraction libraries:** Read raw data from an IMU (e.g.,
MPU-9250 or ICM-42688) over I2C using `smbus2`. Implement the register read sequence from
the datasheet: write the register address, read the 16-bit signed integer output, apply the
scale factor to convert to calibrated acceleration (m/s²) and angular rate (rad/s). No IMU
library.

**Sensor calibration from first principles:** Implement accelerometer 6-position bias
calibration: collect accelerometer means at 6 orientations (each axis up and down), solve for
bias and scale factor using `numpy.linalg.lstsq`. Implement magnetometer hard-iron
calibration: collect readings while slowly rotating the sensor, fit the minimum enclosing
ellipse to the 3D data cloud using `scipy.optimize.minimize`, extract the center as the
hard-iron offset. **Important field note:** On electric UAVs, the motor current drawn through
power cables near the magnetometer produces a throttle-dependent soft-iron field that
invalidates any static calibration. This manifests as a heading error that changes with
throttle setting. The mitigation is to mount the magnetometer on a tail boom or wingtip boom,
as far from the power runs as the airframe allows. Document this in the calibration
procedure.

**Latency and timing measurement:** Measure the round-trip latency of a MAVLink attitude
request-response using `time.perf_counter`. Plot the latency distribution. Identify sources
of jitter: USB interrupt latency (~1 ms), Python GIL, OS scheduler quantum.

**Hardware interface abstraction layer (Day 5):** Refactor the Weeks 7–9 autopilot code to
depend on an interface class `NavigationSource` with a single `get_state() → NavState`
method. Implement two concrete versions: `SimulationNavSource` (reads from the 6-DOF
simulator) and `MAVLinkNavSource` (reads from MAVLink `ATTITUDE` and `GLOBAL_POSITION_INT`
messages). This dependency injection pattern allows the autopilot logic to run unchanged
against the simulation, against a flight controller in HIL, or against a real aircraft —
only the interface implementation changes.

**Logging discipline:** All hardware-facing code in `uav/hardware/` uses Python's `logging`
module with appropriate log levels: `DEBUG` for raw byte streams and register values,
`INFO` for connection events and mode transitions, `WARNING` for out-of-range sensor values
or latency violations, `ERROR` for unrecoverable failures. `print()` is prohibited in any
file under `uav/` after Week 10 — all diagnostic output uses `logging`. This is enforced by
a `flake8` rule in `.pre-commit-config.yaml`.

### Python Ecosystem
`numpy`, `scipy`, `struct` (MAVLink binary encoding), `pyserial` (UART), `smbus2` (I2C),
`time`, `threading`, `logging`, `pytest`

### Capstone Deliverable
`hardware/` — MAVLink byte framer and message parser (built from `struct`), MAVLink message
constructors for key message types, UART interface class wrapping `pyserial`, PWM output via
sysfs, IMU raw register readout and calibration routines, magnetometer calibration with
power-cable interference warning, and the `NavigationSource` abstraction layer. Demonstrated:
Python on the companion computer reading IMU data at 100 Hz and sending attitude commands to
the flight controller, with round-trip latency measured and logged.

### Connection to Larger Goal
This week is the bridge between simulation and the physical world. Every interface built here
— MAVLink, sensor readout, PWM output — is exercised in HIL testing (Week 12) and real
flight (Week 14). The `logging` discipline established here is what makes on-aircraft
debugging possible without an attached terminal.

### References
- MAVLink serialization specification: https://mavlink.io/en/guide/serialization.html
- MAVLink common message set: https://mavlink.io/en/messages/common.html
- PX4 companion computer setup: https://docs.px4.io/main/en/companion_computer/
- MPU-9250 datasheet (register map, scale factor tables, calibration procedure)

---

## Week 12 — Hardware-in-the-Loop Testing and Autopilot Validation

### Theme
Hardware-in-the-loop (HIL) testing runs the student's Python autopilot against the 6-DOF
flight simulator, but with real hardware in the communication path. The flight controller
reads synthetic sensor data formatted as MAVLink, runs its own estimators, and outputs
actuator commands back to the simulator. Bugs that only appear under real-time execution —
timing errors, buffer overflows, latency-induced instabilities — emerge here, before they
can cause a crash in real flight.

### Prerequisite Knowledge
- Week 11: hardware interface layer, MAVLink byte-level implementation, companion computer
  setup
- Weeks 7–10: full autopilot and guidance stack

### Core Concepts

**HIL loop architecture:** The 6-DOF simulator runs on the companion computer. At each
timestep, the simulator generates synthetic sensor readings and encodes them as MAVLink
`HIL_SENSOR` (accelerometer, gyro, magnetometer, barometer, at 200 Hz) and `HIL_GPS`
(position, velocity, at 5 Hz) messages sent over UART to the flight controller. The flight
controller processes these messages, runs its internal EKF, runs its attitude controllers,
and sends back `HIL_ACTUATOR_CONTROLS` messages containing normalized actuator commands. The
companion computer applies these commands to the plant model (through the actuator dynamics
model from Week 7) and advances one simulation timestep.

**Real-time loop rate control:** The HIL loop must execute at a fixed rate with bounded
jitter. Implement using a tight loop with `time.perf_counter` correction: at the end of each
step, compute elapsed time, subtract from the nominal step period, and sleep the remainder.
Measure the actual loop period distribution using a ring buffer and `numpy.histogram`.
Standard deviation should be < 10% of the nominal period.

**Synthetic sensor injection with realistic noise:** The synthetic sensor readings must
include the noise models from Week 3 — accelerometer white noise, gyro bias drift, GPS noise.
Without noise, the flight controller's EKF is unrealistically optimistic. Verify that the
flight controller attitude estimate matches simulation truth within 1°.

**Pitot fallback verification in HIL:** Trigger the pitot blockage flag mid-mission in the
HIL simulation. Verify that the Week 8 airspeed controller correctly switches to the
synthetic airspeed estimate and that altitude and airspeed hold remain stable through the
transition. This failure mode must be verified in HIL before relying on it in real flight.

**Actuator mapping and control mixing:** The flight controller outputs normalized commands for
motor, aileron, elevator, rudder. Implement the airframe-specific mixer in
`hardware/mixer.py` that maps these to individual servo positions. Verify the mixer by
checking that a roll command produces correct differential aileron deflection.

**HIL regression testing:** Re-run the Week 9 5-waypoint mission in HIL. Compare performance
metrics against pure simulation results from Week 10. Differences quantify real-time effects
— latency, EKF differences, timing jitter — that were invisible in pure simulation. Document
each discrepancy and trace it to a root cause.

**Failure mode injection testing:** Inject failures during HIL: GPS dropout (stop sending
`HIL_GPS` messages), IMU spike (inject a single outlier sample), pitot blockage (set blockage
flag in the sensor injector), actuator saturation (clip the command before feeding the
plant). Verify the autopilot degrades gracefully: dead reckoning during GPS dropout, spike
rejection by the EKF, synthetic airspeed fallback during pitot blockage, continued stability
during saturation.

**Ground station via UDP socket:** Build a minimal ground control station using Plotly Dash
that receives MAVLink telemetry forwarded from the companion computer over UDP
(`socket.socket(socket.AF_INET, socket.SOCK_DGRAM)` — Python built-in `socket` module only).
Display real-time attitude, position, autopilot mode, and battery state. The UDP forwarding
thread on the companion computer and the Dash callback on the ground station must be
decoupled by a thread-safe queue (`collections.deque` or `queue.Queue`).

### Python Ecosystem
`numpy`, `scipy`, `struct`, `pyserial`, `socket`, `threading`, `time`, `collections`,
`logging`, `plotly` (Dash), `pytest`

### Capstone Deliverable
`hil/` — the HIL simulation loop, synthetic sensor injector (with noise and pitot blockage
flag), actuator mapper, and airframe mixer. `gcs/` — Plotly Dash ground station receiving
MAVLink telemetry over UDP. Demonstrated: a complete 5-waypoint mission executed in HIL with
cross-track error < 200 m and performance within 20% of pure-simulation results from Week 10.
Pitot blockage failure mode verified in HIL.

### Connection to Larger Goal
HIL testing is the final validation gate before committing to real flight. It has revealed
real-time timing constraints and flight controller EKF behavior that pure simulation cannot
expose. By the end of this week the student has engineering confidence — not optimism — in
their autopilot.

### References
- PX4 HIL documentation: https://docs.px4.io/main/en/simulation/hitl.html
- MAVLink `HIL_SENSOR` / `HIL_ACTUATOR_CONTROLS` message specs:
  https://mavlink.io/en/messages/common.html
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* — Chapter 14

---

## Week 13 — System Integration, Ground Testing, and Pre-Flight Validation

### Theme
The aircraft exists physically. This week is systems integration and formal ground testing:
assembling all hardware, verifying every subsystem on the bench with quantitative acceptance
criteria, and executing a signed pre-flight checklist. Flight is earned through systematic
test, not assumed. The student follows the same structured validation process used in
professional UAV development — every failure mode discovered on the ground is one that cannot
destroy the aircraft in the air.

**AMA compliance note:** Before any outdoor testing (even taxi tests), verify that AMA
membership is active, the aircraft is registered, and the test site is an AMA-chartered field
or otherwise approved for model aircraft operations. This should have been completed as a
background task starting in Week 1 — if not, address it now before Week 14.

### Prerequisite Knowledge
- Weeks 11–12: hardware interface, HIL validation, mixer
- Week 6: propulsion system expected performance metrics
- Week 5: structural weight estimate and CG analysis

### Core Concepts

**Component assembly and CG verification:** Assemble all components. Weigh each individually
and record in a `pandas` DataFrame weight log. Compare to the Week 5 weight estimate —
discrepancies reveal modeling errors that must be understood. Measure the physical CG
location and compare to the Week 4 aerodynamic stability analysis. Adjust CG by repositioning
the battery; verify it is within the allowable CG range. The CG position must satisfy the
Week 4 static margin constraint: `(x_NP - x_CG) / c̄ ≥ 0.10` (10% static margin minimum).

**Servo deflection calibration — quantitative:** With the autopilot armed (propeller removed
for safety), command a series of known deflection angles (−20°, −10°, 0°, +10°, +20°) on
each control surface via the RC transmitter. Measure the actual surface angle with a digital
protractor and record in a `pandas` DataFrame. Fit a linear calibration curve
`δ_actual = slope · δ_cmd + offset` using `numpy.linalg.lstsq`. If the slope deviates from
1.0 by more than 5%, or the residual exceeds 1°, adjust the servo mechanical linkage before
proceeding. Store the calibration coefficients in `UAVConfig` — the autopilot uses these to
correct commanded deflections. A single reversed servo causes loss of control and is a common
cause of first-flight crashes.

**Propulsion bench test and model validation:** Mount the motor-propeller-battery system on a
thrust stand instrumented with a load cell (read via the Week 11 serial interface). Measure
static thrust at 25%, 50%, 75%, and 100% throttle. Compare to the Week 6 `PropulsionSystem`
model prediction. Measure current draw at each throttle setting and verify it does not exceed
ESC and battery C-rating limits. If thrust predictions differ from measurements by > 10%,
identify the discrepancy (propeller coefficients, motor KV, battery internal resistance) and
correct the model.

**IMU vibration characterization:** With the motor running at cruise throttle (aircraft
tethered or held securely), log IMU accelerometer data at 1 kHz. Compute the PSD using
`scipy.signal.welch`. The dominant motor vibration frequency is `f_vib = n_RPM / 60 ·
N_blades`. Verify this frequency is below the IMU's Nyquist frequency (fs/2) and that the
vibration amplitude at the EKF's process noise bandwidth does not contaminate the navigation
solution. If vibration is excessive, add vibration damping mounts.

**Software ground-test run:** With the aircraft on the ground and propeller removed, arm the
autopilot and run through the mission state machine manually (advancing states by RC switch)
while observing control surface deflections and ground station telemetry. Every autopilot
mode transition must be verified before flight: confirm actuator command continuity at each
FSM state transition (the Week 9 integrator pre-loading) by observing the ground station
servo position log.

**Failure mode documentation:** For each critical system, document: what happens if it fails,
how the autopilot responds (with reference to the Week 9 FSM state transitions), and what the
ground operator should do. GPS loss → dead reckoning → return to launch after 30 s. Low
battery → return to launch when SOC < 20%. RC link loss → return to launch after 3 s. Motor
failure → best-glide attitude, alert operator. Pitot blockage → synthetic airspeed fallback,
continue mission with degraded accuracy. This is not a formality — understanding failure
modes is how professional engineers prevent accidents.

**Pre-flight checklist development:** Write a formal checklist in a structured `pandas`
DataFrame: item description, category (physical/avionics/propulsion/software), pass/fail
criterion, measured value, and result column. Categories: physical inspection (control
surface freedom, propeller tightness, connector seating, CG check, servo calibration
slopes), avionics (flight controller boot, sensor health, GPS lock within 60 seconds, RC
link), propulsion (battery charge ≥ 95%, motor run-up, vibration PSD within limits, no
abnormal noise), software (mission loaded and validated, waypoints verified on ground station
map, kill switch tested, logging active). Sign off each item. The checklist is updated each
time a new failure mode is discovered during ground testing.

### Python Ecosystem
`numpy`, `scipy` (`signal.welch` for vibration PSD, `linalg.lstsq` for servo calibration),
`pyserial`, `struct`, `pandas`, `matplotlib`, `plotly`, `logging`

### Capstone Deliverable
Signed pre-flight checklist (all items passed), component weight log (`pandas` DataFrame)
verified against Week 5 estimates, servo deflection calibration curves and coefficients
stored in `UAVConfig`, thrust-stand test report comparing measured thrust to propulsion model
predictions, vibration PSD plot confirming no IMU aliasing, and failure mode documentation.
The aircraft is declared ready for flight when every checklist item is signed off.

### References
- AMA Safety Code: https://www.modelaviation.com/safety
- MIL-STD-882E — System Safety (failure mode documentation methodology)

---

## Week 14 — First Flight, Flight Test Program, and Performance Validation

### Theme
The aircraft flies. This week is a structured flight test program that expands the envelope
progressively: manual flight to verify basic airworthiness, then inner loops, then outer
loops, then full autonomous mission. Flight test is systematic hypothesis testing: predict
behavior from the simulation, measure it in real flight, reconcile differences, and update
the model. The student leaves the semester with a flying autonomous fixed-wing UAV and a
simulation model validated against real flight data.

**Scope note:** Flight test is dominated by weather, logistics, site access, and the
possibility that the aircraft does not behave as predicted on the first attempt. Week 14
therefore scopes its deliverables to what is achievable regardless of how many flights
succeed: manual flight airworthiness assessment, inner loop validation, and flight data
logging are the primary deliverables. System identification and the final performance report
are post-flight analysis tasks that can be completed on the ground after any successful
logging flight — they do not require additional flights.

### Prerequisite Knowledge
- All previous weeks
- Week 13: pre-flight checklist signed off, all ground tests passed
- Active AMA membership and registered aircraft
- Confirmed access to a suitable flying site

### Core Concepts

**Progressive envelope expansion:** Never go to full autonomous navigation on the first
flight. Phase 1: manual RC flight — verify basic airworthiness, trim elevator position, and
control authority. Phase 2: stabilization assist — inner rate loops active, pilot commands
attitude via RC. Phase 3: outer loops — altitude and heading hold engaged. Phase 4: single
waypoint autonomous navigation. Phase 5: full 5-waypoint mission. Each phase gates the next;
no phase is exited until its quantitative performance criteria are met.

**Flight data logging:** Log all available data: IMU at 100 Hz, GPS at 10 Hz, autopilot
state and commands at 50 Hz, MAVLink telemetry, battery SOC. Store as timestamped CSV files
using `pandas`. Verify the logging pipeline is active and recording before each flight —
data is the primary analysis resource. Without logs, flight test is anecdotal.

**Manual flight airworthiness assessment:** On the first flight, fly manually under RC
control. Assess: does the aircraft lift off near the predicted stall speed? Is the trim
elevator consistent with the Week 4/5 design? Are control authority and response rates
appropriate? Any unexpected behavior (unexpected roll-off, uncommanded pitch, excessive
sensitivity) terminates the flight and requires ground investigation before proceeding.

**Autopilot channel validation — quantitative:** For each outer-loop channel, measure actual
closed-loop performance and compare to simulation predictions from Week 10: altitude hold
bandwidth and disturbance rejection (excite with a step command, measure overshoot and
settling time), airspeed hold step response, heading hold step response and cross-track error
during a straight-line segment. Overlay measured and simulated time histories on the same
Plotly chart. Discrepancies are attributed to: unmodeled aerodynamics, actuator nonlinearities,
EKF error, and real turbulence intensity.

**Autonomous mission validation:** Fly the same 5-waypoint mission from Week 9/10. Log
cross-track error, altitude deviation, and waypoint capture events. Compare to simulation
predictions. Visualize the real GPS ground track on the Week 10 Plotly Dash replay
application — the same dashboard now shows real flight data in the same interface used to
analyze simulated data.

**System identification from flight data (post-flight analysis):** Fly doublet maneuvers
(rapid reversal of a control surface) for each axis. Use `scipy.optimize.minimize` to fit
the linearized model stability derivatives to the measured flight response — minimize the sum
of squared errors between simulated and measured state time histories. The fitted parameters
update the 6-DOF simulation to match the real aircraft, improving all future simulation
predictions. This analysis is performed after landing using the flight log CSVs — it does not
require additional flights.

**Final performance report (post-flight analysis):** Produce a Plotly Dash report comparing:
design predictions (Weeks 4–6) vs. simulation predictions (Weeks 7–10) vs. measured flight
performance for stall speed, cruise airspeed, climb rate, turn radius, autopilot tracking
error, and endurance. Quantify the prediction accuracy at each modeling step. This report is
the technical deliverable that closes the semester and can be produced from any successful
logging flight.

**Post-flight model update cycle:** After each flight, update the 6-DOF simulation with
system-identified parameter corrections. Re-run the Week 10 mission simulation with updated
parameters and quantify the prediction improvement. The simulation becomes progressively more
accurate with each flight — this is the iteration cycle of real flight test programs.

### Python Ecosystem
`numpy`, `scipy` (`optimize`, `signal`, `integrate`), `pandas`, `matplotlib`, `plotly`
(Dash), `json`, `logging`, `pytest`

### Capstone Deliverable
Flight log CSVs from all test flights, the GPS ground track visualized in the Week 10 Plotly
Dash replay dashboard, a system identification report comparing pre-flight model parameters
to flight-estimated parameters with residual error quantification, and a final performance
comparison report covering all key metrics across design → simulation → flight. The
post-flight analysis deliverables are independent of flight count — a single successful
logging flight is sufficient to produce them.

### Connection to the Larger Goal
First flight validates everything built in the previous 13 weeks. The student now owns a
complete aerospace engineering workflow — from requirements through aerodynamic design,
structural sizing, propulsion selection, simulation, navigation, autopilot design, hardware
integration, and flight test — implemented entirely in general-purpose Python, with every
domain-specific algorithm built by hand. That is the difference between a tool user and an
engineer.

### References
- Klein & Morelli, *Aircraft System Identification: Theory and Practice* — Chapters 1–5
- Etkin & Reid, *Dynamics of Flight: Stability and Control* — Chapter 1 (flight test
  philosophy)
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* — Chapter 14
- AMA Safety Code: https://www.modelaviation.com/safety

---

---

## Appendix A — Running Software Engineering Thread

The following habits are introduced in Week 1 and enforced throughout every subsequent week.
Each weekly schedule must include at least one milestone that exercises these practices
explicitly.

**Version control:** Git commit at every milestone, with messages referencing the milestone:
`"W4-D2-M1: VLM AIC matrix validated against AVL within 4%"`. Branch per week, merge to
`main` only when all `pytest` tests pass and GitHub Actions CI reports green. The commit
history is a lab notebook.

**Modular architecture:** One concern per module. By Week 10 the project has ≥ 20 modules.
Functions called from two or more places live in a shared module — never copy-pasted. The
MATLAB habit of one enormous script is explicitly named and rejected at every opportunity.

**Automated testing:** Every numerical result has a quantitative `assert np.allclose(result,
expected, atol=tol)` assertion. By Week 10, `pytest` runs ≥ 40 tests in < 3 minutes.
`pytest` fixtures and parametrize are introduced in Week 2 and used throughout.

**Type annotations and `mypy`:** All function signatures carry type annotations from Week 1
onward. `mypy --strict` must report zero errors on every week that introduces new classes or
dataclasses (Weeks 2, 3, 5, 7, 9, and 11 at minimum). This is not optional: a typed
codebase is a documented codebase.

**Unit comments on physical quantities:** `dt = 0.01  # sample period [s], 100 Hz`. Every
physical constant carries units. Every state vector index is named: `V_TAS = x[0]  # true
airspeed [m/s]`. Magic numbers are an error.

**Parameterized configuration:** All aircraft parameters live in a dataclass (`UAVConfig` or
`UAVDesign`). Nothing is hard-coded in simulation or analysis functions. Changing the aircraft
means changing one configuration file. Every configuration dataclass includes a `config_hash`
field (SHA-256 of all field values, computed in `__post_init__`) that is logged alongside
every simulation result. This makes every output traceable to the exact configuration that
produced it.

**Random seed discipline:** All stochastic simulations use `numpy.random.default_rng(seed)`
with the seed stored in the configuration object as `UAVConfig.monte_carlo_seed`. The default
seed is 42. Changing the seed is a deliberate experiment with a documented reason — not a
debugging tool. This ensures that Monte Carlo results are reproducible across sessions and
machines.

**Logging over print:** All code in `uav/` uses Python's `logging` module. Log levels: `DEBUG`
for raw data streams, `INFO` for state transitions and milestone completions, `WARNING` for
out-of-range values, `ERROR` for unrecoverable failures. `print()` is prohibited in `uav/`
after Week 10. This is enforced by a `flake8` rule in `.pre-commit-config.yaml`. The
rationale: on hardware, there is no terminal attached — `logging` output can be redirected
to a file while `print()` is silently discarded.

**CI pipeline:** GitHub Actions runs `pytest` and `mypy --strict` automatically on every push
to any `week-NN` branch. Configured in `.github/workflows/ci.yml` on Day 1 of Week 1. The
`main` branch is protected: merges are blocked if CI is failing.

**Dependency management:** `requirements.in` lists direct dependencies only, unpinned
(human-maintained). `requirements.txt` is generated by `pip freeze --local` and lists all
transitive dependencies with pinned versions. Both files are committed. When deploying to the
companion computer (a different architecture), regenerate `requirements.txt` on the target
platform rather than copying from the development machine.

---

## Appendix B — Week-by-Week Library Progression

All libraries are either Python standard library or general-purpose scientific computing
packages. No aerospace domain library appears anywhere in the curriculum.

| Week | Libraries Introduced or Extended |
|---|---|
| 1 | `numpy`, `scipy.integrate`, `scipy.signal`, `scipy.linalg`, `pandas`, `matplotlib`, `plotly`, `python-control`, `logging` |
| 2 | `dataclasses`, `scipy.optimize.fsolve`, `pytest`, `mypy` |
| 3 | `scipy.spatial.transform` (verification only), `json` |
| 4 | `subprocess`, `scipy.integrate.quad`, `scipy.optimize.brentq`, `scipy.interpolate` |
| 5 | `scipy.optimize.minimize_scalar`, `scipy.integrate.trapezoid` |
| 6 | `scipy.interpolate.CubicSpline`, `scipy.interpolate.RegularGridInterpolator` |
| 7 | `python-control` deeper: `ss2tf`, `rlocus`, `margin`, `bode` |
| 8 | `plotly.dash`, `scipy.signal.lti` |
| 9 | `json` (mission format), `dataclasses` (mission types) |
| 10 | `scipy.signal.cont2discrete`, `scipy.signal.welch`, `plotly.dash` (extended) |
| 11 | `struct`, `pyserial`, `smbus2`, `threading`, `time` |
| 12 | `socket`, `collections.deque`, `queue` |
| 13 | `scipy.signal.welch` (vibration PSD), `scipy.linalg.lstsq` (servo calibration) |
| 14 | `scipy.optimize.minimize` (system ID), `scipy.signal.lsim` |

---

## Appendix C — Software Engineering Standards

This appendix defines the Git workflow, commit conventions, docstring standard, and tooling
configuration used for the entire semester. Everything here is established on Day 1 of Week 1
and maintained without reorganization.

---

### Git Workflow

**Branching:** Create a branch `week-NN` from `main` at the start of each week. Work
exclusively on that branch during the week. Merge to `main` only when the week's capstone
`pytest` suite passes completely and GitHub Actions CI reports green. The `main` branch is
branch-protected and always contains working, tested code.

**Commit message format:** `W{week}-D{day}-M{milestone}: brief description`
Example: `W2-D3-M1: numerical Jacobian verified against Week 1 A matrix`
Every milestone produces a commit. Commits that do not correspond to a milestone use
`W{week}-D{day}: description`.

**What to commit:** Source code, `requirements.txt`, `requirements.in`, test files, summary
documents, the hardware BOM CSV, and the `.github/` workflow directory.
Never commit: `.venv/`, `__pycache__/`, generated figures (`figures/*.html`, `figures/*.svg`),
or large data files (`data/*.npy`, `data/*.csv`). The `.gitignore` at the repository root
enforces these exclusions from Day 1.

**Promotion to `uav/`:** When a week's module is stable and its tests pass, its core
components are copied (not symlinked) into the `uav/` directory. This promotion happens at
the start of the following week, before new development begins. The copy in `uav/` may be
modified as integration requirements become clear; the copy in `weeks/` is frozen as the
standalone reference implementation.

---

### Docstring Standard

All functions use NumPy-style docstrings throughout the semester.

```python
def function_name(param1: float, param2: np.ndarray) -> np.ndarray:
    """
    One-line summary of what the function does.

    Longer description if needed. Explain the mathematical basis and any
    important assumptions (e.g., small-angle, rigid body, ISA atmosphere).

    Parameters
    ----------
    param1 : float
        Description with units. Example: airspeed [m/s].
    param2 : np.ndarray, shape (N,)
        Description with shape and units. Example: state vector [u, w, q, θ].

    Returns
    -------
    np.ndarray, shape (N,)
        Description with shape and units.

    Notes
    -----
    Any implementation notes, references to equations in the textbook,
    or known limitations.
    """
```

Physical constants and all intermediate variables with physical meaning carry unit comments
on the same line: `rho = 1.225  # air density at sea level [kg/m³]`. This is enforced by
code review and by a `flake8` custom rule in `.pre-commit-config.yaml`.

---

### Pre-commit Configuration

The following hooks run automatically on every `git commit`. They are configured in
`.pre-commit-config.yaml` and installed with `pre-commit install` on Day 1.

- **`black`** — automatic code formatting. `black`'s defaults are the standard. The student
  never manually formats code after Day 1.
- **`isort`** — automatic import sorting, compatible with `black` (use `--profile black`).
- **`flake8`** — linting for unused imports, undefined names, line length violations, and a
  custom rule prohibiting `print()` in any file under `uav/`.
  Configured with `max-line-length = 88` to match `black`.

---

### Weekly Skills Summary

After each week's branch merges to `main`, the student writes a skills summary document and
saves it to `docs/weekly_summaries/week_NN_summary.md`. This document is the context
artifact that is added to the Claude Project before generating the next week's tasks.

The summary must include:

- **Project structure** — the directory tree of the completed week's module
- **Packages introduced** — any new libraries added to `requirements.txt` this week
- **Functions written** — a table of every public function: name, signature, module path,
  and one-sentence description of purpose
- **Concepts covered** — a brief list mapping topic to day (e.g., "Kalman filter
  predict-update loop — Day 1 of Week 3")
- **Capstone state** — what `python main.py` produces at end of week: what files it writes,
  what the dashboard shows, what `pytest` reports
- **config_hash** — the SHA-256 hash of the `UAVConfig` or `UAVDesign` in use at end of
  week, so that any simulation result from this week can be traced back to the exact
  configuration that produced it

The summary is intentionally terse — its purpose is to give the schedule generator enough
context to reference prior work correctly, not to re-explain the concepts. It is not a
tutorial; it is an interface specification for the weeks that follow.
