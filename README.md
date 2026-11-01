# Self-Balancing Robot

A low-cost two-wheeled self-balancing robot (ESP32, DC gear motors with encoders, MPU-6050),
built from first principles: the dynamic model, simulator, estimators and controllers are
written from scratch, without robotics libraries. The goal is a balance controller that
combines model-based control with a learned policy, robust to changes of the centre of mass.

<p align="center"><img src="docs/media/model.png" width="420" alt="Planar model of the robot"></p>

## Done so far

- Planar dynamic model derived with the Euler–Lagrange method, in the form
  `M(q) q̈ + h(q, q̇) = B τ`, and checked symbolically ([report](report/main.pdf)).
- NumPy simulator: forward dynamics, explicit Euler and RK4 integrators.
- Physics-based test suite (28 tests): mass matrix properties, equilibrium, agreement with the
  symbolic model, integrator order, energy conservation.
- First experiment: free fall with the motor off.

<p align="center"><img src="docs/media/free_fall.gif" width="520" alt="Free-fall simulation"></p>

## Plan

- [x] Planar dynamic model (Lagrange)
- [x] Simulator and integrators, free-fall experiment
- [ ] Linearisation and controllability
- [ ] PID and LQR in simulation
- [ ] Motor model (torque limits, dead zone, delay)
- [ ] MuJoCo model and cross-validation
- [ ] Bench tests: IMU and encoder drivers, complementary and Kalman filters, motor identification
- [ ] 3D-printed frame, assembly, LQR on the real robot
- [ ] Steering (differential drive)
- [ ] Learned controller: PPO, domain randomisation, adaptation to centre-of-mass changes

## Quick start (WSL2 / Linux)

```bash
bash environment/setup_wsl.sh   # once
pip install -e ".[dev]"
pytest
python experiments/01_free_fall.py
```
