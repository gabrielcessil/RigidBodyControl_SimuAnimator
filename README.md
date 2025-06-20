
# Robotic Mechanism Simulator and Visualizer

## Overview

This repository provides a Python-based tool for simulating and **visualizing the dynamic motion of rigid body mechanisms under a specified control law**. The framework numerically solves the nonlinear dynamics of a mechanism based on the Lagrangian formulation and produces an **interactive HTML animation file** that illustrates both the physical motion (in 3D space) and the evolution of state variables over time.

This tool is designed for academic and educational purposes in the fields of **dynamics, control systems, robotics, and mechanical systems modeling**.

A complete implementation of the **Furuta Pendulum** (rotational inverted pendulum) is included, using the rigorous dynamic model derived in:

> B. S. Cazzolato and Z. Prime, "On the Dynamics of the Furuta Pendulum," *Journal of Control Science and Engineering*, vol. 2011, Article ID 528341. [https://doi.org/10.1155/2011/528341](https://doi.org/10.1155/2011/528341)

---

## Purpose

This framework is developed to serve as an educational and research tool that enables:

- **Dynamic simulation** of rigid body mechanisms subject to external inputs (control laws).
- **Visualization of the resulting motion** through 3D kinematic animation synchronized with the time evolution of system states.
- A platform for validating dynamic models, studying control behavior, and illustrating complex dynamic coupling effects in mechanical systems.

It is particularly suited for applications in:

- Control system education,
- Mechanical systems analysis,
- Nonlinear dynamics demonstration,
- Development and validation of control laws.

---

## Key Features

- General-purpose dynamic solver based on the Euler-Lagrange formulation.
- Modular architecture:
  - `Mechanism`: Defines the system's dynamics and kinematics.
  - `MechanicalSystemAnimation`: Handles 3D visualization and time-series plotting.
- **Output: An interactive HTML file** containing:
  - 3D kinematic animation of the mechanism.
  - Synchronized time plots of all system states (e.g., angles, angular velocities).
- Supports both open-loop and closed-loop simulations, depending on the provided control law.
- Easily extensible to other mechanisms by defining mass, Coriolis, and gravity models.

---

## Value for Dynamics and Control Visualization

This tool provides a **visual and quantitative understanding** of how a given rigid body mechanism responds to external forces or control inputs. It serves as an intuitive complement to the mathematical formulation of dynamic systems by allowing users to:

- Observe dynamic coupling, energy exchanges, and nonlinear effects.
- Validate dynamic models through visual inspection and time-based diagnostics.
- Test and refine control strategies by visually examining system behavior.

---

## Example: Furuta Pendulum Simulation

### Model Description

The Furuta Pendulum is a classical underactuated system composed of:

- A horizontal base arm actuated by a torque (`theta_base`).
- A pendulum arm attached to the end of the base, free to rotate in the vertical plane (`theta_pend`).

This simulation accurately represents the nonlinear coupled dynamics, including full inertia tensors, centripetal, Coriolis, viscous damping, and gravitational forces, following the derivations from Cazzolato and Prime (2011).

### Code Example

```python
from rotary_pendulum import Rotary_Pendulum
from mechanism_animation import MechanicalSystemAnimation
import numpy as np

# Define system parameters (per Cazzolato & Prime, 2011)
params = {
    "m1": 0.300,
    "m2": 0.075,
    "L1": 0.278,
    "L2": 0.300,
    "l1": 0.150,
    "l2": 0.148,
    "J1": 2.48e-2,
    "J2": 3.86e-3,
    "b1": 1e-4,
    "b2": 2.8e-4
}

pendulum = Rotary_Pendulum(params)

def control_law(t, q):
    return np.array([0.2, 0])

initial_state = {
    "theta_base": 0,
    "theta_pend": 0,
    "omega_base": 0,
    "omega_pend": 0
}

pendulum.solve_system(control_law, x0=initial_state, tf=4, steps=400)

animator = MechanicalSystemAnimation(pendulum)
animator.create_animation()
