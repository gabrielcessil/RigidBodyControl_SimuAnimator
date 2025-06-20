from rotary_pendulum import Rotary_Pendulum
from mechanism_animation import MechanicalSystemAnimation
import numpy as np

# Pendulum's constructive parameters
params = {
    "m1": 0.300,    # kg (mass of the base)
    "m2": 0.075,    # kg (mass of the pendulum)
    "L1": 0.278,    # m (length of base arm)
    "L2": 0.300,    # m (length of pendulum)
    "l1": 0.150,    # m (center of mass of base arm)
    "l2": 0.148,    # m (center of mass of pendulum)
    "J1": 2.48e-2,  # kg*m^2 (moment of inertia of base)
    "J2": 3.86e-3,  # kg*m^2 (moment of inertia of pendulum)
    "b1": 1e-4,     # viscous damping on base
    "b2": 2.8e-4    # viscous damping on pendulum joint
}
rotary_pendulum = Rotary_Pendulum(params)

# Example of controlling input signal
past_theta_pend = 0
past_t = -0.00001
def u_base(t, q):
    # theta_pend_ref = np.pi # Reference: upward position
    # theta_base, theta_pend = q # States
    return np.array([0.2, 0])  # Only actuate the base


# Initial state of the mechanism
initial_state = { r"theta_base": 0, # Arbitrary
                  r"theta_pend": 0, # Downward position
                  r"omega_base": 0, # Equilibrium state
                  r"omega_pend": 0} # Equilibrium state

# Time definitions
tf      = 4  # Final instant
t0      = 0 # Initial instant
steps   = 400 # Number of time steps

# Call pendulum mechanism solver
rotary_pendulum.solve_system(control_law=u_base, x0=initial_state, tf=tf, t0=t0, steps=steps)

# Visualize pendulum with generic mechanism animator
animator = MechanicalSystemAnimation(rotary_pendulum)
animator.create_animation()