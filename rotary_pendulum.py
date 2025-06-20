import numpy as np
from scipy.integrate import solve_ivp
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Generic mechanism: Any specific mechanism must inherit this class
class Mechanism:
    def __init__(self):
        self.time = None
        self.states = None

    # Eulerian (Lagrangian) system's dynamic, from typical form:
    # M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) = T (control input)
    # Inputs:
    # - t : float
    #     Current time (used if the control law is time-dependent).
    # - x : ndarray, shape (2 * N,)
    #     State vector of the system, where N is the number of degrees of freedom.
    #     The first N elements are the generalized positions (q),
    #     and the last N elements are the generalized velocities (q_dot):
    #     x = [q1, q2, ..., qN, q1_dot, q2_dot, ..., qN_dot]
    # - control_law : callable
    #     Function that provides the control input (generalized forces or torques).
    #     Must have the signature control_law(t, q) and return an array of shape (N,)

    def dynamics(self, t, x, control_law):
        q = x[:len(x) // 2]         # Generalized positions (first N//2 elementos of x)
        q_dot = x[len(x) // 2:]     # Generalized velocities (last N//2 elementos of x)

        M_eval = self.M(q)
        T_eval = control_law(t, q)
        C_eval = self.C(q, q_dot)
        G_eval = self.G(q)

        q_ddot = np.linalg.inv(M_eval) @ (T_eval - C_eval @ q_dot - G_eval)

        return np.concatenate([q_dot, q_ddot])

    def solve_system(self, control_law, x0, tf=10, t0=0, steps=500):
        # Solved time range
        self.time = np.linspace(t0, tf, steps)
        # Solve via scipy integration
        sol = solve_ivp(self.dynamics,
                        [self.time[0], self.time[-1]],
                        list(x0.values()),
                        t_eval=self.time,
                        args=(control_law,),
                        method='Radau')
        # Save states in proper format
        self.states = dict(zip(x0.keys(), sol.y))
        return self.states

    def direct_kinematics(self, state):
        raise NotImplementedError("direct_kinematics must be implemented in the subclass.")
    def M(self, state):
        raise NotImplementedError("M must be implemented in the subclass, following the rigid body equation of dynamic motion.")
    def G(self, state):
        raise NotImplementedError("G must be implemented in the subclass, following the rigid body equation of dynamic motion.")
    def C(self, state):
        raise NotImplementedError("C must be implemented in the subclass, following the rigid body equation of dynamic motion.")


# Example of mechanism: defined by user
class Rotary_Pendulum(Mechanism):

    def __init__(self, params, origin=[0,0,0]):
        self.params = params
        self.origin = origin

    # Calculate positions in 3D space given a state
    def direct_kinematics(self, state):
        theta_base, theta_pen = state[0], -state[1]

        L1, L2 = self.params["L1"], self.params["L2"]
        x0, y0, z0 = self.origin

        # Posição do braço base
        x1 = L1 * np.cos(theta_base)
        y1 = L1 * np.sin(theta_base)
        z1 = 0

        # Posição do pêndulo
        x2 = x1 + L2 * np.sin(theta_base) * np.sin(theta_pen)
        y2 = y1 - L2 * np.cos(theta_base) * np.sin(theta_pen)
        z2 = - L2 * np.cos(theta_pen)

        return (x0, y0, z0), (x1, y1, z1), (x2, y2, z2)

    # Auxiliary methods
    # Get mass matrix
    def M(self, q):
      # Read parameters
      J1, J2 = self.params['J1'], self.params['J2']
      m1, m2 = self.params['m1'], self.params['m2']
      l1, l2 = self.params['l1'], self.params['l2']
      L1, L2 = self.params['L1'], self.params['L2']
      # Calculate composed parameters
      J1_ = J1+m1*l1**2
      J0_ = J1_+m2*L1**2
      J2_ = J2 +m2*l2**2
      # Calculate mass matrix elements
      M11 = J0_ + J2_*np.sin(q[1])**2
      M12 = m2*L1*l2*np.cos(q[1])
      M21 = m2*L1*l2*np.cos(q[1])
      M22 = J2_
      return np.array([[M11, M12],[M21, M22]])

    # Get centrifugal matrix
    def C(self, q, q_dot):
      # Read parameters
      J1, J2 = self.params['J1'], self.params['J2']
      m1, m2 = self.params['m1'], self.params['m2']
      l1, l2 = self.params['l1'], self.params['l2']
      L1, L2 = self.params['L1'], self.params['L2']
      b1, b2 = self.params['b1'], self.params['b2']
      # Calculate composed parameters
      J2_ = J2 +m2*l2**2
      # Calculate centrifugal parameters elements
      C11 = b1 + 0.5*q_dot[1]*J2_*np.sin(2*q[1])
      C12 = 0.5*q_dot[1]*J2_*np.sin(2*q[1])-m2*L1*l2*np.sin(q[1])*q_dot[1]
      C21 = -0.5*q_dot[0]*J2_*np.sin(2*q[1])
      C22 = b2
      return np.array([[C11,C12],[C21,C22]])

    # Get gravity matrix
    def G(self, q, g=9.81):
      # Get parameters
      m1, m2 = self.params['m1'],self.params['m2']
      L1, L2 = self.params['L1'],self.params['L2']
      # Calculate matrix elements
      G1, G2 = 0, g*m2*L2*np.sin(q[1])
      return np.array([G1,G2])

