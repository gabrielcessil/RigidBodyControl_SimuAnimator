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

    def solve_system(self, control_law, x0, tf=10, t0=0, steps=500, max_step=np.inf):
        # Solved time range
        self.time = np.linspace(t0, tf, steps)
        # Solve via scipy integration

        sol = solve_ivp(self.dynamics,
                        [self.time[0], self.time[-1]],
                        list(x0.values()),
                        t_eval=self.time,
                        args=(control_law,),
                        method='RK45',
                        max_step=max_step
                        )
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


import numpy as np


class KUKA_KR6(Mechanism):

    def __init__(self, params, origin=[0, 0, 0]):
        self.params = params
        self.origin = origin

    # Denavit-Hartenberg transformation
    def dh_transform(self, a, alpha, d, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])


    """
    State Definition:
    -----------------
    The state vector represents the 6 joint angles of the robot in radians, ordered from the base to the end effector:
    
        state = [θ1, θ2, θ3, θ4, θ5, θ6]
    
    Where:
    - θ1: Base rotation (rotation about Z-axis)
    - θ2: Shoulder pitch (arm moves forward/back)
    - θ3: Elbow motion
    - θ4: Wrist pitch (up/down)
    - θ5: Wrist yaw (side rotation)
    - θ6: Wrist roll (rotation around end-effector axis)
    """
    # Calculate forward kinematics (return the positions of each joint)
    def direct_kinematics(self, state):
        q = state
        x0, y0, z0 = self.origin

        # DH parameters
        dh = [
            (0, np.pi / 2, 0.400, q[0]),  # Link 1
            (0.455, 0, 0, q[1]),  # Link 2
            (0.035, np.pi / 2, 0, q[2]),  # Link 3 (offset link at elbow)
            (0, -np.pi / 2, 0.420, q[3]),  # Link 4
            (0, np.pi / 2, 0, q[4]),  # Link 5
            (0, 0, 0.080, q[5])  # Link 6 (wrist to tool flange)
        ]

        T = np.eye(4)
        positions = [(x0, y0, z0)]

        for i in range(6):
            A = self.dh_transform(*dh[i])
            T = T @ A
            pos = T @ np.array([0, 0, 0, 1])
            positions.append((pos[0] + x0, pos[1] + y0, pos[2] + z0))

        return positions

    # Simplified mass matrix (diagonal approximation)
    def M(self, q):
        J = [self.params[f"J{i + 1}"] for i in range(6)]
        return np.diag(J)

    # Simplified centrifugal matrix (viscous damping)
    def C(self, q, q_dot):
        b = [self.params[f"b{i + 1}"] for i in range(6)]
        return np.diag(b)

    # Gravity vector (approximate, along Z)
    def G(self, q, g=9.81):
        m = [self.params[f"m{i + 1}"] for i in range(6)]
        l = [self.params[f"L{i + 1}"] for i in range(6)]

        G = [m[i] * g * l[i] * np.cos(q[i]) for i in range(6)]
        return np.array(G)
