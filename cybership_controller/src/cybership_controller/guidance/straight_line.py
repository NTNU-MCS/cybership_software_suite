import numpy as np
import matplotlib.pyplot as plt
from shoeboxpy.model3dof import Shoebox


class StraightLineGuidance:
    """Maneuvering guidance: straight path rho(theta) + speed assignment theta_dot = v_s.

        g = StraightLineGuidance(lam=0.01, k=8.0)
        g.v_d, g.psi_ref, g.sigma = 0.05, 0.0, 1          # params (slider/box/checkbox)
        g.eta_signal = measured_pose                       # written by a ROS subscriber
        g.set_line(p_end=[3,2], p_start=[0,0])             # p_start=None -> use eta_signal
        eta_d, eta_d_dot, eta_d_ddot = g.step(dt)          # timer callback
    """

    def __init__(self, lam=0.01, k=10.0):
        self.lam = lam              # small regularization (lambda << 1)
        self.k = k                  # tanh ramp steepness, typically k=10
        # ---- incoming ROS signal (subscriber writes here) ----
        self.eta_signal = np.zeros(3)   # measured vessel pose [x, y, psi]
        # ---- parameters (GUI / ROS params) ----
        self.v_d = 0.0              # desired speed [m/s]   (slider)
        self.psi_ref = 0.0          # heading reference [rad] (number box)
        self.sigma = 1              # smooth-acceleration checkbox {0,1}
        # ---- internal path state (set by set_line) ----
        self.p0 = np.zeros(2)
        self.p1 = np.zeros(2)
        self.rho_theta = np.zeros(2)
        self._len = 0.0             # path length ||p1 - p0||
        self.theta = 0.0
        self.active = False

    # ---- define the straight line between two arbitrary points ----
    def set_line(self, p_end, p_start=None):
        if p_start is None:
            p_start = self.eta_signal[:2]        # default: start at the vessel
        self.p0 = np.asarray(p_start, float)[:2].copy()
        self.p1 = np.asarray(p_end,   float)[:2].copy()
        # self.rho_theta = (self.p1 - self.p0) / (1.0 + self.lam)
        self.rho_theta = (self.p1 - self.p0) 
        self._len = float(np.linalg.norm(self.p1 - self.p0))
        if self.v_d < 0.0:
            # self.theta = (1.0 + 2.0 * self.lam) if self.sigma == 1 else 1.0
            self.theta = 1.0
        else:
            self.theta = 0.0
        self.active = True

    def rho(self, theta):
        # return ((1.0 + self.lam) * self.p0 + (self.p1 - self.p0) * theta) / (1.0 + self.lam)
        return (self.p0 + (self.p1 - self.p0) * theta)

    def gamma(self, theta):
        if self.v_d < 0.0:
            # mid = 0.5 * (1.0 + 2.0 * self.lam)
            mid = 0.5 + self.lam
            if theta <= mid:              return np.tanh(self.k * theta)
            if theta <= 1.0 + 2*self.lam: return np.tanh(self.k * (1.0 + 2.0*self.lam - theta))
            return 0.0
        # if theta < -self.lam:  return 0.0
        if theta < -2*self.lam:         return 0.0
        # if theta <= 0.5:       return np.tanh(self.k * (self.lam + theta))
        if theta <= 0.5-self.lam:       return np.tanh(self.k * (theta + 2*self.lam))
        # return np.tanh(self.k * (1.0 + self.lam - theta))
        return np.tanh(self.k * (1.0 - theta))

    def gamma_theta(self, theta):
        if self.v_d < 0.0:
            # mid = 0.5 * (1.0 + 2.0 * self.lam)
            mid = 0.5 + self.lam
            if theta <= mid:              return  self.k * (1.0 - np.tanh(self.k * theta)**2)
            if theta <= 1.0 + 2*self.lam: return -self.k * (1.0 - np.tanh(self.k * (1.0 + 2.0*self.lam - theta))**2)
            return 0.0
        # if theta < -self.lam:  return 0.0
        if theta < -2*self.lam:         return 0.0
        # if theta <= 0.5:       return  self.k * (1.0 - np.tanh(self.k * (self.lam + theta))**2)
        if theta <= 0.5-self.lam:       return  self.k * (1.0 - np.tanh(self.k * (theta + 2*self.lam))**2)
        # return -self.k * (1.0 - np.tanh(self.k * (1.0 + self.lam - theta))**2)
        return -self.k * (1.0 - np.tanh(self.k * (1.0 - theta))**2)

    def speed(self, theta):
        if self._len < 1e-9:
            return 0.0, 0.0
        if self.sigma == 1:
            return (self.gamma(theta)       * self.v_d / self._len,
                    self.gamma_theta(theta) * self.v_d / self._len)
        return self.v_d / self._len, 0.0

    def step(self, dt):
        vs, vs_theta = self.speed(self.theta)
        theta_dot, theta_ddot = vs, vs_theta * vs
        p_d     = self.rho(self.theta)
        pd_dot  = self.rho_theta * theta_dot
        pd_ddot = self.rho_theta * theta_ddot
        eta_d      = np.array([p_d[0],     p_d[1],     self.psi_ref])
        eta_d_dot  = np.array([pd_dot[0],  pd_dot[1],  0.0])
        eta_d_ddot = np.array([pd_ddot[0], pd_ddot[1], 0.0])
        new_theta = self.theta + dt * theta_dot
        self.theta = max(new_theta, 0.0) if self.v_d < 0.0 else min(new_theta, 1.0)
        return eta_d, eta_d_dot, eta_d_ddot

    @property
    def arrived(self):
        if self.v_d < 0.0:
            return self.theta <= 0.0
        return self.theta >= 1.0


class DPTrackingController:
    """Model-based 3-DOF tracking law (feedback linearization incl. Coriolis).

        c = DPTrackingController(model=Shoebox(L,B,T), Kp=..., Kd=...)
        c.eta_signal, c.nu_signal = pose, vel                       # observer subscriber
        c.eta_d_signal, c.eta_d_dot_signal, c.eta_d_ddot_signal = ref  # guidance subscriber
        tau = c.compute_tau()                                       # control timer
    """

    def __init__(self, model, Kp, Kd):
        self.model = model                       # Shoebox: M_eff, D, C_RB, C_A, J
        self.Kp = np.asarray(Kp, dtype=float)
        self.Kd = np.asarray(Kd, dtype=float)
        # ---- incoming ROS signals (subscribers write here) ----
        self.eta_signal       = np.zeros(3)      # measured pose      [x, y, psi]
        self.nu_signal        = np.zeros(3)      # measured velocity  [u, v, r]
        self.eta_d_signal     = np.zeros(3)      # reference pose
        self.eta_d_dot_signal = np.zeros(3)      # reference velocity
        self.eta_d_ddot_signal= np.zeros(3)      # reference acceleration

    @staticmethod
    def Jdot(psi, r):
        c, s = np.cos(psi), np.sin(psi)
        return r * np.array([[-s, -c, 0.0], [c, -s, 0.0], [0.0, 0.0, 0.0]])

    @staticmethod
    def wrap(a):
        # Here you can rather use np.arctan2(np.sin(a), np.cos(a)) to wrap the angle to [-pi, pi]
        # return (a + np.pi) % (2.0 * np.pi) - np.pi
        return np.arctan2(np.sin(a), np.cos(a))

    def compute_tau(self):
        eta = np.asarray(self.eta_signal, float)
        nu  = np.asarray(self.nu_signal,  float)
        eta_d, eta_d_dot, eta_d_ddot = self.eta_d_signal, self.eta_d_dot_signal, self.eta_d_ddot_signal

        M = self.model.M_eff
        D = self.model.D
        J = self.model.J(eta)
        C = self.model.C_RB(nu) + self.model.C_A(nu)

        # We should rewrite this control law, but for now we will keep it as is.
        e = eta - eta_d
        e[2] = self.wrap(e[2])                         # wrap heading error
        e_dot = J @ nu - eta_d_dot                     # eta_dot - eta_d_dot
        a_cmd = eta_d_ddot - self.Kd @ e_dot - self.Kp @ e
        tau = C @ nu + D @ nu + M @ J.T @ (a_cmd - self.Jdot(eta[2], nu[2]) @ nu)
        return tau
