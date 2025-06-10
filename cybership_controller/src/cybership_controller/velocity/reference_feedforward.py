from typing import Optional, Dict, Any
import numpy as np
from cybership_controller.utils import ExponentialSmoothing


class RffVelocityController():
    """Inverse-dynamics velocity controller with PI action and acceleration limiting."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        r"""Initialize the RffVelocityController.

        Args:
            config (Optional[Dict[str, Any]]): Configuration parameters dictionary.
                Defaults to None. Supported keys:
                - "M" (array-like): Inertia matrix, defaults to 3x3 identity matrix
                - "D" (array-like): Damping matrix, defaults to 3x3 zero matrix
                - "Kp" (array-like): Proportional gain matrix, defaults to 3x3 identity matrix
                - "Ki" (array-like): Integral gain matrix, defaults to 3x3 zero matrix
                - "Kd" (array-like): Derivative gain matrix, defaults to 3x3 zero matrix
                - "dt" (float): Time step size in seconds, defaults to 0.01
                - "a_max" (float): Hard acceleration limit, values <= 0 disable limit, defaults to -1.0
                - "I_max" (float): Integral wind-up cap, values <= 0 disable cap, defaults to -1.0
                - "smooth_limit" (bool): Enable smooth limiting behavior, defaults to False

               M = \\text{inertia matrix},\\quad D = \\text{damping matrix},\\quad K_p, K_i, K_d = \\text{gain matrices},\\\\
               dt = \\text{time step size}.


        Equations:
            .. math::
               M = \text{inertia matrix},\quad D = \text{damping matrix},\quad K_p, K_i, K_d = \text{gain matrices},\\
               dt = \text{time step size}.
        """

        self.config = config or {}
        # Matrices and gains
        self.M: np.ndarray = np.array(
            self.config.get("M", np.eye(3)), dtype=float)
        self.D: np.ndarray = np.array(
            self.config.get("D", np.zeros((3, 3))), dtype=float)
        self.Kp: np.ndarray = np.array(
            self.config.get("Kp", np.eye(3)), dtype=float)
        self.Ki: np.ndarray = np.array(
            self.config.get("Ki", np.zeros((3, 3))), dtype=float)
        self.Kd: np.ndarray = np.array(
            self.config.get("Kd", np.zeros((3, 3))), dtype=float)
        self.dt: float = float(
            self.config.get("dt", 0.01))  # seconds

        # Pre-compute inverse inertia
        self.M_inv = np.linalg.inv(self.M)

        # Limits and behaviours
        # hard acc limit (<=0 => off)
        self.a_max: float = float(self.config.get("a_max", -1.0))
        # integral wind-up cap (<=0 => off)
        self.I_max: float = float(self.config.get("I_max", -1.0))
        self.smooth: bool = bool(self.config.get("smooth_limit", False))

        # Internal state
        # for desired-velocity derivative
        self._prev_vd: Optional[np.ndarray] = None
        self._int_e: np.ndarray = np.zeros(3)       # integral of error dt

        self._prev_tau = None

    def update(self, current_velocity: np.ndarray, desired_velocity: np.ndarray, dt: float) -> np.ndarray:
        r"""Compute the force/torque command tau.

        The controller computes tau based on the following equations:

        .. math::
           e = current\_velocity - desired\_velocity

        .. math::
           v_{d\_dot} = \text{smoothed derivative of desired\_velocity}

        .. math::
           e_{dot} = \text{smoothed derivative of } e,\quad \text{where } e = current\_velocity - desired\_velocity

        .. math::
           a_{des} = v_{d\_dot} - K_p \cdot e - K_i \cdot \int e\,dt - K_d \cdot e_{dot}

        .. math::
           \tau = M \cdot a_{des} + D \cdot desired\_velocity

        Args:
            current_velocity (np.ndarray): Current measured velocity.
            desired_velocity (np.ndarray): Desired velocity.
            dt (float): Time step in seconds.

        Raises:
            ValueError: If dt is not positive.

        Returns:
            np.ndarray: Computed force/torque command tau.
        """

        if dt <= 0.0:
            return self._prev_tau if self._prev_tau is not None else np.zeros(3)

        v = current_velocity.reshape(3)
        vd = desired_velocity.reshape(3)

        if not hasattr(self, '_vd_dot_smoother'):
            r = self.config.get("filter_alpha", 0.7)
            self._vd_dot_smoother = ExponentialSmoothing(r=r)

        if not hasattr(self, '_e_dot_smoother'):
            r = self.config.get("filter_alpha", 0.7)
            self._e_dot_smoother = ExponentialSmoothing(r=r)

        raw_vd_dot = (vd - self._prev_vd) / \
            dt if self._prev_vd is not None else np.zeros(3)
        vd_dot = self._vd_dot_smoother(raw_vd_dot)
        self._prev_vd = vd.copy()

        e = v - vd
        raw_e_dot = (v - self._prev_vd) / \
            dt if self._prev_vd is not None else np.zeros(3)
        e_dot = self._e_dot_smoother(raw_e_dot)

        self._int_e = self._int_e + e * dt
        self._int_e = np.clip(self._int_e, -self.I_max, self.I_max)

        a_des = vd_dot - self.Kp @ e - self.Ki @ self._int_e - self.Kd @ e_dot

        tau = self.M @ a_des + self.D @ vd
        self._prev_tau = tau.copy()
        return tau
