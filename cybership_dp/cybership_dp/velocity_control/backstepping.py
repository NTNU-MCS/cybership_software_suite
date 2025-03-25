#!/usr/bin/env python3

import numpy as np

class BacksteppingController:
    def __init__(self, mass, damping, k_gain):
        """
        Initialize the backstepping controller for surge, sway, and yaw.

        Parameters:
            mass (array-like): [m_surge, m_sway, m_yaw] parameters.
            damping (array-like): [d_surge, d_sway, d_yaw] damping coefficients.
            k_gain (array-like): [k_surge, k_sway, k_yaw] control gains.
        """
        self.mass = np.array(mass)
        self.damping = np.array(damping)
        self.k_gain = np.array(k_gain)

    def update(self, v, v_des, dv_des):
        """
        Compute the control input vector for surge, sway, and yaw.

        Parameters:
            v (np.array): Current velocity vector [surge, sway, yaw].
            v_des (np.array): Desired velocity vector [surge, sway, yaw].
            dv_des (np.array): Time derivative of the desired velocity vector.

        Returns:
            u (np.array): Control input vector [u_surge, u_sway, u_yaw].
        """
        # Tracking error for each channel
        e = v - v_des

        # Backstepping control law for each channel:
        # u = damping * v + mass * (dv_des - k_gain * e)
        u = self.damping @ v + self.mass @ (dv_des - self.k_gain * e)
        return u