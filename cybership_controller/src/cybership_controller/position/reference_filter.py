import numpy as np


def wrap_to_pi(angle):
    r"""Wrap an angle to [-pi, pi].

    .. math::
       \mathrm{result} = (\,\mathtt{angle} + \pi\,) \bmod (2\pi) - \pi

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Angle wrapped to the interval [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def Rz(psi):
    r"""Compute a 3DOF rotation matrix about the z-axis.

    .. math::
       R_z(\psi)= \begin{pmatrix}
           \cos(\psi) & -\sin(\psi) & 0\\[1mm]
           \sin(\psi) & \cos(\psi) & 0\\[1mm]
           0 & 0 & 1
       \end{pmatrix}

    Args:
        psi (float): Yaw angle in radians.

    Returns:
        numpy.ndarray: A 3x3 rotation matrix.
    """
    return np.array(
        [[np.cos(psi), -np.sin(psi), 0],
         [np.sin(psi), np.cos(psi), 0],
         [0, 0, 1]]
    )


class ThirdOrderReferenceFilter:
    """Third-order reference filter for guidance."""

    def __init__(self, dt, omega=[0.2, 0.2, 0.2], delta=[1.0, 1.0, 1.0], initial_eta=None):
        """Initialize the third-order reference filter.

        Args:
            dt (float): Time step size.
            omega (list of float, optional): Frequency parameters. Defaults to [0.2, 0.2, 0.2].
            delta (list of float, optional): Damping parameters. Defaults to [1.0, 1.0, 1.0].
            initial_eta (list of float, optional): Initial state [x, y, yaw]. Defaults to None.
        """
        self._dt = dt
        # Keep everything as a float array
        self.eta_d = (
            np.zeros(3) if initial_eta is None else np.array(
                initial_eta, dtype=float)
        )  # [ x, y, yaw ]
        self.eta_d_dot = np.zeros(3)
        self.eta_d_ddot = np.zeros(3)
        self._eta_r = self.eta_d.copy()  # reference target (unwrapped)

        # State vector of 9 elements: [ eta, eta_dot, eta_ddot ]
        self._x = np.concatenate([self.eta_d, self.eta_d_dot, self.eta_d_ddot])

        # Gains
        self._delta = np.eye(3)
        self._w = np.diag(omega)
        O3x3 = np.zeros((3, 3))
        self.Ad = np.block(
            [
                [O3x3, np.eye(3), O3x3],
                [O3x3, O3x3, np.eye(3)],
                [
                    -self._w**3,
                    -(2 * self._delta + np.eye(3)) @ self._w**2,
                    -(2 * self._delta + np.eye(3)) @ self._w,
                ],
            ]
        )
        self.Bd = np.block([[O3x3], [O3x3], [self._w**3]])

    def get_eta_d(self):
        r"""Retrieve the desired pose.

        Returns:
            numpy.ndarray: Desired pose as [x, y, yaw].
        """
        return self.eta_d

    def get_eta_d_dot(self):
        r"""Retrieve the desired velocity in the inertial frame.

        Returns:
            numpy.ndarray: Desired velocity.
        """
        return self.eta_d_dot

    def get_eta_d_ddot(self):
        r"""Retrieve the desired acceleration in the inertial frame.

        Returns:
            numpy.ndarray: Desired acceleration.
        """
        return self.eta_d_ddot

    def get_nu_d(self):
        r"""Retrieve the desired velocity in the body frame.

        Returns:
            numpy.ndarray: Body frame velocity (u, v, r).
        """
        psi = self.eta_d[2]
        return Rz(psi).T @ self.eta_d_dot

    def set_eta_r(self, eta_r):
        r"""Set the reference pose with smooth yaw transition.

        Args:
            eta_r (list or numpy.ndarray): Reference pose [x, y, yaw].
        """
        old_yaw = self._eta_r[2]
        new_yaw = eta_r[2]

        # Wrap new yaw to [-pi, pi]
        new_yaw = wrap_to_pi(new_yaw)

        # Get minimal difference
        diff = wrap_to_pi(new_yaw - old_yaw)
        continuous_yaw = old_yaw + diff  # small step only

        self._eta_r = np.array([eta_r[0], eta_r[1], continuous_yaw])

    def update(self):
        r"""Integrate the filter for one time step.

        The state is updated as follows:

        .. math::
           \dot{x} = A_d x + B_d \eta_r

        .. math::
           x_{\mathrm{new}} = x + dt \cdot \dot{x}

        Updates the internal state based on the reference pose.
        """
        x_dot = self.Ad @ self._x + self.Bd @ self._eta_r
        self._x += self._dt * x_dot

        # Extract the updated states
        self.eta_d = self._x[:3]
        self.eta_d_dot = self._x[3:6]
        self.eta_d_ddot = self._x[6:]


def saturate(x, z):
    r"""Apply a smooth saturation to the input.

    .. math::
       \mathrm{saturated\_value} = \frac{x}{\lvert x \rvert + z}

    Args:
        x (numpy.ndarray or float): Input value.
        z (numpy.ndarray or float): Saturation parameter.

    Returns:
        numpy.ndarray or float: Saturated value between -1 and 1.
    """
    return x / (np.abs(x) + z)
