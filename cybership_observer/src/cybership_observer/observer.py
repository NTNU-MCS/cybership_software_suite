from typing import Optional, Tuple

import numpy as np
from numpy.typing import ArrayLike, NDArray

# Default parameters kept at module level for clarity and reuse
DEFAULT_M = np.array(
    [[9.9, 0.0, 0.0], [0.0, 10.8, 0.23], [0.0, 0.23, 0.89]], dtype=float
)
DEFAULT_D = np.array(
    [[1.35, 0.0, 0.0], [0.0, 14.25, 2.13], [0.0, 2.13, 1.07]], dtype=float
)
DEFAULT_TB = 142.0 * np.eye(3)


class LuenbergerObserver:
    """Luenberger-style observer for 3-DOF planar vessel states.

    Estimates pose `eta_hat` (3x1), body velocities `nu_hat` (3x1),
    and an additive bias `b_hat` (3x1). Mass (`M`), damping (`D`) and
    bias time-constant matrix (`Tb`) are configurable at construction.
    """

    def __init__(
        self,
        M: Optional[NDArray] = None,
        D: Optional[NDArray] = None,
        Tb: Optional[NDArray] = None,
        eta_hat: Optional[ArrayLike] = None,
        nu_hat: Optional[ArrayLike] = None,
        b_hat: Optional[ArrayLike] = None,
    ) -> None:
        # Assign mass matrix
        if M is None:
            self.M = DEFAULT_M.copy()
        else:
            self.M = np.asarray(M, dtype=float)

        # Assign damping matrix
        if D is None:
            self.D = DEFAULT_D.copy()
        else:
            self.D = np.asarray(D, dtype=float)

        # Validate shapes early
        self._validate_3x3("M", self.M)
        self._validate_3x3("D", self.D)

        # Precompute inverse of M
        self.M_inv = np.linalg.inv(self.M)

        # Assign bias time-constant matrix
        if Tb is None:
            self.Tb = DEFAULT_TB.copy()
        else:
            self.Tb = np.asarray(Tb, dtype=float)

        self._validate_3x3("Tb", self.Tb)
        self.Tb_inv = np.linalg.inv(self.Tb)

        # State estimates as (3,1) column vectors
        self.eta_hat: NDArray = self._col(eta_hat)
        self.nu_hat: NDArray = self._col(nu_hat)
        self.b_hat: NDArray = self._col(b_hat)

        # expose the attributes
        self.eta_hat = self.eta_hat
        self.nu_hat = self.nu_hat
        self.b_hat = self.b_hat

    @staticmethod
    def _Rz(psi: float) -> NDArray:
        c, s = np.cos(psi), np.sin(psi)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return (a + np.pi) % (2 * np.pi) - np.pi

    def _col(self, x: Optional[ArrayLike]) -> NDArray:
        """Return a (3,1) column vector from input or zeros if None.

        Ensures callers always get a consistent column-shaped array.
        """
        if x is None:
            return np.zeros((3, 1), dtype=float)
        arr = np.asarray(x, dtype=float)
        arr = arr.reshape(3, -1)
        if arr.shape[1] != 1:
            arr = arr.reshape(3, 1)
        return arr

    def _validate_3x3(self, name: str, mat: NDArray) -> None:
        if mat.shape != (3, 3):
            raise ValueError(f"{name} must be a (3,3) matrix")

    def step(
        self,
        eta_meas: ArrayLike,
        tau_meas: ArrayLike,
        L1: NDArray,
        L2: NDArray,
        L3: NDArray,
        dt: float,
        dead_reckoning: bool = False,
    ) -> Tuple[NDArray, NDArray, NDArray]:
        """Perform one observer update step.

        Args:
            eta_meas: measured pose-like vector (3,) or (3,1): [x, y, psi]
            tau_meas: measured input/moment vector (3,) or (3,1)
            L1, L2, L3: observer gain matrices (3x3)
            dt: timestep in seconds
            dead_reckoning: if True, disable measurement correction

        Returns:
            (eta_hat, nu_hat, b_hat) each as (3,1) numpy arrays.
        """

        eta = np.asarray(eta_meas, dtype=float).reshape(3, 1)
        tau = np.asarray(tau_meas, dtype=float).reshape(3, 1)

        # normalize yaw and compute rotation
        raw_psi = float(eta[2, 0])
        psi = self._wrap_pi(raw_psi)
        R = self._Rz(psi)

        # measurement residual (with yaw wrapping)
        y_tilde = eta - self.eta_hat
        y_tilde[2, 0] = self._wrap_pi(float(y_tilde[2, 0]))

        if dead_reckoning:
            y_tilde[:] = 0.0

        # Eq. (11)
        eta_dot = R @ self.nu_hat + L1 @ y_tilde
        nu_dot = self.M_inv @ (-self.D @ self.nu_hat + self.b_hat + tau + L2 @ (R.T @ y_tilde))
        b_dot = -self.Tb_inv @ self.b_hat + L3 @ (R.T @ y_tilde)

        # Euler integration
        self.eta_hat = self.eta_hat + dt * eta_dot
        self.nu_hat = self.nu_hat + dt * nu_dot
        self.b_hat = self.b_hat + dt * b_dot

        self.eta_hat[2, 0] = self._wrap_pi(float(self.eta_hat[2, 0]))

        return self.eta_hat, self.nu_hat, self.b_hat