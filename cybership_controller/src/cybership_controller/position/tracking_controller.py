r"""Simplified body-frame 3-DOF PID tracking controller with integral action.

Body-frame PID-with-feedforward law for a 3 degree-of-freedom surface vessel
(the Cybership in the MC-Lab). Given the NED reference trajectory from guidance,
it returns the generalized control force :math:`\tau`.

**Frames and states**

* Pose in the Earth-fixed (NED) frame, :math:`\eta = [x,\, y,\, \psi]^{\top}`.
* Body-fixed velocity, :math:`\nu = [u,\, v,\, r]^{\top}`.
* Rotation :math:`R(\psi)` (the model's ``J``), orthonormal so
  :math:`R^{-1} = R^{\top}`, with kinematics :math:`\dot{\eta} = R(\psi)\,\nu`.
"""

import time
import numpy as np


class DPTrackingController:
    r"""Body-frame PID tracking law with reference feedforward and anti-windup.

    .. math::

        \begin{aligned}
        \nu_d        &= R(\psi)^{\top}\,\dot{\eta}_d \\
        \dot{\nu}_d  &= \tfrac{d}{dt}\!\left(R(\psi)^{\top}\right)\dot{\eta}_d
                        + R(\psi)^{\top}\,\ddot{\eta}_d \\
        e_1          &= R(\psi)^{\top}(\eta - \eta_d) \quad(\text{heading wrapped}),
                        \qquad e_2 = \nu - \nu_d \\
        \dot{\xi}    &= e_1 \\
        P &= -K_p\,e_1, \quad I = w\,(-K_i\,\xi), \quad D = -K_d\,e_2 \\
        \tau &= D_{mat}\,\nu + \underbrace{(P + I + D)}_{\text{PID}} + M\,\dot{\nu}_d
        \end{aligned}

    Here :math:`\xi` is the integral of the body-frame pose error,
    :math:`w \in \{0, 1\}` is the per-DOF :attr:`windup_flag`, and
    :math:`D_{mat}` is the damping matrix (distinct from the derivative term
    :math:`D`). The terms :math:`P, I, D` are stored each call for logging (see
    :attr:`term_P`, :attr:`term_I`, :attr:`term_D`).

    :param model: Vessel model exposing ``M_eff``, ``D`` and ``J(eta)``.
    :param Kp: Proportional gain, 1-D vector (diagonal) or 3x3 matrix.
    :param Kd: Derivative gain, 1-D vector or 3x3 matrix.
    :param Ki: Integral gain, 1-D vector or 3x3 matrix. Defaults to zero.
    :param u_max: Upper rail on the PID effort :math:`P + I + D` used by the
        anti-windup logic. ``None`` disables saturation and anti-windup.
    :param u_min: Lower rail on the PID effort. Defaults to ``-u_max``.
    :param i_max: Optional hard clamp on the integral state :math:`\xi`.
    """

    def __init__(self, model, Kp, Kd, Ki=None,
                 u_max=None, u_min=None, i_max=None):
        self.model = model
        self.Kp = self._as_matrix(Kp)
        self.Kd = self._as_matrix(Kd)
        self.Ki = self._as_matrix(np.zeros(3) if Ki is None else Ki)

        # PID-effort saturation rails used for anti-windup; None -> no limits.
        self.u_max = None if u_max is None else np.asarray(u_max, float)
        if self.u_max is None:
            self.u_min = None
        else:
            self.u_min = (-self.u_max if u_min is None
                          else np.asarray(u_min, float))

        # Optional hard clamp on the integral state (backstop). None -> off.
        self.i_max = None if i_max is None else np.asarray(i_max, float)

        # ---- incoming ROS signals (subscribers write here) ----
        self.eta_signal        = np.zeros(3)     # measured pose      [x, y, psi]
        self.nu_signal         = np.zeros(3)     # measured velocity  [u, v, r]
        self.eta_d_signal      = np.zeros(3)     # reference pose
        self.eta_d_dot_signal  = np.zeros(3)     # reference velocity
        self.eta_d_ddot_signal = np.zeros(3)     # reference acceleration

        # ---- integrator state / telemetry ----
        self.xi = np.zeros(3)                    # integral of body-frame error e1
        self.windup_flag = np.ones(3)            # per DOF: 1 = integrating, 0 = frozen
        self._t_prev = None                      # for internal dt fallback

        # ---- logged PID terms (refreshed every compute_tau call) ----
        self.term_P   = np.zeros(3)
        self.term_I   = np.zeros(3)
        self.term_D   = np.zeros(3)
        self.term_PID = np.zeros(3)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _as_matrix(K):
        r"""Promote a 1-D gain vector to a diagonal matrix; pass 3x3 through.

        :param K: Gain given as a length-3 vector or a 3x3 matrix.
        :returns: A 3x3 ``numpy`` array.
        """
        K = np.asarray(K, dtype=float)
        return np.diag(K) if K.ndim == 1 else K

    @staticmethod
    def Jdot(psi, r):
        r"""Time derivative of the rotation matrix :math:`\dot{R}(\psi, r)`.

        With :math:`\dot{\psi} = r`,

        .. math::

            \dot{R}(\psi, r) = r
            \begin{bmatrix}
            -\sin\psi & -\cos\psi & 0 \\
             \cos\psi & -\sin\psi & 0 \\
             0        &  0        & 0
            \end{bmatrix}.

        Its transpose gives :math:`\tfrac{d}{dt}(R^{\top})`, used in
        :math:`\dot{\nu}_d`.

        :param psi: Heading :math:`\psi` in radians.
        :param r: Yaw rate :math:`r` in rad/s.
        :returns: The 3x3 matrix :math:`\dot{R}`.
        """
        c, s = np.cos(psi), np.sin(psi)
        return r * np.array([[-s, -c, 0.0], [c, -s, 0.0], [0.0, 0.0, 0.0]])

    @staticmethod
    def wrap(a):
        r"""Wrap an angle to :math:`(-\pi, \pi]` via
        :math:`\mathrm{atan2}(\sin a, \cos a)`.

        :param a: Angle in radians.
        :returns: Equivalent angle in :math:`(-\pi, \pi]`.
        """
        return np.arctan2(np.sin(a), np.cos(a))

    def reset(self):
        r"""Clear the integrator and timing.

        Call when the **SetDP** push-button is pressed so each DP run starts from
        :math:`\xi = 0`.
        """
        self.xi = np.zeros(3)
        self.windup_flag = np.ones(3)
        self._t_prev = None

    def _dt(self, dt):
        r"""Resolve the integration step :math:`\Delta t`.

        :param dt: Period in seconds, or ``None`` to use a monotonic wall clock.
        :returns: Step :math:`\Delta t` in seconds (``0.0`` on the first
            wall-clock call).
        """
        if dt is not None:
            return float(dt)
        now = time.monotonic()
        if self._t_prev is None:
            self._t_prev = now
            return 0.0
        dt = now - self._t_prev
        self._t_prev = now
        return dt

    # ------------------------------------------------------------------- output
    def compute_tau(self, dt=None):
        r"""Evaluate the PID tracking law and return the generalized force.

        **Anti-windup (per-DOF conditional integration).** The integral term
        enters the PID effort :math:`u = P + I + D` directly, so one integration
        step (:math:`\xi \mathrel{+}= e_1\,\Delta t`) moves that effort by

        .. math::

            d_u = -K_i\,e_1 .

        With the rail violation :math:`\mathrm{excess} = u - \mathrm{sat}(u)`
        (nonzero only where saturated), DOF :math:`i` is frozen
        (:math:`w_i = 0`, no accumulation) iff

        .. math::

            \mathrm{excess}_i \cdot d_{u,i} > 0,

        i.e. iff integrating would push that saturated component further past its
        rail; otherwise it integrates. Because the law uses :math:`I = -K_i\,\xi`
        (negative gain), this is the sign-correct form of the advisor's
        "integrate only if it does not worsen saturation"; the textbook
        ``e1 < 0`` / ``e1 > 0`` tests apply to the ``I = +Ki*xi`` convention and
        invert here.

        :param dt: Control-timer period :math:`\Delta t` in seconds. If ``None``,
            a wall-clock difference between calls is used.
        :returns: Generalized force :math:`\tau` of shape (3,) = ``[Fx, Fy, Mz]``.
        """
        dt = self._dt(dt)

        eta = np.asarray(self.eta_signal, float)
        nu  = np.asarray(self.nu_signal,  float)
        eta_d      = np.asarray(self.eta_d_signal,      float)
        eta_d_dot  = np.asarray(self.eta_d_dot_signal,  float)
        eta_d_ddot = np.asarray(self.eta_d_ddot_signal, float)

        M    = self.model.M_eff
        Dmat = self.model.D                              # damping matrix
        R    = self.model.J(eta)                         # rotation R(psi)
        Rt   = R.T                                        # R(psi)'
        Rt_dot = self.Jdot(eta[2], nu[2]).T              # d/dt( R(psi)' )

        # ---- body-velocity reference and its derivative ----
        nu_d     = Rt @ eta_d_dot
        nu_d_dot = Rt_dot @ eta_d_dot + Rt @ eta_d_ddot

        # ---- tracking errors (body frame) ----
        eta_err = eta - eta_d
        eta_err[2] = self.wrap(eta_err[2])               # wrap heading error
        e1 = Rt @ eta_err                                # body-frame pose error
        e2 = nu - nu_d                                   # body velocity error

        # ---- PID terms ----
        P = -self.Kp @ e1
        Dterm = -self.Kd @ e2
        I_full = -self.Ki @ self.xi                      # ungated integral term

        # ---- per-DOF conditional-integration anti-windup ----
        # Test with the full integral so the decision is stable across steps.
        if self.u_max is None:
            self.windup_flag = np.ones(3)
        else:
            u_test = P + I_full + Dterm
            excess = u_test - np.clip(u_test, self.u_min, self.u_max)
            d_u    = -self.Ki @ e1                       # effort move per integration
            self.windup_flag = np.where(excess * d_u > 0.0, 0.0, 1.0)

        # ---- gated integral term, PID, and force ----
        I = self.windup_flag * I_full                    # I = windup_flag * (-Ki*xi)
        PID = P + I + Dterm
        tau = Dmat @ nu + PID + M @ nu_d_dot

        # ---- advance integrator on active DOFs only ----
        if dt > 0.0:
            self.xi = self.xi + dt * (self.windup_flag * e1)
            if self.i_max is not None:                   # optional backstop clamp
                self.xi = np.clip(self.xi, -self.i_max, self.i_max)

        # ---- store terms for logging ----
        self.term_P, self.term_I, self.term_D, self.term_PID = P, I, Dterm, PID
        return tau