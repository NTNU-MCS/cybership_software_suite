import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar


class LOSGuidance:
    def __init__(self, pts, V_d, delta, window_len=None, mu=0.0):
        """
        pts: (N,2) waypoints (NED)
        V_d: desired speed [m/s]
        delta: look-ahead distance [m] (arc-length)
        window_len: local search window [m] around s_prev (default: 4*delta)
        mu: small gradient gain for along-track stabilization [0..0.5], 0 to disable
        """
        pts = np.asarray(pts)
        self.tck, self.u_wp = splprep(
            [pts[:, 0], pts[:, 1]], s=0.1, k=min(3, len(pts)-1)
        )

        # Precompute dense arc-length lookup: u_grid <-> s_grid
        self.u_grid = np.linspace(0.0, 1.0, 2000)
        xy = np.array(splev(self.u_grid, self.tck)).T
        dxy = np.diff(xy, axis=0)
        seg = np.hypot(dxy[:, 0], dxy[:, 1])
        s_grid = np.concatenate(([0.0], np.cumsum(seg)))
        self.s_grid = s_grid
        self.path_len = float(s_grid[-1])

        self.V_d = float(V_d)
        self.delta = float(delta)
        self.window_len = 8.0 * self.delta if window_len is None else float(window_len)

        self.mu = float(mu)  # small gradient to reduce along-track error

        # Persistent arc-length parameter s (initialize lazily on first call)
        self.s = None

    def _u_from_s(self, s):
        s = np.clip(s, 0.0, self.path_len)
        i = np.searchsorted(self.s_grid, s)
        if i == 0:
            return self.u_grid[0]
        if i >= len(self.s_grid):
            return self.u_grid[-1]
        s0, s1 = self.s_grid[i-1], self.s_grid[i]
        u0, u1 = self.u_grid[i-1], self.u_grid[i]
        t = 0.0 if s1 == s0 else (s - s0)/(s1 - s0)
        return u0 + t*(u1 - u0)

    def _pos_from_s(self, s):
        u = self._u_from_s(s)
        x, y = splev(u, self.tck)
        return np.array([x, y])

    def _tangent_from_s(self, s):
        u = self._u_from_s(s)
        dx, dy = splev(u, self.tck, der=1)
        t = np.array([dx, dy])
        nrm = np.linalg.norm(t)
        return t / (nrm + 1e-12), nrm  # unit tangent, |p'(u)| (scaled)

    def _project_local(self, x, y, s_guess, ds_ff):
        """
        Project (x,y) to arc-length s within a local window around s_guess.
        Adds a quadratic bias toward s_guess + ds_ff to prefer forward progress.
        """
        w = self.window_len
        a = max(0.0, s_guess - 0.5*w)
        b = min(self.path_len, s_guess + 0.5*w)
        if b <= a:  # fallback
            a, b = 0.0, self.path_len

        target = np.array([x, y])
        s_target = s_guess + ds_ff

        def cost(s):
            p = self._pos_from_s(s)
            d = p - target
            # distance term + progress bias term (lambda weights)
            return d@d + 1e-3*(s - s_target)**2

        res = minimize_scalar(cost, bounds=(
            a, b), method='bounded', options={'xatol': 1e-6})
        return float(res.x)

    def reset_to_nearest(self, x, y):
        """Call once before the loop to initialize s to the global nearest point."""
        target = np.array([x, y])

        def cost_u(u):
            px, py = splev(u, self.tck)
            d = np.array([px, py]) - target
            return d@d
        res = minimize_scalar(cost_u, bounds=(0.0, 1.0), method='bounded')
        u0 = float(res.x)
        # convert to arc length
        i = np.searchsorted(self.u_grid, u0)
        if i == 0:
            self.s = 0.0
        elif i >= len(self.u_grid):
            self.s = self.path_len
        else:
            u0a, u0b = self.u_grid[i-1], self.u_grid[i]
            s0a, s0b = self.s_grid[i-1], self.s_grid[i]
            t = (u0 - u0a) / (u0b - u0a + 1e-12)
            self.s = s0a + t*(s0b - s0a)

    def guidance(self, x, y, dt=None):
        """
        Compute heading + NED velocity command.
        If dt is given, advance by V_d*dt along the curve; otherwise use delta lookahead.
        Returns (chi_d [rad], vel_cmd [2], extras dict)
        """
        if self.s is None:
            self.reset_to_nearest(x, y)

        # feedforward advance in arc length
        ds_ff = (self.V_d * (dt if dt is not None else 0.0))

        # optional gradient correction to reduce along-track error (small mu)
        # ṡ_c = -mu * t_hat^T * e  where e = p(s) - current position
        # discretize as ds_grad ~ ṡ_c * dt
        ds_grad = 0.0
        if dt is not None and self.mu > 0.0:
            p_s = self._pos_from_s(self.s)
            t_hat, _ = self._tangent_from_s(self.s)
            e = p_s - np.array([x, y])
            ds_grad = - self.mu * float(t_hat.dot(e)) * dt

        # predict prior to projection
        s_pred = np.clip(self.s + ds_ff + ds_grad, 0.0, self.path_len)

        # local projection (prevents jumping at crossings)
        s0 = self._project_local(x, y, s_pred, ds_ff)

        # choose look-ahead by arc length
        s_la = min(s0 + self.delta, self.path_len)
        p_la = self._pos_from_s(s_la)

        # heading toward lookahead
        dx, dy = p_la[0] - x, p_la[1] - y
        chi_d = np.arctan2(dy, dx)

        # velocity command in NED
        Vx = self.V_d * np.cos(chi_d)
        Vy = self.V_d * np.sin(chi_d)

        # commit parameter for next call
        self.s = s0

        return chi_d, np.array([Vx, Vy]), {"s": self.s, "s_la": s_la, "path_len": self.path_len, "lookahead_point": p_la}


if __name__ == "__main__":
    # Example waypoints in NED (north-east)
    waypoints = np.array([
        [0.0, 0.0],
        [10.0, 5.0],
        [20.0, 0.0],
        [30.0, -5.0]
    ])

    # Initialize LOS guidance
    desired_speed = 1.5  # m/s
    lookahead = 5.0      # meters
    los = LOSGuidance(waypoints, V_d=desired_speed, delta=lookahead)

    # Simulate evaluation at several positions
    test_positions = [
        (2.0, 0.5),
        (8.0, 3.0),
        (15.0, -1.0),
        (25.0, -4.0)
    ]

    print("LOS Guidance Commands:")
    for pos in test_positions:
        chi_d, vel_cmd = los.guidance(*pos)
        print(
            f"Position {pos} -> Heading: {np.degrees(chi_d):.1f}°, Vel_CMD: [{vel_cmd[0]:.2f}, {vel_cmd[1]:.2f}]")
