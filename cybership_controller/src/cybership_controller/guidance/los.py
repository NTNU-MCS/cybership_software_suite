import numpy as np
from scipy.interpolate import splprep, splev

class LOSGuidance:
    def __init__(self, pts, V_d, delta):
        """
        pts: (N,2) waypoints (NED)
        V_d: desired speed [m/s]
        delta: look-ahead distance [m] (arc-length)
        """
        pts = np.asarray(pts)
        self.tck, self.u_wp = splprep(
            [pts[:, 0], pts[:, 1]], s=0.1, k=min(3, len(pts)-1)
        )

        # Precompute dense arc-length lookup: u_grid <-> s_grid
        self.u_grid = np.linspace(0.0, 1.0, 2000)
        self.xy = np.array(splev(self.u_grid, self.tck)).T
        dxy = np.diff(self.xy, axis=0)
        seg = np.hypot(dxy[:, 0], dxy[:, 1])
        s_grid = np.concatenate(([0.0], np.cumsum(seg)))
        self.s_grid = s_grid
        self.path_len = float(s_grid[-1])

        self.V_d = float(V_d)
        self.delta = float(delta)

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

    def guidance(self, x, y, dt=None):
        """
        Compute heading + NED velocity command using purely geometric LOS.
        Returns (chi_d [rad], vel_cmd [2], extras dict)
        """
        # 1. Find the true closest point geometrically using the dense grid
        # Vectorized distance squared (faster than minimizing a scalar function)
        dists_sq = (self.xy[:, 0] - x)**2 + (self.xy[:, 1] - y)**2
        idx_closest = np.argmin(dists_sq)
        s_closest = self.s_grid[idx_closest]

        # 2. Advance by the lookahead distance Delta
        s_la = min(s_closest + self.delta, self.path_len)
        p_la = self._pos_from_s(s_la)

        # 3. Calculate heading toward the lookahead point
        dx = p_la[0] - x
        dy = p_la[1] - y
        chi_d = np.arctan2(dy, dx)

        # 4. Velocity command in NED
        Vx = self.V_d * np.cos(chi_d)
        Vy = self.V_d * np.sin(chi_d)

        return chi_d, np.array([Vx, Vy]), {"s_closest": s_closest, "s_la": s_la, "lookahead_point": p_la}


if __name__ == "__main__":
    # Example waypoints in NED (north-east)
    waypoints = np.array([
        [0.0, 0.0],
        [10.0, 5.0],
        [20.0, 0.0],
        [30.0, -5.0]
    ])

    desired_speed = 1.5  # m/s
    lookahead = 5.0      # meters
    los = LOSGuidance(waypoints, V_d=desired_speed, delta=lookahead)

    test_positions = [
        (2.0, 0.5),
        (8.0, 3.0),
        (15.0, -1.0),
        (25.0, -4.0)
    ]

    print("LOS Guidance Commands:")
    for pos in test_positions:
        # Note: Unpacking 3 variables here to match the returned tuple
        chi_d, vel_cmd, extras = los.guidance(*pos)
        print(f"Position {pos} -> Heading: {np.degrees(chi_d):.1f}°, Vel_CMD: [{vel_cmd[0]:.2f}, {vel_cmd[1]:.2f}]")