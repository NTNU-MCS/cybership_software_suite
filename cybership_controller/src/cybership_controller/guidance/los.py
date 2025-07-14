import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar

class LOSGuidance:
    def __init__(self, pts, V_d, delta):
        """
        Initialize the LOS guidance module.

        :param pts: array-like of shape (N, 2), waypoints in NED coordinates
        :param V_d: desired speed (m/s)
        :param delta: look-ahead distance (m)
        """
        pts = np.asarray(pts)
        # Fit cubic B-spline to the waypoints
        self.tck, _ = splprep([pts[:, 0], pts[:, 1]], s=0, k=3)
        self.V_d = V_d
        self.delta = delta

        # Approximate total path length
        samples = np.array(splev(np.linspace(0, 1, 500), self.tck)).T
        diffs = np.diff(samples, axis=0)
        self.path_len = np.sum(np.hypot(diffs[:, 0], diffs[:, 1]))

    def _project(self, x, y):
        """
        Project the point (x, y) onto the spline to find the parameter s.
        """
        def cost(u):
            px, py = splev(u, self.tck)
            return (px - x)**2 + (py - y)**2

        res = minimize_scalar(cost, bounds=(0, 1), method='bounded')
        return res.x

    def guidance(self, x, y):
        """
        Compute the LOS guidance command.

        :param x: current NED x-position (m)
        :param y: current NED y-position (m)
        :return: (chi_d, vel_cmd) where chi_d is desired heading (rad),
                 and vel_cmd is a 2-element array [Vx, Vy] in NED.
        """
        # 1) Project onto spline
        s0 = self._project(x, y)

        # 2) Compute look-ahead parameter increment
        ds = self.delta / self.path_len
        s_la = min(s0 + ds, 1.0)

        # 3) Evaluate look-ahead point
        x_la, y_la = splev(s_la, self.tck)

        # 4) Compute desired heading
        dx, dy = x_la - x, y_la - y
        chi_d = np.arctan2(dy, dx)

        # 5) Compute velocity command in NED
        Vx = self.V_d * np.cos(chi_d)
        Vy = self.V_d * np.sin(chi_d)

        return chi_d, np.array([Vx, Vy])


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
        print(f"Position {pos} -> Heading: {np.degrees(chi_d):.1f}°, Vel_CMD: [{vel_cmd[0]:.2f}, {vel_cmd[1]:.2f}]")

