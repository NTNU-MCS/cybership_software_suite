"""ROS2 wrapper node for the LuenbergerObserver.

Subscribes to fixed topics so they can be remapped externally. Runs the
observer in a timer callback, drops stale sensor readings using a configurable
`max_age`, and applies an optional TF frame prefix to published headers.

Publishes estimated `eta` as `PoseStamped`, `nu` as `TwistStamped`, and
`b` as `Vector3Stamped`.
"""
from __future__ import annotations

from typing import Optional, Sequence
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
    WrenchStamped,
    TwistStamped,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry

from .observer import LuenbergerObserver


def _parse_matrix(param: Sequence[float], name: str) -> np.ndarray:
    arr = np.asarray(param, dtype=float)
    if arr.size != 9:
        raise ValueError(f"Parameter {name} must contain 9 elements (3x3)")
    return arr.reshape((3, 3))


class ObserverNode(Node):
    def __init__(self) -> None:
        super().__init__("luenberger_observer")

        # parameters
        self.declare_parameter("tf_prefix", "")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("max_age", 0.2)  # seconds

        # gains: flattened 9-element lists
        self.declare_parameter("L1", [0.0] * 9)
        self.declare_parameter("L2", [0.0] * 9)
        self.declare_parameter("L3", [0.0] * 9)

        self.tf_prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value
        rate = float(self.get_parameter("rate").get_parameter_value().double_value)
        self.max_age = float(self.get_parameter("max_age").get_parameter_value().double_value)

        L1_param = self.get_parameter("L1").get_parameter_value().double_array_value
        L2_param = self.get_parameter("L2").get_parameter_value().double_array_value
        L3_param = self.get_parameter("L3").get_parameter_value().double_array_value

        L1 = _parse_matrix(L1_param, "L1")
        L2 = _parse_matrix(L2_param, "L2")
        L3 = _parse_matrix(L3_param, "L3")

        # instantiate internal observer
        self.obs = LuenbergerObserver()

        # message caches
        self.last_pose: Optional[PoseWithCovarianceStamped] = None
        self.last_pose_time: Optional[float] = None
        self.last_tau: Optional[WrenchStamped] = None
        self.last_tau_time: Optional[float] = None

        # publishers use fixed topic names so they can be remapped externally
        self.pub_odom = self.create_publisher(Odometry, "observation/odom", 10)
        self.pub_eta = self.create_publisher(PoseStamped, "observation/eta", 10)
        self.pub_nu = self.create_publisher(TwistStamped, "observation/nu", 10)
        self.pub_b = self.create_publisher(Vector3Stamped, "observation/b", 10)

        # subscriptions
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._on_pose, 10)
        self.create_subscription(WrenchStamped, "tau", self._on_tau, 10)

        self._L1 = L1
        self._L2 = L2
        self._L3 = L3

        # timer
        self._rate = rate
        self._dt = 1.0 / rate
        self._last_time = time.monotonic()
        self.create_timer(self._dt, self._on_timer)

        self.get_logger().info("Luenberger observer node started")

    def _prefixed_frame(self, frame_id: str) -> str:
        if not self.tf_prefix:
            return frame_id
        return f"{self.tf_prefix}/{frame_id}"

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_pose = msg
        self.last_pose_time = self._now_seconds()

    def _on_tau(self, msg: WrenchStamped) -> None:
        self.last_tau = msg
        self.last_tau_time = self._now_seconds()

    def _now_seconds(self) -> float:
        return time.monotonic()

    def _on_timer(self) -> None:
        now = self._now_seconds()
        dt = now - self._last_time if self._last_time is not None else self._dt
        # clamp dt to reasonable positive value
        if dt <= 0.0:
            dt = self._dt
        self._last_time = now

        # prepare measurement eta
        dead_reckoning = True
        if self.last_pose is not None and self.last_pose_time is not None:
            age = now - self.last_pose_time
            if age <= self.max_age:
                p = self.last_pose.pose.pose
                x = float(p.position.x)
                y = float(p.position.y)
                quaternion = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                _, _, psi = euler_from_quaternion(quaternion)
                eta_meas = np.array([x, y, psi], dtype=float)
                dead_reckoning = False
            else:
                eta_meas = np.zeros(3, dtype=float)
        else:
            eta_meas = np.zeros(3, dtype=float)

        # prepare tau (use zeros if missing or stale)
        if self.last_tau is not None and self.last_tau_time is not None and (now - self.last_tau_time) <= self.max_age:
            w = self.last_tau.wrench
            # map to 3-DOF: [Fx, Fy, Nz]
            tau_meas = np.array([w.force.x, w.force.y, w.torque.z], dtype=float)
        else:
            tau_meas = np.zeros(3, dtype=float)

        # perform observer step
        try:
            eta_hat, nu_hat, b_hat = self.obs.step(eta_meas, tau_meas, self._L1, self._L2, self._L3, dt, dead_reckoning=dead_reckoning)
        except Exception as e:
            self.get_logger().error(f"Observer step failed: {e}")
            return

        # publish eta + nu together as Odometry for RViz
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self._prefixed_frame("map")
        odom_msg.child_frame_id = self._prefixed_frame("base_link")

        odom_msg.pose.pose.position.x = float(eta_hat[0, 0])
        odom_msg.pose.pose.position.y = float(eta_hat[1, 0])
        psi = float(eta_hat[2, 0])
        odom_msg.pose.pose.orientation.z = math.sin(psi / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(psi / 2.0)

        odom_msg.twist.twist.linear.x = float(nu_hat[0, 0])
        odom_msg.twist.twist.linear.y = float(nu_hat[1, 0])
        odom_msg.twist.twist.angular.z = float(nu_hat[2, 0])

        self.pub_odom.publish(odom_msg)

        # publish eta and nu separately for consumers that want the raw signals
        eta_msg = PoseStamped()
        eta_msg.header = odom_msg.header
        eta_msg.pose = odom_msg.pose.pose
        self.pub_eta.publish(eta_msg)

        nu_msg = TwistStamped()
        nu_msg.header = odom_msg.header
        nu_msg.twist = odom_msg.twist.twist
        self.pub_nu.publish(nu_msg)

        # publish b as Vector3Stamped
        b_msg = Vector3Stamped()
        b_msg.header = odom_msg.header
        b_msg.vector.x = float(b_hat[0, 0])
        b_msg.vector.y = float(b_hat[1, 0])
        b_msg.vector.z = float(b_hat[2, 0])
        self.pub_b.publish(b_msg)

    def destroy_node(self) -> None:  # type: ignore[override]
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
