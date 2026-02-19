from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

from geometry_msgs.msg import Wrench


@dataclass(frozen=True)
class ThrusterPublisherSpec:
    key: str
    topic: str
    actuator_index: int
    mapping: Dict[str, int]


class BaseForceControllerROS(LifecycleNode):
    """Lifecycle-enabled base class for thrust allocation.

    Subclasses must implement:
      - `_thruster_publisher_specs()`
      - `_initialize_thrusters()` (sets `self.actuators`)
      - `_initialize_allocator()` (sets `self.allocator`)
    """

    def __init__(
        self,
        *args,
        node_name: str,
        allow_undeclared_parameters: bool = True,
        **kwargs,
    ):
        super().__init__(
            *args,
            node_name=node_name,
            allow_undeclared_parameters=allow_undeclared_parameters,
            **kwargs,
        )

        self.actuators = []
        self.allocator = None

        self.tau_cmd = np.zeros((6, 1), dtype=np.float32)
        self._timer = None
        self._force_sub = None

        self._pub_specs = []
        self._pubs = {}

        self.declare_parameter("frequency", 1.0)
        self.declare_parameter("autostart", True)

        # Optional: publish zero commands when deactivating.
        self.declare_parameter("publish_zero_on_deactivate", True)

        if bool(self.get_parameter("autostart").value):
            self._autostart_lifecycle()

    def _autostart_lifecycle(self) -> None:
        trigger_configure = getattr(self, "trigger_configure", None)
        trigger_activate = getattr(self, "trigger_activate", None)

        if not callable(trigger_configure) or not callable(trigger_activate):
            self.get_logger().warn(
                "Lifecycle autostart requested, but Lifecycle trigger methods are missing; "
                "node will stay unconfigured/inactive."
            )
            return

        try:
            trigger_configure()
            trigger_activate()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Lifecycle autostart failed: {exc}")

    def _thruster_publisher_specs(self) -> list[ThrusterPublisherSpec]:
        raise NotImplementedError

    def _initialize_thrusters(self) -> None:
        raise NotImplementedError

    def _initialize_allocator(self) -> None:
        raise NotImplementedError

    def _get_frequency(self) -> float:
        freq = float(self.get_parameter("frequency").value)
        if freq <= 0.0:
            self.get_logger().warn(
                f"Invalid frequency {freq}; falling back to 1.0 Hz"
            )
            return 1.0
        return freq

    def on_configure(self, state) -> TransitionCallbackReturn:
        _ = state

        self.freq = self._get_frequency()

        if self._force_sub is None:
            self._force_sub = self.create_subscription(
                Wrench, "control/force/command", self.force_callback, 10
            )

        self._initialize_thrusters()
        self._initialize_allocator()

        if self.allocator is None:
            self.get_logger().error("Allocator was not initialized")
            return TransitionCallbackReturn.FAILURE

        # Create lifecycle publishers.
        self._pub_specs = self._thruster_publisher_specs()
        for spec in self._pub_specs:
            if spec.key in self._pubs:
                continue
            self._pubs[spec.key] = self.create_publisher(Wrench, spec.topic, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        _ = state

        if self._timer is None:
            self._timer = self.create_timer(1.0 / float(self.freq), self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        _ = state

        if bool(self.get_parameter("publish_zero_on_deactivate").value):
            self._publish_zero_commands()

        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        _ = state

        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None

        if self._force_sub is not None:
            try:
                self.destroy_subscription(self._force_sub)
            except Exception:
                pass
            self._force_sub = None

        for pub in self._pubs.values():
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        self._pubs.clear()
        self._pub_specs = []

        self.actuators = []
        self.allocator = None

        return TransitionCallbackReturn.SUCCESS

    def force_callback(self, msg: Wrench) -> None:
        self.tau_cmd = np.array(
            [
                msg.force.x,
                msg.force.y,
                msg.force.z,
                msg.torque.x,
                msg.torque.y,
                msg.torque.z,
            ],
            dtype=np.float32,
        ).reshape((6, 1))

    def timer_callback(self) -> None:
        if self.allocator is None or not self.actuators:
            return

        try:
            self.allocator.allocate(tau=self.tau_cmd)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Allocation failed: {exc}")
            return

        self._publish_thruster_commands()

    def _publish_thruster_commands(self) -> None:
        for spec in self._pub_specs:
            pub = self._pubs.get(spec.key)
            if pub is None:
                continue

            msg = Wrench()
            force_vec = self.actuators[spec.actuator_index].force
            self._apply_mapping(msg=msg, vector=force_vec, mapping=spec.mapping)
            pub.publish(msg)

    def _publish_zero_commands(self) -> None:
        for spec in self._pub_specs:
            pub = self._pubs.get(spec.key)
            if pub is None:
                continue
            pub.publish(Wrench())

    @staticmethod
    def _apply_mapping(*, msg: Wrench, vector: np.ndarray, mapping: Dict[str, int]) -> None:
        for field_path, index in mapping.items():
            value = float(vector[index]) if index < len(vector) else 0.0
            root, leaf = field_path.split(".", 1)
            target = getattr(msg, root)
            setattr(target, leaf, value)
