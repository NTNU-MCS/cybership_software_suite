#!/usr/bin/env python3
"""Standalone ROS 2 node that showcases nested parameter handling."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParameterSandbox(Node):
    """Node that declares nested parameters and updates them through callbacks."""

    def __init__(self) -> None:
        super().__init__("parameter_sandbox")

        # Declare nested parameters in one call so they can be overridden via YAML/CLI.
        self.declare_parameters(
            "",
            [
                ("controller.enabled", True),
                ("controller.gains.kp", 1.5)
            ],
        )

        self._parameter_cache = self._snapshot_parameters()

        self.add_on_set_parameters_callback(self._on_parameter_update)
        self.add_post_set_parameters_callback(self._post_parameter_update)

        self._log_timer = self.create_timer(2.0, self._publish_snapshot)


    def _on_parameter_update(self, params: list[Parameter]) -> SetParametersResult:
        """Validate updates before they are committed."""

        return SetParametersResult(successful=True)

    def _post_parameter_update(self, params: list[Parameter]) -> None:
        """Update the cache after parameters have been updated."""

        print(f"Updated parameters: {[param.name for param in params]}")

    def _snapshot_parameters(self) -> dict[str, float | str | bool]:
        """Capture the parameters we care about into a nested dictionary."""

        return {
            "controller": {
                "enabled": self.get_parameter("controller.enabled").get_parameter_value().bool_value,
                "gains": {
                    axis: self.get_parameter(f"controller.gains.{axis}").value
                    for axis in ["kp"]
                },
            },
        }

    def _flatten_cache(self, nested: dict[str, object], prefix: str = "") -> dict[str, object]:
        """Flatten nested dict for compact logging output."""

        flat: dict[str, object] = {}
        for key, value in nested.items():
            qualified = f"{prefix}{key}" if not prefix else f"{prefix}.{key}"
            if isinstance(value, dict):
                flat.update(self._flatten_cache(value, qualified))
            else:
                flat[qualified] = value
        return flat

    def _publish_snapshot(self) -> None:
        """Periodically show the current parameter values."""

        snapshot = self._flatten_cache(self._parameter_cache)
        formatted = ", ".join(f"{key}={value}" for key, value in snapshot.items())
        self.get_logger().info(f"Current parameters: {formatted}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ParameterSandbox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
