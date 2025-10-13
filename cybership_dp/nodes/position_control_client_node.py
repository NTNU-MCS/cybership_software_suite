#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# This code is part of the MCSimPython toolbox and repository
# Created By: Jan-Erik Hygen
# Created Date: 2023-01-30,
# Revised: 2025-01-31 Kristian Magnus Roen   Now fitting the MC-Gym for csad.
# Revised: 2025-03-26 Emir Cem Gezer Applied to the Cybership Software Suite
# Tested:
# Copyright (C) 2025: NTNU, Trondheim
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import time
import numpy as np
import rclpy

from rclpy.executors import MultiThreadedExecutor
from cybership_tests.go_to_client import NavigateToPoseClient




def main(args=None):
    rclpy.init(args=args)
    client_node = NavigateToPoseClient()
    executor = MultiThreadedExecutor()

    executor.add_node(client_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        client_node.get_logger().info("Keyboard interrupt, shutting down...")

    finally:
        client_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
