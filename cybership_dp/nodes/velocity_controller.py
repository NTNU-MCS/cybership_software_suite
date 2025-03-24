#!/usr/bin/env python3

import cybership_dp.voyager.force_controller
import cybership_dp.voyager.velocity_controller
import rclpy
import rclpy.executors
import rclpy.node

import geometry_msgs.msg
import rcl_interfaces.msg

import os
import sys
import argparse

import numpy as np

import cybership_utilities.utilities
import cybership_utilities.launch

import cybership_dp.voyager
import cybership_dp.enterprise


class VelocityControllerManager():

    def __init__(self):

        self.vessel_model: str = None

        parser = argparse.ArgumentParser()
        parser.add_argument("--vessel-model", type=str, choices=cybership_utilities.utilities.VESSEL_MODELS, required=True)
        self.args, _ = parser.parse_known_args()

    def initialize(self) -> rclpy.node.Node:

        if self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_VOYAGER:
            return cybership_dp.voyager.velocity_controller.BacksteppingVelocityControllerROS()

        elif self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_ENTERPRISE:
            print("Force controller for C/S Enterprise is not implemented yet.")

        elif self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_DRILLSHIP:
            print("Force controller for C/S Enterprise is not implemented yet.")

            return None


def main(args=None):
    rclpy.init(args=args)

    manager = VelocityControllerManager()

    node = manager.initialize()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
