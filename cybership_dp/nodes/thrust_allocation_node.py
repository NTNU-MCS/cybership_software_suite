#!/usr/bin/env python3

import cybership_dp.drillship.force_controller
import cybership_dp.enterprise.force_controller
import cybership_dp.voyager.force_controller
import rclpy
import rclpy.executors
import rclpy.node

import geometry_msgs.msg
import rcl_interfaces.msg

import os
import sys
import argparse

import numpy as np
import skadipy

import cybership_utilities.utilities
import cybership_utilities.launch

import cybership_dp.voyager
import cybership_dp.enterprise
import cybership_dp.drillship


class ForceControllerManager():

    def __init__(self):

        self.vessel_model: str = None

        parser = argparse.ArgumentParser()
        parser.add_argument("--vessel-model", type=str, choices=cybership_utilities.utilities.VESSEL_MODELS, required=True)
        self.args, _ = parser.parse_known_args()

    def initialize(self) -> rclpy.node.Node:

        if self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_VOYAGER:
            return cybership_dp.voyager.force_controller.ForceControllerROS()

        elif self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_ENTERPRISE:
            return cybership_dp.enterprise.force_controller.ForceControllerROS()

        elif self.args.vessel_model == cybership_utilities.utilities.VESSEL_MODEL_DRILLSHIP:
            return cybership_dp.drillship.force_controller.ForceControllerROS()

        return None


def main(args=None):
    rclpy.init(args=args)

    manager = ForceControllerManager()

    node = manager.initialize()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
