#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import (
    Wrench,
    TransformStamped,
    WrenchStamped,
    PoseWithCovarianceStamped,
)
from tf2_ros import TransformBroadcaster
from rosgraph_msgs.msg import Clock

import shoeboxpy
import shoeboxpy.model6dof
import skadipy.actuator
import skadipy.allocator

# ----------------------------------------------------------------------------
# Abstract Base Class
# ----------------------------------------------------------------------------

class BaseSimulator(Node, ABC):
    """
    Abstract base class for a vessel simulator.
    Provides common parameter handling, reset, odometry and TF publishing, and simulation iteration.
    Thruster topic setup is delegated to the subclass.
    """

    def __init__(self, node_name="base_simulator"):
        super().__init__(node_name=node_name)

        # Common initial state: pose and velocity
        self.eta0 = np.zeros((6, 1))
        self.nu0 = np.zeros((6, 1))
        # Thruster command vector (u) will be defined by the subclass since its dimension can vary.
        self.u = None

        # Create vessel and allocator (subclasses provide concrete implementations)
        self.vessel = self._create_vessel()
        self.allocator = self._init_allocator()
        self.allocator.compute_configuration_matrix()

        # Declare and read parameters for initial conditions
        self._declare_parameters()
        self._read_parameters()
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Common publishers for odometry and TF
        self.odom = Odometry()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_odom = self.create_publisher(Odometry, "measurement/odom", 1)
        self.publisher_pose = self.create_publisher(PoseWithCovarianceStamped, "measurement/pose", 1)

        # Common reset service
        self.reset_service = self.create_service(Empty, "simulator/reset", self.reset_simulation)

        # Let subclass handle thruster topics and callbacks
        self.setup_thrusters()

        # Simulation time step and timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.iterate)

    # ------------------------------------------------------------------------
    # ABSTRACT METHODS (to be implemented in subclasses)
    # ------------------------------------------------------------------------

    @abstractmethod
    def _create_vessel(self):
        """
        Create and return the vessel (e.g., a Shoebox instance) with desired geometry.
        """
        pass

    @abstractmethod
    def _init_allocator(self):
        """
        Create and return the thruster allocator (e.g., skadipy PseudoInverse) with proper thruster configuration.
        """
        pass

    @abstractmethod
    def setup_thrusters(self):
        """
        Set up thruster-related publishers and subscriptions.
        This method should also initialize self.u with the appropriate dimensions.
        """
        pass

    # ------------------------------------------------------------------------
    # PARAMETER HANDLING (common to all simulators)
    # ------------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameters(
            namespace='initial_conditions.position',
            parameters=[
                ('x', 0.0),
                ('y', 0.0),
                ('yaw', 0.0),
            ]
        )
        self.declare_parameters(
            namespace='initial_conditions.velocity',
            parameters=[
                ('surge', 0.0),
                ('sway', 0.0),
                ('yaw_rate', 0.0),
            ]
        )

    def _read_parameters(self):
        self.eta0[0] = self.get_parameter('initial_conditions.position.x').value
        self.eta0[1] = self.get_parameter('initial_conditions.position.y').value
        self.eta0[5] = self.get_parameter('initial_conditions.position.yaw').value

        self.nu0[0] = self.get_parameter('initial_conditions.velocity.surge').value
        self.nu0[1] = self.get_parameter('initial_conditions.velocity.sway').value
        self.nu0[5] = self.get_parameter('initial_conditions.velocity.yaw_rate').value

    def _on_parameter_change(self, params):
        successful = True
        new_eta0 = self.eta0.copy()
        new_nu0 = self.nu0.copy()

        for param in params:
            name = param.name
            value = param.value
            if name == 'initial_conditions.position.x':
                new_eta0[0] = float(value)
            elif name == 'initial_conditions.position.y':
                new_eta0[1] = float(value)
            elif name == 'initial_conditions.position.yaw':
                new_eta0[5] = float(value)
            elif name == 'initial_conditions.velocity.surge':
                new_nu0[0] = float(value)
            elif name == 'initial_conditions.velocity.sway':
                new_nu0[1] = float(value)
            elif name == 'initial_conditions.velocity.yaw_rate':
                new_nu0[5] = float(value)

        if successful:
            self.eta0 = new_eta0
            self.nu0 = new_nu0

        return SetParametersResult(successful=successful)

    # ------------------------------------------------------------------------
    # SIMULATION ITERATION & RESET (common)
    # ------------------------------------------------------------------------

    def reset_simulation(self, request, response):
        self.eta0 = np.zeros((6, 1))
        self.nu0 = np.zeros((6, 1))
        if self.u is not None:
            self.u = np.zeros_like(self.u)
        self.vessel.eta = self.eta0
        self.vessel.nu = self.nu0
        self._read_parameters()
        return response

    def iterate(self):
        """
        Timer callback: compute thruster force, step the vessel, and publish odometry/TF.
        Assumes that self.u has been initialized by the subclass.
        """
        tau = (self.allocator._b_matrix @ self.u).flatten()
        self.vessel.step(tau=tau, dt=self.dt)
        self.publish_odom()
        self.publish_tf()

    def publish_odom(self):
        eta, nu = self.vessel.get_states()
        rot = R.from_euler('xyz', eta[3:6], degrees=False)
        quat = rot.as_quat()  # [x, y, z, w]

        self.odom.header.frame_id = "world"
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.child_frame_id = "base_link"

        self.odom.pose.pose.position.x = eta[0].item()
        self.odom.pose.pose.position.y = eta[1].item()
        self.odom.pose.pose.position.z = eta[2].item()
        self.odom.pose.pose.orientation.x = quat[0].item()
        self.odom.pose.pose.orientation.y = quat[1].item()
        self.odom.pose.pose.orientation.z = quat[2].item()
        self.odom.pose.pose.orientation.w = quat[3].item()

        self.odom.twist.twist.linear.x = nu[0].item()
        self.odom.twist.twist.linear.y = nu[1].item()
        self.odom.twist.twist.linear.z = nu[2].item()
        self.odom.twist.twist.angular.x = nu[3].item()
        self.odom.twist.twist.angular.y = nu[4].item()
        self.odom.twist.twist.angular.z = nu[5].item()

        self.publisher_odom.publish(self.odom)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = self.odom.header
        pose_msg.pose = self.odom.pose
        self.publisher_pose.publish(pose_msg)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.odom.pose.pose.position.x
        t.transform.translation.y = self.odom.pose.pose.position.y
        t.transform.translation.z = self.odom.pose.pose.position.z
        t.transform.rotation.x = self.odom.pose.pose.orientation.x
        t.transform.rotation.y = self.odom.pose.pose.orientation.y
        t.transform.rotation.z = self.odom.pose.pose.orientation.z
        t.transform.rotation.w = self.odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

