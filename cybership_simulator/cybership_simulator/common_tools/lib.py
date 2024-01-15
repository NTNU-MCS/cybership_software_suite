#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no
# Year: 2022
# Copyright (C) 2023 NTNU Marine Cybernetics Laboratory

import rospy
import numpy as np
import dynamic_reconfigure.client
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
from cse_messages.msg import observer_message, reference_message, s_message
from common_tools.math_tools import quat2eul, rad2pipi

class Qualisys():
    """
    Retrieves qualisys measurements by listening to the /qualisys/CSE1/odom topic.
    It converts the quaternions to euler angles and publishes a 1x3 measurement vector
    to the topic /CSS/eta
    """
    def __init__(self):
        self.odom = Odometry()
        self.eta = np.zeros(3)
        self.message = Float64MultiArray()
        self.pub = rospy.Publisher('/CSE1/eta', Float64MultiArray, queue_size=1)

    def callback(self, data):
        self.odom = data
        w = self.odom.pose.pose.orientation.w
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z

        self.eta[0] = self.odom.pose.pose.position.x
        self.eta[1] = self.odom.pose.pose.position.y
        self.eta[2] = quat2eul(x, y, w, z)[2]
        self.eta[2] = rad2pipi(self.eta[2])
        self.message.data = self.eta
        self.pub.publish(self.message)

    def get_data(self):
        return self.eta

# class DS4_Controller():
#     """
#     The controller listens to the /joy topic and maps all input signals from the DS4 to a variable that can be called
#     """
#     def __init__(self):
#         self.x = self.square = self.circle = self.triangle = self.rightArrow = self.leftArrow = self.upArrow = self.DownArrow = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.share = self.options = self.PS = self.pad = 0
#         self.lStickX = self.lStickY = self.rStickX = self.rStickY = self.L2A = self.R2A = 0.0

#     def updateState(self, data):
#         self.x = data.buttons[3]
#         self.square = data.buttons[0]
#         self.circle = data.buttons[2]
#         self.triangle = data.buttons[1]
#         self.rightArrow = data.buttons[16]
#         self.leftArrow = data.buttons[14]
#         self.upArrow = data.buttons[15]
#         self.DownArrow = data.buttons[17]
#         self.L1 = data.buttons[4]
#         self.R1 = data.buttons[6]
#         self.L2 = data.buttons[5]
#         self.R2 = data.buttons[7]
#         self.L3 = data.buttons[12]
#         self.R3 = data.buttons[13]
#         self.options = data.buttons[9]
#         self.share = data.buttons[8]
#         self.PS = data.buttons[10]
#         self.pad = data.buttons[11]

#         self.lStickX = -data.axes[0]
#         self.lStickY = data.axes[1]
#         self.rStickX = -data.axes[2]
#         self.rStickY = data.axes[3]
#         self.L2A = data.axes[4]
#         self.R2A = data.axes[5]


class Forces():
    def __init__(self):
        self.msg = Float64MultiArray()
        self.pub = rospy.Publisher('/CSEI/tau', Float64MultiArray, queue_size=1)
        self.tau = np.zeros(3)

    def get_data(self):
        return self.tau

    def callback(self, msg):
        self.tau = np.array(msg.data)

    def publish(self, tau):
        self.msg.data = tau
        self.pub.publish(self.msg)

class UVector():
    """
    The UVector initializing and publishing the computed actuator commands
    """
    def __init__(self):
        #self.leftRotorThrust = 0.0
        #self.rightRotorThrust = 0.0
        #self.bowRotorThrust = 0.0
        #self.leftRotorAngle = 0.0
        #self.rightRotorAngle = 0.0
        self.Udata = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub = rospy.Publisher('CSEI/u', Float64MultiArray, queue_size = 1)
        self.message = Float64MultiArray()

    def publish(self, data):
        self.message.data = data
        self.pub.publish(self.message)

    def callback(self, udata):
        self.Udata = udata



class Observer_Converser():
    def __init__(self):
        self.observer_msg = observer_message()
        self.pub = rospy.Publisher('CSEI/observer', observer_message, queue_size=1)
        self.eta_hat = np.array([0, 0, 0])
        self.nu_hat = np.array([0, 0, 0])
        self.bias_hat = np.array([0, 0 ,0])

    def callback(self, msg):
        self.eta_hat = msg.eta
        self.nu_hat = msg.nu
        self.bias_hat = msg.bias

    # Publishes the to the CSEI/observer topic
    def publish(self, eta_hat, nu_hat, bias_hat):
        self.observer_msg.eta = eta_hat
        self.observer_msg.nu = nu_hat
        self.observer_msg.bias = bias_hat
        self.pub.publish(self.observer_msg)

    def get_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat

class Reference_Converser():
    # Initialize the guidance parameters in [0; 0; 0]
    def __init__(self):
        self.ref_msg = reference_message()
        self.pub = rospy.Publisher('/CSEI/reference', reference_message, queue_size=1)
        self.eta_d = np.array([0, 0, 0])
        self.eta_ds = np.array([0, 0, 0])
        self.eta_ds2 = np.array([0, 0, 0])
        self.w = 0.0
        self.v_s = 0.0
        self.v_ss = 0.0
    # Callback function is called when the topic is updated

    def callback(self, msg):
        self.eta_d = np.array(msg.eta_d)
        self.eta_ds = np.array(msg.eta_ds)
        self.eta_ds2 = np.array(msg.eta_ds2)
        self.w = msg.w
        self.v_s = msg.v_s
        self.v_ss = msg.v_ss

    # Publishes new gains to the reference topic. These should be numpy arrays with n=3
    def publish(self, eta_d, eta_ds, eta_ds2, w, v_s, v_ss):
        self.ref_msg.eta_d = eta_d
        self.ref_msg.eta_ds = eta_ds
        self.ref_msg.eta_ds2 = eta_ds2
        self.ref_msg.w = w
        self.ref_msg.v_s = v_s
        self.ref_msg.v_ss = v_ss
        self.pub.publish(self.ref_msg)

    # Retrieve the references from the object
    def get_ref(self):
        return self.eta_d, self.eta_ds, self.eta_ds2

    def get_speed_assignemt(self):
        return self.v_s, self.v_ss, self.w

class Paramterization_Variable():
    def __init__(self):
        self.msg = s_message()
        self.pub = rospy.Publisher('/CSEI/s', s_message, queue_size=1)
        self.s = 0.0
        self.s_dot = 0.0

    def get_s(self):
        return self.s, self.s_dot

    def publish(self, s, s_dot):
        self.msg.s = s
        self.msg.s_dot = s_dot
        self.pub.publish(self.msg)

    def callback(self, msg):
        self.s = msg.s
        self.s_dot = msg.s_dot

class Gains():
    """
    Controller gains retrieves the parameters from the dynamic_reconfigure server.
    """
    # Initialize all gains to zero
    def __init__(self):
        self.L1 = np.zeros(3)
        self.L2 = np.zeros(3)
        self.L3 = np.zeros(3)

        self.Kp = np.zeros(3)
        self.Kd = np.zeros(3)
        self.Ki = np.zeros(3)

        self.mu = 0
        self.Uref = 0

    # Retrieves the gaines
    def get_ctrl_gains(self):
        return self.Kp, self.Kd, self.Ki, self.mu, self.Uref

    def get_obs_gains(self):
        return self.L1, self.L2, self.L3

    # Updates gains everytime the parameters are tuned
    def callback(self, config):
        self.L1 = self.string2array(config.L1)
        self.L2 = self.string2array(config.L2)
        self.L3 = self.string2array(config.L3)

        self.Kp = self.string2array(config.Kp)
        self.Kd = self.string2array(config.Kd)
        self.Ki = self.string2array(config.Ki)

        self.mu = config.mu
        self.Uref = config.U_ref

    # dynamic_reconfigure does not handle arrays, so gains like L1 or KP are strings on the form "x11,x12,x13"
    # the server to limit the number of variables. This function converts
    # the string into a numpy array when they are retrieved. Very scuffed :^)

    def string2array(self, string):
        return np.array(list(map(float, string.split(',')))) # Not proud of this one

# Build the objects to be imported
# ps4 = DS4_Controller()
# tau = Forces()
# u_data = UVector()
# qualisys = Qualisys()
# observer = Observer_Converser()
# reference = Reference_Converser()
# s_p = Paramterization_Variable()
# gains = Gains()

# Initialize controller node
def controllNodeInit():
    controller_node = rospy.init_node('controll_node')
    rospy.Subscriber("/joy", Joy, ps4.updateState)
    rospy.Subscriber("CSEI/u", Float64MultiArray, u_data.callback)
    rospy.Subscriber("CSEI/observer", observer_message, observer.callback)
    rospy.Subscriber("CSEI/reference", reference_message, reference.callback)
    rospy.Subscriber("CSEI/s", s_message, s_p.callback)
    #gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = gains.callback)
    return controller_node


def thrustAllocationNodeInit():
    thrust_allocation_node = rospy.init_node('Thrust_allocation_node')
    rospy.Subscriber("/CSEI/tau", Float64MultiArray, tau.callback)
    return thrust_allocation_node


def guidanceNodeInit():
    guidance_node = rospy.init_node('guidance_node')
    rospy.Subscriber("/CSEI/observer", observer_message, observer.callback)
    rospy.Subscriber("CSEI/reference", reference_message, reference.callback)
    rospy.Subscriber("CSEI/s", s_message, s_p.callback)
    rospy.Subscriber("/joy", Joy, ps4.updateState)
    #gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = gains.callback)
    return guidance_node


def observerNodeInit():
    observer_node = rospy.init_node('observer_node')
    rospy.Subscriber("/qualisys/CSEI/odom", Odometry, qualisys.callback)
    rospy.Subscriber("/joy", Joy, ps4.updateState)
    rospy.Subscriber("/CSEI/u", Float64MultiArray, u_data.callback)
    rospy.Subscriber("/CSEI/observer", observer_message, observer.callback)
    rospy.Subscriber("/CSEI/tau", Float64MultiArray, tau.callback)
    #gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = gains.callback)
    return observer_node

# Destroy node when prompted
def nodeEnd(node):
    node.destroy_node()