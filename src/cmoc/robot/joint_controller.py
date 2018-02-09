#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Federico Ruiz-Ugalde
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

from numpy import array, amin, amax, pi
import numpy


class Controller_type(object):
    position = 0
    velocity = 1


class Controller_mass_sim(object):
    def __init__(self, kp, kvp, inertia, stiffness, joint_num, joint_limits):
        """
        Simulates a controller/mass system.
        kp_m: kp controller parameter divided by the virtual mass bigger is
              equivalent to increase the controller speed or decrease the mass
        joint_num: quantity of joints
        joint_limits: the joint limits
        """
        self.joint_num = joint_num
        # self.kp=kp # kp for position controller
        # self.kvp=kvp # kp for velocity controller
        # self.inertia=inertia # inertia
        self.set_param([kp, kvp, inertia, stiffness])
        self.set_joint_limits(joint_limits)
        self.q = array([0.0] * joint_num)  # joint angle
        self.qv = array([0.0] * joint_num)  # joint speed
        self.q_ref = array([0.0] * joint_num)  # joint desired position
        self.q_ref_old = array([0.0] * joint_num)
        self.qv_ref = self.qv  # desired speed
        self.qv_max = array(
            [180.0 * pi / 180.0] * joint_num)  # maximum joint speed
        self.ext_torque = array([0.0] * joint_num)
        self.acc = array(
            [0.0] * joint_num
        )  # acceleration produced by the controller and the virtual mass
        self.last_time = 0.0
        self.controller = Controller_type.velocity

    def set_controller(self, controller_type):
        self.controller = controller_type

    def set_initial_time(self, time):
        self.last_time = time

    def set_joint_limits(self, limits):
        self.low_limits, self.high_limits = zip(*limits)

    def set_q_ref(self, q_ref):
        self.q_ref = q_ref

    def set_qv_ref(self, qv_ref):
        """Sets the desired joint velocities"""
        self.qv_ref = qv_ref

    def set_qv_max(self, qv_max):
        self.qv_max = qv_max

    def set_q(self, q):
        self.q = q
        self.qv = array([0.0] * self.joint_num)
        self.acc = array([0.0] * self.joint_num)

    def set_param(self, param):
        self.kp = param[0]
        self.kvp = param[1]
        self.inertia = param[2]
        self.stiffness = array(param[3])
        # print 'Stiffness', self.stiffness
        self.ks1 = self.stiffness / (self.kvp * self.kp)
        if numpy.any(self.stiffness == array([0.] * len(self.stiffness))):
            self.ks2 = array([1.] * len(self.stiffness))
        else:
            self.ks2 = self.kvp * self.kp / self.stiffness
        # print 'ks', self.ks1, self.ks2

    def set_ext_torque(self, torque):
        self.ext_torque = torque

    def update(self, time):
        self.delta_time = time - self.last_time
        self.last_time = time
        # position control
        if self.controller == Controller_type.position:
            # print 'Position', self.q_ref
            self.qv_ref = self.kp * (self.q_ref - self.q)
        # limiting speed

        # velocity control
        self.torque_int = self.kvp * (self.qv_ref - self.qv)
        # acceleration calculation
        ks1 = array([1.0] * len(self.ks1))
        ks2 = array([1.0] * len(self.ks2))
        # if self.ks1 element is smaller than 1. then multiply by torque_int,
        # otherwise multiply self.ks2 element with ext_torque
        for i, num in enumerate(self.ks1.tolist()):
            if num < 1.0:
                ks1[i] = num
                ks2[i] = 1.0
            else:
                ks1[i] = 1.0
                ks2[i] = self.ks2[i]
        # print 'Torque int', self.torque_int, 'Torque ext', self.ext_torque, 'ks', ks1, ks2 # noqa
        self.acc = (
            ks1 * self.torque_int + self.ext_torque * ks2) / self.inertia
        # print 'Acc', self.acc
        # motion equations
        self.q = self.q + self.qv * self.delta_time + self.acc * (
            self.delta_time**2) / 2.0
        self.qv = self.qv + self.acc * self.delta_time
        # Check for joint limits
        q_unlimited = self.q
        self.q = amin(
            array(
                [amax(array([self.low_limits, self.q]), 0), self.high_limits]),
            0)
        # speed must be zero for limited joints
        changed = q_unlimited - self.q
        for i in range(self.joint_num):
            if changed[i] != 0.:
                self.qv[i] = 0.
        # print 'qv', self.qv

    def get_q(self, time):
        self.update(time)
        return (self.q)
