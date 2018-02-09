#!/usr/bin/python
# Copyright (c) 2011 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
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

import sys
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort
from numpy import array, identity, dot, pi
from numpy.linalg import norm
import yarp
from scipy.signal import iirfilter, lfilter, lfiltic
from arcospyu.numeric import quat
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.kdl_helpers import (
    rot_vector_angle, my_adddelta, my_diff, kdlframe_to_narray,
    narray_to_kdlframe, narray_to_kdltwist, kdltwist_to_narray, my_adddelta,
    my_get_euler_zyx, rpy_to_rot_matrix)
from arcospyu.control.control_loop import Controlloop
# from helpers import Roboviewer_objects, Joint_sim, Finger_sim, Hand_sim,
# Controlloop,
import time
from numpy import max as npmax
from arcospyu.signal_proc.filters import Filter_vector


def wait_valid_object_pose(
        object_pos_handle, last_object_pose, max_object_jump_dist,
        blocking=True):
    # getting current object position
    valid_object_pos = False
    while not valid_object_pos:
        xo_new = object_pos_handle.get_object_pos(blocking)
        if len(xo_new) == 0:
            return array([])
        if (norm(xo_new[:3, 3] - last_object_pose[:3, 3]) <
                max_object_jump_dist):
            valid_object_pos = True
        else:
            print "\nObject jumping around too much\n"
    return (xo_new)


class Object_pos_handle(object):
    def __init__(self, basename, simulation=True):
        self.simulation = simulation
        if self.simulation:
            self.port = new_port(
                basename + "/xo:in", "in", "/slider_sim/xo:out", timeout=10.)
        else:
            self.port = new_port(
                basename + "/xo:in", "in", "/marker/object_pos:o", timeout=10.)
            pass  # TODO: create port for marker tracking in case of using cop

    def get_object_pos_mean(self, num_measurements=10, timeout=-1):
        object_pose_data = array([0.] * 7)
        init_time = time.time()
        counter = 0
        while (counter < num_measurements):
            if time.time() - init_time > timeout:
                print "Timeout"
                return (array([]))
        # for i in xrange(num_measurements):
            pose = self.get_object_pos(blocking=False)
            if len(pose) == 0:
                pass
                # print "No object data"
            else:
                init_time = time.time()
                object_pose_data[:3] += pose[:3, 3]
                object_pose_data[3:] += quat.from_matrix(pose)
                counter += 1
        object_pose_data /= num_measurements
        object_pose_data[3:] /= norm(object_pose_data[3:])
        return (
            quat.to_matrix_fixed(object_pose_data[3:], r=object_pose_data[:3]))

    def get_object_pos(self, blocking=True):
        if self.simulation:
            result = readListPort(self.port, blocking=blocking)
            if result:
                xo = array(result).reshape((4, 4))
            else:
                return array([])
        else:
            result = readListPort(self.port, blocking=blocking)
            if result:
                xo = array(result).reshape((4, 4))
            else:
                return array([])
            # xo=quat.to_matrix(result[3:7], r=result[:3])

            pass  # TODO, get object from marker tracking in case of using cop
        return (xo)


def finger_joint_move(
        hand_handle, finger, joint_pos, wait=0.,
        goal_precision=0.):  # joint_pos in degrees
    hand_handle.set_ref(finger, array(joint_pos) * pi / 180.0)
    hand_handle.update_controller_refs(list_fingers=[finger])
    if wait != 0.:
        init_time = time.time()
        while (wait == -1) or (time.time() - init_time < wait):
            hand_handle.update_sensor_data()
            cur_joint_pos = hand_handle.fingers[finger].get_joint_pos()
            dist_to_goal = norm(cur_joint_pos - joint_pos * pi / 180.)
            print "cur joint pos", cur_joint_pos * 180. / pi
            print "goal joint pos", joint_pos
            print "Distance to goal", dist_to_goal * 180. / pi
            print "Goal precision", goal_precision
            if dist_to_goal < goal_precision * pi / 180.:
                print "Goal reached"
                return
        print "Timeout, goal never reached with requested precision"


def read_finger(yarp_port, blocking, finger):
    result = yarp_port.read(blocking)
    bottle = result.get(finger).asList()
    # print "Result", bottle
    trans = array(map(yarp.Value.asDouble, map(bottle.get, range(3))))
    rot = array(map(yarp.Value.asDouble, map(bottle.get, range(3, 7))))
    force = array(map(yarp.Value.asDouble, map(bottle.get, range(7, 10))))
    return (trans, rot, force)


def move_robot_fast(
        hand_handle,
        finger,
        arm_handle,
        pose,
        goal_precision,
        extra_tool=identity(4),
        update_finger_cart=True):
    # hand_handle.update_sensor_data()
    if update_finger_cart:
        finger_cart = hand_handle.fingers[finger].get_cart_pos()
        tool_pose = dot(finger_cart, extra_tool)
        # print "Finger cart pos", finger_cart
        arm_handle.setTool(narray_to_kdlframe(tool_pose))
    # arm_handle.gotoFrame(pose.reshape(16), wait=0.0, goal_precision=goal_precision) # noqa
    arm_handle.gotoPose(pose[:3, 3], pose[:3, :3].reshape(9))


def move_robot(
        hand_handle,
        finger,
        arm_handle,
        pose,
        goal_precision,
        wait=10.0,
        extra_tool=identity(4),
        update_finger_cart=True):
    if update_finger_cart:
        hand_handle.update_sensor_data()
        finger_cart = hand_handle.fingers[finger].get_cart_pos()
        tool_pose = dot(finger_cart, extra_tool)
        print "Finger cart pos", finger_cart
        print "SET TOOL"
        arm_handle.setTool(narray_to_kdlframe(tool_pose))
    print "GOTO POSE"
    arm_handle.gotoFrame(
        pose.reshape(16), wait=wait, goal_precision=goal_precision)
    print "END GOTO POSE"


def near(num1, num2, epsilon=1e-10):
    return (abs(num1 - num2) < epsilon)


def find_force_noise_levels(
        force_handle, finger, filter_order=3, filter_freq=0.3,
        measurements=10):
    vector_filter = Filter_vector(order=filter_order, freq=filter_freq)
    # stabilizing filter
    for i in xrange(measurements):
        force = force_handle.get_force(finger, update=True)
        vector_filter.filter(force)
    # real measurement
    max_force = array([0.] * 3)
    max_norm_force = 0.
    for i in xrange(measurements):
        force = force_handle.get_force(finger, update=True)
        force_filtered = vector_filter.filter(force)
        max_force = npmax(array([max_force, force_filtered]), axis=0)

        norm_force = norm(force_filtered)
        max_norm_force = max(max_norm_force, norm_force)
    return (max_force, max_norm_force)
