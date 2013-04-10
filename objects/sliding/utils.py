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
sys.path.append("../../control/motionControl")
sys.path.append("../../tools/python/numeric/")

from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port, yarpListToList, bottle_to_list
from numpy import array, identity, dot, concatenate, sign, cross, arccos, pi, arctan, tan, cos, sin, sign, exp, arctan2, mean
from numpy.linalg import norm
import yarp
from scipy.signal import iirfilter, lfilter, lfiltic
from arcospyu.numeric import quat
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta, my_diff, kdlframe_to_narray, narray_to_kdlframe, narray_to_kdltwist, kdltwist_to_narray, my_adddelta, my_get_euler_zyx, rpy_to_rot_matrix
from arcospyu.control.control_loop import Controlloop
#from helpers import Roboviewer_objects, Joint_sim, Finger_sim, Hand_sim, Controlloop, 
import time
from numpy.random import normal
from numpy import max as npmax



def wait_valid_object_pose(object_pos_handle, last_object_pose, max_object_jump_dist, blocking=True):
    #getting current object position
    valid_object_pos=False
    while not valid_object_pos:
        xo_new=object_pos_handle.get_object_pos(blocking)
        if len(xo_new)==0:
            return array([])
        if norm(xo_new[:3,3]-last_object_pose[:3,3]) < max_object_jump_dist:
            valid_object_pos=True
        else:
            print
            print "Object jumping around too much"
            print
    return(xo_new)


class Force_handle(object):
    def __init__(self, baseportname, remote_name):
        self.port=new_port(baseportname+"/force:i", "in", remote_name, timeout=10.0)
        self.update_data(blocking=True)

    def update_data(self, blocking=True):
        pending=self.port.getPendingReads()
        for i in xrange(pending):
            self.port.read()
        bottle=self.port.read(blocking)
        self.data=bottle_to_list(bottle)

    def get_pose(self, finger, update=True, blocking=True, extra_tool=identity(4)):
        if update:
            self.update_data(blocking)
        finger_data=array(self.data[finger])
        return(dot(quat.to_matrix_fixed(finger_data[3:7], r=finger_data[:3]), extra_tool))

    def get_data(self, blocking=True, update=True):
        if update:
            self.update_data(blocking=blocking)
        return(self.data)

    def get_data_finger(self, finger, blocking=True, update=True):
        if update:
            self.update_data()
        return(array(self.data[finger]))

    def get_force(self, finger, blocking=True, update=True):
        if update:
            self.update_data(blocking=blocking)
        return(self.data[finger][-3:])

class Object_pos_handle(object):
    def __init__(self, basename, simulation=True):
        self.simulation=simulation
        if self.simulation:
            self.port=new_port(basename+"/xo:in", "in", "/slider_sim/xo:out", timeout=10.)
        else:
            self.port=new_port(basename+"/xo:in", "in", "/marker/object_pos:o", timeout=10.)
            pass #TODO: create port for marker tracking in case of using cop

    def get_object_pos_mean(self, num_measurements=10, timeout=-1):
        object_pose_data=array([0.]*7)
        init_time=time.time()
        counter=0
        while (counter<num_measurements):
            if time.time()-init_time>timeout:
                print "Timeout"
                return(array([]))
        #for i in xrange(num_measurements):
            pose=self.get_object_pos(blocking=False)
            if len(pose)==0:
                pass
                #print "No object data"
            else:
                init_time=time.time()
                object_pose_data[:3]+=pose[:3,3]
                object_pose_data[3:]+=quat.from_matrix(pose)
                counter+=1
        object_pose_data/=num_measurements
        object_pose_data[3:]/=norm(object_pose_data[3:])
        return(quat.to_matrix_fixed(object_pose_data[3:], r=object_pose_data[:3]))

    def get_object_pos(self, blocking=True):
        if self.simulation:
            result=readListPort(self.port,blocking=blocking)
            if result:
                xo=array(result).reshape((4,4))
            else:
                return array([])
        else:
            result=readListPort(self.port,blocking=blocking)
            if result:
                xo=array(result).reshape((4,4))
            else:
                return array([])
            #xo=quat.to_matrix(result[3:7], r=result[:3])
            
            pass #TODO, get object from marker tracking in case of using cop
        return(xo)

class Filter_vector():
    def __init__(self, dim=3, order=2, freq=0.7, y=[], x=[]):
        self.dim=dim
        self.filters=[]
        for i in xrange(dim):
            if len(y)>0:
                self.filters.append(Filter(order=order, freq=freq, y=y[i], x=x[i]))
            else:
                self.filters.append(Filter(order=order, freq=freq))

    def filter(self, data):
        return(array([self.filters[i].filter(array([data[i]]))[0] for i in xrange(self.dim)]))


class Filter():
    def __init__(self, order=2, freq=0.7,y=[],x=[]):
         self.b,self.a=iirfilter(order, freq, btype="lowpass")
         if len(y)>0:
             print "here"
             self.z=lfiltic(self.b,self.a, y, x=x)
         else:
             self.z=array([0.]*order)
         #self.z=lfiltic(self.b,self.a, y,x=x)

    def filter(self,raw_data):
        #print "Raw", array(raw_data)
        y,self.z=lfilter(self.b,self.a,raw_data,zi=self.z, axis=0)
        return(y)


def finger_joint_move(hand_handle, finger, joint_pos, wait=0., goal_precision=0.): #joint_pos in degrees
    hand_handle.set_ref(finger,array(joint_pos)*pi/180.0)
    hand_handle.update_controller_refs(list_fingers=[finger])
    if wait!=0.:
        init_time=time.time()
        while (wait==-1) or (time.time()-init_time<wait):
            hand_handle.update_sensor_data()
            cur_joint_pos=hand_handle.fingers[finger].get_joint_pos()
            dist_to_goal=norm(cur_joint_pos-joint_pos*pi/180.)
            print "cur joint pos", cur_joint_pos*180./pi
            print "goal joint pos", joint_pos
            print "Distance to goal", dist_to_goal*180./pi
            print "Goal precision", goal_precision
            if dist_to_goal<goal_precision*pi/180.:
                print "Goal reached"
                return
        print "Timeout, goal never reached with requested precision"


def add_noise_to_pose(pose, dist_noise=0.02, angle_noise=5.0*pi/180.0):
    noise_pose=identity(4)
    noise_pose[:3,3]=normal(loc=pose[:3,3], scale=dist_noise, size=(3))
    random_angles=normal(loc=0.,scale=angle_noise, size=(3))
    noise_pose[:3,:3]=dot(pose[:3,:3],rpy_to_rot_matrix(random_angles))
    return(noise_pose)


def move_robot_fast(hand_handle, finger, arm_handle, pose, goal_precision, extra_tool=identity(4), update_finger_cart=True):
    #hand_handle.update_sensor_data()
    if update_finger_cart:
        finger_cart=hand_handle.fingers[finger].get_cart_pos()
        tool_pose=dot(finger_cart, extra_tool)
    #print "Finger cart pos", finger_cart
        arm_handle.setTool(narray_to_kdlframe(tool_pose))
    #arm_handle.gotoFrame(pose.reshape(16), wait=0.0, goal_precision=goal_precision)
    arm_handle.gotoPose(pose[:3,3], pose[:3,:3].reshape(9))

def move_robot(hand_handle, finger, arm_handle, pose, goal_precision, wait=10.0, extra_tool=identity(4), update_finger_cart=True):
    if update_finger_cart:
        hand_handle.update_sensor_data()
        finger_cart=hand_handle.fingers[finger].get_cart_pos()
        tool_pose=dot(finger_cart, extra_tool)
        print "Finger cart pos", finger_cart
        print "SET TOOL"
        arm_handle.setTool(narray_to_kdlframe(tool_pose))
    print "GOTO POSE"
    arm_handle.gotoFrame(pose.reshape(16), wait=wait, goal_precision=goal_precision)
    print "END GOTO POSE"

def homo_matrix(rot_m=identity(3),trans=array([0.]*3)):
    m=identity(4)
    m[:3,:3]=rot_m
    m[:3,3]=trans
    return(m)


def rot_z(angle):
    return(array([[cos(angle), -sin(angle), 0.],
                  [sin(angle), cos(angle),0.],
                  [0.,0.,1.]]))

def rot_x(angle):
    return(array([[1., 0., 0.],
                  [0., cos(angle), -sin(angle)],
                  [0., sin(angle), cos(angle)]]))

def rot_y(angle):
    return(array([[cos(angle), 0, sin(angle)],
                  [0., 1., 0.],
                  [-sin(angle), 0., cos(angle)]]))


def random_homo_matrix(center_pos=array([0.]*3), noise_level=array([0.02,0.02,0.02,5.0*pi/180.0,5.0*pi/180.0,5.0*pi/180.0])):
    #incomplete function, not finished
    out_matrix=identity(4)
    out_matrix[:3,3]=normal(loc=array(center_pos), scale=noise_level[:3], size=(3))
    random_angles=normal(loc=array(center_angles),scale=noise_level[2])
    table_object_normal=-array(self.box_planes[self.table_object_face][0])
    print "table normal", table_object_normal
    random_rot_frame=identity(4)
    random_rot_frame[:2,3]=0*array([0.0,0.01]) #camera offset error
    random_rot_frame[:3,:3]=rot_vector_angle(table_object_normal, random_z_angle)
    self.box_pose_out=dot(self.box_pose_out, random_rot_frame)


def find_force_noise_levels(force_handle, finger, filter_order=3, filter_freq=0.3, measurements=10):
    vector_filter=Filter_vector(order=filter_order, freq=filter_freq)
    #stabilizing filter
    for i in xrange(measurements):
        force=force_handle.get_force(finger, update=True)
        vector_filter.filter(force)
    #real measurement
    max_force=array([0.]*3)
    max_norm_force=0.
    for i in xrange(measurements):
        force=force_handle.get_force(finger, update=True)
        force_filtered=vector_filter.filter(force)
        max_force=npmax(array([max_force, force_filtered]), axis=0)

        norm_force=norm(force_filtered)
        max_norm_force=max(max_norm_force, norm_force)
    return(max_force, max_norm_force)
