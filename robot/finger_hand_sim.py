#!/usr/bin/python
# Copyright (c) 2009,2010 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Alexis Maldonado-Herrera <maldonad at in.tum.de>, Federico Ruiz-Ugalde <ruizf at in.tum.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your optin) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import cmoc.robot.sahand as sahand
from arcospyu.control.nanosleep import nanosleep
from time import sleep
from math import *
from numpy import array,concatenate
import sys
import yarp
from optparse import OptionParser
import time
from sahand_ros_sim import ROSRate, ROSPublisher



go_on=True
import signal
def signal_handler(signum, frame):
    print("Got a signal, getting out!")
    global go_on
    go_on=False
    import sys
    sys.exit()



class DummyFinger(object):
    def __init__(self):
        self.angles=[0.0, 0.0, 0.0]
        self.speeds=[0.0, 0.0, 0.0]
        self.torques=[0.0, 0.0, 0.0]


class Finger_sim(object):
    def __init__(self,port,basename,finger_num, hands_config):
        #sys.path.append("../../temp/hand_cart")
        from cmoc.robot.joint_controller import Controller_mass_sim, Controller_type
        import time
        self.enabled=True
        self.ref_angles=array((5.,5.,0.))
        self.ref_vels=array((20.,20.,20.))
        self.ext_torques_offset=array((0.,0.,0.))
        self.ext_torques=array((0.,0.,0.))
        self.speed=array((0.,0.,0.))
        self._angles=array((0.,0.,0.))
        #from hands_kin import fingers_lim
        fingers_lim=hands_config.fingers_lim
        self.kp=0.8
        self.kvp=0.8
        self.inertia=0.1
        self.sim_stiffness=self.kvp*self.kp
        self.sim_param=[self.kp,self.kvp,self.inertia,self.sim_stiffness]
        if finger_num==0:
            self.num_joints=4
        else:
            self.num_joints=3
        self._stiffness=array([self.sim_stiffness]*3)*pi/180.0
        self.sim=Controller_mass_sim(self.kp,self.kvp,self.inertia,self.stiffness,self.num_joints,reversed(fingers_lim[finger_num]))
        self.sim.set_initial_time(time.time())
        self.sim.set_controller(Controller_type.position)
        self.torque_in_port=yarp.BufferedPortBottle()
        self.torque_in_port.open(basename+str(port-1)+"/finger"+str(finger_num)+"/torque_in")
        self.sim_param_port=yarp.BufferedPortBottle()
        self.sim_param_port.open(basename+str(port-1)+"/finger"+str(finger_num)+"/sim_param")
        self.finger_num=finger_num
        

    def enable(self):
        self.enabled=True

    def _set_stiffness(self,stiffness):
        #print "Set stiffness", stiffness
        self._stiffness=array(stiffness)
        if self.num_joints==4:
            self.sim_param[3]=concatenate((self._stiffness,array([1000.])))*180.0/pi
        else:
            self.sim_param[3]=self._stiffness*180.0/pi
        self.sim.set_param(self.sim_param)
        #self.update()

    def _get_stiffness(self):
        return(self._stiffness)

    stiffness = property(_get_stiffness, _set_stiffness)

    def _get_angles(self):
        return(self._angles)

    angles=property(_get_angles)

    def _get_speed(self):
        self.speed=self.sim.qv[:3]*180.0/pi
        return(self.speed)

    speeds=property(_get_speed)

    def _get_torques(self):
        return(self.ext_torques+self.ext_torques_offset)

    torques=property(_get_torques)

    def set_ext_torque(self,torque):
        self.ext_torques=torque
        #sim.set_ext_torque(self.ext_torques+self.ext_torques_offset)

    def move(self, angles, vels=(100.,100.,100.), interval=0.1):
        print "moving", angles
        self.ref_angles=array(angles)
        self.ref_vels=array(vels)
        self.sim.set_q_ref(self.ref_angles*pi/180.0)
        #sim.set_qv_max(self.ref_vels)
        #self.update()

    def clear_torque_sensor_offset(self):
        self.ext_torques_offset=-array(self.ext_torques)

    def update(self):
        #print "updating"
        #TODO simulation goes here
        #use: self.ext_torques_offset, self.stiffness, self.ref_angles, self.ref_vels
        #self.angles, self.speed, self.ext_torques
        #and for thumb also self.base_angle
        sim_param_bottle=self.sim_param_port.read(False)
        if sim_param_bottle:
            sim_param=array(map(yarp.Value.asDouble,map(sim_param_bottle.get,range(sim_param_bottle.size()))))[::-1]
            self.sim.set_param(sim_param)
        torque_in_bottle=self.torque_in_port.read(False)
        if torque_in_bottle:
            torque_in=array(map(yarp.Value.asDouble,map(torque_in_bottle.get,range(torque_in_bottle.size()))))[::-1]
            if self.finger_num==0:
                self.sim.set_ext_torque(concatenate((torque_in,array([0.]))))
            else:
                self.sim.set_ext_torque(torque_in)
            self.ext_torques=torque_in
            #print "torque_in", self.ext_torques

        #print "Finger num", self.finger_num
        self._angles=self.sim.get_q(time.time())[:3]*180.0/pi

class Thumb_sim(Finger_sim):
    def __init__(self,port,basename,finger, hands_config):
        Finger_sim.__init__(self,port,basename,finger, hands_config)
        self.ref_angles=array((5.,5.,0.,0.))
        self.max_base_vel=100.
        self.ref_vels=array((20.,20.,20.,self.max_base_vel))
        self.base_braked=False
        self._base_angle=0.

    def unbrake_base(self):
        self.base_braked=False

    def _get_base_angle(self):
        return(self._base_angle)
    
    base_angle=property(_get_base_angle)

    def move(self,angles, vels=(100.,100.,100.), interval=0.1):
        self.ref_angles[:3]=array(angles)
        self.ref_vels[:3]=array(vels)
        self.sim.set_q_ref(self.ref_angles*pi/180.0)

    def move_base(self,angle):
        self.ref_angles[-1]=angle
        print "Moving base", self.ref_angles
        self.sim.set_q_ref(self.ref_angles*pi/180.0)

    def update(self):
        Finger_sim.update(self)
        self._base_angle=self.sim.q[-1]*180.0/pi

class Hand_sim(object):
    def __init__(self,port,basename, hands_config):
        self.port=port
        self.thumb=Thumb_sim(self.port,basename,0, hands_config)
        self.first=Finger_sim(self.port,basename,1, hands_config)
        self.middle=Finger_sim(self.port,basename,2, hands_config)
        self.ring=Finger_sim(self.port,basename,3, hands_config)
        self.emergency_stop=False

    def __iter__(self):
        return iter([self.thumb, self.first, self.middle, self.ring])

    def disable_emergency_stop(self):
        self.emergency_stop=False

    def get_hand_config(self):
        if self.port==sahand.PORT_1:
            return(1)
        elif self.port==sahand.PORT_2:
            return(2)
        else:
            return(0)

    def set_controller(self,controller):
        #TODO
        pass

class Sahboard_sim(object):
    def __init__(self,basename, hands_config, enabled_hands=[True,True]):
        self.enabled_hands=enabled_hands
        self.hands=[]
        self.hands.append(Hand_sim(sahand.PORT_1,basename, hands_config))
        self.hands.append(Hand_sim(sahand.PORT_2,basename, hands_config))
        self.counter=0
        self.previous_time=time.time()
        self.cur_time=time.time()
        self.frequency=60. #TODO: change frequency
        self.period=1.0/self.frequency
        self.ref_next_time=self.cur_time+self.period
            

    def is_connected(self,port):
        if port == sahand.PORT_1:
            if self.enabled_hands[0]:
                return(True)
        elif port == sahand.PORT_2:
            if self.enabled_hands[1]:
                return(True)
        return(False)

    def get_counter(self):
        self.cur_time=time.time()
        wait_time=self.ref_next_time-self.cur_time
        if wait_time>0:
            #print "Waiting for ", wait_time
            time.sleep(wait_time)
        else:
            print "Not enough time"
        self.ref_next_time+=self.period
        for hand in self.hands:
            for finger in hand:
                #print "Hand", hand.port
                finger.update()
        self.counter+=1
        return(self.counter)
