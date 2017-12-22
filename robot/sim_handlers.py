#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Alexis Maldonado Herrera <maldonad at cs.tum.edu> Federico Ruiz-Ugalde <memeruiz@gmail.com>
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
import yarp
from numpy import identity, array
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, bottle_to_list, yarp_connect_blocking
from cmoc.robot.hand_sim_handler import Hand
from arcospyu.robot_tools.lafik import Lafik
from arcospyu.config_parser.config_parser import import_config
from PyKDL import Wrench, Vector
from numpy import dot, array, concatenate

class Joint_sim():
    def __init__(self,portbasename,jointsimportbasename):
        self.torque_out_port=yarp.BufferedPortBottle()
        self.torque_out_port.open(portbasename+"/joint_sim_client/torque_out")
        yarp.Network.connect(portbasename+"/joint_sim_client/torque_out", jointsimportbasename+"/torque_in")

    def set_torques(self,torques):
        torque_out_bottle=self.torque_out_port.prepare()
        torque_out_bottle.clear()
        for i in torques:
            torque_out_bottle.addDouble(i)
        self.torque_out_port.write()

class Finger_sim():
    def __init__(self,portbasename,finger,fingersimbaseportname):
        self.torque_out_port=yarp.BufferedPortBottle()
        basename=portbasename+"/finger"+str(finger)
        self.torque_out_port.open(basename+"/torque_out")
        yarp.Network.connect(basename+"/torque_out", fingersimbaseportname+"/torque_in")

    def set_torques(self,torques):
        torque_out_bottle=self.torque_out_port.prepare()
        torque_out_bottle.clear()
        for i in torques:
            torque_out_bottle.addDouble(i)
        self.torque_out_port.write()

class Hand_sim():
    def __init__(self,handedness,sahand_port):
        self.fingers_sim_client=[]
        for i in xrange(5):
            self.fingers_sim_client.append(Finger_sim("/torque_sim_"+handedness,i,"/sahand"+str(sahand_port)+"/finger"+str(i)))
        #self.fingers_sim_client=map(Finger_sim, [["/torque_sim_"+handedness,i,"/sahand"+str(sahand_port)+"/finger"+str(i)] for i in xrange(4)])

    def set_torques(self,torques):
        for torque, finger_sim_client in zip(torques,self.fingers_sim_client):
            finger_sim_client.set_torques(torque)

    def set_torques_np(self,torques):
        #print "Torques", torques
        for i in xrange(5):
            #print "Torques", torques[i,:]
            self.fingers_sim_client[i].set_torques(torques[i,:])

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

class Handle_arm_joints():
    def __init__(self, arm_portbasename, handlername="/handle_arm_joints"):
        import imp
        prename=arm_portbasename
        full_name=prename+handlername

        self.q_in_port=yarp.BufferedPortBottle()
        self.q_in_port.open(full_name+"/qin")

        yarp_connect_blocking(prename+"/bridge/encoders" , full_name+"/qin")
        self.angles=[0.]*7

    def get_angles(self, blocking=True):
        if not blocking:
            if not (self.q_in_port.getPendingReads()>0):
                return(self.angles)
        q_in_bottle=self.q_in_port.read(blocking)
        self.angles=map(yarp.Value.asDouble,map(q_in_bottle.get,range(q_in_bottle.size())))
        return(self.angles)

class Sim_torques():
    '''class that connects to arms and hands and calculates torques given a force'''
    def __init__(self, config_lwr_right, config_lwr_left, config_hand, baseportname="/torque_sim"):
        arm_joint_num=7
        self.fingers_cant = 5
        self.enabled_left = not (config_lwr_left == None)
        self.rarmj_handle=Handle_arm_joints("/lwr/right",handlername=baseportname+"/arm_joints")
        self.rhand=Hand(config_hand, handedness="right",sahand_number=0, portprefix=baseportname+"/hand_handler")
        self.rarm_robot=Lafik(import_config(config_lwr_right))
        self.sim_forces_right=[array([0.]*3)]*self.fingers_cant
        self.est_forces_right=[array([0.]*3)]*self.fingers_cant
        self.finger_tip_right=[array([[1.,0.,0.,0.],
                                      [0.,1.,0.,0.],
                                      [0.,0.,1.,0.],
                                      [0.,0.,0.,1.]])]*self.fingers_cant
        self.rhand_torques=array([[0.]*3]*self.fingers_cant)        
        self.arm_torques_right=array([0.]*arm_joint_num)
        if self.enabled_left:
            self.larmj_handle=Handle_arm_joints("/lwr/left",handlername=baseportname+"/arm_joints")
            self.lhand=Hand(config_hand, handedness="left",sahand_number=1, portprefix=baseportname+"/hand_handler")
            self.larm_robot=Lafik(import_config(config_lwr_left))
            self.sim_forces_left=[array([0.]*3)]*self.fingers_cant
            self.est_forces_left=[array([0.]*3)]*self.fingers_cant
            self.finger_tip_left=[array([[1.,0.,0.,0.],
                                         [0.,1.,0.,0.],
                                         [0.,0.,1.,0.],
                                         [0.,0.,0.,1.]])]*self.fingers_cant
            self.lhand_torques=array([[0.]*3]*self.fingers_cant)
            self.arm_torques_left=array([0.]*arm_joint_num)

        self.wrench=Wrench()

    def update_data(self):
        self.rarm_robot.jntsList=self.rarmj_handle.get_angles(False)
        self.rarm_frame=array(self.rarm_robot.frame).reshape((4,4))
        self.rhand.update_sensor_data()
        if self.enabled_left:
            self.larm_robot.jntsList=self.larmj_handle.get_angles(False)
            self.larm_frame=array(self.larm_robot.frame).reshape((4,4))
            self.lhand.update_sensor_data()
    
    def set_sim_forces(self,arm,finger,force):
        if arm=="right":
            self.sim_forces_right[finger]=array(force)
        if arm=="left":
            self.sim_forces_left[finger]=array(force)

    def get_torques_all(self,arm=0):
        #arm_joint_num=7
        #hand_torques=[]
        if arm==0: #right
                self.arm_torques_right*=0
                for finger,force in enumerate(self.sim_forces_right):
                    #referencing fingertip force to arm end effector force
                    self.finger_tip_right[finger]=self.get_cart_pos(arm,finger)
                    finger_tip_to_hand=self.rarm_frame[:,3][:3]-self.finger_tip_right[finger][:,3][:3]
                    #self.wrench=Wrench()
                    self.wrench.force=Vector(force[0],force[1],force[2])
                    arm_wrench=self.wrench.RefPoint(Vector(finger_tip_to_hand[0],finger_tip_to_hand[1],finger_tip_to_hand[2]))
                    #print "Finger: ", finger, "Arm_wrench", arm_wrench

                    #calculating arm torques
                    self.rarm_robot.update_jac()
                    self.arm_torques_right+=self.rarm_robot.calc_torques(array([arm_wrench[i] for i in xrange(6)]))
                    #print "Arm torques", self.arm_torques_right
                    #calculating hand torques
                    hand_force=dot(self.rarm_frame[:3,:3].T,array(force)) #with respect to hand base
                    #hand_torques=self.rhand.fingers[finger].kinematics.calc_torques(hand_force.reshape((6,1)).T)
                    #hand_torques.append(self.rhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3)))))
                    self.rhand_torques[finger,:]=(self.rhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3)))))
                #print "Finger:", finger, "Hand torques", self.rhand_torques
                #print "Arm_torques final:", self.arm_torques_right
                return(self.arm_torques_right,self.rhand_torques)
        if arm==1:
                self.arm_torques_left*=0
                for finger,force in enumerate(self.sim_forces_left):
                    #referencing fingertip force to arm end effector force
                    self.finger_tip_left[finger]=self.get_cart_pos(arm,finger)
                    finger_tip_to_hand=self.larm_frame[:,3][:3]-self.finger_tip_left[finger][:,3][:3]
                    #wrench=Wrench()
                    self.wrench.force=Vector(force[0],force[1],force[2])
                    arm_wrench=self.wrench.RefPoint(Vector(finger_tip_to_hand[0],finger_tip_to_hand[1],finger_tip_to_hand[2]))
                    #print "Arm_wrench", arm_wrench

                    #calculating arm torques
                    self.larm_robot.update_jac()
                    self.arm_torques_left+=self.larm_robot.calc_torques(array([arm_wrench[i] for i in xrange(6)]))
                    #calculating hand torques
                    hand_force=dot(self.larm_frame[:3,:3].T,array(force)) #with respect to hand base
                    #hand_torques=self.rhand.fingers[finger].kinematics.calc_torques(hand_force.reshape((6,1)).T)
                    #hand_torques.append(self.lhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3)))))
                    self.lhand_torques[finger,:]=(self.lhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3)))))
                #print "Hand torques", self.lhand_torques
                return(self.arm_torques_left,self.lhand_torques)

    def get_torques(self,arm,finger,force):
        if arm=="right":
            rhand_frame=self.rhand.fingers[finger].get_cart_pos()
            finger_tip=self.get_cart_pos(arm,finger)
            #print "finger tip", finger_tip
            #print "rhand frame", rhand_frame
            #print "rarm frame", self.rarm_frame
            finger_tip_to_hand=self.rarm_frame[:,3][:3]-finger_tip[:,3][:3]
            
            wrench=Wrench()
            wrench.force=Vector(force[0],force[1],force[2])
            arm_wrench=wrench.RefPoint(Vector(finger_tip_to_hand[0],finger_tip_to_hand[1],finger_tip_to_hand[2]))
            #print "Arm_wrench", arm_wrench
            self.rarm_robot.update_jac()
            arm_torques=self.rarm_robot.calc_torques(array([arm_wrench[i] for i in xrange(6)]))
            #hand torques
            hand_force=dot(self.rarm_frame[:3,:3].T,array(force))
            #hand_torques=self.rhand.fingers[finger].kinematics.calc_torques(hand_force.reshape((6,1)).T)
            hand_torques=self.rhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3))))
            return(arm_torques,hand_torques)
        if arm=="left":
            larm_frame=array(self.larm_frame).reshape((4,4))
            lhand_frame=self.lhand.fingers[finger].get_cart_pos()
            finger_tip=self.get_cart_pos(arm,finger)
            finger_tip_to_hand=larm_frame[:,3][:3]-finger_tip[:,3][:3]
            
            wrench=Wrench()
            wrench.force=Vector(force[0],force[1],force[2])
            arm_wrench=wrench.RefPoint(Vector(finger_tip_to_hand[0],finger_tip_to_hand[1],finger_tip_to_hand[2]))
            #print "Arm_wrench", arm_wrench
            self.larm_robot.update_jac()
            arm_torques=self.larm_robot.calc_torques(array([arm_wrench[i] for i in xrange(6)]))
            #hand torques
            hand_force=dot(larm_frame[:3,:3].T,array(force))
            #hand_torques=self.rhand.fingers[finger].kinematics.calc_torques(hand_force.reshape((6,1)).T)
            hand_torques=self.lhand.fingers[finger].kinematics.calc_torques(concatenate((hand_force,array([0.]*3))))
            return(arm_torques,hand_torques)

    def get_cart_pos(self,arm,finger):
        if (arm==0) or (arm=="right"): #right
            return(dot(self.rarm_frame,self.rhand.fingers[finger].get_cart_pos()))
        if (arm==1) or (arm=="left"):
            return(dot(self.larm_frame,self.lhand.fingers[finger].get_cart_pos()))

    def get_force(self,arm,finger):
        if (arm==0) or (arm=="right"): #right
            #return(self.rhand.fingers[finger].get_forces()[:3])
            return(dot(self.rarm_frame[:3,:3],self.rhand.fingers[finger].get_forces()[:3]))
        if (arm==1) or (arm=="left"):
            return(dot(self.larm_frame[:3,:3],self.lhand.fingers[finger].get_forces()[:3]))

    def get_forces(self):
        for finger in xrange(self.fingers_cant):
            #print "Getting forces: ", finger, self.fingers_cant
            #if finger==0:
            #    print "Torques", self.rhand.fingers[finger].get_torques()
            self.est_forces_right[finger]=self.get_force(0,finger)
            if self.enabled_left:
                self.est_forces_left[finger]=self.get_force(1,finger)
        if self.enabled_left:
            self.est_forces_left[finger]=self.get_force(1,finger)
            return(self.est_forces_right, self.est_forces_left)
        else:
            return(self.est_forces_right, None)

def main():
    return(False)



if __name__ == "__main__":
    main()


