#!/usr/bin/python
# Copyright (c) 2016 ARCOS-LAB, Universidad de Costa RIca
# Authors: Federico Ruiz-Ugalde, Daniel Garcia Vaglio
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

import sys, time

from numpy import array, identity, dot, concatenate, sign, cross, arccos, pi, arctan, tan, cos, sin, sign, exp, arctan2, mean
from numpy.linalg import pinv, inv, norm
import PyKDL as kdl
import yarp
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port, write_bottle_lists, yarpListToList, bottle_to_list
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.control.control_loop import Controlloop
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta, my_diff, my_get_euler_zyx
from cmoc.robot.hand_sim_handler import Hand
from cmoc.objects.sliding.utils import Object_pos_handle, wait_valid_object_pose, move_robot, finger_joint_move

from cmoc.robot.sim_handlers import Force_handle
from arcospyu.robot_tools.robot_trans import homo_matrix
import optparse
from vfclik.handlers import HandleArm, HandleJController, HandleBridge
from arcospyu.config_parser.config_parser import import_config

# FIXME: right now all thumb commands use 4 angles. But the first one must be 0. This should be fixed, so that the thumb only uses 3 anlges.
# DLR HIT II does not have that extra joint. 

class Control(Controlloop):
    
    ''' 
    visualize is in charge to handle the objects for visualization
    recieves as parameters the points and vectors that have to be drawn
    '''
    
    def visualize(self,points=[],vectors=[]):
        for local,ref_frame,vis_id in points:
            if len(local)==3:
                #for points
                pose=dot(ref_frame,homo_matrix(trans=local))
            else:
                #for frames
                pose=dot(ref_frame,local)
            self.view_objects.send_prop(vis_id,"pose",
                                        pose.reshape(16).tolist())
            
        for vector_local,ref_frame_vector,point_local,ref_frame_point,vis_id in vectors:
            point=dot(ref_frame_point,homo_matrix(trans=point_local))
            self.view_objects.send_prop(vis_id,"pose",
                                        point.reshape(16).tolist())
            vector=dot(ref_frame_vector[:3,:3],vector_local)
            self.view_objects.send_prop(vis_id,"axis",self.arrow_length*vector/norm(vector))
            
    '''
    set_params is in charge of setting the parameters for the control loop
    and doing some initial processes, like initializing servers and calibration
    stuff
    '''
    def set_params(self,params):
        
        vars(self).update(params)
        print "Simulation", self.simulation
        
        if self.simulation:
            self.rel_pose_marker_finger=identity(4)
        else:
            self.rel_pose_marker_finger=rel_pose_marker_finger
        
        self.arrow_length=0.1

        #section for the viewer
        self.view_objects=Roboviewer_objects("/slider_control","/lwr/roboviewer",counter=300)
        self.goal_id=self.view_objects.create_object("frame")
        self.view_objects.send_prop(self.goal_id,"scale",[0.1,0.1,0.4])
        self.view_objects.send_prop(self.goal_id,"timeout",[-1])
        goal_vis=identity(4)
        self.view_objects.send_prop(self.goal_id,"pose", goal_vis.reshape(16).tolist())
        
        # yarp ports
        base_name="/planar_control"
                        
        #Arm/finger control initialization.
        
        arm='right'
        hand='right'
        robot='lwr'
        fingers=[0,1,2,3,4]
        arm_portbasename="/"+robot+"/"+arm
        
        self.finger=5 #ring finger right hand **changed to 5 to use PINKY**
        
        #angle format [base, proximal, distal]
        finger_initial_pos=array([0.,6.,6.]) #angles in degrees
        thumb_finger_initial_pos=array([0.1,0.,15.,15.]) #angles in degrees
        thumb_finger_pushing_pos=array([0.1,0.,10.,15.]) #angles in degrees
        finger_pushing_pos=array([10.,50.,40.]) #angles in degrees
        
        
        #initial cartesian pose. array for each arm. 
        initial_pose={
            'right': array([[-1., 0., 0., 0.8],
                            [0., 0., 1., -0.1],
                            [0., 1., 0., 1.14],
                            [0., 0., 0., 1.]]),
            'left': array([[-1., 0., 0., 0.74],
                           [0., 0., 1., -0.1],
                           [0., 1., 0., 1.14],
                           [0., 0., 0., 1.]])
            }
        
        self.goal_precision=[0.03, 3.0*pi/180.0]
        
        #initial angles in every joint
        initial_joint_pos={
            "right": array([0, -1.2, 0.7, 1.4, 0.35, -1.4, 0]),
            "left": array([0.78, 1.6, -0.4, -1.3, 1, 0.5, 0.7])
            }
        
        #setting up Arms
        self.harm=HandleArm(arm_portbasename,handlername=base_name+"/arm")
        self.harm_joint=HandleJController(arm_portbasename,handlername=base_name+"arm_joint")
        #self.lharm=HandleArm(arm_portbasename_left,handlername=base_name+"arm_left")
        #self.lharm_joint=HandleJController(arm_portbasename_left,handlername=base_name+"arm_left_joint")
        self.harm_bridge=HandleBridge(arm_portbasename,handlername=base_name+"arm_bridge",torso=False)
        #self.lharm_bridge=HandleBridge(arm_portbasename_left,handlername=base_name+"arm_left_bridge",torso=False)

        #Setting up Hands
        self.hand_right_server=Hand(self.config_hands, handedness="right",portprefix=base_name,sahand_number=0)
        #self.hand_left_server=Hand(self.config_hands, handedness="left",portprefix=base_name,sahand_number=1)
        self.hforce=Force_handle(base_name, "/torque_sim/force_out")
        hand_calib=not self.simulation

        #Start hands calibration, if required
        if hand_calib:
            print "Setting calibration offsets and factors"
            for finger,(angle_offset,torque_factor) in enumerate(zip(angle_calibration_data_right,torque_calibration_factors_right)):
                self.hand_right_server.fingers[finger].angle_offsets=angle_offset
                self.hand_right_server.fingers[finger].torque_calibration_factors=torque_factor
                print "Torque factor", self.hand_right_server.fingers[finger].torque_calibration_factors
                self.hand_right_server.update_angle_offsets(list_fingers=[finger])
            #for finger,(angle_offset,torque_factor) in enumerate(zip(angle_calibration_data_left,torque_calibration_factors_left)):
                #self.hand_left_server.fingers[finger].angle_offsets=angle_offset
                #self.hand_left_server.fingers[finger].torque_calibration_factors=torque_factor
                #self.hand_left_server.update_angle_offsets(list_fingers=[finger])
                
        time.sleep(1)

        #set hand parameters
        max_stiffness_finger=array([28.64,28.64,8.59])
        finger_max_speed=pi
        finger_speed=finger_max_speed/2.
        for i in fingers:
            self.hand_right_server.set_params(i,[finger_speed,finger_speed,finger_speed]+(max_stiffness_finger).tolist())
            #self.hand_left_server.set_params(i,[finger_speed,finger_speed,finger_speed]+(max_stiffness_finger).tolist())
        #self.hand_left_server.update_controller_params()
        self.hand_right_server.update_controller_params()
        #0.64 (soft for simulation) 40 (soft for real robot)
        max_stiffness_arm=array([240.0]*7)
        self.harm.set_stiffness(max_stiffness_arm)

        finger_pushing_pos=array([0.,2.,50.])
        #start testing commands
        finger_joint_move(self.hand_right_server, 0, array([0.0,0.,30.0,30.]), wait=-1, goal_precision=5.1)
        for i in range(4):
            finger_joint_move(self.hand_right_server, i+1, finger_pushing_pos, wait=-1, goal_precision=5.)
            #finger_joint_move(self.hand_left_server,  i+1, finger_pushing_pos, wait=-1, goal_precision=5.)
        for i in range(4):
            finger_joint_move(self.hand_right_server, i+1, finger_initial_pos, wait=-1, goal_precision=5.)
            #finger_joint_move(self.hand_left_server,  i+1, finger_initial_pos, wait=-1, goal_precision=5.)
            
            
    def end(self):
        print "end"
        
        
    def process(self):
        '''
        We need an implementation here
        '''
        print "in process"
        
def main():
    parser=optparse.OptionParser("usage: %prog [options]")
    parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
    parser.add_option("-c", "--finger_cal_data", dest="finger_cal _data_filename", default="robot_description/arcosbot/kinematics/sahand/calibration_data/finger_calibration_data.py", type="string", help="Finger calibration data filename")
    parser.add_option("-f", "--config_hands", dest="config_hands", default="../../../robot_description/arcosbot/kinematics/sahand/hands_kin.py", type="string", help="hands config filename")
    (options, args)= parser.parse_args(sys.argv[1:])
    
    config_hands=import_config(options.config_hands)
    
    control_loop=Control(15.)
    control_loop.loop(simulation=options.sim, config_hands=config_hands)
    
if __name__=="__main__":
    main()
    
