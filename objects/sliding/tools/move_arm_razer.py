#!/usr/bin/python

#made by daniel

from cmoc.objects.sliding.robot_params import force_factor
from arcospyu.dprint import dprint, eprint, iprint
from arcospyu.dprint import d as dp
import sys, time
sys.path.append("../../control/motionControl")
sys.path.append("../../tools/python/rawkey")

from numpy import array, identity, dot, concatenate, sign, cross, arccos, pi, arctan, tan, cos, sin, sign, exp, arctan2, mean
from numpy.linalg import pinv, inv, norm
import PyKDL as kdl
import yarp
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port, write_bottle_lists, yarpListToList, bottle_to_list
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.rawkey.rawkey import Raw_key, Keys
from arcospyu.control.control_loop import Controlloop
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta, my_diff, my_get_euler_zyx, rpy_to_rot_matrix
#from helpers import Roboviewer_objects, Joint_sim, Finger_sim, Hand_sim, Controlloop, kdlframe_to_narray, narray_to_kdlframe, narray_to_kdltwist, kdltwist_to_narray, rot_vector_angle, my_adddelta, my_get_euler_zyx
#from rawkey import Raw_key, Keys
from arcospyu.numeric import quat
from scipy.signal import iirfilter, lfilter, lfiltic
from cmoc.robot.hand_sim_handler import Hand
from cmoc.objects.sliding.utils import Object_pos_handle, wait_valid_object_pose, move_robot, finger_joint_move
from cmoc.robot.sim_handlers import Force_handle
from arcospyu.robot_tools.robot_trans import rot_z, homo_matrix
import optparse
#from webcam_calibration_values import rel_pose_marker_finger
#from finger_calibration_data import *
import pickle

from arcospyu.computer_graphics.ray_tracing import find_crossing_faces, most_far_away_face_from_point
#from cmoc.objects.sliding.box import vc_from_vfinger, , forces_to_no_map, force_c_to_force_o_map, vc_to_vo_linear_model, fc_to_nc_matrix
from cmoc.objects.sliding.box import forces_to_no_map, force_c_to_force_o_map, vc_to_vo_linear_model, fc_to_nc_matrix, nc_to_forces_map, vo_to_vc
from cmoc.objects.sliding.box_control import calc_finger_start_pos, calc_finger_orient, approach_until_touch
from arcospyu.signal_proc.filters import  Filter_vector, Filter
from arcospyu.numeric.lin_alg import angle_from_a_to_b, vector_saturation2
from vfclik.handlers import HandleArm, HandleJController, HandleBridge


class Control(Controlloop):
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

     def set_params(self,params):
          vars(self).update(params)
          self.rel_pose_marker_finger=identity(4)
          self.view_objects=Roboviewer_objects("/slider_control","/lwr/roboviewer",counter=300)
          self.goal_id=self.view_objects.create_object("frame")
          self.view_objects.send_prop(self.goal_id,"scale",[0.1,0.1,0.4])
          self.view_objects.send_prop(self.goal_id,"timeout",[-1])
          goal_vis=identity(4)
          self.view_objects.send_prop(self.goal_id,"pose",goal_vis.reshape(16).tolist())

          
          self.finger_pose=identity(4)
          
          arm='right'
          hand='right'
          robot='lwr'
          fingers=[0,1,2,3]
          arm_portbasename="/"+robot+"/"+arm
          arm_portbasename_left="/"+robot+"/left"
          
          self.finger=3 #ring finger right hand
          finger_initial_pos=array([0.,6.,6.]) #angles in degrees
          thumb_finger_initial_pos=array([60.,0.,15.,15.]) #angles in degrees
          thumb_finger_pushing_pos=array([0.1,0.,10.,10.]) #angles in degrees
          finger_pushing_pos=array([0.,5.,40.]) #angles in degrees
          
          self.goal_precision=[0.03, 3.0*pi/180.0]
          
          initial_joint_pos={
               "right": array([0, -1.2, 0.7, 1.4, 0.35, -1.4, 0]),
               "left": array([0.78, 1.6, -0.4, -1.3, 1, 0.5, 0.7])
          }

          cam_away_joint_pos={
               "right": array([60., -35, 45, 45, 0., -90, 0])*pi/180.,
               "left": array([-30., 45, -45, -70, -90., 90, 45])*pi/180.
          }
          
          
          #setting up
          base_name="/planar_control"
          self.harm=HandleArm(arm_portbasename,handlername=base_name+"/arm")
          self.harm_joint=HandleJController(arm_portbasename,handlername=base_name+"arm_joint")
          self.harm_bridge=HandleBridge(arm_portbasename,handlername=base_name+"arm_bridge",torso=False)
          
          self.hand_right_server=Hand(self.config_hands, handedness="right",portprefix=base_name,sahand_number=0)
          
          #self.hforce=Force_handle(base_name, "/torque_sim/force_out")
          hand_calib=not self.simulation
          #if False:
          if hand_calib:
               print "Setting calibration offsets and factors"
               for finger,(angle_offset,torque_factor) in enumerate(zip(angle_calibration_data_right,torque_calibration_factors_right)):
                    self.hand_right_server.fingers[finger].angle_offsets=angle_offset
                    self.hand_right_server.fingers[finger].torque_calibration_factors=torque_factor
                    print "Torque factor", self.hand_right_server.fingers[finger].torque_calibration_factors
                    self.hand_right_server.update_angle_offsets(list_fingers=[finger])
               for finger,(angle_offset,torque_factor) in enumerate(zip(angle_calibration_data_left,torque_calibration_factors_left)):
                    pass
                    

          time.sleep(1)
          
          max_stiffness_finger=array([28.64,28.64,8.59])
          finger_max_speed=pi
          finger_speed=finger_max_speed/2.
          for i in fingers:
               self.hand_right_server.set_params(i,[finger_speed,finger_speed,finger_speed]+(max_stiffness_finger).tolist())
               
          
          self.hand_right_server.update_controller_params()
          #0.64 (soft for simulation) 40 (soft for real robot)
          max_stiffness_arm=array([240.0]*7)
          self.harm.set_stiffness(max_stiffness_arm)

          self.harm_bridge.cartesian_controller()
          
          #comm with razer
          self.raw_key=Raw_key()
          base_name="/xfinger_feeder"
          self.xfinger_in_port=new_port(base_name+"/xfinger:in", "in", "/razer", timeout=2)
          
          self.pose_right=identity(4)
                    
          self.offset_rot_matrix_scaled=array([[1., 0., 0.],
                                               [0.,-1., 0.],
                                               [0., 0.,-1.]])
          
          
          self.right_pos_frame=array([[0. ,0. ,1.],
                                      [1. ,0. ,0.],
                                      [0. ,1. ,0.]])
                    
          
          self.data=array(readListPort(self.xfinger_in_port, True))
          self.data_right=self.data[:12]
                    
          self.pos_right=dot( self.right_pos_frame, self.data_right[:3])
                    
          self.initial_razer_pos_right=self.pos_right

                    
          self.rot_data_right=array([[self.data_right[3], self.data_right[4], self.data_right[5]],
                                     [self.data_right[6], self.data_right[7], self.data_right[8]],
                                     [self.data_right[9], self.data_right[10], self.data_right[11]]])
          
          self.req_right=array([[0.0, 0.0, 1.0],
                                [0.0, 1.0, 0.0],
                                [1.0, 0.0, 0.0]])
          
          
          self.rot_right=self.req_right
          
          self.init_inv_right=inv(self.rot_data_right)
                    
          
          self.pose_right=array([[self.rot_right[0,0],self.rot_right[0,1],self.rot_right[0,2],0.7],
                                 [self.rot_right[1,0],self.rot_right[1,1],self.rot_right[1,2],0.2],
                                 [self.rot_right[1,0],self.rot_right[1,1],self.rot_right[2,2],0.9],
                                 [0.,0.,0.,1.]])
          

          print "Waiting to get initial pose"
          time.sleep(5)
          initial_right_pose = array(self.harm.getPose())
          initial_right_pose = initial_right_pose.reshape(4,4)
          # FIXME: DEBUG
          print "INITIAL RAZER POS: ", self.initial_razer_pos_right
          print "INITIAL ROBOT POS:  ", initial_right_pose
          print "Setting initial pose..."
          #self.initial_pos_right = initial_right_pose

          ###raw_input()
          
                    
          move_robot(self.hand_right_server, self.finger, self.harm, initial_right_pose, [0.03, 5.*pi/180.], wait=0.)
          self.initial_pos_right = [initial_right_pose[0,3],
                                    initial_right_pose[1,3],
                                    initial_right_pose[2,3]]
          self.initial_rot_right = initial_right_pose[:3,:3]
          print "initial position done"
          time.sleep(5)
          #sometimes hapens that data is inverted, reason not known yet.
          self.sgn=1
          #self.roll0=my_get_euler_zyx(self.rot_left)

     def process(self):
          chars=self.raw_key.get_num_chars()
          if len(chars)>0:
            if chars==Keys.m:
                self.sgn*=-1
          self.data=array(readListPort(self.xfinger_in_port, True))
          self.data_right=self.data[:12]          

          self.pos_right=dot( self.sgn*self.right_pos_frame, self.data_right[:3])          
          
          self.rot_data_right=array([[self.data_right[3], self.data_right[4], self.data_right[5]],
                                     [self.data_right[6], self.data_right[7], self.data_right[8]],
                                     [self.data_right[9], self.data_right[10], self.data_right[11]]])

          self.rot_right=dot(self.req_right, inv(dot(self.init_inv_right,self.rot_data_right)))

          # HACK: Set fixed orientation
          self.rot_right=array([[-1.,0.,0.],
                                [0.,1.,0.],
                                [0.,0.,-1.]])
          


          #[roll, pitch, yaw]=my_get_euler_zyx(self.rot_right)
          #print "ROLL, PITCH, YAW", [roll, pitch, yaw], "RIGHT"

          self.pose_right=array([[self.initial_rot_right[0,0], self.initial_rot_right[0,1], self.initial_rot_right[0,2],
                                  (0.001*(self.pos_right[0]-self.initial_razer_pos_right[0])+self.initial_pos_right[0])],
                                 [self.initial_rot_right[1,0], self.initial_rot_right[1,1], self.initial_rot_right[1,2],
                                  (0.001*(self.pos_right[1]-self.initial_razer_pos_right[1])+self.initial_pos_right[1])],
                                 [self.initial_rot_right[1,0], self.initial_rot_right[1,1], self.initial_rot_right[2,2],
                                  (0.001*(self.pos_right[2]-self.initial_razer_pos_right[2])+self.initial_pos_right[2])],
                                 [0.0,0.0,0.0,1.0]])

          print "pos_right    :", self.pos_right
          print "initial robot:", [self.initial_pos_right[0], self.initial_pos_right[1], self.initial_pos_right[2]]
          print "initial razer:", self.initial_razer_pos_right

          move_robot(self.hand_right_server, self.finger, self.harm, self.pose_right, [0.03, 5.*pi/180.], wait=0.)
          

def main():
    parser=optparse.OptionParser("usage: %prog [options]")
    parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
    parser.add_option("-c", "--finger_cal_data", dest="finger_cal _data_filename", default="~/local/src/robot_description/tum-rosie/kinematics/sahand/calibration_data/finger_calibration_data.py", type="string", help="Finger calibration data filename")
    parser.add_option("-f", "--config_hands", dest="config_hands", default="~/local/src/robot_descriptions/tum-rosie/kinematics/sahand/hands_kin.py", type="string", help="hands config filename")
    parser.add_option("-o", "--object_params_file", dest="object_params_filename", default="~/local/src/cmoc/objects/sliding/objects/object_params.py", type="string", help="Object parameters file")
    (options, args)= parser.parse_args(sys.argv[1:])
    from arcospyu.config_parser.config_parser import import_config
    #fcd=import_config(options.finger_cal_data_filename)
    config_hands=import_config(options.config_hands)
    object_params=import_config(options.object_params_filename)

    control_loop=Control(15.)
    control_loop.loop(simulation=options.sim, config_hands=config_hands, object_params=object_params)


def from_euler_to_matrix(x,y,z):
     matrix_x=array([[1.0,0.,0.],
                     [0.,cos(x), sin(x)],
                     [0.,-sin(x), cos(x)]])
     matrix_y=array([[cos(y),0.,sin(y)],
                     [0.,1., 0.],
                     [-sin(y),0., cos(y)]])
     matrix_z=array([[cos(z),sin(z),0.],
                     [-sin(z),cos(z), 0.],
                     [0.,0., 1.]])
     return dot(matrix_z, dot(matrix_y,matrix_x))
     #return dot(matrix_x, dot(matrix_y,matrix_z))

if __name__=="__main__":
    main()
        
