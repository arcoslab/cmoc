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

from robot_params import force_factor
from arcospyu.dprint import dprint, eprint, iprint
from arcospyu.dprint import d as dp
import sys, time, LS
sys.path.append("../../control/motionControl")
sys.path.append("../../tools/python/numeric/")
sys.path.append("../../robots/kimp/hand/hand_cartesian/")
sys.path.append("../../tools/python/computer_graphics")
sys.path.append("../../perception/webcam_baseframe_calibration/")
sys.path.append("../../../robot_descriptions/tum-rosie/kinematics/sahand/calibration_data/")

#sys.path.append("../../tools/python/rawkey")
from arcospyu.computer_graphics.pluecker_test import polygon_pluecker_test, triangle_ray_intersection, plane_ray_intersection
from numpy import array, identity, dot, concatenate, sign, cross, arccos, pi, arctan, tan, cos, sin, sign, exp, arctan2, mean
from numpy.linalg import pinv, inv, norm
import PyKDL as kdl
import yarp
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port, write_bottle_lists, yarpListToList, bottle_to_list
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.control.control_loop import Controlloop
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta, my_diff, my_get_euler_zyx
#from helpers import Roboviewer_objects, Joint_sim, Finger_sim, Hand_sim, Controlloop, kdlframe_to_narray, narray_to_kdlframe, narray_to_kdltwist, kdltwist_to_narray, rot_vector_angle, my_adddelta, my_get_euler_zyx
#from rawkey import Raw_key, Keys
from arcospyu.numeric import quat
import planar_sliding as ps
from scipy.signal import iirfilter, lfilter, lfiltic
from cmoc.robot.hand_sim_handler import Hand
from utils import Object_pos_handle, Force_handle, Filter_vector, Filter, wait_valid_object_pose, move_robot, rot_z, homo_matrix, finger_joint_move, find_force_noise_levels
import optparse
from object_params import Object_params
from webcam_calibration_values import rel_pose_marker_finger
from finger_calibration_data import *
import pickle




def find_crossing_faces(vector,vector_pos, vertices_faces, planes):
    crossing_faces={}
    for face in vertices_faces:
        temp=array(polygon_pluecker_test(vertices_faces[face],
                                        vector_pos,vector/norm(vector)))
#        print "pluecker test", face, temp
        crossing_face=False
        if all(temp>0.) or all(temp<0.):
#            print "Finger will cross face:", face
            crossing_face=True
        elif all(temp>=0.) or all(temp<=0.):
#            print "not positive"
            #if the zeros are only two and are next to each other then it
            #crosses the face
            i=0
            j=0
            while i<len(temp)-j:
                num=temp[i]
#                print "num", num
                if num==0. and not crossing_face:
#                    print "zero"
                    crossing_face=True #we suppose beforehand
                    i+=1
                    if i==len(temp):
                        num=temp[0]
                    else:
                        num=temp[i]
                    i+=1
                    if num==0.:
 #                       print "crossing face, two zeros"
                         pass
                    else:
 #                       print "crossing face, one zero"
                        if i==2:
 #                           print "Don't check for last item"
                            j=1
                elif num==0. and crossing_face:
                    crossing_face=False #we rectify wrong previews assumption
                    break
                else:
                    i+=1
        if crossing_face:
            #calculate where is crossing the face
            vertices=vertices_faces[face]
            #intersection=triangle_ray_intersection(vertices[0],vertices[1],
            #                                       vertices[2],vector_pos,vector)
            #intersection=plane_ray_intersection(array(planes[face][0]),
            #                                    array(planes[face][1]),
            #                                    vector_pos,vector)
            intersection=ps.plane_ray_intersec2( array(planes[face][0]),
                                              array(planes[face][1]),
                                              vector,vector_pos)
            intersection[0]*=norm(vector)
            print "intersection", intersection
            crossing_faces[face]=[vertices_faces[face],temp,intersection]
    print "crossing_faces", crossing_faces
    return(crossing_faces)

def most_far_away_face_from_point(crossing_faces_local, crossing_faces_reference_frame,  point):
    dist=-1
    final_face=-1
    for face in crossing_faces_local:
        touch_point_local=crossing_faces_local[face][2][1]
        touch_point=dot(crossing_faces_reference_frame,homo_matrix(trans=touch_point_local))[:3,3]
        print "Touch point", face, touch_point
        cur_dist=norm(touch_point-point)
        if dist<0.:
            dist=cur_dist
            final_touch_point=touch_point
            final_face=face
        else:
            if cur_dist>dist:
                dist=cur_dist
                final_touch_point=touch_point
                final_face=face
    return(final_touch_point,final_face)

def calc_finger_start_pos(finger_offset, touch_point, object_pos, touch_face, box_planes):
    touch_point_local=dot(inv(object_pos), homo_matrix(trans=touch_point))[:3,3]
    print "touch local", touch_point_local, "plane", box_planes[touch_face][0]
    return(dot(object_pos,homo_matrix(trans=touch_point_local+array(box_planes[touch_face][0])*finger_offset))[:3,3])

def calc_finger_orient(finger_base_orient, table_normal, object_pos, face):
    if face==0:
        angle=0.
    elif face==1:
        angle=180*pi/180.0
    elif face==2:
        angle=90*pi/180.0
    elif face==3:
        angle=-90*pi/180.0
    return(dot(object_pos[:3,:3],dot(rot_vector_angle(table_normal, angle), finger_base_orient)))

def homo_matrix(rot_m=identity(3),trans=array([0.]*3)):
    m=identity(4)
    m[:3,:3]=rot_m
    m[:3,3]=trans
    return(m)


def angle_from_a_to_b(a,b):
    s=sign(cross(a,b))
    #print "s", s
    if s==0.:
        s=1.
    return(arccos(min(dot(a,b),1.))*s)

def approach_until_touch(hand_handle, finger, arm_handle, force_handle, touch_point, force_threshold, goal_precision, speed, start_pose, sim, extra_tool=identity(4)):

    filters_order=3
    filters_freq=0.2
    max_noise_force, max_noise_norm_force=find_force_noise_levels(force_handle, finger, filter_order=filters_order, filter_freq=filters_freq, measurements=50)
    print "noise levels", max_noise_force, max_noise_norm_force
    #if False:
    if not sim:
        force_threshold=max_noise_norm_force*1.3
    raw_input()

    force=array([0.]*3)
    init_time=time.time()
    old_time=init_time
    first=True
    force_filter=Filter_vector(dim=3, order=filters_order, freq=filters_freq)

    while norm(force)<force_threshold:
        cur_time=time.time()
        period=cur_time-old_time
        old_time=cur_time
        data=force_handle.get_data()[finger]
        print "data", data, force_factor
        force=array(data[-3:])*force_factor
        force=force_filter.filter(force)
        eprint("FORCE: ", norm(force), "Threshold", force_threshold)
        cur_pose=quat.to_matrix_fixed(data[3:7], r=data[:3])
        if first:
            vel_vector=(touch_point[:2]-start_pose[:2,3])
            vel_vector/=norm(vel_vector)
            first=False
            pose=start_pose
        step=vel_vector*speed*period
        pose[:2,3]+=step
        move_robot(hand_handle, finger, arm_handle, pose, goal_precision, wait=0.1, extra_tool=extra_tool)
        print "Pos", cur_pose, "Force", force
    #stop robot
    #cur_pose=quat.to_matrix_fixed(data[3:7], r=data[:3])
    #cur_pose[:3,:3]=pose[:3,:3]
    #move_robot(hand_handle, finger, arm_handle, cur_pose, goal_precision, extra_tool=extra_tool)

def vo_to_vc(vo_local,contact_pos_local,fmax,mmax):
    A=ps.forces_to_no_map(fmax,mmax)
    B_inv=ps.nc_to_forces_map(fmax,mmax,contact_pos_local)
    C=ps.force_c_to_force_o_map(contact_pos_local)

    #system calculation
    P=dot(A,dot(C,B_inv))
    P_inv=pinv(P)
    vc_local=dot(P_inv,vo_local)/mmax**2
    return(vc_local)

def vector_saturation(vector_min, vector_max, vector):
    angle_min_max=arccos(dot(vector_min,vector_max)/(norm(vector_min)*norm(vector_max)))
    angle_max=arccos(dot(vector,vector_max)/(norm(vector)*norm(vector_max)))
    angle_min=arccos(dot(vector,vector_min)/(norm(vector)*norm(vector_min)))
    print "angles", angle_min_max, angle_max, angle_min
    if (angle_max > angle_min_max) or (angle_min > angle_min_max):
        print "limiting"
        if angle_max < angle_min:
            return(vector_max)
        else:
            return(vector_min)
    return(vector)

def vector_saturation2(vector1, vector2, vector_test):
    #saturates the direction
    vector1n=vector1/norm(vector1)
    vector2n=vector2/norm(vector2)
    if norm(vector_test) > 0.00000001:
        vector_testn=vector_test/norm(vector_test)
    else:
        vector_testn=array([1.0,0.,0.])
    #print "vector1n", vector1n, "vector2n", vector2n, "vector test", vector_testn
    proj_vector1n=dot(vector_testn,vector1n)
    proj_vector2n=dot(vector_testn,vector2n)
    #if proj_vector1n<0. or proj_vector2n<0.:
    #    print "input vector not in the expected direction", proj_vector1n, proj_vector2n
        #return(array([0.,0.,0.]))
    if proj_vector1n<=proj_vector2n:
        # test is done using vector1n
        proj_vector2n_in_1n=dot(vector1n,vector2n)
        if proj_vector1n<proj_vector2n_in_1n:
            print "Vector outside vector2n", vector2n
            return(vector2n)
        else:
            return(vector_testn)
    else:
        #test using vector2n
        proj_vector1n_in_2n=dot(vector2n,vector1n)
        if proj_vector2n<proj_vector1n_in_2n:
            print "Vector outside vector1n", vector1n
            return(vector1n)
        else:
            return(vector_testn)

def read_finger(yarp_port, blocking, finger):
    result=yarp_port.read(blocking)
    bottle=result.get(finger).asList()
    #print "Result", bottle
    trans=array(map(yarp.Value.asDouble,map(bottle.get, range(3))))
    rot=array(map(yarp.Value.asDouble,map(bottle.get, range(3,7))))
    force=array(map(yarp.Value.asDouble,map(bottle.get, range(7,10))))
    return(trans,rot,force)

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
        #store lists
        dp.level=dp.INFO
        dprint("Starting control parameters (Initialization)")
        self.first=True
        self.update_finger_cart=True
        self.store_ref=[]
        self.store_xo=[]
        self.store_side=[]
        self.store_side_rot=[]
        self.store_box_pose=[]
        self.store_box_pose2=[]
        self.store_box_pos_filtered=[]
        self.store_box_orient_filtered=[]
        self.store_force_data=[]
        self.store_finger_pose_global=[]
        self.store_finger_pose=[]
        self.store_rxy_global=[]
        self.store_rxy=[]
        self.store_rxy_angle=[]
        self.store_sliding_angle=[]
        self.store_stop_angle=[]
        self.store_event=[]
        self.store_mu=[]
        self.store_mu_max=[]
        self.store_mu_min=[]
        self.store_ncmax_loc=[]
        self.store_ncmin_loc=[]
        self.store_ncmax=[]
        self.store_ncmin=[]
        self.store_box_rot_finger=[]
        self.store_d=[]
        self.store_b_e=[]
        self.store_n=[]
        self.store_phi_a=[]
        self.store_phi=[]
        self.store_phi_goal=[]
        self.store_e=[]
        self.store_r=[]
        self.store_a=[]
        self.store_b=[]
        self.store_vtest1=[]
        self.store_d_low=[]
        self.store_stop_angle_ctrl=[]
        self.store_vtest2_raw=[]
        self.store_vtest2=[]
        self.store_d_extra=[]
        self.store_control_e=[]
        self.store_control_phi=[]
        self.store_control_e_scaled=[]
        self.store_control_phi_scaled=[]
        self.store_vc_local4=[]
        self.store_vc_local2=[]
        self.store_vc=[]
        self.store_vc_sat_temp=[]
        self.store_cur_time=[]
        self.store_finger_pose_cmd=[]
        self.store_finger_pose2=[]

        print "Params", params
        vars(self).update(params)
        print "Simulation", self.simulation
        #if True:
        if self.simulation:
            self.rel_pose_marker_finger=identity(4)
        else:
        #if True:
            self.rel_pose_marker_finger=rel_pose_marker_finger
        #raw_input()
        self.arrow_length=0.1
        #viewer
        self.view_objects=Roboviewer_objects("/slider_control","/lwr/roboviewer",counter=300)
        self.goal_id=self.view_objects.create_object("frame")
        self.view_objects.send_prop(self.goal_id,"scale",[0.1,0.1,0.4])
        self.view_objects.send_prop(self.goal_id,"timeout",[-1])
        #self.view_objects.send_prop(self.goal_id,"color",[0,1,0])
        goal_vis=identity(4)
        self.view_objects.send_prop(self.goal_id,"pose",
                                    goal_vis.reshape(16).tolist())

        # self.ncmin_id=self.view_objects.create_object("arrow")
        # self.view_objects.send_prop(self.ncmin_id,"scale",[0.05,0.05,0.0125])
        # ncmin_id_offset=identity(4)
        # ncmin_id_offset[:3,3]=array([0.,0.,-1.])
        # self.view_objects.send_prop(self.ncmin_id,"pose_offset",ncmin_id_offset.reshape(16).tolist())
        # self.view_objects.send_prop(self.ncmin_id,"color",[0,1,1])
        # self.view_objects.send_prop(self.ncmin_id,"timeout",[-1])

        # self.ncmax_id=self.view_objects.create_object("arrow")
        # self.view_objects.send_prop(self.ncmax_id,"scale",[0.05,0.05,0.0125])
        # ncmax_id_offset=identity(4)
        # ncmax_id_offset[:3,3]=array([0.,0.,-1.])
        # self.view_objects.send_prop(self.ncmax_id,"pose_offset",ncmax_id_offset.reshape(16).tolist())
        # self.view_objects.send_prop(self.ncmax_id,"color",[0,1,1])
        # self.view_objects.send_prop(self.ncmax_id,"timeout",[-1])

        #model parameters
        self.obj_par=Object_params()

        self.finger_object_face=1
        self.table_object_face=5

        self.box_sides_angle=arctan(self.obj_par.box_dim[1]/self.obj_par.box_dim[0])

        # yarp ports
        base_name="/planar_control"
        #self.finger_pose_out_port=new_port(base_name+"/xfinger:out",
        #                          "out", "/slider_sim/xfinger:in")
        self.finger_pose=identity(4)
        self.finger_sign=1

        self.finger_pose[:3,3]=array([1*self.obj_par.box_dim[0]/2.,-0.8*self.obj_par.box_dim[1]/2,0.])
        self.finger_pose[:3,3]=array([1*self.obj_par.box_dim[0]/2.,-0.*self.obj_par.box_dim[1]/2,0.])
#        self.finger_pose[:3,3]=array([0*self.obj_par.box_dim[0]/2.,self.obj_par.box_dim[1]/2,0.])
#        self.finger_pose[:3,3]=array([-1*self.obj_par.box_dim[0]/2.,0.1*self.obj_par.box_dim[1]/2,0.])
#        self.finger_pose[:3,3]=array([0*self.obj_par.box_dim[0]/2.,-self.obj_par.box_dim[1]/2,0.])


        #Goal #Adquire goal from outside
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([.65, 0.05, 1.1])
        angle=-25.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 3 #Adquire goal from outside
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([1.2, -0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 2  #Adquire goal from outside
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([0.7, -0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 4. Testing starting away from corners
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([0.9, -0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 5. Testing starting away from corners
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([0.7, 0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 7. Goal push y_pos test real robot
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([1.1, -0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 8. Goal general test real robot
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([0.75, -0.15, 1.1])
        angle=25.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()


        #Goal 6. Goal general test real robot
        self.xo_ref=identity(4)
        self.xo_ref[:3,3]=array([0.7, -0.1, 1.1])
        angle=-0.*pi/180.0
        self.table_normal=array([0.,0.,1.])
        self.xo_ref[:3,:3]=rot_vector_angle(self.table_normal,angle)
        print "xo ref", self.xo_ref
        #raw_input()

        self.view_objects.send_prop(self.goal_id,"pose",
                                    self.xo_ref.reshape(16).tolist())

        #Arm/finger control initialization.
        from vfclik.handlers import HandleArm, HandleJController, HandleBridge
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

        self.base_pushing_orient=array([[0.,0.,-1.], # Initial finger orientation. Or base finger transformation
                              [-1.,0.,0.],
                              [0.,1.,0.]])
        #goal_precision=[0.01, 0.5*pi/180.0]
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
        self.harm=HandleArm(arm_portbasename,handlername=base_name+"/arm")
        self.harm_joint=HandleJController(arm_portbasename,handlername=base_name+"arm_joint")
        self.lharm=HandleArm(arm_portbasename_left,handlername=base_name+"arm_left")
        self.lharm_joint=HandleJController(arm_portbasename_left,handlername=base_name+"arm_left_joint")
        self.harm_bridge=HandleBridge(arm_portbasename,handlername=base_name+"arm_bridge",torso=False)
        self.lharm_bridge=HandleBridge(arm_portbasename_left,handlername=base_name+"arm_left_bridge",torso=False)
        self.hand_right_server=Hand(self.config_hands, handedness="right",portprefix=base_name,sahand_number=0)
        self.hand_left_server=Hand(self.config_hands, handedness="left",portprefix=base_name,sahand_number=1)
        self.hforce=Force_handle(base_name, "/torque_sim/force_out")
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
                self.hand_left_server.fingers[finger].angle_offsets=angle_offset
                self.hand_left_server.fingers[finger].torque_calibration_factors=torque_factor
                self.hand_left_server.update_angle_offsets(list_fingers=[finger])

        time.sleep(1)

        max_stiffness_finger=array([28.64,28.64,8.59])
        finger_max_speed=pi
        finger_speed=finger_max_speed/2.
        for i in fingers:
            self.hand_right_server.set_params(i,[finger_speed,finger_speed,finger_speed]+(max_stiffness_finger).tolist())
            self.hand_left_server.set_params(i,[finger_speed,finger_speed,finger_speed]+(max_stiffness_finger).tolist())
        self.hand_left_server.update_controller_params()
        self.hand_right_server.update_controller_params()
        #0.64 (soft for simulation) 40 (soft for real robot)
        max_stiffness_arm=array([240.0]*7)
        #max_stiffness_arm=array([1000.0]*7)
        self.harm.set_stiffness(max_stiffness_arm)

        #Initial finger joint pos
        finger_joint_move(self.hand_right_server, 0, thumb_finger_pushing_pos, wait=-1, goal_precision=5.)
        finger_joint_move(self.hand_right_server, self.finger, finger_pushing_pos, wait=-1, goal_precision=5.)

        #Send arm to away for looking for object position.
        self.harm_bridge.joint_controller()
        self.harm_joint.set_ref_js(cam_away_joint_pos["right"],wait=-1,goal_precision=[0.1]*7)
        self.lharm_bridge.joint_controller()
        self.lharm_joint.set_ref_js(cam_away_joint_pos["left"],wait=-1,goal_precision=[0.1]*7)
        #raw_input("Put object in scene. Check that it gets correctly detect")

        sim_model=self.simulation # When using model simulation, otherwise object position from marker tracking
        self.object_pos_handle=Object_pos_handle(base_name, simulation=sim_model)

        cont="n"
        while cont!="y":
            #getting current box position (TODO: make this more stable, read multiple times, wait until is stable, wait for aceptance from user)
            print "Getting initial object position"
            self.xo=self.object_pos_handle.get_object_pos()
            print "Box global pose", self.xo
            self.xo_ref[2,3]=self.xo[2,3]
            self.box_pose=dot(inv(self.xo_ref), self.xo)
            cont=raw_input("If right, press \"y\"")
            #self.box_pose=identity(4)
            #self.box_pose[:3,3]=array([3.,2,0.])

        #Send finger goal to somewhere a bit away from object.
        self.goal_vector=self.xo[:3,3]-self.xo_ref[:3,3]
        print "Goal vector", self.goal_vector
        self.goal_vector_object_local=dot(inv(self.xo[:3,:3]),self.goal_vector)
        print "Goal vector object local", self.goal_vector_object_local
        crossing_faces_local=find_crossing_faces(self.goal_vector_object_local, array([0.,0.,0]), self.obj_par.box_vertices, self.obj_par.box_planes)
        touch_point,self.touch_face=most_far_away_face_from_point(crossing_faces_local, self.xo, self.xo_ref[:3,3])
        print "Touch point global", touch_point, self.touch_face
        raw_input()
        #eliminate touch point that is too near to corners
        no_touch_angle=0.8*self.box_sides_angle
        rxy_rel=touch_point-self.xo[:3,3]
        rxy=dot(inv(self.xo[:3,:3]),rxy_rel)
        rxy_angle=arctan2(rxy[1],rxy[0])
        print "rxy", rxy, "rxy angle", rxy_angle*180./pi, "box_side angle" , self.box_sides_angle*180./pi
        print "box angle limit", no_touch_angle*180./pi
        if (rxy_angle>=no_touch_angle and rxy_angle<=self.box_sides_angle) or (rxy_angle-pi/2.>=no_touch_angle and rxy_angle-pi/2.<=self.box_sides_angle) or (rxy_angle+pi/2.>=no_touch_angle and rxy_angle+pi/2.<=self.box_sides_angle) or (rxy_angle+pi>=no_touch_angle and rxy_angle+pi<=self.box_sides_angle):
            print "Moving touch point away from corner, negative rotation"
            touch_point=dot(self.xo, homo_matrix(trans=(dot(rot_z(-(self.box_sides_angle-no_touch_angle)),rxy))))[:3,3]
        if (rxy_angle<=-no_touch_angle and rxy_angle>=-self.box_sides_angle) or (rxy_angle-pi/2.<=-no_touch_angle and rxy_angle-pi/2.>=-self.box_sides_angle) or (rxy_angle+pi/2.<=-no_touch_angle and rxy_angle+pi/2.>=-self.box_sides_angle) or (rxy_angle-pi<=-no_touch_angle and rxy_angle-pi>=-self.box_sides_angle):
            print "Moving touch point away from corner, positive rotation"
            touch_point=dot(self.xo, homo_matrix(trans=(dot(rot_z(self.box_sides_angle-no_touch_angle),rxy))))[:3,3]

        raw_input()
        self.finger_start_offset=identity(4)
        #self.finger_start_offset[:2,3]=array([0.03,0.,0.])
        avoid_col, finger_start_pose, new_touch_point=self.finger_away_touch_point(touch_point)
        dprint("New touch point", new_touch_point)
        dprint("finger start pose", finger_start_pose)
        dprint("avoid col", avoid_col)
        #self.view_objects.send_prop(self.goal_id,"pose",
        #                            avoid_col.reshape(16).tolist())
        self.harm_bridge.cartesian_controller()
        move_robot(self.hand_right_server, self.finger, self.harm, avoid_col, [0.03, 5.*pi/180.], extra_tool=self.rel_pose_marker_finger)
        dprint("avoid colision position")
        raw_input()
        #move finger to start position
        move_robot(self.hand_right_server, self.finger, self.harm, finger_start_pose, [0.03, 5.*pi/180.], wait=0., extra_tool=self.rel_pose_marker_finger)
        move_robot(self.hand_right_server, self.finger, self.harm, finger_start_pose, [0.03, 5.*pi/180.],extra_tool=self.rel_pose_marker_finger)
        #time.sleep(1)
        #raw_input()
        #finger_offset=0.05
        #self.finger_start_pos=calc_finger_start_pos(finger_offset, touch_point, self.xo, self.touch_face, self.obj_par.box_planes)
        #print "Finger Start position", self.finger_start_pos
        #self.finger_start_orient=calc_finger_orient(self.base_pushing_orient, self.table_normal, self.xo, self.touch_face)
        #print "Finger start orient", self.finger_start_orient
        #temp=identity(4)
        #temp[:3,:3]=self.finger_start_orient
        #temp[:3,3]=self.finger_start_pos
        #box_z_offset=0.04
        #temp[2,3]+=box_z_offset #box z offset for pushing

        #Move finger to start position but on top of object (to avoid collisions)
        #avoid_col=array(temp)
        #avoid_col[2,3]+=self.obj_par.box_dim[2]
        #self.harm_bridge.cartesian_controller()
        #move_robot(self.hand_right_server, self.finger, self.harm, avoid_col, [0.03, 5.*pi/180.])

        #move finger to start position
        #move_robot(self.hand_right_server, self.finger, self.harm, temp, [0.03, 5.*pi/180.], wait=0.)
        #move_robot(self.hand_right_server, self.finger, self.harm, temp, [0.03, 5.*pi/180.])
        #time.sleep(1)

        eprint("Approach until touch")
        #Approach object in straight line until it touches the object (feels some force) then stop. In this point we are ready for control.
        force_threshold=0.01 #smaller than static friction force
        finger_approach_speed=0.003
        approach_until_touch(self.hand_right_server, self.finger, self.harm, self.hforce, new_touch_point, force_threshold, self.goal_precision, finger_approach_speed, finger_start_pose, self.simulation, extra_tool=self.rel_pose_marker_finger)
        eprint("Done")
        raw_input()

        #getting finger position
        force_data=self.hforce.get_data_finger(self.finger)
        force_data*=force_factor
        self.finger_pose_global=quat.to_matrix_fixed(force_data[3:7], r=force_data[:3])
        print "Global Finger_pose from robot", self.finger_pose_global[:3,3]

        #getting current object position
        self.xo=self.object_pos_handle.get_object_pos()
        self.box_pose=dot(inv(self.xo_ref), self.xo) #with respect to goal
        print "Global box pose", self.xo[:3,3]
        print "Box pose with respect to goal", self.box_pose[:3,3]

        #finger position with respect to box
        self.finger_pose=dot(inv(self.xo_ref), self.finger_pose_global)
        print "finger position with respect to goal", self.finger_pose[:3,3]

        raw_input("ready to control, press enter")



        #self.fc_in_port=new_port(base_name+"/ffinger:in","in", "/slider_sim/ffinger:out", timeout=10.)

        self.vo=array([0.01,0.01,0.01]) #???

        self.last_time=time.time()
        self.d_last=None
        self.stop=False

        #for control output filtering
        self.vtest2_buff=array([0.]*6)
        self.vtest2_buff_pos=0

        #filter for finger control output (To simulate arm)
        self.enable_filter=False
        order=5
        freq=0.3
        self.finger_pose_out=array(self.finger_pose)
        print "Filter", self.finger_pose_out[0,3]
        self.filter_x=Filter(order=order,freq=freq,y=array([self.finger_pose_out[0,3]]*order),x=array([self.finger_pose_out[0,3]]*order))
        self.filter_y=Filter(order=order,freq=freq,y=array([self.finger_pose_out[1,3]]*order),x=array([self.finger_pose_out[1,3]]*order))
        self.control_ei=0.

        #object orientation and position error
        self.object_pos_filter=Filter_vector(order=5,freq=0.1)
        self.object_orient_filter=Filter_vector(order=5,freq=0.1)

        #max distance between object position measurements to decide to ignore new measurement
        #useful for eliminating bad object detections from the marker detection system
        self.max_object_jump_dist=0.03

        self.cur_time=time.time()



    def end(self):
        # Move finger away from object
        # Move finger to higher position than object
        # Quit, or restart
        avoid_col, finger_start_pose, new_touch_point=self.finger_away_touch_point(self.finger_pose2[:3,3], z_offset=False)
        print "cur_finger pose (with rel marker pose)", self.finger_pose_global
        print "cur cmd finger pose" , self.finger_pose2[:3,3]
        print "finger_start_pose" , finger_start_pose
        #raw_input("Finished")
        self.harm_bridge.cartesian_controller()
        #move finger to start position
        move_robot(self.hand_right_server, self.finger, self.harm, finger_start_pose, [0.01, 5.*pi/180.], extra_tool=self.rel_pose_marker_finger)
        #raw_input("go up")
        move_robot(self.hand_right_server, self.finger, self.harm, avoid_col, [0.03, 5.*pi/180.], extra_tool=self.rel_pose_marker_finger)
        control_filename="control"+"_"+time.strftime("%d.%m.%Y_%H.%M.%S")+".dat"
        control_file=open(control_filename,"wb")
        store={
            "ref": self.store_ref,
            "xo": self.store_xo,
            "side": self.store_side,
            "side_rot": self.store_side_rot,
        "box_pose": self.store_box_pose,
        "box_pose2": self.store_box_pose2,
        "box_pos_filtered": self.store_box_pos_filtered,
        "box_orient_filtered": self.store_box_orient_filtered,
        "force_data": self.store_force_data,
        "finger_pose_global": self.store_finger_pose_global,
        "finger_pose": self.store_finger_pose,
        "rxy_global": self.store_rxy_global,
        "rxy": self.store_rxy,
        "rxy_angle": self.store_rxy_angle,
        "sliding_angle": self.store_sliding_angle,
        "stop_angle": self.store_stop_angle,
        "event": self.store_event,
        "mu": self.store_mu,
        "mu_max": self.store_mu_max,
        "mu_min": self.store_mu_min,
        "ncmax_loc": self.store_ncmax_loc,
        "ncmin_loc": self.store_ncmin_loc,
        "ncmax": self.store_ncmax,
        "ncmin": self.store_ncmin,
        "box_rot_finger": self.store_box_rot_finger,
        "d": self.store_d,
        "b_e": self.store_b_e,
        "n": self.store_n,
        "phi_a": self.store_phi_a,
        "phi": self.store_phi,
        "phi_goal": self.store_phi_goal,
        "e": self.store_e,
        "r": self.store_r,
        "a": self.store_a,
        "b": self.store_b,
        "vtest1": self.store_vtest1,
        "d_low": self.store_d_low,
        "stop_angle_ctrl": self.store_stop_angle_ctrl,
        "vtest2_raw": self.store_vtest2_raw,
        "vtest2": self.store_vtest2,
        "d_extra": self.store_d_extra,
        "control_e": self.store_control_e,
        "control_phi": self.store_control_phi,
        "control_e_scaled": self.store_control_e_scaled,
        "control_phi_scaled": self.store_control_phi_scaled,
        "vc_local4": self.store_vc_local4,
        "vc_local2": self.store_vc_local2,
        "vc": self.store_vc,
        "vc_sat_temp": self.store_vc_sat_temp,
        "cur_time": self.store_cur_time,
        "finger_pose_cmd": self.store_finger_pose_cmd,
        "finger_pose2": self.store_finger_pose2,
            }
        pickle.dump(store, control_file)
        control_file.close()

    def finger_away_touch_point(self,touch_point, z_offset=True):
        '''Before running this, please update object position and touch face'''
        finger_offset=0.02
        self.finger_start_pos=calc_finger_start_pos(finger_offset, touch_point, self.xo, self.touch_face, self.obj_par.box_planes)
        print "Finger Start position", self.finger_start_pos
        self.finger_start_orient=calc_finger_orient(self.base_pushing_orient, self.table_normal, self.xo, self.touch_face)
        print "Finger start orient", self.finger_start_orient
        finger_start_pose=identity(4)
        finger_start_pose[:3,:3]=self.finger_start_orient
        finger_start_pose[:3,3]=self.finger_start_pos
        if z_offset:
            box_z_offset=-self.obj_par.box_dim[2]/2.+0.05
        else:
            box_z_offset=0.
        finger_start_pose[2,3]+=box_z_offset #box z offset for pushing

        new_touch_point=array(touch_point)
        new_touch_point[2]+=box_z_offset

        #Move finger to start position but on top of object (to avoid collisions)
        avoid_col=array(finger_start_pose)
        avoid_col[2,3]+=self.obj_par.box_dim[2]
        return(avoid_col,finger_start_pose, new_touch_point)

    def process(self):
        print
        eprint("---------START---------")
        #Visualizing goal
        goal_vis=identity(4)
        goal_vis[:3,:3]=self.xo_ref[:3,:3]
        goal_vis[:3,3]=self.xo_ref[:3,3]
        #goal_vis[2,3]+=self.obj_par.box_dim[2]/2.
        self.view_objects.send_prop(self.goal_id,"pose",
                                    goal_vis.reshape(16).tolist())

        #getting current object position
        self.xo=wait_valid_object_pose(self.object_pos_handle, self.xo, self.max_object_jump_dist)
        self.box_pose=dot(inv(self.xo_ref), self.xo) #with respect to goal
        #print "Global box pose", self.xo[:3,3]
        #print "Box pose with respect to goal", self.box_pose[:3,3]
        self.finger_start_orient=calc_finger_orient(self.base_pushing_orient, self.table_normal, self.xo, self.touch_face)


        #getting finger position
        force_data=self.hforce.get_data_finger(self.finger)
        force_data[-3:]*=force_factor
        self.finger_pose_global=quat.to_matrix_fixed(force_data[3:7], r=force_data[:3])
        self.finger_pose_global=dot(self.finger_pose_global, self.rel_pose_marker_finger)
        #print "Global Finger_pose from robot", self.finger_pose_global[:3,3]
        #finger position with respect to goal
        self.finger_pose=dot(inv(self.xo_ref), self.finger_pose_global)
        if self.first:
            self.first_finger_pose=array(self.finger_pose)
            self.finger_pose_cmd_special=array(self.finger_pose)

        #print "box pos", self.box_pose[:3,:3]
        print "box pos", self.xo[:3,3]
        print "Goal pos", self.xo_ref[:3,3]
        print "local box pos control", self.box_pose[:3,3]
        print "Local finger position", self.finger_pose[:3,3]

        #ncmin, ncmax right side box
        mu=0.8*self.obj_par.friction_coef_finger_object #10 % security band

        #rx, ry
        rxy_global=self.finger_pose[:3,3]-self.box_pose[:3,3]
        rxy=dot(inv(self.box_pose[:3,:3]),rxy_global)[:2]
        rxy_angle=arctan2(rxy[1],rxy[0])
        print "rxy", rxy, "rxy angle", rxy_angle*180./pi, "mu", mu

        sliding_angle=0.7*self.box_sides_angle
        stop_angle=0.9*self.box_sides_angle
        stop_angle=1.*self.box_sides_angle # to include corners
        if rxy_angle>=-stop_angle and rxy_angle<=stop_angle: #this is for changing the allowed sliding movement
            self.store_event.append([self.cur_time,"away from corners"])
            # Here we allow sliding in only one direction.
            if rxy_angle>-sliding_angle:
                self.store_event.append([self.cur_time,"nearing low corner, limiting sliding"])
                print "Max sliding"
                mu_max=0.9
            else:
                print "Max sat"
                mu_max=mu
            if rxy_angle<sliding_angle:
                self.store_event.append([self.cur_time,"nearing high corner, limiting sliding"])
                mu_min=0.9
            else:
                mu_min=mu
#            mu_max=mu
#            mu_min=mu
            ncmax_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)-2*rxy[0]*rxy[1]*mu_max),
                              -2*rxy[0]*rxy[1]+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu_max,0.])
            ncmin_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)+2*rxy[0]*rxy[1]*mu_min),
                              -2*rxy[0]*rxy[1]-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu_min,0.])
            #LC_c_matrix=ps.nc_to_forces_map(self.obj_par.fmax, self.obj_par.mmax, rxy)
            #table_face_normal=array([0.,0.,1.])
            #finger_face_normal=array([1.,0.,0.])
            #ncmax_loc,ncmin_loc,fc_max,fc_min=ps.nc_max_min(table_face_normal, finger_face_normal, self.obj_par.friction_coef_finger_object, inv(LC_c_matrix))
            ncmax=dot(self.box_pose[:3,:3],ncmax_loc)
            ncmin=dot(self.box_pose[:3,:3],ncmin_loc)
            print "right side"
            side="x_pos"
        elif (rxy_angle-pi/2.)>=-stop_angle and (rxy_angle-pi/2.)<=stop_angle: 
            self.store_event.append([self.cur_time,"away from corners"])
            # Here we allow sliding in only one direction.
            if rxy_angle-pi/2.>-sliding_angle:
                self.store_event.append([self.cur_time,"nearing low corner, limiting sliding"])
                print "Max sliding"
                mu_max=0.9
            else:
                print "Max sat"
                mu_max=mu
            if rxy_angle-pi/2.<sliding_angle:
                self.store_event.append([self.cur_time,"nearing high corner, limiting sliding"])
                mu_min=0.9
            else:
                mu_min=mu
#            mu_max=mu
#            mu_min=mu
            ncmax_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)*mu_max-2*rxy[0]*rxy[1]),
                              -2*rxy[0]*rxy[1]*mu_max+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2),0.])
            ncmin_loc=-array([(-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)*mu_min-2*rxy[0]*rxy[1]),
                              2*rxy[0]*rxy[1]*mu_min+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2),0.])
            #LC_c_matrix=ps.nc_to_forces_map(self.obj_par.fmax, self.obj_par.mmax, rxy)
            #table_face_normal=array([0.,0.,1.])
            #finger_face_normal=array([1.,0.,0.])
            #ncmax_loc,ncmin_loc,fc_max,fc_min=ps.nc_max_min(table_face_normal, finger_face_normal, self.obj_par.friction_coef_finger_object, inv(LC_c_matrix))
            ncmax=dot(self.box_pose[:3,:3],ncmax_loc)
            ncmin=dot(self.box_pose[:3,:3],ncmin_loc)
            # ncmax_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)-2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            # ncmin_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)+2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            print "top side"
            side="y_pos"
        elif (rxy_angle+pi/2.)>=-stop_angle and (rxy_angle+pi/2.)<=stop_angle:
            self.store_event.append([self.cur_time,"away from corners"])
            # Here we allow sliding in only one direction.
            if rxy_angle+pi/2.>-sliding_angle:
                self.store_event.append([self.cur_time,"nearing low corner, limiting sliding"])
                print "Min sliding"
                mu_min=0.9
            else:
                print "Min sat"
                mu_min=mu
            if rxy_angle+pi/2.<sliding_angle:
                self.store_event.append([self.cur_time,"nearing high corner, limiting sliding"])
                print "Max sliding"
                mu_max=0.9
            else:
                print "Max sat"
                mu_max=mu
#            mu_max=mu
#            mu_min=mu
            ncmax_loc=array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)*mu_max-2*rxy[0]*rxy[1]),
                              -2*rxy[0]*rxy[1]*mu_max+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2),0.])
            ncmin_loc=array([(-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)*mu_min-2*rxy[0]*rxy[1]),
                              2*rxy[0]*rxy[1]*mu_min+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2),0.])
            #LC_c_matrix=ps.nc_to_forces_map(self.obj_par.fmax, self.obj_par.mmax, rxy)
            #table_face_normal=array([0.,0.,1.])
            #finger_face_normal=array([1.,0.,0.])
            #ncmax_loc,ncmin_loc,fc_max,fc_min=ps.nc_max_min(table_face_normal, finger_face_normal, self.obj_par.friction_coef_finger_object, inv(LC_c_matrix))
            ncmax=dot(self.box_pose[:3,:3],ncmax_loc)
            ncmin=dot(self.box_pose[:3,:3],ncmin_loc)
            # ncmax_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)-2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            # ncmin_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)+2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            print "lower side"
            side="y_neg"
        elif (((rxy_angle-pi)>=-stop_angle) and ((rxy_angle-pi)<=0.)) or (((rxy_angle+pi)<=stop_angle) and ((rxy_angle+pi)>=0.)):
            self.store_event.append([self.cur_time,"away from corners"])
            # Here we allow sliding in only one direction.
            if rxy_angle+pi/2.>-sliding_angle:
                self.store_event.append([self.cur_time,"nearing low corner, limiting sliding"])
                print "Max sliding"
                mu_max=0.9
            else:
                print "Max sat"
                mu_max=mu
            if rxy_angle+pi/2.<sliding_angle:
                self.store_event.append([self.cur_time,"nearing high corner, limiting sliding"])
                mu_min=0.9
            else:
                mu_min=mu
#            mu_max=mu
#            mu_min=mu
            ncmax_loc=array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)-2*rxy[0]*rxy[1]*mu_max),
                              -2*rxy[0]*rxy[1]+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu_max,0.])
            ncmin_loc=array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)+2*rxy[0]*rxy[1]*mu_min),
                              -2*rxy[0]*rxy[1]-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu_min,0.])
            #LC_c_matrix=ps.nc_to_forces_map(self.obj_par.fmax, self.obj_par.mmax, rxy)
            #table_face_normal=array([0.,0.,1.])
            #finger_face_normal=array([1.,0.,0.])
            #ncmax_loc,ncmin_loc,fc_max,fc_min=ps.nc_max_min(table_face_normal, finger_face_normal, self.obj_par.friction_coef_finger_object, inv(LC_c_matrix))
            ncmax=dot(self.box_pose[:3,:3],ncmax_loc)
            ncmin=dot(self.box_pose[:3,:3],ncmin_loc)
            # ncmax_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)-2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]+2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            # ncmin_loc=-array([(2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[1]**2)+2*rxy[0]*rxy[1]*mu),
            #                   -2*rxy[0]*rxy[1]-2*((self.obj_par.mmax**2/self.obj_par.fmax**2)+rxy[0]**2)*mu,0.])
            print "left side"
            side="x_neg"

        ncmax_loc/=abs(ncmax_loc[0])
        ncmin_loc/=abs(ncmin_loc[0])
        print "mcmax_loc", ncmax_loc, "ncmin_loc", ncmin_loc

        if side=="x_pos":
            side_rot=0.
        elif side=="y_pos":
            side_rot=pi/2.
        elif side=="x_neg":
            side_rot=-pi
        elif side=="y_neg":
            side_rot=-pi/2.
        rxy_angle2=rxy_angle-side_rot
        self.box_pose2=dot(dot(inv(dot(self.xo_ref,homo_matrix(rot_m=rot_z(side_rot)))),self.xo), homo_matrix(rot_m=rot_z(side_rot)))

        #filtered object data
        self.box_pos_filtered=self.object_pos_filter.filter(self.box_pose2[:3,3])
        self.box_orient_filtered=self.object_orient_filter.filter(self.box_pose2[:3,0]) #takes the first column of the orientation to used later to decide to stop or not near the goal
        print "Box pos filtered", self.box_pos_filtered
        print "Box orient filtered", self.box_orient_filtered

            #self.box_pose2=dot(self.box_pose,homo_matrix(rot=rot_z(pi/2.)))
        print "rxy_angle2, angle relative to face", rxy_angle2*180./pi
        #rotate box to finger angle
        rot_frame=identity(4)
        rot_frame[:3,:3]=rot_z(rxy_angle2)
        #box_rot_finger=dot(rot_frame,self.box_pose)
        box_rot_finger=dot(self.box_pose2, rot_frame)
        print "Box pos finger correction", box_rot_finger[:3,3]

        # d, e, phi_a, phi, n
        d=norm(box_rot_finger[:2,3])
        b_e=1.2
        #b_e=1.5
        b_e=1.2
        n=sign(self.finger_sign)*arctan(box_rot_finger[1,3]/abs(box_rot_finger[0,3]))
        n=arctan2(box_rot_finger[1,3],(box_rot_finger[0,3]))
        phi_a=(2./b_e)*n
        if phi_a>pi*0.9:
            phi_a=0.9*pi
        if phi_a<-0.9*pi:
            phi_a=-0.9*pi
        phi=my_get_euler_zyx(box_rot_finger[:3,:3])[0]
        phi_goal=my_get_euler_zyx(self.box_pose[:3,:3])[0]
        e=phi_a-phi

        r=norm(rxy[:2]) #do the right thing
        print "phi", phi*180/pi, "d", d, "n", n*180/pi, "phi_a", phi_a*180/pi, "e", e*180/pi, "r", r, "phi goal", phi_goal*180./pi
        a=1/(1+self.obj_par.fmax**2*r**2/self.obj_par.mmax**2)
        b=r/((self.obj_par.mmax**2/self.obj_par.fmax**2)+r**2)
        print "a:", a, "b:", b
        c1=5.
        c2=4.
        c3=4.

        #with pseudoinverse
        A=array([[cos(n-phi),a*sin(n-phi)],
                 [-2*sin(n-phi)/(b_e*d),a*(2/(b_e*d))*cos(n-phi)-b],
                 [0,b]])
        A=array([[cos(n-phi),a*sin(n-phi)],
                 [-2*sin(n-phi)/(b_e*d),-b],
                 [0,b]])
        pA=pinv(A)
#        print "Matrix", A
#        print "inverse", pA
        if d>1.:
            d_extra=1.
        else:
            d_extra=d
        if d>0.05:
            d_extra2=0.05
        else:
            d_extra2=0.05
        y=array([-c1*d_extra2,-c2*e*d_extra**2,-c3*phi/exp(d)])
        #y=array([-c1*d,-c2*e, -c3*phi/exp(d)])
        #y=array([-c1*cos(n-phi)**2/(cos(n-phi)**2+e**2)*d,-(c2+c1*d**2/(cos(n-phi)**2+e**2))*e, c2*e+(c1/(cos(n-phi)**2+e**2))*((2/b_e)*cos(n-phi)*sin(n-phi)+d**2*e)])
        vc_local3=dot(pA,y)
#        print "vc local3", vc_local3

        #Intuitive
        #forward velocity control rule
        if self.simulation:
            finger_speed=0.02
            vc_angle_factor_slope=5.
        else:
            finger_speed=0.015
            vc_angle_factor_slope=1.
        vtest1=-finger_speed #*self.finger_sign
        d_low=max(self.obj_par.box_dim[0],self.obj_par.box_dim[1])/1.5 #should depend on the size of the object but only there must be a minimum for the precision of the sensors 
        d_low2=min(self.obj_par.box_dim[0],self.obj_par.box_dim[1])/4. #should depend on the size of the object but only there must be a minimum for the precision of the sensors 
        #d_low=0.001
        stop_angle_ctrl=0.
        d_low_factor=1.
        if d<d_low:
            print "Slowing down"
            #forward velocity control rule (during slow down)
            #vtest1=(vtest1/d_low)*d
            if d<d_low2:
                d_low_factor=d/d_low
                d_low_factor*=d_low_factor
            #check for stopping, 90 to object orient from distance vector
            stop_angle_ctrl=dot(self.box_orient_filtered/norm(self.box_orient_filtered), self.box_pos_filtered/norm(self.box_pos_filtered))
            print "Stop angle", arccos(min(1.,stop_angle_ctrl))*180./pi
            if (stop_angle_ctrl < cos(88.*pi/180.)) or (d<0.002):
                self.store_event.append([self.cur_time, "Control stopped"])
                self.stop=True
            #if not self.d_last:
            #    self.d_last=d
            #    self.stop=False
            #if self.d_last<d:
            #    self.stop=True
            #else:
            #    self.d_last=d
        if self.stop:
            print "Stopped"
            vtest1=0.
        vtest2=e*abs(vtest1)*205.0*d**2
        if d>0.4:
            d_extra=0.4
        else:
            d_extra=d
#        d_extra=d
#        self.control_ei+=-0.05*vtest1*e*b
        e_boundary=2.*pi/180.
        #lateral velocity control rule for aligning with auxiliary angle
        control_e=-830.*e*vtest1+self.control_ei
        #if abs(e)>e_boundary:
        #    control_e=-3.*e*vtest1*(b*d_extra)**2+self.control_ei
        #else:
        #    control_e_max=-3.*e_boundary*vtest1*(b*d_extra)**2
        #    control_e_min=3.*e_boundary*vtest1*(b*d_extra)**2
        #    print "Sliding mode, control max min:", control_e_max, control_e_min
#            max=(
        #    control_e=-10.*e*vtest1
#        control_e=(0.6*e+0.6*2*d*tan(n-phi)/b_e)/(b/a)

#        self.control_ei+=0.01*e
#        control_e=0.1*e+self.control_ei
#        control_e=-35.*e*vtest1/b
        #lateral velocity control rule for aligning with final angle
        control_phi=45.*phi_goal*vtest1
#        control_phi=0.
        #complete lateral velocity control rule
        vtest2=control_e*(d_extra)**2+control_phi/(exp(b*d))
        vtest2=control_e*(d_extra)**2+control_phi/(exp(b*d/a))
        control_phi_scaled=control_phi/(exp(b*d/(2*a)))
        control_phi_scaled=control_phi/(exp(b*d/a))
        control_e_scaled=control_e*(d_extra)**2
        vtest2=control_e_scaled+control_phi_scaled
        vtest2_raw=vtest2
        print "control phi scaled", control_phi_scaled, "control e scaled", control_e_scaled
#        vtest2/=b
        #vtest2*=self.finger_sign
#        vtest2=self.filter.filter(array([vtest2]))

        print "Control parts, v1:", vtest1, "v2:", vtest2, "e:", control_e, "phi:", control_phi
        if abs(n)<1*pi/180.:
            print "error low"
#            vtest2=0.
        u=15.8 #velocity ratio limit (limits the lateral speed to u time the forward speed)
        if abs(vtest2)>u*abs(vtest1):
            print "Lateral speed limiting" 
            self.store_event.append([self.cur_time, "limit lateral speed because of u"])
            vtest2=sign(vtest2)*u*abs(vtest1)
        print "New vtest" , vtest1, vtest2
        #vtest2 filter
        self.vtest2_buff[self.vtest2_buff_pos]=vtest2
        self.vtest2_buff_pos+=1
        if self.vtest2_buff_pos>=len(self.vtest2_buff):
            self.vtest2_buff_pos=0
#        vtest2=self.vtest2_buff.mean()
        vc_local4=array([vtest1,vtest2,0])
        print "vc local4 HERE:", vc_local4
        #slow down speed if lateral speed increases.
        vc_angle_temp=dot(array([-1.,0.]),vc_local4[:2]/norm(vc_local4[:2]))
        vc_angle=arccos(min(1., vc_angle_temp))
        #vc_angle_factor=1./(vc_angle_factor_slope*vc_angle+1.)
        #vc_angle_factor=vc_angle_temp+1.
        #print "Vc angle factor" , vc_angle_factor, "vc angle" , vc_angle*180./pi

        vc_local4=dot(rot_z(rxy_angle2),vc_local4)
        vc_local4=vc_local4[:2]
        print "vc_local4", vc_local4

        #analytical
        v2=(-c2*e-2*c1*d*tan(n-phi)/(b_e*d))/((2./(b_e*d))*tan(n-phi)*sin(n-phi)+(2./(b_e*d))*cos(n-phi)-(b/a))
        v1=(-c1*d-sin(n-phi)*v2)/cos(n-phi)
        vc1=v1
        vc2=v2/a
#        print "vc1", vc1, "vc2", vc2
        vc_local2=array([vc1,vc2,0.])
        vc_dir=vc_local2/norm(vc_local2)
#        print "vcdir", vc_dir
        #vc_dir=array([-1.,0.])
        #print "angle1", arccos(dot(vc_dir[:2],ncmax_loc[:2]))*180./pi, "angle2", arccos(dot(vc_dir[:2],ncmin_loc[:2]))*180./pi
        #vc_sat_temp=vector_saturation(ncmin_loc,ncmax_loc,vc_local2[:2])
        #vc_sat_temp/=norm(vc_sat_temp)
        #for the left side
        #vc_local2[:2]=abs(vc_local2[0])*vc_sat_temp
        #print "saturated vector", vc_local2
        #vc_local2*=2.
        #if norm(vc_local2)> 0.2:
        #    print "limiting speed"
            #vc_local2*=0.2/norm(vc_local2)
        vc_local2=concatenate((vc_local4,array([0.])))

        # angle deviation calculation
        self.object_goal=self.xo_ref[:2,3]-self.box_pose[:2,3]
        self.object_goal/=norm(self.object_goal)
        self.dir_box_finger=self.box_pose[:2,3]-self.finger_pose[:2,3]
        self.dir_box_finger/=norm(self.dir_box_finger)
        self.finger_goal=self.xo_ref[:2,3]-self.finger_pose[:2,3]
        self.finger_goal/=norm(self.finger_goal)
        #print "dir vector", self.object_goal, "dir_box_finger", self.dir_box_finger

        #proportional control
        xo_error=array([0.]*3)
        xo_error[2]=angle_from_a_to_b(self.finger_goal,self.object_goal)
        xo_error[:2]=self.xo_ref[:2,3]-self.box_pose[:2,3]
#        if norm(xo_error[2])<1.*pi/180.0:
#            xo_error[2]=0.
        slow_down_dist=0.5*min(self.obj_par.box_dim[0],self.obj_par.box_dim[1])
        angle_dist_norm=norm(xo_error[:2])/xo_error[2]
        #xo_error[2]/=angle_dist_norm
        #if norm(xo_error[:2])<slow_down_dist:
        #    percent=norm(xo_error[:2])/slow_down_dist
        #    kw=1.2*0.
        #else:
        #    kw=3.
        kw=4.*norm(xo_error[:2])
        kw=30.
        #print "kw", kw
        kxy=0.1/norm(xo_error[:2])
        #if norm(xo_error[:2])<0.05:
        #    modulate=0.
        #else:
        #    modulate=1.
        modulate=1.
        #kxy=1.
        #kw=kxy*30.
        kxy=0.1
        kw=kxy*100.
        k=modulate*array([[kxy,0.,0.],
                 [0.,kxy,0.],
                 [0.,0.,kw]])
        #print "Error", xo_error, norm(xo_error[:2]), slow_down_dist
        self.vo=dot(k,xo_error)
        #if norm(self.vo[:2])>0.5:
        #    self.vo[:2]/=norm(self.vo[:2])
        #    self.vo[:2]*=0.5
        #print "Vo", self.vo

        vo_local=dot(inv(self.box_pose[:3,:3]),self.vo)
        finger_pos_local=dot(inv(self.box_pose),self.finger_pose)
        contact_pos_local=finger_pos_local #this only if the finger is really
        #touching
        vc_local=concatenate((vo_to_vc(vo_local,
                                       contact_pos_local[:3,3],
                                       self.obj_par.fmax,self.obj_par.mmax), array([0.])))
        #max_finger_speed=0.5
        #if norm(vc_local)>max_finger_speed:
        #    print "limiting speed"
        #    #vc_local=max_finger_speed*vc_local/norm(vc_local)
        #print "Vc local", vc_local
        #if norm(xo_error[:2])>slow_down_dist:
        #    print "constant speed"
        #    speed=max_finger_speed
        #elif norm(xo_error[:2])>0.1*slow_down_dist:
        #    print "slowing down"
        #    speed=max_finger_speed-(slow_down_dist-norm(xo_error[:2]))*(max_finger_speed/(0.9*slow_down_dist))
        #else:
        #    print "stopped"
        #    speed=0.
        #print "Speed", speed
        #vc_local*=(speed/norm(vc_local))
        #vc=concatenate((dot(self.box_pose[:3,:3],vc_local),array([0.]*3)))

        #converting to goal reference frame
        print "vc_local2", vc_local2
        vc=dot(rot_z(side_rot),dot(self.box_pose2[:3,:3],vc_local2))
        vc[2]=0.
        #saturating vc
        ncmax[2]=0.
        ncmin[2]=0.
        print "nc max min" , ncmax, ncmin
        print "vc raw:", vc
        vc_sat_temp=norm(vc)*vector_saturation2(ncmin,ncmax,vc)
        print "Saturation temp", vc_sat_temp
        sat_speed=finger_speed
        if norm(vc_sat_temp)> sat_speed:
            eprint("Limiting speed")
        if norm(vc_sat_temp)>0.000000001:
            vc_sat_temp=sat_speed*vc_sat_temp/norm(vc_sat_temp)
            #vc_sat_temp=sat_speed*vc_sat_temp/vc
        #modulating vc_sat_temp for pushing angle.
        vc_sat_temp*=d_low_factor
        #minimal speed
        if not self.stop and (norm(vc_sat_temp)<0.3*sat_speed):
            print "Saturating to minimal speed"
            vc_sat_temp=0.3*sat_speed*vc_sat_temp/norm(vc_sat_temp)
        #vtest1=d/d_low  #(vtest1/d_low)*d
        #nc_extra=ncmax_loc
        #nc_extra[1]-=0.4
        #vc_sat_temp=dot(self.box_pose[:3,:3],0.05*nc_extra)
        #vc_sat_temp=0.05*vector_saturation2(ncmin,ncmax,vc_sat_temp)
        vc=concatenate((vc_sat_temp,array([0.]*3)))
        #vc_sat_temp/=norm(vc_sat_temp)

#        vc=array([0.]*6)
#        vc=concatenate((dot(self.box_pose[:3,:3],vc_local2),array([0.]*3)))
        print "Vc saturated", vc, "Norm vc", norm(vc)

        #raw_input()

        self.cur_time=time.time()
        dt=self.cur_time-self.last_time
        self.last_time=self.cur_time
        self.finger_pose_cmd_special=my_adddelta(self.finger_pose_cmd_special,vc,dt)
        update_method="cmd"
        #update_method="finger_fixed_z"
        #update_method="finger"
        if update_method=="cmd":
            print "Finger update from cmd_pose"
            self.finger_pose_cmd=self.finger_pose_cmd_special
        elif update_method=="finger_fixed_z":
            print "Finger update from finger pose z fixed"
            self.finger_pose_cmd=my_adddelta(self.finger_pose,vc,dt)
            self.finger_pose_cmd[2,3]=self.first_finger_pose[2,3]
        else:
            print "Finger update from finger pose"
            self.finger_pose_cmd=my_adddelta(self.finger_pose,vc,dt)
        self.finger_pose_out=array(self.finger_pose_cmd)
        self.finger_pose_out[0,3]=self.filter_x.filter(array([self.finger_pose_out[0,3]]))
        self.finger_pose_out[1,3]=self.filter_y.filter(array([self.finger_pose_out[1,3]]))
        if self.stop:
            self.finger_pose_out[:2,3]=self.finger_pose_cmd[:2,3]
        print "finger pose", self.finger_pose[:2,3], self.finger_pose_out[:2,3]
        if self.enable_filter:
            self.finger_pose_out2=dot(self.xo_ref, self.finger_pose_out)
            #write_narray_port(self.finger_pose_out_port,self.finger_pose_out2[:3,3])
        else:
            self.finger_pose2=dot(self.xo_ref, self.finger_pose_cmd)
            dprint("Commanded finger pose", self.finger_pose2[:3,3])
            #write_narray_port(self.finger_pose_out_port,self.finger_pose_global[:3,3])
            self.finger_pose2[:3,:3]=self.finger_start_orient
            move_robot(self.hand_right_server, self.finger, self.harm, self.finger_pose2, self.goal_precision, extra_tool=self.rel_pose_marker_finger, update_finger_cart=self.update_finger_cart)
            eprint("After")
            #move_robot(self.hand_right_server, self.finger, self.harm, self.finger_pose2, self.goal_precision)
            if self.update_finger_cart:
                self.update_finger_cart=False
                pass

        #min max velocity vectors visualization
        #vis_vectors=[(dot(self.xo_ref[:3,:3],ncmin),identity(4), self.finger_pose_global[:3,3], identity(4), self.ncmin_id)]
        #vis_vectors+=[(dot(self.xo_ref[:3,:3],ncmax),identity(4), self.finger_pose_global[:3,3], identity(4), self.ncmax_id)]
        vis_vectors=[]
        vis_points=[]
        self.visualize(points=vis_points, vectors=vis_vectors)
        if self.stop:
            self.cont=False #Stops the cycle

        #storing data
        self.store_ref.append(self.xo_ref)
        self.store_xo.append(self.xo)
        self.store_side.append(side)
        self.store_side_rot.append(side_rot)
        self.store_box_pose.append(self.box_pose)
        self.store_box_pose2.append(self.box_pose2)
        self.store_box_pos_filtered.append(self.box_pos_filtered)
        self.store_box_orient_filtered.append(self.box_orient_filtered)
        self.store_force_data.append(force_data)
        self.store_finger_pose_global.append(self.finger_pose_global)
        self.store_finger_pose.append(self.finger_pose)
        self.store_rxy_global.append(rxy_global)
        self.store_rxy.append(rxy)
        self.store_rxy_angle.append(rxy_angle)
        self.store_sliding_angle.append(sliding_angle)
        self.store_stop_angle.append(stop_angle)
        self.store_mu.append(mu)
        self.store_mu_max.append(mu_max)
        self.store_mu_min.append(mu_min)
        self.store_ncmax_loc.append(ncmax_loc)
        self.store_ncmin_loc.append(ncmin_loc)
        self.store_ncmax.append(ncmax)
        self.store_ncmin.append(ncmin)
        self.store_box_rot_finger.append(box_rot_finger)
        self.store_d.append(d)
        self.store_b_e.append(b_e)
        self.store_n.append(n)
        self.store_phi_a.append(phi_a)
        self.store_phi.append(phi)
        self.store_phi_goal.append(phi_goal)
        self.store_e.append(e)
        self.store_r.append(r)
        self.store_a.append(a)
        self.store_b.append(b)
        self.store_vtest1.append(vtest1)
        self.store_d_low.append(d_low)
        self.store_stop_angle_ctrl.append(stop_angle_ctrl)
        self.store_vtest2_raw.append(vtest2_raw)
        self.store_vtest2.append(vtest2)
        self.store_d_extra.append(d_extra)
        self.store_control_e.append(control_e)
        self.store_control_phi.append(control_phi)
        self.store_control_e_scaled.append(control_e_scaled)
        self.store_control_phi_scaled.append(control_phi_scaled)
        self.store_vc_local4.append(vc_local4)
        self.store_vc_local2.append(vc_local2)
        self.store_vc.append(vc)
        self.store_vc_sat_temp.append(vc_sat_temp)
        self.store_cur_time.append(self.cur_time)
        self.store_finger_pose_cmd.append(self.finger_pose_cmd)
        self.store_finger_pose2.append(self.finger_pose2)
        self.first=False
        eprint("---------END---------")


def main():
    parser=optparse.OptionParser("usage: %prog [options]")
    parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
    parser.add_option("-c", "--finger_cal_data", dest="finger_cal_data_filename", default="robot_description/tum-rosie/kinematics/sahand/calibration_data/finger_calibration_data.py", type="string", help="Finger calibration data filename")
    parser.add_option("-f", "--config_hands", dest="config_hands", default="robot_description/tum-rosie/kinematics/sahand/hands_kin.py", type="string", help="hands config filename")
    (options, args)= parser.parse_args(sys.argv[1:])
    from arcospyu.config_parser.config_parser import import_config
    #fcd=import_config(options.finger_cal_data_filename)
    config_hands=import_config(options.config_hands)

    control_loop=Control(25.)
    control_loop.loop(simulation=options.sim, config_hands=config_hands)

if __name__=="__main__":
    main()