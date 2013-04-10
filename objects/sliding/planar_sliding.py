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
sys.path.append("../../tools/python/computer_graphics")
from pyrovito.pyrovito_utils import Roboviewer_objects
from arcospyu.kdl_helpers import rot_vector_angle, my_adddelta, my_diff
from arcospyu.control.control_loop import Controlloop

#from helpers import Roboviewer_objects, Joint_sim, Finger_sim, Hand_sim, Controlloop, kdlframe_to_narray, narray_to_kdlframe, narray_to_kdltwist, kdltwist_to_narray, rot_vector_angle, my_adddelta, my_diff
from numpy import array,identity, concatenate,dot ,sqrt, cross, arctan, pi
from numpy.linalg import norm,inv
from numpy.random import normal
from PyKDL import Frame,addDelta, Twist, diff
from arcospyu.computer_graphics.pluecker_test import polygon_pluecker_test, triangle_ray_intersection, plane_ray_intersection
#from pluecker_test import polygon_pluecker_test, triangle_ray_intersection, plane_ray_intersection
import LS, time
from arcospyu.yarp_tools.yarp_comm_helpers import new_port, readListPort, write_narray_port, write_bottle_lists
from utils import Force_handle
from model_functions import fc_to_nc_matrix
from object_params import Object_params
from box import nc_to_forces_map, forces_to_no_map, force_c_to_force_o_map

def norm_nc_under_vc(fmax,mmax,contact_pos,fc2):
    nc=mmax*sqrt(1/( (mmax**2/fmax**2+contact_pos[1]**2)*fc2[0]**2 +
                     (mmax**2/fmax**2+contact_pos[0]**2)*fc2[1]**2 -
                     2*contact_pos[0]*contact_pos[1]*fc2[0]*fc2[1]))
    return(nc)


def homo_matrix(rot_m=identity(3),trans=array([0.]*3)):
    m=identity(4)
    m[:3,:3]=rot_m
    m[:3,3]=trans
    return(m)


def norm_Vc_to_norm_Vo(contact_pos_local,cor_local,Vc):
    norm_rccor=norm(contact_pos_local[:2]-cor_local[:2])
    norm_rocor=norm(-cor_local[:2])
    norm_Vo=norm(Vc)*norm_rocor/norm_rccor
    return(norm_Vo)


def forces_to_nc_map(fmax,mmax,contact_local):
    B=2*array([[mmax**2/fmax**2+contact_local[1]**2,-contact_local[0]*contact_local[1]],
               [-contact_local[0]*contact_local[1],mmax**2/fmax**2+contact_local[0]**2]])
    return(B)

def vo_to_vc_map(contact_local):
    D=array([[1,0,-contact_local[1]],
             [0,1,contact_local[0]]])
    return(D)

def ellipse_in_c(mmax,fmax,contact_local,fc_local):
    one=(1/mmax**2)*((mmax**2/fmax**2+contact_local[1]**2)*fc_local[0]**2+
                     (mmax**2/fmax**2+contact_local[0]**2)*fc_local[1]**2-
                     2*contact_local[0]*contact_local[1]*fc_local[0]*fc_local[1])
    print "Ellipse in c", one

def nc_max_min(table_face_normal,finger_face_normal,
               friction_coef_finger_object,LC_c_matrix):
    #rotation is over the table_face_normal_axis
    #We rotate finger_face_normal by arctan(friction_coef) to obtain corresponding
    #forces.
    #We apply the this forces through B to obtain nc_max_min
    rot_mat=rot_vector_angle(-table_face_normal,
                             arctan(friction_coef_finger_object))
    fc_max=dot(rot_mat,-finger_face_normal)
    nc_max=concatenate((dot(LC_c_matrix,fc_max[:2]),array([0.])))
    nc_max/=norm(nc_max)
    rot_mat=rot_vector_angle(-table_face_normal,
                             -arctan(friction_coef_finger_object))
    fc_min=dot(rot_mat,-finger_face_normal)
    nc_min=concatenate((dot(LC_c_matrix,fc_min[:2]),array([0.])))
    nc_min/=norm(nc_min)
    #print "fc max,min", fc_max, fc_min
    return(nc_max,nc_min,fc_max,fc_min)

def vc_from_vfinger(vfinger,finger_pos, box_vertices, box_planes, finger_face_normal,
                    table_face_normal, friction_coef_finger_object,fmax,mmax):
    
        
    LC_c_matrix=nc_to_forces_map(fmax,mmax,finger_pos)
    nc_max,nc_min,fc_max,fc_min=nc_max_min(table_face_normal, finger_face_normal,
                             friction_coef_finger_object,inv(LC_c_matrix)) #TODO change to non inverse
    vfinger=concatenate((vfinger[:2],array([0.])))
    vfingera=array(vfinger)
    vfingera/=norm(vfingera)
    print "Nc max, min", nc_max, nc_min, vfingera
    w1=dot(cross(nc_min,vfingera),-table_face_normal)
    w2=dot(cross(nc_max,vfingera),-table_face_normal)
    if w1<0:
        #Vfinger smaller than nc_mix, finger slides to the right
        vc=nc_min*dot(vfinger,-finger_face_normal)/\
            dot(nc_min,-finger_face_normal) #projection of vfinger on nc_min that
                                            #is parallel to finger_face_normal
        print "Slide right"
        result=1
    elif w2>0:
        #Vfinger bigger than nc_max, finger slides to the left
        vc=nc_max*dot(vfinger,-finger_face_normal)/\
            dot(nc_max,-finger_face_normal)
        print "Slide left"
        result=-1
    else:
        #finger not sliding
        vc=vfinger
        result=0
    #vc=vfinger
    return(vc,result)

def vc_to_vo_linear_model(vc_local,contact_pos_local,fmax,mmax):
    A=forces_to_no_map(fmax,mmax)
    B_inv=nc_to_forces_map(fmax,mmax,contact_pos_local)
    C=force_c_to_force_o_map(contact_pos_local)

    #system calculation
    fc_temp=dot(B_inv,vc_local[:2]) #this force has an incorrect magnitude
    fo_temp=dot(C,fc_temp) #this force has an incorrect magnitude
    vo_local=dot(A,fo_temp) #this velocity has an incorrect magnitude
    vo_local*=mmax**2 #correcting velocity magnitude

    #Internal system step values
    nc_under_vc=norm_nc_under_vc(fmax,mmax, contact_pos_local, fc_temp)
    fc_local=concatenate((fc_temp[:2],array([0.])))* nc_under_vc
    fo_local=fo_temp*nc_under_vc

    return(vo_local,fc_local, fo_local)

def near(num1,num2,epsilon=1e-10):
    return(abs(num1-num2)<epsilon)

def plane_ray_intersec2(normal_plane,point_plane,vector,pos):
    d=dot((point_plane-pos),normal_plane)/dot(vector,normal_plane)
    result=pos+vector*d
    return([d,result])

def find_finger_object_face(vfinger,pos_finger,vertices_faces, box_planes):
    crossing_faces={}
    for face in vertices_faces:
        temp=array(polygon_pluecker_test(vertices_faces[face],
                                        pos_finger,vfinger/norm(vfinger)))
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
            #                                       vertices[2],pos_finger,vfinger)
            #intersection=plane_ray_intersection(array(box_planes[face][0]),
            #                                    array(box_planes[face][1]),
            #                                    pos_finger,vfinger)
            intersection=plane_ray_intersec2( array(box_planes[face][0]),
                                              array(box_planes[face][1]),
                                              vfinger,pos_finger)
            intersection[0]*=norm(vfinger)
            print "intersection", intersection
            crossing_faces[face]=[vertices_faces[face],temp,intersection]
    print "crossing_faces", crossing_faces
    first=True
    dist_min=None
    for face in crossing_faces:
        if first:
            dist_min=[face,crossing_faces[face][2][0]]
            first=False
        else:
            if abs(crossing_faces[face][2][0]) < abs(dist_min[1]):
                dist_min=[face,crossing_faces[face][2][0]]
    print "dist min", dist_min
    if dist_min and dist_min[1]<=0. :
        for face in crossing_faces:
            if face!=dist_min[0]:
                if crossing_faces[face][2][0]>=0.:
                    return(True,dist_min[0])
    return(False,None)

class Loop(Controlloop):

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
        self.arrow_length=0.1
        vars(self).update(params) #takes the dictionary params and puts everything in variables of the class
        base_name="/slider_sim"
        #self.xfinger_in_port=new_port(base_name+"/xfinger:in","in", "/xfinger_feeder/xfinger:out",timeout=1.)
        self.hforce=Force_handle(base_name, "/torque_sim/force_out")
        self.finger=3

        self.xo_out_port=new_port(base_name+"/xo:out","out", "/planar_control/xo:in", timeout=1.)
        self.fc_out_port=new_port(base_name+"/ffinger:out","out", "/torque_sim/force_in", timeout=1.)
        #do a parameter input port (for simulation only)
        #variables
        #robot-object contact
        #self.Vc=array([0.1,0.,0.])
        #self.contact_pos_local=array([-0.5,-0.2,0.])
        #self.contact_pose=identity(4)
        #self.contact_pose[:3,3]=array([-0.5,-0.2,0.])
        #self.contact_vel_twist=array([0.,0.0,0.,0.,0.,0.])
        #self.contact_vel_twist[:3]=self.Vc

        self.finger_vel_twist=array([0.,0.,0.,0.,0.,0.])
        self.finger_data_good=False
        self.finger_data_good_counter=0
        self.finger_data_good_max_counter=10
        self.finger_pose=identity(4)
        #self.finger_pose[:3,3]=array(readListPort(self.xfinger_in_port,blocking=True))
        self.last_time=time.time()

        #self.Vc=array([0.1,0.,0.])
        #self.contact_pose[:3,3]=array([-1,-0.5,0.])

        #box
        self.obj_par=Object_params()

        self.finger_object_face=1
        self.table_object_face=5
        self.view_objects.send_prop(self.box_id,"scale",self.obj_par.box_dim)
        box_id_offset=identity(4)
        box_id_offset[:3,3]=array([0.,0.,-0.5])
        self.view_objects.send_prop(self.box_id,"pose_offset",
                                    box_id_offset.reshape(16).tolist())

        #objects
        table_height=0.85
        object_height=table_height+self.obj_par.box_dim[2]/2.

        #object in front (works)
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.8, 0.1,object_height])
        angle=-15.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)

        #object in front 2 (works)
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.8, 0. ,object_height])
        angle=-15.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)

        #object in front 2 (works)
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.82, -0.1 ,object_height])
        angle=-5.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)




        #object to the right away side 45
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.9, -0.5,object_height])
        angle=-45*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)




        #Goal 3.1 object to the right away side
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([1.05, -0.5,object_height])
        angle=-0.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)


        #Goal 2.1 object to the right away side, not so far away
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.86, -0.45,object_height])
        angle=-45.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)


        #Goal 4.1 object to the right away side, not so far away, testing, starting away from corners
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([1., -0.2,object_height])
        angle=5.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)

        #Goal 5.1 object to the right away side, not so far away, testing, starting away from corners
        self.box_pose=identity(4)
        self.box_pose[:3,3]=array([0.9, -0.1,object_height])
        angle=-5.*pi/180.0
        table_normal=array([0.,0.,1.])
        self.box_pose[:3,:3]=rot_vector_angle(table_normal,angle)



        self.box_vel_twist=array([0.0,0.0,0.,0.,0.,0.])
        self.box_pose_out=array(self.box_pose)

        self.test_pose=identity(4)
        self.test_pose[1,3]=-1.
        self.test_vel_twist=array([0.,0.0,0.,0.,0.,0.])
        self.test_vel_twist[:3]=self.finger_vel_twist[:3]

        self.cor_local=array([0.]*3)


    def process(self):
        vis_vectors=[]
        vis_points=[]
        print

        #reading finger pose and calculating new speed
        self.new_finger_pose=identity(4)
        #print "here"
        self.cur_time=time.time()
        dt=self.cur_time-self.last_time
        self.last_time=self.cur_time
        self.finger_data_good_counter+=1
        if self.finger_data_good_counter>self.finger_data_good_max_counter:
            self.finger_data_good=False

        #get finger pos
        force_pos_data=self.hforce.get_data_finger(self.finger)
        self.new_finger_pose[:3,3]=force_pos_data[:3]
        self.finger_vel_twist[:3]=my_diff(self.finger_pose,
                                              self.new_finger_pose, dt)[:3]
        self.finger_pose=self.new_finger_pose
        print "finger vel", self.finger_vel_twist

        # finger local calculation
        self.finger_vel_local=dot(inv(self.box_pose[:3,:3]),self.finger_vel_twist[:3])
        self.finger_pos_local=dot(inv(self.box_pose),self.finger_pose)[:3,3]
#        print "Finger pos local", self.finger_pos_local


        #first we find if finger is touching box
        touch,self.finger_object_face=find_finger_object_face(self.finger_vel_local,self.finger_pos_local,
                                      self.obj_par.box_vertices, self.obj_par.box_planes)

        #for finger vel z
        print "vel local", self.finger_vel_local
        if self.finger_vel_local[2]!=0.:
            friction_coef_finger_object_xy=self.obj_par.friction_coef_finger_object*\
                                        abs(self.finger_vel_local[1])/\
                                        norm(array([self.finger_vel_local[1],
                                                    self.finger_vel_local[2]]))
            # friction_coef_finger_object_xy= abs(self.finger_vel_local[1])/\
            # norm(array([self.finger_vel_local[1],
            # self.finger_vel_local[2]]))
        else:
            friction_coef_finger_object_xy=self.obj_par.friction_coef_finger_object
        friction_coef_finger_object_xy=self.obj_par.friction_coef_finger_object # TODO 

        fmax2=self.obj_par.friction_coef_object_table*\
                   (self.obj_par.weight_force-self.obj_par.friction_finger_z_force)
        mmax2=self.obj_par.mmax_base*(self.obj_par.weight_force-self.obj_par.friction_finger_z_force)
        print "fmax2, mmax2", fmax2,mmax2, friction_coef_finger_object_xy
        #LC_c_matrix=nc_to_forces_map(fmax2,mmax2,self.finger_pos_local)
        #nc_max,nc_min,fc_max,fc_min=nc_max_min(
        #    array(self.obj_par.box_planes[self.table_object_face][0]),
        #    array(self.obj_par.box_planes[
        #    self.finger_object_face][0]),
        #    friction_coef_finger_object_xy,
        #    inv(LC_c_matrix))

        #end finger vel z

        if touch:
            #velocity of box in point of contact (when finger slides or not)
            vc_local,result=vc_from_vfinger(self.finger_vel_local,
                                            self.finger_pos_local,
                                       self.obj_par.box_vertices,self.obj_par.box_planes,
                                array(self.obj_par.box_planes[self.finger_object_face][0]),
                                array(self.obj_par.box_planes[self.table_object_face][0]),
                                friction_coef_finger_object_xy,fmax2,
                                       mmax2)
            # print "Vc local", vc_local

            #finger touching and pushing object
            #table-object friction model
            Vo_local,fc_local,fo_local=vc_to_vo_linear_model(
                vc_local, self.finger_pos_local, fmax2, mmax2)
            self.cor_local=array([-Vo_local[1],Vo_local[0],0.])/Vo_local[2]
            Vo_local=array([Vo_local[0],Vo_local[1],0.,0.,0.,Vo_local[2]])
            # self.obj_par.friction_finger_z_force=fc_local[1]*self.finger_vel_local[2]/\
            # self.finger_vel_local[1]
            if self.finger_vel_local[2]!=0.:
                self.obj_par.friction_finger_z_force=self.obj_par.friction_coef_finger_object*fc_local[0]*self.finger_vel_local[2]/norm(array([self.finger_vel_local[1],self.finger_vel_local[2]]))
            else:
                self.obj_par.friction_finger_z_force=0.
            print "fc local, vc local", fc_local, vc_local/norm(vc_local)
            print "force z", self.obj_par.friction_finger_z_force
        else:
            vc_local=array([0.]*3)
            #finger not touching, no movement
            Vo_local=array([0.]*6)
            fo_local=array([0.]*3)
            fc_local=array([0.]*3)
            #cor_local=array([0.]*3) #commented so that it displays last position
        # box twist
        self.box_vel_twist[:3]=dot(self.box_pose[:3,:3],
                                   Vo_local[:3])
        self.box_vel_twist[3:]=Vo_local[3:]
        #fc_local to fc
        self.fc=dot(self.box_pose[:3,:3],fc_local)


        #box visualization
        box_top_pose=array(self.box_pose)
        #box_top_pose[2,3]=+0.2
        vis_points+=[(self.box_pose_out,identity(4),self.box_id), 
                    #(self.contact_pos_local,box_top_pose,self.contact_id),
                    (self.finger_pose,identity(4),self.contact_id),
                    (self.cor_local,box_top_pose,self.cor_id)]
        foxy_local=array(fo_local)
        foxy_local[2]=0.
        voxy_local=array(Vo_local)[:3]
        voxy_local[2]=0.
        vis_vectors+=[(self.finger_vel_twist[:3],identity(4),
                      self.finger_pose[:3,3], identity(4),
                      self.vc_id),
                      #(fc_local,self.box_pose,
                      #self.finger_pos_local, box_top_pose,
                      #self.fc_id),
                      (self.fc,identity(4),
                      self.finger_pose[:3,3], identity(4),
                      self.fc_id),
                      #(foxy_local,self.box_pose,
                      #array([0.,0.,0.]), box_top_pose,
                      #self.fo_id),
                      #(voxy_local,self.box_pose,
                      #array([0.,0.,0.]), box_top_pose,
                      #self.vo_id)
                      ]
                      

        #test visualization
#        vis_points+=[(self.test_pose,identity(4),self.test_id)]

        #visualization update
        self.visualize(points=vis_points, vectors=vis_vectors)


        #position update
        self.box_pose=my_adddelta(self.box_pose,self.box_vel_twist,dt)

        #gaussian noise to output information.
        self.box_pose_out=array(self.box_pose)
        noise_level=0.01*array([0.02,0.02,5.0*pi/180.0])
        self.box_pose_out[:2,3]=normal(loc=self.box_pose_out[:2,3], scale=noise_level[:2], size=(2))
        random_z_angle=normal(loc=0.,scale=noise_level[2])
        table_object_normal=-array(self.obj_par.box_planes[self.table_object_face][0])
        print "table normal", table_object_normal
        random_rot_frame=identity(4)
        random_rot_frame[:2,3]=0*array([0.0,0.01]) #camera offset error
        random_rot_frame[:3,:3]=rot_vector_angle(table_object_normal, random_z_angle)
        self.box_pose_out=dot(self.box_pose_out, random_rot_frame)
        #self.test_pose=my_adddelta(self.test_pose,self.test_vel_twist,self.period)
        #self.contact_pose=my_adddelta(self.contact_pose,self.contact_vel_twist,
        #                              self.period)
        #self.finger_pose=my_adddelta(self.finger_pose,self.finger_vel_twist,
                                      #self.period)

        #send through yarp
        #write_narray_port(self.fc_out_port,self.fc)
        write_bottle_lists(self.fc_out_port,[concatenate((array([self.finger]),-self.fc))])
        write_narray_port(self.xo_out_port,self.box_pose_out.reshape(16))


def main():
    view_objects=Roboviewer_objects("/planar_slider","/lwr/roboviewer", counter=10000)
    box_id=view_objects.create_object("box")
    view_objects.send_prop(box_id,"scale",[1.,1.,0.1])
    view_objects.send_prop(box_id,"timeout",[10])
    view_objects.send_prop(box_id,"color",[0.5,0.5,0.5])

    contact_id=view_objects.create_object("sphere")
    view_objects.send_prop(contact_id,"scale",[0.01,0.01,0.01])
    view_objects.send_prop(contact_id,"timeout",[-1])

    cor_id=view_objects.create_object("sphere")
    view_objects.send_prop(cor_id,"scale",[0.02,0.02,0.02])
    view_objects.send_prop(cor_id,"timeout",[-1])
    view_objects.send_prop(cor_id,"color",[1,0,0])

    vc_id=view_objects.create_object("arrow")
    view_objects.send_prop(vc_id,"scale",[0.05,0.05,0.0125])
    vc_id_offset=identity(4)
    vc_id_offset[:3,3]=array([0.,0.,-1.])
    view_objects.send_prop(vc_id,"pose_offset",vc_id_offset.reshape(16).tolist())
    view_objects.send_prop(vc_id,"timeout",[-1])
    view_objects.send_prop(vc_id,"color",[1,0,0])

    fc_id=view_objects.create_object("arrow")
    view_objects.send_prop(fc_id,"scale",[0.05,0.05,0.0125])
    fc_id_offset=identity(4)
    fc_id_offset[:3,3]=array([0.,0.,-1.])
    view_objects.send_prop(fc_id,"pose_offset",fc_id_offset.reshape(16).tolist())
    view_objects.send_prop(fc_id,"timeout",[-1])
    view_objects.send_prop(fc_id,"color",[0,1,0])

    vo_id=view_objects.create_object("arrow")
    view_objects.send_prop(vo_id,"scale",[0.05,0.05,0.0125])
    vo_id_offset=identity(4)
    vo_id_offset[:3,3]=array([0.,0.,-1.])
    view_objects.send_prop(vo_id,"pose_offset",vo_id_offset.reshape(16).tolist())
    view_objects.send_prop(vo_id,"timeout",[-1])
    view_objects.send_prop(vo_id,"color",[1,0,0])

    fo_id=view_objects.create_object("arrow")
    view_objects.send_prop(fo_id,"scale",[0.05,0.05,0.0125])
    fo_id_offset=identity(4)
    fo_id_offset[:3,3]=array([0.,0.,-1.])
    view_objects.send_prop(fo_id,"pose_offset",fo_id_offset.reshape(16).tolist())
    view_objects.send_prop(fo_id,"timeout",[-1])
    view_objects.send_prop(fo_id,"color",[0,1,0])

#    test_id=view_objects.create_object("sphere")
#    view_objects.send_prop(test_id,"scale",[0.1,0.1,0.1])
#    view_objects.send_prop(test_id,"timeout",[-1])
#    view_objects.send_prop(test_id,"color",[1,0,0])


    control_loop=Loop(15)
    control_loop.loop(view_objects=view_objects,box_id=box_id,contact_id=contact_id,vc_id=vc_id,fc_id=fc_id,cor_id=cor_id,fo_id=fo_id,vo_id=vo_id)
    

    raw_input()


if __name__=="__main__":
    main()
