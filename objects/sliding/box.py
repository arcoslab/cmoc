#!/usr/bin/env python
# Copyright (c) 2013 Federico Ruiz Ugalde
# Author: Federico Ruiz Ugalde <memeruiz at gmail.com>
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

from numpy import array
from arcospyu.robot_tools.robot_trans import homo_matrix
from arcospyu.computer_graphics.pluecker_test import polygon_pluecker_test
from arcospyu.computer_graphics.ray_tracing import plane_ray_intersec2
from numpy.linalg import norm, inv, pinv
from arcospyu.kdl_helpers.kdl_helpers import rot_vector_angle
from numpy import dot, arctan, cross, sqrt, concatenate



def forces_to_no_map(fmax,mmax):
    A=array([[2/fmax**2,0,0], # maps forces to nc
             [0,2/fmax**2,0],
             [0,0,2/mmax**2]])
    return(A)

def nc_to_forces_map(fmax,mmax,contact_local):
    a=(4*(mmax**2/fmax**2+contact_local[1]**2)*(mmax**2/fmax**2+contact_local[0]**2)
       -4*contact_local[0]**2*contact_local[1]**2)
    B_inv=(2./a)*array( #maps nc to forces
        [[mmax**2/fmax**2+contact_local[0]**2,contact_local[0]*contact_local[1]],
         [contact_local[0]*contact_local[1],mmax**2/fmax**2+contact_local[1]**2]])
    return(B_inv)

def force_c_to_force_o_map(contact_local):
    C=array([[1,0], #maps elipsoid to contact point
             [0,1],
             [-contact_local[1],contact_local[0]]])
    return(C)


##planar sliding simple


def fc_to_nc_matrix(fmax,mmax,contact_pos):
    m=2*array([[mmax**2/fmax**2+contact_pos[1]**2,-contact_pos[0]*contact_pos[1]],
               [-contact_pos[0]*contact_pos[1],mmax**2/fmax**2+contact_pos[0]**2]])
    return(m)

### planar sliding

def norm_nc_under_vc(fmax,mmax,contact_pos,fc2):
    nc=mmax*sqrt(1/( (mmax**2/fmax**2+contact_pos[1]**2)*fc2[0]**2 +
                     (mmax**2/fmax**2+contact_pos[0]**2)*fc2[1]**2 -
                     2*contact_pos[0]*contact_pos[1]*fc2[0]*fc2[1]))
    return(nc)

def norm_Vc_to_norm_Vo(contact_pos_local,cor_local,Vc):
    norm_rccor=norm(contact_pos_local[:2]-cor_local[:2])
    norm_rocor=norm(-cor_local[:2])
    norm_Vo=norm(Vc)*norm_rocor/norm_rccor
    return(norm_Vo)


def forces_to_nc_map(fmax,mmax,contact_local):
    B=2*array([[mmax**2/fmax**2+contact_local[1]**2,-contact_local[0]*contact_local[1]],
               [-contact_local[0]*contact_local[1],mmax**2/fmax**2+contact_local[0]**2]])
    return(B)


def vo_to_vc(vo_local,contact_pos_local,fmax,mmax):
    A=forces_to_no_map(fmax,mmax)
    B_inv=nc_to_forces_map(fmax,mmax,contact_pos_local)
    C=force_c_to_force_o_map(contact_pos_local)

    #system calculation
    P=dot(A,dot(C,B_inv))
    P_inv=pinv(P)
    vc_local=dot(P_inv,vo_local)/mmax**2
    return(vc_local)


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
