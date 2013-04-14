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

from numpy import dot, pi, array, identity
from numpy.linalg import inv, norm
from arcospyu.robot_tools.robot_trans import homo_matrix
from arcospyu.kdl_helpers import rot_vector_angle
from cmoc.objects.sliding.utils import move_robot, find_force_noise_levels
import time
from arcospyu.signal_proc.filters import  Filter_vector, Filter
from arcospyu.numeric import quat
from arcospyu.dprint import dprint, eprint, iprint

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

def approach_until_touch(hand_handle, finger, arm_handle, force_handle, touch_point, force_threshold, goal_precision, speed, start_pose, sim, force_factor, extra_tool=identity(4)):

    filters_order=3
    filters_freq=0.2
    max_noise_force, max_noise_norm_force=find_force_noise_levels(force_handle, finger, filter_order=filters_order, filter_freq=filters_freq, measurements=50)
    print "noise levels", max_noise_force, max_noise_norm_force
    #if False:
    if not sim:
        force_threshold=max_noise_norm_force*1.3
    #raw_input()

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
    dprint("STop movement")
    cur_pose=quat.to_matrix_fixed(data[3:7], r=data[:3])
    cur_pose[:3,:3]=pose[:3,:3]
    move_robot(hand_handle, finger, arm_handle, cur_pose, goal_precision, extra_tool=extra_tool)
