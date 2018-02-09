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

# Object parameters file.
from numpy import array, identity, pi
from cmoc.objects.sliding import LS


class Object_params(object):
    def __init__(self):
        # model parameters
        self.box_dim = [0.062, 0.093, 0.18]
        self.box_center = array([0., 0., 0])
        self.box_planes = [
            [(1., 0., 0), (self.box_dim[0] / 2., 0., 0.)],  #(normal, point)
            [(-1., 0., 0), (-self.box_dim[0] / 2., 0., 0.)],
            [(0., 1., 0), (0., self.box_dim[1] / 2., 0.)],
            [(0., -1., 0), (0, -self.box_dim[1] / 2., 0.)],
            [(0., 0., 1), (0., 0., self.box_dim[2] / 2.)],
            [(0., 0., -1), (0., 0., -self.box_dim[2] / 2.)]
        ]
        self.box_vertices = {
            0:
            array(
                [
                    [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ]
                ]),
            1:
            array(
                [
                    [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ]
                ]),
            2:
            array(
                [
                    [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ]
                ]),
            3:
            array(
                [
                    [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ]
                ]),
            4:
            array(
                [
                    [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        self.box_dim[2] / 2
                    ]
                ]),
            5:
            array(
                [
                    [
                        -self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ], [
                        -self.box_dim[0] / 2, -self.box_dim[1] / 2,
                        -self.box_dim[2] / 2
                    ]
                ])
        }
        self.weight_balance = 0.520
        self.weight = 0.510163341925  # kilos
        self.weight_force = 9.81 * self.weight
        self.friction_finger_z_force = 0.
        self.friction_coef_object_table = 0.240329579329
        self.friction_coef_object_table_static = 0.313318050154
        self.friction_coef_finger_object = 0.733039972538
        self.friction_coef_finger_object_static = 0.884649835141
        self.fmax = self.friction_coef_object_table * (
            self.weight_force - self.friction_finger_z_force)
        self.mmax_base = LS.calc_mmax_analytic(
            self.friction_coef_object_table, 1., self.box_dim[0],
            self.box_dim[1])
        self.mmax = self.mmax_base * (
            self.weight_force - self.friction_finger_z_force)
        print "fmax, mmax", self.fmax, self.mmax

        self.tilt_height = self.box_dim[2] / 2. - 0.2 * self.box_dim[2]
        self.max_tilt_angle = 4. * pi / 180.
        self.markers_transformations={ # this matrices represent what's the pose of the marker with respect of the center of the object
            "/4x4_1": array([[0., 0., -1., -self.box_dim[0]/2.],
                             [-1., 0., 0., 0.02],
                             [0., 1., 0., 0.023+0.035/2.+0.0],
                             [0., 0., 0., 1.]])
            #            "/4x4_1": identity(4)
            }
