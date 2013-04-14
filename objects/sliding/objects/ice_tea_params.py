#!/usr/bin/env python
#Object parameters file.
from numpy import array, identity, pi
from cmoc.objects.sliding import LS

class Object_params(object):
    def __init__(self):
        #model parameters
        #self.box_dim=[0.094, 0.094, 0.23]
        self.box_dim=[0.09, 0.09, 0.23]
        self.box_center=array([0.,0.,0])
        self.box_planes=[[(1.,0.,0),(self.box_dim[0]/2.,0.,0.)], #(normal, point)
                         [(-1.,0.,0),(-self.box_dim[0]/2.,0.,0.)],
                         [(0.,1.,0),(0.,self.box_dim[1]/2.,0.)],
                         [(0.,-1.,0),(0,-self.box_dim[1]/2.,0.)],
                         [(0.,0.,1),(0.,0.,self.box_dim[2]/2.)],
                         [(0.,0.,-1),(0.,0.,-self.box_dim[2]/2.)]]
        self.box_vertices={
            0: array([[self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2],
                      [self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2]]),
            1: array([[-self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2],
                      [-self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2],
                      [-self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2],
                      [-self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2]]),
            2: array([[-self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2],
                      [-self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2]]),
            3: array([[self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2],
                      [self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2],
                      [-self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2],
                      [-self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2]]),
            4: array([[-self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2],
                      [self.box_dim[0]/2,-self.box_dim[1]/2,self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2],
                      [-self.box_dim[0]/2,self.box_dim[1]/2,self.box_dim[2]/2]]),
            5: array([[-self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2],
                      [self.box_dim[0]/2,self.box_dim[1]/2,-self.box_dim[2]/2],
                      [self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2],
                      [-self.box_dim[0]/2,-self.box_dim[1]/2,-self.box_dim[2]/2]])}
        self.weight_balance=0.4
        self.weight=0.418999761588 #kilos
        self.weight_force=9.81*self.weight
        self.friction_finger_z_force=0.
        self.friction_coef_object_table_static=0.297760967638
        self.friction_coef_finger_object_static=0.692988300356
        self.friction_coef_object_table=0.269154097429
        self.friction_coef_finger_object=0.567399204934
        self.fmax=self.friction_coef_object_table*(self.weight_force-self.friction_finger_z_force)
        self.mmax_base=LS.calc_mmax_analytic(self.friction_coef_object_table,1.,self.box_dim[0],self.box_dim[1])
        self.mmax=self.mmax_base*(self.weight_force-self.friction_finger_z_force)
        print "fmax, mmax", self.fmax, self.mmax

        self.tilt_height=self.box_dim[2]/2.-0.1*self.box_dim[2]
        self.max_tilt_angle=3.*pi/180.
        self.markers_transformations={ #this matrices represent what's the pose of the marker with respect of the center of the object
            "/4x4_1": array([[0., 0., -1., -self.box_dim[0]/2.],
                             [-1., 0., 0., 0.02],
                             [0., 1., 0., 0.0625],
                             [0., 0., 0., 1.]])
#            "/4x4_1": identity(4)
            }
