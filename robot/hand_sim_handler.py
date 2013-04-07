#!/usr/bin/python

import sys,signal
sys.path.append("../sahand_api")
#from sahand.py import THUMB,FIRST_FINGER,MIDDLE_FINGER,RING_FINGER
from numpy import array,identity,zeros, dot, pi, concatenate, diag
from numpy.linalg import pinv, svd
import yarp
#from hands_kin import fingers_lim,fingers_coupling, motor_locked_joints,sensor_locked_joints, sensor_finger_coupling
#import hands_kin as kin
import numpy
#sys.path.append("../../../../control/motionControl")
#sys.path.append("../python-pyrovito")
#sys.path.append("../vflibrary")
from arcospyu.dprint import dprint,d, Dprint
d.level=Dprint.ERROR


from multiprocessing import Process, Queue


import time
from arcospyu.control import nanosleep
import pickle


THUMB=0
FIRST_FINGER=1
MIDDLE_FINGER=2
RING_FINGER=3
all_finger_list=[THUMB,FIRST_FINGER,MIDDLE_FINGER,RING_FINGER]

def weighted_pseudo_inverse(matrix, weight_left, weight_right):
    U,s,Vh=svd(dot(dot(weight_left,matrix),weight_right))
    Uw=dot(weight_left,U)
    Vhw=dot(weight_right,Vh)
    
    Si=zeros((matrix.shape[1],matrix.shape[0]))
    for i,j in enumerate(s):
        Si[i,i]=1.0/j
    return(dot(Vhw.T,dot(Si,Uw.T)))

def kdl_jac_to_numpy_array(in_jac):
    out_jac=zeros((in_jac.rows(),in_jac.columns()))
    for row in xrange(in_jac.rows()):
        for col in xrange(in_jac.columns()):
            out_jac[row,col]=in_jac[row,col]
    return(out_jac)

def kdl_frame_to_numpy_array(kdl_frame):
    out_frame=zeros((4,4))
    for row in xrange(4):
        for col in xrange(4):
            out_frame[row,col]=kdl_frame[row,col]
    return(out_frame)

def kdl_jntarray_to_numpy_array(kdl_array):
    out=array([0.0]*kdl_array.rows())
    for row in xrange(kdl_array.rows()):
        out[row]=kdl_array[row]
    return out

class Finger_controller_cart_pos(object):
    '''Cart position controller'''

    def __init__(self,parent):
        import vfLibrary as vf
        self.parent=parent
        self.name="cart_pos"
        self.ref=numpy.identity(4)
        #TODO: set nice reference for finger( the best would be current first position
        self.params=array([0.0]*self.parent.torque_controlled_num_joints)
        self.control_signal_jpos=array([0.0]*self.parent.num_joints)
        self.control_stiffness=False
        self.controller_joint_velocity=Finger_controller_joint_vel(self.parent)
        self.vfDB=vf.vectorFieldLibrary()
        self.vector_field=self.vfDB[1]() #Point attractor
        self.slow_down_distance=0.01
        print numpy.concatenate((self.ref.reshape(16),numpy.array([self.slow_down_distance])))
        self.vector_field.setParams(numpy.concatenate((self.ref.reshape(16),numpy.array([self.slow_down_distance]))))
        self.control_signal_jpos=array([0.0]*self.parent.num_joints)
        self.control_signal_jvel=array([0.0]*self.parent.torque_controlled_num_joints)
        self.control_signal_jstiff=array([0.0]*self.parent.torque_controlled_num_joints)

    def set_refs(self,ref):
        ''' Sets the cartesian position controller reference values
        ref: 4x4 frame matrix as a 16 values list or as a 4x4 numpy array
        units: The position part is in meters'''
        temp_ref=array(ref)
        if temp_ref.ndim==1:
            if len(temp_ref)==16:
                self.ref=temp_ref.reshape((4,4))
        elif (temp_ref.ndim==2) and (temp_ref.shape==(4,4)):
            self.ref=temp_ref
        else:
            print "wrong reference size"

    def set_params(self,params):
        '''Sets the cartesian position controller parameters
        params: stiffness [base, proximal, distal]
        units: Nm/degree TODO: change to Nm/radian
        '''
        if len(params) != self.parent.torque_controlled_num_joints:
            print "incorrect params, ignoring"
        else:
            self.params=array(params)


    def update_control_signals(self):
        '''Control signal calculation, should calculate position, velocity, stiffness (optional) '''
        # Put your control algorithm here. fill in control_signal_jpos, control_signal_jvel, control_signal_jstiff signals
        #get current cartesian position
        self.parent.kinematics.get_cart_pos()
        self.cur_frame_np=kdl_frame_to_numpy_array(self.parent.kinematics.cur_frame_kdl)
        self.cur_frame_np.resize(16)
        pos=self.vector_field.getVector(self.cur_frame_np)[:3]*self.vector_field.getScalar(self.cur_frame_np)[0]
#        print pos
        rot=self.vector_field.getVector(self.cur_frame_np)[3:]*self.vector_field.getScalar(self.cur_frame_np)[1]
#        print rot
        self.out_cart_vel=concatenate((pos,rot))
        #print self.out_cart_vel
        #calculate the error
        #Multiply by some constant
        #convert to joint speeds
        if self.parent.kinematics.calc_vik(self.out_cart_vel)<0:
            print "Error during velocity inverse kinematics"
            return(-1)
        else:
#            print "Number of joints", self.parent.kinematics.motor_chain.getNrOfUnlockedJoints()
#            print "Qdot out", kdl_jntarray_to_numpy_array(self.parent.kinematics.qdot_out_kdl)
            self.controller_joint_velocity.set_refs(dot(pinv(self.parent.kinematics.coupling_matrix),kdl_jntarray_to_numpy_array(self.parent.kinematics.qdot_out_kdl)))
            self.controller_joint_velocity.update_control_signals()
            self.control_signal_jpos=self.controller_joint_velocity.control_signal_jpos
            self.control_signal_jvel=self.controller_joint_velocity.control_signal_jvel
            self.control_signal_jstiff=self.controller_joint_velocity.control_signal_jstiff

class Finger_controller_joint_vel(object):
    '''Joint velocity controller'''

    def __init__(self,parent):
        self.parent=parent
        self.name="joint_vel"
        self.ref=array([0.0]*self.parent.num_joints)
        self.params=array([0.0]*self.parent.torque_controlled_num_joints)
        self.control_signal_jpos=array([0.0]*self.parent.num_joints)
        self.control_stiffness=False

    def set_refs(self,ref):
        '''Sets the joint velocity controller reference values
        ref: python list or numpy array of joint velocities [(thumb_base(opt)), base, proximal, distal]
        units: radians/s
        '''
        if len(ref) != self.parent.num_joints:
            print "incorrect reference, ignoring"
            print "Quantity of values must be:", self.parent.num_joints
        else:
            self.ref=array(ref)

    def set_params(self,params):
        '''Sets the joint velocity controller parameters
        params: stiffness [base, proximal, distal]
        units: Nm/degree TODO: change to Nm/radian
        '''
        if len(params) != self.parent.torque_controlled_num_joints:
            print "incorrect params, ignoring"
        else:
            self.params=array(params)

    def update_control_signals(self):
        '''Control signal calculation, should return 6 or 9 parameters: position, velocity, stiffness (optional) '''
        # Put your control algorithm here. fill in control_signal_jpos, control_signal_jvel, control_signal_jstiff signals
        #This controller just sends extreme positions with a variable velocity. For zero velocity it must tell the finger to go where it is right now.
        self.control_signal_jpos=self.control_signal_jpos
        biggers=numpy.greater(self.ref,array([0.0]*self.parent.num_joints))
        smallers=numpy.less(self.ref,array([0.0]*self.parent.num_joints))
        for i,(less,greater,ref) in enumerate(zip(smallers,biggers,self.ref)):
            if ref!=0:
                if less:
                    self.control_signal_jpos[i]=self.parent.kinematics.limits[i][0]
                if greater:
                    self.control_signal_jpos[i]=self.parent.kinematics.limits[i][1]
            else:
                self.control_signal_jpos[i]=self.parent.cur_joint_pos[i] #NEW, test
        print "Current position", self.parent.cur_joint_pos*180/pi
        print "Control position", self.control_signal_jpos*180/pi
        self.control_signal_jvel=numpy.abs(self.ref[-3:])
        self.control_signal_jstiff=self.params

class Finger_controller_sahand_joint_pos(object):
    '''Controller class for the internal sahand controller
    It makes available the controller methods
    This class is pretty stupid, it basically just stores information
    to send later to the hand.
    '''
    
    def __init__(self,parent):
        '''parent can be used in the controller for reading current data (with sahand controllers this is not necessary) '''
        from numpy import concatenate
        self.parent=parent
        self.ref_joint_pos=array([0.0]*self.parent.num_joints)
        if self.parent.num_joints==4:
            self.ref_joint_pos=array([1.4, 0. ,0.2, 0.2])
        else:
            self.ref_joint_pos=array([0. ,0.12, 0.12])
        self.ref=self.ref_joint_pos
        joint_pos_controller_vel=array([50.0*pi/180.0]*parent.torque_controlled_num_joints)
        joint_pos_controller_stiffness=array([0.01*180.0/pi]*parent.torque_controlled_num_joints)
        self.params=concatenate((joint_pos_controller_vel,joint_pos_controller_stiffness))
        self.name="sahand"
        self.control_stiffness=False

    def set_refs(self,ref,offsets=False):
        '''Sets the position controller reference values (for the internal hand position controller)
        ref: python list or numpy array of joint angles. [base,proximal,distal] (for all but thumb finger)
        units: radians
        '''
        if len(ref)==self.parent.num_joints:
            if offsets:
                #print "ref", ref, "offsets", self.parent.angle_offsets
                self.ref=array(ref)+self.parent.angle_offsets #Be carefull could be + for some other reasons. Check the calibration procedure
                #print "new ref", self.ref
            else:
                self.ref=array(ref)
        else:
            print "Incorrect number of reference values"

    def set_params(self,params):
        '''Sets the position controller params (for the internal hand position controller)
        params: python list or numpy array. [vel_base,vel_proximal,vel_distal,stiff_base,stiff_proximal,stiff_distal] (for all but thumb finger)
        units: radians/s and Nm/radians
        Max speed: 180 deg/s = pi rad/s
        Max stiffness: 0.5Nm/deg,0.5Nm/deg,0.15Nm/deg= 28.65Nm/rad, 28.65Nm/rad, 8.59Nm/rad
        '''
        if len(params)==len(self.params):
            self.params=array(params)
        else:
            print "Incorrect number of parameters"

    def update_control_signals(self):
        '''Control signal calculation, should return 6 or 9 parameters: position, velocity, stiffness (optional)
        or one more if the finger is the thumb
        '''
        # Put your control algorithm here. fill in control_signal_jpos, control_signal_jvel, control_signal_jstiff signals
        self.control_signal_jpos=self.ref # 4 values for thumb , 3 for other fingers
        self.control_signal_jvel=self.params[:3] # 3 values for speed
        self.control_signal_jstiff=self.params[3:6] # 3 values for stiffness

class Kinematics(object):
    '''This deals with the finger kinematics, forces, speeds, and positions in cartesian are here calculated, also the inverses of this'''
    def __init__(self,parent,handedness):
        '''parent is the finger parent'''

        #kinematics
        self.parent=parent
        self.config_hand=self.parent.config_hand
        ch=self.config_hand
        #from hands_kin import hands_kin, fingers_lim, fingers_coupling, motor_locked_joints, sensor_locked_joints, sensor_finger_coupling
        from PyKDL import Chain,ChainFkSolverPos_recursive,JntArray,Frame
        from PyKDL import ChainFkSolverVel_recursive,FrameVel
        from PyKDL import ChainJntToJacSolver, Jacobian, JntArrayVel
        from PyKDL import ChainIkSolverVel_wdls, Twist
        self.segments=ch.hands_kin[handedness][self.parent.num_finger]
        self.limits=ch.fingers_lim[self.parent.num_finger]
        self.coupling_matrix=ch.fingers_coupling[self.parent.num_finger]
        self.sensor_finger_coupling=ch.sensor_finger_coupling
        self.motor_locked_joints=ch.motor_locked_joints[self.parent.num_finger]
        self.sensor_locked_joints=ch.sensor_locked_joints[self.parent.num_finger]
        print "Motor_locked", self.motor_locked_joints
        print "Sensor_locked", self.sensor_locked_joints

        #chain motor
        self.motor_chain=Chain()
        for segment in self.segments:
            self.motor_chain.addSegment(segment)
        self.motor_chain.setCoupling(self.motor_locked_joints,self.coupling_matrix.tolist())

        #chain sensor
        self.sensor_chain=Chain(self.motor_chain)
        self.sensor_chain.setCoupling(self.sensor_locked_joints,self.sensor_finger_coupling.tolist())

        #pos fk
        self.pfk_solver=ChainFkSolverPos_recursive(self.motor_chain)
        self.cur_jpos_kdl=JntArray(self.motor_chain.getNrOfJoints())
        self.cur_frame_kdl=Frame()

        #vel fk
        self.vfk_solver=ChainFkSolverVel_recursive(self.motor_chain)
        self.cur_jvel_kdl=JntArrayVel(self.motor_chain.getNrOfJoints()) #TODO for the thumb we don't have joint velocity yet, it has to be calculated.
        self.cur_framevel_kdl=FrameVel()

        #vel ifk
        from numpy import finfo, double
        self.smallest=finfo(double).eps
        self.vik_solver=ChainIkSolverVel_wdls(self.motor_chain,self.smallest)
        #TODO: make set weights methods
        self.qdot_out_kdl=JntArray(self.motor_chain.getNrOfUnlockedJoints())
        self.tw=Twist()

        #jac solver
        self.jac_solver_sensors=ChainJntToJacSolver(self.sensor_chain)
        self.jac_solver_sensors
        self.jac_sensors=Jacobian(self.sensor_chain.getNrOfIndJoints())
        self.forces=array([0.0]*6)

        #finger calibration values
        #angle offsets in radians of the finger joints
        self.base_angle_offset=0.0
        self.proximal_angle_offset=0.0
        self.distal_angle_offset=0.0
        self.distal2_angle_offset=0.0

        #weight matrices for the statics inverse
        self.static_weight_left=diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        self.static_weight_right=identity(self.sensor_chain.getNrOfIndJoints())
        self.static_weight_right=identity(self.sensor_chain.getNrOfUnlockedJoints())

    def get_cart_pos(self,segment_num=-1):
        if self.pfk_solver.JntToCart(self.cur_jpos_kdl,self.cur_frame_kdl,segment_num)<0:
            print "Error in cart pos calculation"

    def get_cart_vel(self):
        #TODO: not tested yet
#        for i,(angle,speed) in enumerate(zip(self.parent.cur_joint_pos,self.parent.cur_joint_vel)):
#            self.cur_jpos_kdl[i]=angle
#            self.cur_jvel_kdl.q[i]=angle
#            self.cur_jvel_kdl.qdot[i]=speed
        if self.vfk_solver.JntToCart(self.cur_jvel_kdl,self.cur_framevel_kdl)<0:
            print "Error in cart vel calculation"

    def calc_vik(self,tw_list):
        if len(tw_list)==6:
            for i in xrange(6):
                self.tw[i]=tw_list[i]
            if self.vik_solver.CartToJnt(self.cur_jpos_kdl,self.tw,self.qdot_out_kdl)<0:
                print "Error in velocity inverse kinematics"
                return(-1)
        else:
            print "Error wrong number of values"
            return(-1)
        return(0)

    def update_cur_jvel_kdl(self):
        j=0
        temp=concatenate((self.parent.cur_joint_vel,array([self.parent.cur_joint_vel[-1]])))
        for i,locked in enumerate(self.sensor_locked_joints):
            self.cur_jvel_kdl.q[i]=self.cur_jpos_kdl[i]
            if locked:
                self.cur_jvel_kdl.qdot[i]=temp[j]
                j+=1
            else:
                self.cur_jvel_kdl.qdot[i]=0.0
        

    def update_cur_jpos_kdl(self):
        j=0
        temp=concatenate((self.parent.cur_joint_pos,array([self.parent.cur_joint_pos[-1]])))
        for i,locked in enumerate(self.sensor_locked_joints):
            if locked:
                self.cur_jpos_kdl[i]=temp[j]
                j+=1
            else:
                self.cur_jpos_kdl[i]=0.0


    def get_forces(self):
        self.update_cur_jpos_kdl()
        #print "Test1"
        #print "Cur jpos kdl", [self.cur_jpos_kdl[i] for i in xrange(8)]
        if self.jac_solver_sensors.JntToJac(self.cur_jpos_kdl,self.jac_sensors,True)<0:
            print "Error in sensor jacobian calculation"
            return
        #print "Test2"

        jac_sensors_np=kdl_jac_to_numpy_array(self.jac_sensors)
#        print "pinv(jac.T)", pinv(jac_sensors_np.T)
#        print "cur torques", self.parent.cur_torques
        #print "Test3"
        try:
            self.forces=dot(weighted_pseudo_inverse(jac_sensors_np.T,self.static_weight_right,self.static_weight_left),self.parent.cur_torques)
        except Exception,e:
            print "Error during force calculation, force value not updated. Error:", e

    def calc_torques(self,forces):
        if self.jac_solver_sensors.JntToJac(self.cur_jpos_kdl,self.jac_sensors,True)<0:
            print 
            print "Error in sensor jacobian calculation"
            return -1
        jac_sensors_np=kdl_jac_to_numpy_array(self.jac_sensors)
        torques=dot(jac_sensors_np.T,forces.T)
        #.resize(self.sensor_chain.getNrOfUnlockedJoints())
        #print "Torques", torques
        return(torques)
        #TODO: check for negative values in all the solvers calculations
        
class Finger(object):
    def __init__(self, config_hand, num_finger,handedness):
        self.config_hand=config_hand
        self.handedness=handedness
        self.num_finger=num_finger
        if self.num_finger==THUMB:
            self.num_joints=4
        else:
            self.num_joints=3
        self.torque_controlled_num_joints=3
        self.cur_joint_pos=array([0.0]*self.num_joints)
        self.cur_joint_vel=array([0.0]*self.num_joints)
        self.cur_torques=array([0.0]*self.torque_controlled_num_joints)

        self.cart_vel_dim=6
        self.forces_dim=6

        self.cur_cart_pos=identity(4)
        self.cur_cart_vel=array([0.0]*self.cart_vel_dim)
        self.cur_forces=array([0.0]*self.forces_dim)
        self.controller=Finger_controller_sahand_joint_pos(self)

        #torque calibration factors
        #self.base_torque_factor=((kin.dist_proximal_distal1-kin.pos_sensor_proximal1)+kin.dist_distal1_distal2+kin.dist_distal2_tip)/(kin.dist_proximal_distal1+kin.dist_distal1_distal2+kin.dist_distal2_tip)
        self.base_torque_factor=1.002
#        self.proximal_torque_factor=float(self.base_torque_factor)
        self.proximal_torque_factor=0.900
        #self.distal_torque_factor=(kin.dist_distal2_tip-kin.pos_sensor_distal)/(kin.dist_distal1_distal2+kin.dist_distal2_tip)
        self.distal_torque_factor=0.49218
        print "base_torque_factor", self.base_torque_factor
        print "proximal_torque_factor", self.proximal_torque_factor
        print "distal torque_factor", self.distal_torque_factor
        self.torque_calibration_factors=array([1.0]*self.torque_controlled_num_joints)

        #angle calibration offsets
        self.angle_offsets=array([0.0]*self.num_joints)

        self.kinematics=Kinematics(self,self.handedness)

    def set_controller(self,controller):
        '''controller: Use one of the controller classes'''
        self.controller=controller(self)

    def get_controller_name(self):
        return self.controller.name
        
    def update_cur_joint_pos(self,pos,offset=True):
        #This function is called automatically by the hand object when new data is retrieved.
        #Here we apply the joint offsets
        if offset:
            self.cur_joint_pos=array(pos)-self.angle_offsets #check
        else:
            self.cur_joint_pos=array(pos)
        self.kinematics.update_cur_jpos_kdl()

    def update_cur_joint_vel(self,vel):
        if len(vel)==self.num_joints:
            self.cur_joint_vel=array(vel)
            self.kinematics.update_cur_jvel_kdl()
        else:
            print "Wrong velocity dimension"

    def update_cur_torques(self,torques,factors=True):
        #This function is called automatically by the hand object
        #here is where the torque adjustment takes place.
        #self.cur_torques=array([torques[0]*self.base_torque_factor,torques[1]*self.proximal_torque_factor,torques[2]*self.distal_torque_factor])
#        self.cur_torques=array([torques[0]*self.base_torque_factor,torques[1]*self.proximal_torque_factor*0.72,torques[2]*self.distal_torque_factor])#0.62
# factors: 1.3,1.3,0.8 look fine
        if factors:
            dprint("With factors", self.torque_calibration_factors)
            self.cur_torques=array(torques)*self.torque_calibration_factors
            #self.cur_torques=array(torques)
        else:
            self.cur_torques=array(torques)
        #print "torques:", torques, self.cur_torques

    def get_joint_pos(self):
        '''
        Returns the current joint position
        returns: array([distal,proximal,base])
        units: radians
        '''
        return(self.cur_joint_pos)

    def get_joint_vel(self):
        '''
        Returns the current joint velocity
        returns: array([distal,proximal,base])
        units: radians/s
        '''
        return(self.cur_joint_vel)

    def get_torques(self):
        '''
        Returns the current torques
        returns: array([distal,proximal,base])
        units: N
        '''
        return(self.cur_torques)

    def get_cart_pos(self,segment_num=-1):
        '''
        Returns the current cartesian position of the finger tip
        returns: array(homogenous matrix)
        units: rot matrix unitless+meters
        '''
        #TODO
        if segment_num==-1:
            segment_num=len(self.kinematics.segments)
        self.kinematics.get_cart_pos(segment_num=segment_num)
        for row in xrange(3):
            for col in xrange(4):
                self.cur_cart_pos[row,col]=self.kinematics.cur_frame_kdl[row,col]
        self.cur_cart_pos[3,0]=0.
        self.cur_cart_pos[3,1]=0.
        self.cur_cart_pos[3,2]=0.
        self.cur_cart_pos[3,3]=1.
        return(self.cur_cart_pos)

    def get_cart_vel(self):
        '''
        Returns the current cartesian velocity of the finger tip
        returns: array([velx,vely,velz,velrotx,velroty,velrotz])
        units: m/s, radians/s
        '''
        #TODO
        self.kinematics.get_cart_vel()
        twist=self.kinematics.cur_framevel_kdl.GetTwist()
        for i in xrange(self.cart_vel_dim):
            self.cur_cart_vel[i]=twist[i]
        return(self.cur_cart_vel)

    def get_forces(self):
        '''
        Returns the current forces on the finger tip
        returns: array([forcex,forcey,forcez,torquex,torquey,torquez]) (wrench?)
        units: N, Nm
        '''
        #TODO
        self.kinematics.get_forces()
        self.cur_forces=self.kinematics.forces
        return(self.cur_forces)

def controller_f(cmd_queue,ack_queue,handedness,sahand_number,portprefix,sahand_port_name_prefix, config_hand, cycle_frequency=5.0):
    import signal
    sequence_number=0
    angles_field=range(3)
    speeds_field=range(3,6)
    torques_field=range(6,9)
    SEQUENCE_FIELD=0
    THUMB_ANGLE_FIELD=1
    fingers_string=["thumb","first","middle","ring"]
    client_port_name_suffix="/hand_client"
    fingers=[]
    num_fingers_from_server=4
    for num_finger in xrange(num_fingers_from_server):
        fingers.append(Finger(config_hand, num_finger,handedness))
    yarp.Network.init()
    hand_port=yarp.BufferedPortBottle()
    hand_port.open(portprefix+client_port_name_suffix+str(sahand_number))
    hand_port_ctrl=yarp.BufferedPortBottle()
    hand_port_ctrl.open(portprefix+client_port_name_suffix+str(sahand_number)+"/ctrl")
    yarp.Network.connect(sahand_port_name_prefix+str(sahand_number)+"/out",portprefix+client_port_name_suffix+str(sahand_number))
    yarp.Network.connect(portprefix+client_port_name_suffix+str(sahand_number),sahand_port_name_prefix+str(sahand_number)+"/in")
    yarp.Network.connect(portprefix+client_port_name_suffix+str(sahand_number)+"/ctrl",sahand_port_name_prefix+"/cmd")

    def terminate_handler(signum,stack_frame):
        print 'Signal handler called with signal', signum
        hand_port.close()
        hand_port_ctrl.close()
        yarp.Network.fini()
        sys.exit()

    def update_sensor_data(fingers,hand_port):
        '''Updates finger sensor data
        returns the sequence number
        '''
        #TODO for left hand we have to change the sign of the base angle for all the fingers to match the kinematic description.
#        print "test here"
        #init_time=time.time()

        pendingreads=hand_port.getPendingReads()
        if pendingreads>0:
            bottle=hand_port.read()
#            print "bottle", bottle
            #reading data from server (all fingers always)
            sequence_number=bottle.get(SEQUENCE_FIELD).asInt()
            for fingerbottle,finger in zip(map(yarp.Value.asList,map(bottle.get,range(2,bottle.size()))),all_finger_list):
                if finger==THUMB:
                    thumb_angle=bottle.get(THUMB_ANGLE_FIELD).asDouble() #for thumb position
                    thumb_speed=0.0 #TODO: Calculate thumb speed
                    fingers[finger].update_cur_joint_pos([thumb_angle]+map(yarp.Value.asDouble,map(fingerbottle.get,angles_field)))
                    fingers[finger].update_cur_joint_vel([thumb_speed]+map(yarp.Value.asDouble,map(fingerbottle.get,speeds_field)))
                    #TODO: check if the thumb angle value should go at the beggining or at the end of the joint array
                else:
                    fingers[finger].update_cur_joint_pos(map(yarp.Value.asDouble,map(fingerbottle.get,angles_field)))
                    fingers[finger].update_cur_joint_vel(map(yarp.Value.asDouble,map(fingerbottle.get,speeds_field)))
                #fingers[finger].update_cur_torques(array(kin.torque_factors)*array(map(yarp.Value.asDouble,map(fingerbottle.get,torques_field))))
                fingers[finger].update_cur_torques(array(map(yarp.Value.asDouble,map(fingerbottle.get,torques_field))),factors=False)

            #final_time=time.time()
            #print "Time", final_time-init_time
    def send_ctrl_signal_server(fingers,hand_port,stiffness=False,list_fingers=[]):
        if len(list_fingers)>0:
            cmd_bottle=hand_port.prepare()
            cmd_bottle.clear()
            for finger in list_fingers:
                finger_name=fingers_string[finger]
                cmd_bottle_finger=cmd_bottle.addList()
                cmd_bottle_finger.addString(finger_name)
                cmd_bottle_finger_data=cmd_bottle_finger.addList()
                    #for transmiting also stiffness TODO: use control signal here now
                    #data=fingers[finger].controller.ref[:3].tolist()+fingers[finger].controller.params.tolist()
                if stiffness:
                    data=fingers[finger].controller.control_signal_jpos.tolist()[-3:]+fingers[finger].controller.control_signal_jvel.tolist()+fingers[finger].controller.control_signal_jstiff.tolist()
                else:
                    data=fingers[finger].controller.control_signal_jpos.tolist()[-3:]+fingers[finger].controller.control_signal_jvel.tolist()
                for item in data:
                    cmd_bottle_finger_data.addDouble(item)
                if finger==THUMB:
                    #seting thumb angle
                    cmd_bottle_finger.addDouble(fingers[finger].controller.control_signal_jpos[0])
#            print "Bottle", cmd_bottle.toString()
            hand_port.writeStrict()
        
    def send_ctrl_server(ctrl_port,ctrl_cmd):
        cmd_bottle=ctrl_port.prepare()
        cmd_bottle.clear()
        cmd_bottle.addString(ctrl_cmd)
        ctrl_port.writeStrict()

    signal.signal(signal.SIGTERM,terminate_handler)
    signal.signal(signal.SIGINT,terminate_handler)
    sequence_number=0
    angles_field=range(3)
    speeds_field=range(3,6)
    torques_field=range(6,9)
    SEQUENCE_FIELD=0
    THUMB_ANGLE_FIELD=1
    fingers_state=[]
    num_of_parameters_server=3
    last_time=time.time()
    out_data=array([[[0.]*4]*3]*4)

    while True:
        new_cmd=False
        if all([fingers[num_finger].controller.name=="sahand" for num_finger in xrange(num_fingers_from_server)]):
            #print "All fingers are set to sahand controller"
            cmd=cmd_queue.get(True) #blocks completely because all controllers are sahand and sahand control is done inside the hand
            #print "Command received"
            new_cmd=True
        else:
            if not cmd_queue.empty():
                cmd=cmd_queue.get()
                new_cmd=True
        if new_cmd:
            #print "New command received", cmd
            if cmd[0]=="update_sensor_data":
                #print "Updating sensor data"
                #print "Update sensor data"

                sequence_number=update_sensor_data(fingers,hand_port)
#                data=[]
#                for num_finger in xrange(num_fingers_from_server):
#                    data.append(fingers[num_finger].get_joint_pos())
#                    data.append(fingers[num_finger].get_joint_vel())
#                    data.append(fingers[num_finger].get_torques())
                for num_finger in xrange(num_fingers_from_server):
                    if num_finger==0:
                        out_data[num_finger,0,:]=fingers[num_finger].get_joint_pos()
                        out_data[num_finger,1,:]=fingers[num_finger].get_joint_vel()
                    else:
                        out_data[num_finger,0,:3]=fingers[num_finger].get_joint_pos()
                        out_data[num_finger,1,:3]=fingers[num_finger].get_joint_vel()
                    out_data[num_finger,2,:3]=fingers[num_finger].get_torques()
                        
                #ack_queue.put(["update_sensor_data_finished",out_data])
                ack_queue.put(out_data)

            elif cmd[0]=="set_controller":
                #cmd[1] is finger number, cmd[2] is an initialize object of a controller class
                fingers[cmd[1]].set_controller(cmd[2])
            elif cmd[0]=="update_ref":
                #cmd[1] is finger number, cmd[2] is the new reference
                #print "update ref"
                fingers[cmd[1]].controller.set_refs(cmd[2],offsets=True)
                if fingers[cmd[1]].controller.name=="sahand":
                    fingers[cmd[1]].controller.update_control_signals()
                    send_ctrl_signal_server(fingers,hand_port,stiffness=False,list_fingers=[cmd[1]])
            elif cmd[0]=="update_angle_offsets":
                #cmd[1] is finger number, cmd[2] is the new reference
                fingers[cmd[1]].angle_offsets=cmd[2]
                if fingers[cmd[1]].controller.name=="sahand":
                    fingers[cmd[1]].controller.update_control_signals()
                    send_ctrl_signal_server(fingers,hand_port,stiffness=False,list_fingers=[cmd[1]])
            elif cmd[0]=="update_params": 
                #cmd[1] is finger number, cmd[2] is the new parameters
                fingers[cmd[1]].controller.set_params(cmd[2])
                if fingers[cmd[1]].controller.name=="sahand":
                    fingers[cmd[1]].controller.update_control_signals()
                    send_ctrl_signal_server(fingers,hand_port,stiffness=True,list_fingers=[cmd[1]])
            elif cmd[0]=="update_cycle_freq":
                cycle_frequency=cmd[1]
            elif (cmd[0]=="calibrate") or (cmd[0]=="string_cmd"):
                send_ctrl_server(hand_port_ctrl,cmd[1])
            else:
                print "Warning: Command ", cmd[0], " not recognized"

        #Here goes the control cycle for other type of controllers    
        sequence_number=update_sensor_data(fingers,hand_port)
        #updated data now available in fingers[finger].get_*
        for num_finger in xrange(num_fingers_from_server):
            if fingers[num_finger].controller.name!="sahand":
                fingers[num_finger].controller.update_control_signals() #control processing is done here
                send_ctrl_signal_server(fingers,hand_port,fingers[num_finger].controller.control_stiffness,list_fingers=[num_finger]) #send signals to server
            else:
                #don't do anything since control signals are sent when the references or parameters are updated
                pass

        #timing calculation and waiting
        new_time=time.time()
        diff_time=new_time-last_time
        period_time=1.0/cycle_frequency
        wait_time=period_time-diff_time
#        print "Wait time:", wait_time
#        print "All controllers sahand?", all([fingers[num_finger].controller.name=="sahand" for num_finger in xrange(num_fingers_from_server)])
        if (wait_time<0.0) or (all([fingers[num_finger].controller.name=="sahand" for num_finger in xrange(num_fingers_from_server)])):
            #print "Desired control cycle not attained, use a slower control cycle or get a faster computer" #this is wrong FIX TODO
            wait_time=0.0
        last_time=new_time
        sleep_sec=int(wait_time)
        sleep_nsec=(wait_time-sleep_sec)*1000000000
        nanosleep.nanosleep(sleep_sec,int(sleep_nsec))
                
    hand_port.close()
    yarp.Network.fini()

class Hand(object):
    '''Lets controls the kimp robot hands
    it starts another process which is responsible for the control and comunication of the robot
    when we select to use the sahand internal controller, the other process only relays the information from here.
    This class creates an object for each finger, each finger have an associated controller, each finger can have a different controller at the same time.
    '''
    def __init__(self,config_hand, handedness="right",sahand_number=0,portprefix="",sahand_port_name_prefix="/sahand"):
        self.config_hand=config_hand
        self.cmd_queue=Queue()
        self.ack_queue=Queue()
        self.sahand_number=sahand_number
        self.controller_p=Process(target=controller_f,args=(self.cmd_queue,self.ack_queue,handedness,self.sahand_number,portprefix,sahand_port_name_prefix, self.config_hand))
        self.handedness=handedness
        self.fingers=[]
#        self.fingers_string=["thumb","first","middle","ring"]
        self.num_fingers_from_server=4
        for num_finger in xrange(self.num_fingers_from_server):
            self.fingers.append(Finger(self.config_hand, num_finger,handedness))
        self.controller_p.start()
#        for i in xrange(num_fingers_from_server):
#            self.fingers_state.append([])
#            for j in xrange(num_of_parameters):
#                self.giners_state[i].append([0.0]*3)
#        self.fingers_state.append([])

    def __del__(self):
        #TODO maybe move the hand to a initial position and set stiffness low.
        #stop the process
        print "Deleting hand"
        self.controller_p.terminate()
        self.controller_p.join()
        pass

    def set_controller(self,controller,finger_list=all_finger_list):
        '''sets the finger controller (its a controller class)'''
        for finger in finger_list:
            print "set controller finger:", finger
            self.fingers[finger].set_controller(controller)
            self.cmd_queue.put(["set_controller",finger,controller])

    def pause_controller(self,finger_list=all_finger_list):
        '''This pauses sending commands to the server for the given fingers'''
        #It should actualy send a command to the fingers that will stop them.
        #We have to comunitcate this to the server somehow. TODO
        for finger in finger_list:
            if self.fingers[finger].controller.get_name()=="sahand":
                print "This controller is always running just call control when you want to set a new reference"
            else:
                print "For other types of controllers we have to start a processes that calls control continuously"
                #TODO

    def resume_controller(self,finger_list=all_finger_list):
        '''This resumes sending commands to the server for the given fingers'''
        for finger in finger_list:
            if self.fingers[finger].controller.get_name()=="sahand":
                print "This controller is always running just call control when you want to set a new reference"
            else:
                print "For other types of controllers we have to start a processes that calls control continuously"
                #TODO

    def update_sensor_data(self):
        #updates the joint values on all the fingers from the controller process
        init_time=time.time()
        self.cmd_queue.put(["update_sensor_data"])
        result=self.ack_queue.get()
        final_time=time.time()
        #print "Time", final_time-init_time
#        print "Update result", result
        #if result[0]=="update_sensor_data_finished":
#            print "New data received"
        for num_finger in xrange(self.num_fingers_from_server):
                finger_data=result
                if num_finger==0:
                    self.fingers[num_finger].update_cur_joint_pos(finger_data[num_finger,0],offset=False)
                    self.fingers[num_finger].update_cur_joint_vel(finger_data[num_finger,1])
                else:
                    self.fingers[num_finger].update_cur_joint_pos(finger_data[num_finger,0,:3],offset=False)
                    self.fingers[num_finger].update_cur_joint_vel(finger_data[num_finger,1,:3])
                self.fingers[num_finger].update_cur_torques(finger_data[num_finger,2,:3],factors=True)
        #else:
        #    print "Error" #TODO
        #    print "Data not updated"

    def update_controller_refs(self,list_fingers=all_finger_list):
        '''Sends the current local reference values to the controller process for the selected fingers'''
        for num_finger in list_fingers:
            self.cmd_queue.put(["update_ref",num_finger,self.fingers[num_finger].controller.ref])

    def update_angle_offsets(self,list_fingers=all_finger_list):
        '''Sends the current local reference values to the controller process for the selected fingers'''
        for num_finger in list_fingers:
            self.cmd_queue.put(["update_angle_offsets",num_finger,self.fingers[num_finger].angle_offsets])

    def update_controller_params(self,list_fingers=all_finger_list):
        '''Sends the current local parameter values to the controller process for the selected fingers'''
        for num_finger in list_fingers:
            self.cmd_queue.put(["update_params",num_finger,self.fingers[num_finger].controller.params])

    def update_cycle_frequency(self,cycle_frequency):
        self.cmd_queue.put(["update_cycle_freq",cycle_frequency])

    def set_ref(self,finger,ref, offsets=False):
        '''Sets the reference of the selected finger'''
        self.fingers[finger].controller.set_refs(ref,offsets=offsets)
        
    def set_params(self,finger,params):
        '''Sets the reference of the selected finger'''
        self.fingers[finger].controller.set_params(params)

    def calibrate(self,simple=True):
        #stop local controller, calibrate, then restart local controller TODO
        if simple:
            ctrl_cmd='scal'+str(self.sahand_number)
        else:
            ctrl_cmd='cal'+str(self.sahand_number)
        self.cmd_queue.put(["calibrate",ctrl_cmd])

    def send_string_cmd_server(self,cmd):
        self.cmd_queue.put(["string_cmd",cmd])
    
#We make the hand object with the finger inside. We can enable or disable individual fingers during the constructor, disabled fingers means that Hand_client will not transmit data to control this fingers, receiving still happens.
#We get data from the fingers by reading from the hand the data: hand.update_joints() or hand.update_cart() (update cart does an update_joints internally) and then accessing the finger data with: hand.fingers[thumb].get_joint_pos() hand.fingers[thumb].get_joint_torque() hand.fingers[thumb].get_cart_pos() hand.fingers[thumb].get_force() (can also add for cartesian and joint velocity)
#There is a question open: When we update, should we update the hand server data only or some of the calculated data also (depending on a given list)?
#To move the fingers we first set the desired position/velocity/torque/force with: hand.fingers[thumb].set_joint_pos() .set_cart_pos() .set_joint_vel() .set_cart_vel() .set_torque() .set_force() and then we call hand.move(). There will be a different controller for each finger. The selected controller is determined by the last sent command.
