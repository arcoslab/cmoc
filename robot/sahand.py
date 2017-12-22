from numpy import array, pi
from wessling_hand.hand import Hand as Whand

CONTROLLER_IMPEDANCE = 4
CONTROLLER_POSITION = 2 
PORT_1 = 1
PORT_2 = 2

class Finger(object):
    def __init__(self, port, finger_n, parent):
        self.finger_n=finger_n
        self.parent=parent
        self.enabled=False
        self._ref_angles=array([0.0]*3)
        self._ref_speeds=array([120.0]*3)
        self._stiffness=array([0.023]*3)
        self._angles=array([0.0]*3)
        self._speeds=array([0.0]*3)
        self._torques=array([0.0]*3)
        self._torques_filter_buffer=[array([0.0]*3)]*100
        self._torques_offset=array([0.0]*3)

    def enable(self):
        self.parent.whand.enable(self.finger_n)
        self.parent.whand.send_cmd()
        #Send enable command
        pass

    def _set_stiffness(self,stiffness):
        # Send stiffness command to real hand
        self._stiffness=array(stiffness)
        self.send_cmds()

    def _get_stiffness(self):
        return(self._stiffness.tolist())

    stiffness = property(_get_stiffness, _set_stiffness)

    def _get_angles(self):
        return(self._angles)

    angles=property(_get_angles)

    def _get_speed(self):
        return(self._speeds)

    speeds=property(_get_speed)

    def _get_torques(self):
        return(self._torques-self._torques_offset)

    torques=property(_get_torques)

    def move(self, angles, speeds=(100.,100.,100.), interval=0.1):
        print "moving", angles
        self._ref_angles=array(angles)
        self._ref_speeds=array(speeds)
        self.send_cmds()
        #self.sim.set_q_ref(self.ref_angles*pi/180.0)
        #sim.set_qv_max(self.ref_vels)
        #self.update()
        #TODO: Send move command

    def clear_torque_sensor_offset(self):
        filtered_torque=0.
        for torque in self._torques_filter_buffer:
            filtered_torque+=torque
        filtered_torque/=len(self._torques_filter_buffer)
        print "Filtered torque: ", filtered_torque
        print "Defining torque offset"
        self._torques_offset=filtered_torque
        # TODO: read current torques and use them to substract future torques
        # this must be executed always in a predefined hand/fingers orientations to be useful together with calibration data
        pass

    def update(self):
        #move angle, speed and torque here
        self._angles=array(self.parent.whand.get_pos(self.finger_n))
        self._speeds=array(self.parent.whand.get_speed(self.finger_n))
        self._torques=array(self.parent.whand.get_torque(self.finger_n))
        self._torques_filter_buffer.pop(0)
        self._torques_filter_buffer.append(self._torques)

    def send_cmds(self):
        self.parent.whand.set_stiffnesses(self.finger_n, self._stiffness.tolist())
        self.parent.whand.set_speeds(self.finger_n, self._ref_speeds.tolist())
        self.parent.whand.set_poss(self.finger_n, self._ref_angles.tolist())
        self.parent.whand.send_cmd()

class Thumb(Finger):
    def __init__(self, port, finger_n, parent):
        Finger.__init__(self,port, finger_n, parent)
        pass

    def unbrake_base(self):
        pass

    def _get_base_angle(self):
        #New hand does not have this joint
        return(0.0)
    
    base_angle=property(_get_base_angle)

class Hand:
    def __init__(self, port, basename="/handclient"):
        self.whand=Whand(basename)
        self.port=port
        self.thumb=Thumb(self.port, 0, self)
        self.first=Finger(self.port, 1,  self)
        self.middle=Finger(self.port, 2, self)
        self.ring=Finger(self.port, 3, self)
        self.pinky=Finger(self.port, 4, self)

    def __iter__(self):
        return iter([self.thumb, self.first, self.middle, self.ring, self.pinky])

    def disable_emergency_stop(self):
        #TODO: Not important in wessling hand
        pass

    def get_hand_config(self):
        if self.port==PORT_1:
            return(1)
        elif self.port==PORT_2:
            return(2)
        else:
            return(0)

    def set_controller(self,controller):
        if (controller == CONTROLLER_IMPEDANCE) or (controller == CONTROLLER_POSITION):
            self.whand.set_controller_mode(1)
            self.whand.send_cmd()
        #TODO: Not important in wessling hand
        pass

    def update(self):
        self.thumb.update()
        self.first.update()
        self.middle.update()
        self.ring.update()
        self.pinky.update()

class Wessling_controller:
    def __init__(self, basename=""):
        #detect how many hands are connected and available some how. For now there is only support for one hand
        self.counter=0
        self.enabled_hands=[True, False]
        self.hands=[]
        if self.enabled_hands[0]:
            self.hands.append(Hand(PORT_1, basename+"/1"))
        if self.enabled_hands[1]:
            self.hands.append(Hand(PORT_2, basename+"/2"))

    def is_connected(self, port=PORT_1):
        if port in [PORT_1, PORT_2]:
            return self.enabled_hands[port-1]
        print("ERROR! Invalid hand "+str(port))
        return False

    def get_counter(self):
        #TODO: Check if data is new and increase counter
        # for now, using this function to receive new data
        counter=self.counter
        self.counter+=1
        for hand in self.hands:
            hand.whand.update_input()
            hand.update()
        return(counter)
