#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Alexis Maldonado Herrera <maldonad at cs.tum.edu> Federico Ruiz-Ugalde <memeruiz@gmail.com>
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


class Joint_sim():
    def __init__(self,portbasename,jointsimportbasename):
        self.torque_out_port=yarp.BufferedPortBottle()
        self.torque_out_port.open(portbasename+"/joint_sim_client/torque_out")
        yarp.Network.connect(portbasename+"/joint_sim_client/torque_out", jointsimportbasename+"/torque_in")

    def set_torques(self,torques):
        torque_out_bottle=self.torque_out_port.prepare()
        torque_out_bottle.clear()
        for i in torques:
            torque_out_bottle.addDouble(i)
        self.torque_out_port.write()

class Finger_sim():
    def __init__(self,portbasename,finger,fingersimbaseportname):
        self.torque_out_port=yarp.BufferedPortBottle()
        basename=portbasename+"/finger"+str(finger)
        self.torque_out_port.open(basename+"/torque_out")
        yarp.Network.connect(basename+"/torque_out", fingersimbaseportname+"/torque_in")

    def set_torques(self,torques):
        torque_out_bottle=self.torque_out_port.prepare()
        torque_out_bottle.clear()
        for i in torques:
            torque_out_bottle.addDouble(i)
        self.torque_out_port.write()

class Hand_sim():
    def __init__(self,handedness,sahand_port):
        self.fingers_sim_client=[]
        for i in xrange(4):
            self.fingers_sim_client.append(Finger_sim("/torque_sim_"+handedness,i,"/sahand"+str(sahand_port)+"/finger"+str(i)))
        #self.fingers_sim_client=map(Finger_sim, [["/torque_sim_"+handedness,i,"/sahand"+str(sahand_port)+"/finger"+str(i)] for i in xrange(4)])

    def set_torques(self,torques):
        for torque, finger_sim_client in zip(torques,self.fingers_sim_client):
            finger_sim_client.set_torques(torque)

    def set_torques_np(self,torques):
        #print "Torques", torques
        for i in xrange(4):
            #print "Torques", torques[i,:]
            self.fingers_sim_client[i].set_torques(torques[i,:])

def main():
    return(False)



if __name__ == "__main__":
    main()


