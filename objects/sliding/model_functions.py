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

from numpy import array

def fc_to_nc_matrix(fmax,mmax,contact_pos):
    m=2*array([[mmax**2/fmax**2+contact_pos[1]**2,-contact_pos[0]*contact_pos[1]],
               [-contact_pos[0]*contact_pos[1],mmax**2/fmax**2+contact_pos[0]**2]])
    return(m)
