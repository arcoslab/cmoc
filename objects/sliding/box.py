from numpy import array

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
