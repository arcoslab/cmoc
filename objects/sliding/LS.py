#!/usr/bin/python

from scipy.integrate import dblquad
from numpy import sqrt, exp, e, power
from numpy import arange, pi, sin, cos, tan, array, arctan2, arctan, dot, arcsinh
from numpy.linalg import norm
import matplotlib.pyplot as plt
from multiprocessing import Pool
from mpl_toolkits.mplot3d import Axes3D
import time,os


fay=lambda u,p,cx,cy,x,y: u*p*(x-cx)/sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy))
fax=lambda u,p,cx,cy,x,y: u*p*(cy-y)/sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy))
def fx(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=0.00001):
    fax2=lambda y,x: fax(u,p,cx,cy,x,y)
    return(dblquad(fax2,l1l,l1h,lambda x: l2l, lambda x: l2h,epsabs=eps, epsrel=eps))

def fy(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=0.00001):
    fay2=lambda y,x: fay(u,p,cx,cy,x,y)
    return(dblquad(fay2,l1l,l1h,lambda x: l2l, lambda x: l2h,epsabs=eps, epsrel=eps))

def m(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=0.00001):
    fay2=lambda x,y: fay(u,p,cx,cy,x,y)
    fax2=lambda x,y: fax(u,p,cx,cy,x,y)
    m=lambda y,x: x*fay2(x,y)-y*fax2(x,y)
    return(dblquad(m,l1l,l1h,lambda x:l2l, lambda x:l2h,epsabs=eps, epsrel=eps))

def LS_point((l1l,l1h,l2l,l2h,cx,cy,u,p),eps=0.1):
    return((fx(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=eps)[0],fy(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=eps)[0],m(l1l,l1h,l2l,l2h,cx,cy,u,p,eps=eps)[0]))

def calc_fmax(u,fn):
    return(u*fn)

def calc_mmax(u,fn,l1,l2):
    cx=0.0001
    cy=0.0001
    p=fn/(l1*l2)
    return(m(-l1/2.,l1/2,-l2/2,l2/2,cx,cy,u,p,eps=0.0001))

def calc_mmax_analytic(u,fn,l1,l2):
    #this works for centered object in l1/2 and l2/2 suposing center of rotation is in 0,0
    p=fn/(l1*l2)
    mmax=u*p*((l2**3)*arcsinh(l1/abs(l2))+(l1**3)*arcsinh(l2/l1)+2*l1*l2*sqrt((l2**2)+(l1**2)))/12
    return(mmax)



def create_data(cxrange,cyrange,step,l1,l2,u,den):
    list=[]
    p=Pool()
    arguments=[]
    steps=8.00001
    for cy in arange(-2.*l2,2.*l2,2.*l2/steps):
        for cx in arange(-2.*l1,2.*l1,2.*l1/steps):
            arguments.append((l1,l2,cx,cy,u,den))

    for i in arange(steps/2.):
        for j in arange(steps/2.):
            cx=2.*l1*exp(j)/e
            cy=2.*l2*exp(i)/e
            cx=2.*l1*power(1.7,j)/1.7
            cy=2.*l2*power(1.7,i)/1.7
            arguments.append((l1,l2,cx,cy,u,den))
            arguments.append((l1,l2,-cx,cy,u,den))
            arguments.append((l1,l2,cx,-cy,u,den))
            arguments.append((l1,l2,-cx,-cy,u,den))
#    for cy in arange(-cyrange,cyrange,step):
#        print "processing cy:", cy
#        LS_spec=lambda cx: LS_point(1,1,cx,cy,1,1)
#        arguments=[]
#        for cx in arange(-cxrange,cxrange,step):
#            arguments.append((1,1,cx,cy,1,1))
    print arguments
    list+=p.map(LS_point,arguments)
#        for cx in arange(-cxrange,cxrange,step):
#            list.append([fx(1,1,cx,cy,1,1)[0], fy(1,1,cx,cy,1,1)[0], m(1,1,cx,cy,1,1)[0]])
#        print list
    return(list)

def Rz(angle):
    return(array([[cos(angle),-sin(angle),0.0],[sin(angle),cos(angle),0],[0,0,1]]))

def Rx(angle):
    return(array([[1.,0,0],[0,cos(angle),-sin(angle)],[0,sin(angle),cos(angle)]]))

def Scale(x,y,z):
    return(array([[x,0.,0.],[0.,y,0.],[0.,0.,z]]))

def Shear(m):
    return(array([[1.,0.,0.],[0.,1.,m],[0.,0.,1.]]))

def create_data2(angle_steps_dir,angle_steps_rad,l1l,l1h,l2l,l2h,u,den):
    p=Pool()
    arguments=[]

    for a in arange(0.0,2*pi,2.*pi/(angle_steps_dir)): #cor direction
        for b in arange(pi/2./angle_steps_rad,pi/2.,pi/2./angle_steps_rad): #cor radius
            cx=(((l1h-l1l)/2.)*cos(a)/tan(b))+(l1h-l1l)/2.+l1l
            cy=((l2h-l2l)/2.)*sin(a)/tan(b)+(l2h-l2l)/2.+l2l
            arguments.append((l1l,l1h,l2l,l2h,cx,cy,u,den))
            #arguments.append((l1,l2,-cx,cy,u,den))

    #print arguments
    #raw_input()
    points=p.map(LS_point,arguments)
#        for cx in arange(-cxrange,cxrange,step):
#            list.append([fx(1,1,cx,cy,1,1)[0], fy(1,1,cx,cy,1,1)[0], m(1,1,cx,cy,1,1)[0]])
#        print list

    centroid=array(((l1h-l1l)/2+l1l,(l2h-l2l)/2+l2l))
    inc_angle=arctan(norm(centroid))*180/pi
    #inc_angle=0.
    dir_angle=arctan2(centroid[1],centroid[0])*180/pi
    print "centroid", centroid
    print "inclination angle", inc_angle
    print "direction angle", dir_angle
    temp=[]
    x=1.
    y=1/sqrt(1+norm(centroid)*norm(centroid))
    #y=1.
    fn=den*((l1h-l1l)*(l2h-l2l))
    mmax=calc_mmax_analytic(u,fn,l1h-l1l,l2h-l2l)
    z=1./(y*mmax)
    m=-0.5*sqrt(2)*norm(centroid)*0.0
    for i in points:
        temp.append(dot(Shear(m),dot(Scale(x,y,z),dot(Rx(-inc_angle*pi/180.),dot(Rz(-dir_angle*pi/180.0),array(i))))))
        

    return(temp)

def ellipsoid_points((Fx,Fy)):
    Fz=sqrt(1-(Fx**2+Fy**2))
    return((Fx,Fy,Fz))

def create_data_ellipsoid(angle_steps_dir,angle_steps_rad):
    p=Pool()
    arguments=[]

    for a in arange(0.0,2*pi,2.*pi/(angle_steps_dir)): #cor direction
        for b in arange(0.000001,pi/2.,pi/2./angle_steps_rad): #cor radius
            Fx=cos(a)*cos(b)
            Fy=sin(a)*cos(b)
            arguments.append((Fx,Fy))

    points=p.map(ellipsoid_points,arguments)

    return(points)

def superellipsoid_points((Fx,Fy,n1,n2,n3)):
    Fz=(1.-((abs(Fx))**n1+(abs(Fy))**n2))**(1/n3)
    print "Fz", Fx,Fy,Fz
    return((Fx,Fy,Fz))

def create_data_superellipsoid(angle_steps_dir,angle_steps_rad):
    p=Pool()
    arguments=[]

    for a in arange(0.0,2*pi,2.*pi/(angle_steps_dir)): #cor direction
        for b in arange(0.000001,pi/2.,pi/2./angle_steps_rad): #cor radius
            Fx=cos(a)*cos(b)
            Fy=sin(a)*cos(b)
            arguments.append((Fx,Fy,2.,3.3,1.1)) 

    points=p.map(superellipsoid_points,arguments)

    return(points)




def main():
    init_time=time.time()
#    list=create_data(2,2.01,0.30000001,1,1,1,1)
    l1l=-0.5
    l1h=0.5
    l2l=-10
    l2h=10
    u=1.
    p=1/((l1h-l1l)*(l2h-l2l))
    steps=15.
    points=create_data2(steps,l1l,l1h,l2l,l2h,u,p)
    print "\a"
    print "Processing time:", time.time()-init_time
    #print list
    a,b,c=zip(*points)
    fig=plt.figure()
    ax=Axes3D(fig)
    ax.set_xlabel("Fx")
    ax.set_ylabel("Fy")
    ax.set_zlabel("m")
    ax.scatter(a,b,c,'z')
    fname="ls_"+str(time.time())+".png"
    fig.savefig(fname)
    plt.show()



def main2():

    #object properties
    l1=1.
    l2=1.
    u=1.
    fn=1.
    p=fn/(l1*l2)
    fmax=calc_fmax(u,fn)
    mmax=calc_mmax_analytic(u,fn,l1,l2)
    print "Fmax, Mmax", fmax, mmax


    #generation steps
    steps=3.1
    steps_dir=40
    steps_rad=27.1
    steps_rad=15.1

    #plot stuff
    plt.ion()
    init_time=time.time()
    fig=plt.figure()
    colors=['r','g','b','c', 'm', 'y', 'k', 'w']
    color_n=0
    plots=[]
    ax=Axes3D(fig)
    ax.set_xlabel("Fx")
    ax.set_ylabel("Fy")
    ax.set_zlabel("m")

    #Coordinates generation
    coords=[]
    for x in arange(-0*l1/2,l1/2,l1/steps):
        coords.append((x,0*-l2/2))
        #coords.append((x,l2/2))
    for y in arange(-l2/2,l2/2,l2/steps):
        #coords.append((-l1/2,y))
        coords.append((l1/2,y))
    for x in arange(-l1/2,l1/2,l1/steps):
        #coords.append((x,-l2/2))
        coords.append((-x,l2/2))
    for y in arange(-l2/2,l2/2,l2/steps):
        coords.append((-l1/2,-y))
        #coords.append((l1/2,y))
    lss=[]
    offset=0. #coordinate origin offset (from center of object)

    #plot superellipsoid
    lss=create_data_superellipsoid(steps_dir,steps_rad)
    a,b,c=zip(*lss)
    #wframe=ax.scatter(a,b,c,'z',c='r')
    #plt.draw()
    #raw_input()
    #ax.collections.remove(wframe)
    wframe = None

    #plot ellipsoid
    lss=create_data_ellipsoid(steps_dir,steps_rad)
    a,b,c=zip(*lss)
    wframe=ax.scatter(a,b,c,'z',c='g')
    plt.draw()
    raw_input()
    #ax.collections.remove(wframe)
    wframe = None

    for num,coord in enumerate(coords):
        l1l=coord[0]-l1/2+offset
        l1h=coord[0]-l1/2+l1+offset
        l2l=coord[1]-l2/2+offset
        l2h=coord[1]-l2/2+l2+offset
        print "Processing: l1l l1h l2l l2h", l1l, l1h, l2l, l2h
        lss=create_data2(steps_dir,steps_rad,l1l,l1h,l2l,l2h,u,p)
        a,b,c=zip(*lss)
        color=colors[color_n]
        color_n+=1
        if color_n==len(colors):
            color_n=0
        oldcol = wframe
        color='b'
        wframe=ax.scatter(a,b,c,'z',c=color)
        #wframe=ax.plot_wireframe(array([a]),array([b]),array([c]),rstride=1, cstride=1)
        if oldcol is not None:
            ax.collections.remove(oldcol)
        #wframe.color(color)
        plt.draw()
        if num==0:
            raw_input()
        fname = '_tmp%03d.png'%num
        print 'Saving frame', fname
        fig.savefig(fname)
        last_num=num


    print 'Making movie animation.mpg - this make take a while'
    os.system("cp "+"_tmp000.png"+" "+'_tmp%03d.png'%(last_num+1))
    os.system("cp "+"_tmp000.png"+" "+'_tmp%03d.png'%(last_num+2))
    os.system("mencoder 'mf://_tmp*.png' -mf type=png:fps=10 -ovc lavc -lavcopts vcodec=wmv2 -oac copy -o animation%d.mpg"%(time.time()))
    os.system("rm _tmp*.png")
    print "\a"
    print "Processing time:", time.time()-init_time
    #print list
    #plt.show()


#import cProfile, pstats

if __name__=="__main__":
    main2()
#    cProfile.run('main()')
#    p = pstats.Stats("main")
#    p.sort_stats('cumulative').print_stats(10)



