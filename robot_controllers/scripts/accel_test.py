#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import PyKDL

import sys
import pylab
import scipy.signal
from math import pi,asin

msgs = None
msgs2 = None

def odomCallback(msg):
    global msgs
    if msgs != None:
        msgs.append(msg)

def imuCallback(msg):
    global msgs2
    if msgs2 != None:
        msgs2.append(msg)

def cmdVel(vel):
    global msgs, msgs2
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, odomCallback)
    sub2 = rospy.Subscriber('/imu', Imu, imuCallback)
    dt = 0.1
    rate = rospy.Rate(1/dt) # 10hz
    twist = Twist() 
    t = 0.0
    msgs = []
    msgs2 = []
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()
        t+=dt
        if t < 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif t < 4:
            twist.linear.x = 0.0
            twist.angular.z = 4.0
        elif t < 6:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            break
    data = msgs
    data2 = msgs2
    msgs = None
    msgs2 = None

    filt = pylab.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.55, 0.6, 0.55, 0.5, 0.4, 0.3, 0.2, 0.1])
    filt /= filt.sum()

    t = pylab.array([d.header.stamp.to_sec() for d in data])
    x_vel = [d.twist.twist.linear.x for d in data]
    x_vel = pylab.correlate(x_vel,filt,mode='same')
    x_acc = [0.0] + [ (x_vel[i]-x_vel[i-1])/(t[i]-t[i-1]) for i in range(1,len(x_vel))]
    x_acc = pylab.correlate(x_acc,filt,mode='same')

    r_vel = [d.twist.twist.angular.z for d in data]
    r_vel = pylab.correlate(r_vel,filt,mode='same')
    r_acc = [0.0] + [ (r_vel[i]-r_vel[i-1])/(t[i]-t[i-1]) for i in range(1,len(r_vel))]
    r_acc = pylab.correlate(r_acc,filt,mode='same')

    z = []
    for d in data:
        o = d.pose.pose.orientation
        q = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w)
        z.append(q.GetRPY()[2])
    z=pylab.array(z)
    zd = z[1:]-z[:-1]
    zd = pylab.mod(zd+pi,2*pi)-pi
    zd = [0.0]+[i for i in zd]
    z = pylab.cumsum(zd)

    t2 = pylab.array([d.header.stamp.to_sec() for d in data2])
    imu = pylab.array([d.angular_velocity.z for d in data2])
    z2 = [0.0]
    for i in range(1,len(t2)):        
        z2.append(z2[-1] + 0.5*(imu[i]+imu[i-1])*(t2[i]-t2[i-1]))
    z2 = pylab.array(z2)
    #z2 = pylab.cumsum(imu)*(t2[1]-t2[0])

    t2 = t2-t[0]
    t  = t-t[0]

    if False:
        pylab.figure('x')
        pylab.plot(t,x_vel,'r')
        #pylab.plot(t,x_vel2,'b')
        pylab.plot(t,x_acc,'g')

    if False:
        pylab.figure('r')
        pylab.plot(t,r_vel,'r')
        #pylab.plot(t,x_vel2,'b')
        pylab.plot(t,r_acc,'g')

    pylab.figure('z')
    pylab.plot(t2,imu*180/pi,'b')
    pylab.plot(t,z*180/pi,'r')
    pylab.plot(t2,z2*180/pi,'g')


    pylab.show()


if __name__ == '__main__':
    try:
        cmdVel(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
