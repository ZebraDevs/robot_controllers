#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import pylab
import scipy.signal

msgs = None

def odomCallback(msg):
    global msgs
    if msgs != None:
        msgs.append(msg)

def cmdVel(vel):
    global msgs
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, odomCallback)
    dt = 0.1
    rate = rospy.Rate(1/dt) # 10hz
    twist = Twist() 
    t = 0.0
    msgs = []
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()
        t+=dt
        if t < 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif t < 6:
            twist.linear.x = 0.0
            twist.angular.z = vel
        elif t < 10:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            break
    data = msgs
    msgs = None

    filt = pylab.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.55, 0.6, 0.55, 0.5, 0.4, 0.3, 0.2, 0.1])
    filt /= filt.sum()

    t = pylab.array([d.header.stamp.to_sec() for d in data])
    t = t-t[0]
    x_vel = [d.twist.twist.linear.x for d in data]
    x_vel = pylab.correlate(x_vel,filt,mode='same')
    x_acc = [0.0] + [ (x_vel[i]-x_vel[i-1])/(t[i]-t[i-1]) for i in range(1,len(x_vel))]
    x_acc = pylab.correlate(x_acc,filt,mode='same')

    r_vel = [d.twist.twist.angular.z for d in data]
    r_vel = pylab.correlate(r_vel,filt,mode='same')
    r_acc = [0.0] + [ (r_vel[i]-r_vel[i-1])/(t[i]-t[i-1]) for i in range(1,len(r_vel))]
    r_acc = pylab.correlate(r_acc,filt,mode='same')

    pylab.figure('x')
    pylab.plot(t,x_vel,'r')
    #pylab.plot(t,x_vel2,'b')
    pylab.plot(t,x_acc,'g')

    pylab.figure('r')
    pylab.plot(t,r_vel,'r')
    #pylab.plot(t,x_vel2,'b')
    pylab.plot(t,r_acc,'g')

    pylab.show()


if __name__ == '__main__':
    try:
        cmdVel(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
