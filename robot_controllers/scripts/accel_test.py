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
        elif t < 6:
            twist.linear.x = vel
        elif t < 10:
            twist.linear.x = 0.0
        else:
            break
    data = msgs
    msgs = None
    t = pylab.array([d.header.stamp.to_sec() for d in data])
    t = t-t[0]
    x_vel = [d.twist.twist.linear.x for d in data]
    #x_pos = [d.pose.pose.position.x for d in data]
    #x_vel2 = [0.0] + [ (x_pos[i]-x_pos[i-1])/(t[i]-t[i-1]) for i in range(1,len(x_vel))]

    filt = pylab.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.55, 0.6, 0.55, 0.5, 0.4, 0.3, 0.2, 0.1])
    #filt = pylab.ones(30)
    filt /= filt.sum()
    x_vel = pylab.correlate(x_vel,filt,mode='same')
    x_acc = [0.0] + [ (x_vel[i]-x_vel[i-1])/(t[i]-t[i-1]) for i in range(1,len(x_vel))]
    x_acc = pylab.correlate(x_acc,filt,mode='same')
    #filt = scipy.signal.butter(10,0.5)
    #print(filt)
    #x_acc = scipy.signal.lfilter(filt[0],filt[1],x_acc)    

    pylab.figure()
    pylab.plot(t,x_vel,'r')
    #pylab.plot(t,x_vel2,'b')
    pylab.plot(t,x_acc,'g')
    pylab.show()


if __name__ == '__main__':
    try:
        cmdVel(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
