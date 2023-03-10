#!/usr/bin/env python
#coding: UTF-8

import sys, rospy, math, tf
import numpy as np
from pimouse_ros.msg import MotorFreqs, PulseCount, Info
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

FREQ = 10 #Frequency[Hz]
CTR_P = 0.1 #Countrol Period[s]
DPP = 0.9 #Deg per Pulse[deg]
RPP = 0.0157 #Rad per Pulse[rad]
N = 6.98 #Number of Pulse for turn[-]
T = 0.094 #Tread[m]
D = 0.046 #Diameter[m]
R = 0.023 #Radious[m]	

class Mouse():
    def __init__(self):
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
	self.flag=0

        self.pulse_count = PulseCount()
	self.last_pulse_count = PulseCount()
	self.info = Info()
        self.devfile = ['/dev/rtcounter_r1','/dev/rtcounter_l1']
        self.pub = rospy.Publisher('pulsecounter', PulseCount, queue_size = 1)
        self.pub2 = rospy.Publisher('info', Info, queue_size = 1)

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()
        self.calc_r()
        #self.last_time = self.cur_time
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time,"base_link","odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position = Point(self.x,self.y,0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)
        self.last_time = self.cur_time

    def calc_r(self):
	flag = self.read_count()
        d_t = self.cur_time.to_sec() - self.last_time.to_sec()
	d_r = self.pulse_count.right - self.last_pulse_count.right
	d_l = self.pulse_count.left - self.last_pulse_count.left
	if(d_r != 0 and d_l != 0):
	    omega_r = (d_r * RPP)/d_t
	    omega_l = (d_l * RPP)/d_t
	    v_r = omega_r * R
	    v_l = omega_l * R
	    v = float(v_l + v_r)/2.0
	    omega = (v_r - v_l)/T
	    self.th += omega*d_t
            self.x += (v*d_t)* np.cos(self.th)
            self.y += (v*d_t)* np.sin(self.th)
	
	    if flag == 0:
	        self.last_pulse_count.right = self.pulse_count.right
	        self.last_pulse_count.left = self.pulse_count.left
	    else:	
	        self.last_pulse_count.right = 0
	        self.last_pulse_count.left = 0

	    #write_map(self.x*100, self.y*100, self.th*(180.0/np.pi), 'a')
	    #return 1
	#write_map(self.x, self.y, self.th*(180.0/np.pi), 'a')
	self.info.x = self.x
	self.info.y = self.y
	self.info.th = self.th * 180.0/np.pi
	self.pub2.publish(self.info)
	return 0

    def read_count(self):
        r = open(self.devfile[0], 'r')
        r_value = int(r.readline())
        self.pulse_count.right = r_value
        r.close()

        l = open(self.devfile[1], 'r')
        l_value = int(l.readline())
        self.pulse_count.left = l_value
        l.close()

	self.pub.publish(self.pulse_count)

	#???????????????????????????
	if (abs(r_value) >= 30000) or (abs(l_value) >= 30000):
	    self.reset_count()
	    return -1		 #?????????????????????????????????

	return 0		 #???????????????

    def reset_count(self):
        r = open(self.devfile[0], 'w')
        r.write("0")
        r.close()
        l = open(self.devfile[1], 'w')
        l.write("0")
        l.close()

def init_map():
    write_map(0,0,0,'w')

def write_map(x,y,th,op):
    f=open('/home/rairai/catkin_ws/src/pimouse_slam/scripts/log.csv',op)
    f.write(str(x)+ "," + str(y)+","+str(th)+"\n")
    f.close()

if __name__ == '__main__':
    rospy.init_node('mouse')
    #init_map()
    m = Mouse()
    m.reset_count()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
