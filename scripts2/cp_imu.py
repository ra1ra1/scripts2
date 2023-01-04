#!/usr/bin/env python
#coding: UTF-8

import sys, rospy, math, tf
import numpy as np
from pimouse_ros.msg import MotorFreqs, PulseCount, Info
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry
from subprocess import call
from subprocess import Popen

FREQ = 10 #Frequency[Hz]
CTR_P = 0.1 #Countrol Period[s]
DPP = 0.9 #Deg per Pulse[deg]
RPP = 0.0157 #Rad per Pulse[rad]
N = 6.98 #Number of Pulse for turn[-]
T = 0.094 #Tread[m]
D = 0.046 #Diameter[m]
R = 0.023 #Radious[m]

PLOT_DATA = '/home/rairai/catkin_ws/src/pimouse_slam/scripts/plot_data.sh'	

class Mouse():
    def __init__(self):
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
	self.v = 0.0
	self.a = 0.0
	self.ax = 0.0
	self.filtered_ax = 0.0
	self.prev_ax = 0.0
	self.omega = 0.0
	self.cnt=0
	self.imu_data = Imu()

	self.info = Info()
        self.devfile = ['/dev/rtcounter_r1','/dev/rtcounter_l1']
        self.pub2 = rospy.Publisher('info', Info, queue_size = 1)
	self.pub3 = rospy.Publisher('data', Imu, queue_size = 1)
	self.sub = rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time
	self.t = 0.0

    def send_odom(self):
        self.cur_time = rospy.Time.now()
        #self.calc_r()
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
	
	write_data(self.cur_time, self.a, 'a')

        self.pub_odom.publish(odom)
        self.last_time = self.cur_time
	
    def plot_a(self):
        self.cur_time = rospy.Time.now()
	dt = self.cur_time.to_sec() - self.last_time.to_sec()
	self.t += CTR_P
	self.th += self.omega * dt
	#print(self.t, self.omega, self.th * 180.0/np.pi)
	write_data(self.t, self.ax,  self.filtered_ax, 'a')
	if self.cnt % 50 == 0:
	    #plot_data = Popen(PLOT_DATA, shell=True)
	    self.cnt = 0
	self.cnt += 1
	self.last_time = self.cur_time

    def calc_r(self):
        d_t = self.cur_time.to_sec() - self.last_time.to_sec()
	self.v += self.a * d_t
	self.x += self.v * d_t
	print(self.a, self.v)
	print(self.x * 100)
	self.info.x = self.x * 100 #m -> cm
	self.info.y = self.y * 100 #m -> cm
	self.info.th = self.th * 180.0/np.pi #rad -> deg
	self.pub2.publish(self.info)
	return 0

    def callback_imu(self, message):
	self.imu_data = message 
	self.pub3.publish(self.imu_data)
	self.omega = self.imu_data.angular_velocity.z
	self.ax = self.imu_data.linear_acceleration.x
	self.filter_acceleration()
	#if abs(self.omega) < 0.02: self.omega = 0.0 

    def filter_acceleration(self):
	ALPHA = 0.1
	self.filtered_ax = ALPHA * self.ax + (1.0-ALPHA) * self.prev_ax
	self.prev_ax = self.filtered_ax 

def init_data():
    write_data(0,0,0,'w')

def write_data(t,data1,data2,op):
    f=open('/home/rairai/catkin_ws/src/pimouse_slam/scripts/data.csv',op)
    f.write(str(t) + "," + str(data1) + "," + str(data2) + "\n")
    f.close()

if __name__ == '__main__':
    rospy.init_node('mouse')
    init_data()
    m = Mouse()
    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
        m.plot_a()
        rate.sleep()
