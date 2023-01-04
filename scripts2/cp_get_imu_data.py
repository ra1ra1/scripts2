#!/usr/bin/env python
#coding: UTF-8

import sys, rospy, math, tf, time
import numpy as np
from pimouse_ros.msg import MotorFreqs, PulseCount, Info
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry
from subprocess import call
from subprocess import Popen
from std_msgs.msg import Float64

FREQ = 100 #Frequency[Hz]
CTR_P = 0.01 #Countrol Period[s]

PLOT_DATA = '/home/rairai/catkin_ws/src/pimouse_slam/scripts/plot_data.sh'
DATA = '/home/rairai/catkin_ws/src/pimouse_slam/scripts/data.csv'	
BUZZER = '/dev/rtbuzzer0'

class Mouse():
    def __init__(self):
        self.pub_odom = rospy.Publisher('get_imu_data', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
	self.v = 0.0
	self.v_imu = 0.0
	self.a = 0.0
	self.ax, self.filtered_ax,  self.prev_ax = 0.0, 0.0, 0.0
	self.omega, self.filtered_omega,  self.prev_omega = 0.0, 0.0, 0.0
	self.cnt= 1
	self.n = 1
	self.wait_cnt = 5
	self.flag = 0
	self.sum, self.offset = 0.0, 0.0
	self.imu_data = Imu()
	self.vel = Twist()

	self.info = Info()
        self.devfile = ['/dev/rtcounter_r1','/dev/rtcounter_l1']
        self.pub2 = rospy.Publisher('info', Info, queue_size = 1)
	self.pub3 = rospy.Publisher('ax', Float64, queue_size = 1)
	self.pub4 = rospy.Publisher('v_imu', Float64, queue_size = 1)
	self.pub5 = rospy.Publisher('offset', Float64, queue_size = 1)
	self.sub = rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)
	self.sub2 = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel)

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time
	self.t = 0.0

    def get_offset(self):
	if self.cnt % 50 == 0:
	    self.update_offset()
	if self.cnt >= 10000: self.cnt = 0
	self.cnt += 1
   
    def update_offset(self):
	Kn = 1.0/self.n
	self.offset = self.offset + Kn * (-1*self.imu_data.linear_acceleration.x - self.offset)
	self.n += 1
	self.pub5.publish(self.offset)

    def filter_ax(self):
	if abs(self.ax) < 0.1: self.ax = 0.0

    def calc_v(self):
	#if self.n > 10: self.v_imu += self.ax * CTR_P
	if self.vel.linear.x == 0.0: self.v_imu = 0.0
	else: self.v_imu += self.ax * CTR_P
	#self.vel.linear.x != 0.0: self.v_imu += self.ax * CTR_P
	#self.pub4.publish(self.v_imu)
	
    def plot_a(self):
	#self.cur_time = rospy.Time.now()
	#dt = self.cur_time.to_sec() - self.last_time.to_sec()
	self.t += CTR_P
	#print(self.t, self.omega, self.th * 180.0/np.pi)
	write_data(self.t, self.ax,  self.filtered_ax, 'a')
	if self.cnt % (FREQ*2) == 0:
	    #plot_data = Popen(PLOT_DATA, shell=True)
	    self.cnt = 0
	self.cnt += 1
	#self.last_time = self.cur_time

    def callback_imu(self, message):
	self.imu_data = message 

    def get_ax(self):
	self.ax = -1*self.imu_data.linear_acceleration.x-self.offset
        self.filter_ax()
        self.calc_v()
	self.pub3.publish(self.ax)
	self.pub4.publish(self.v_imu)

    def callback_cmd_vel(self, message):
	self.vel = message
	if self.vel.linear.x == 0.0:
	    #self.wait_cnt = 50 
	    self.flag = 0
	    self.n = 1
	    self.cnt = 0
	    #print(self.vel)
	else: self.flag = 1

    def filter_imu_data(self):
        ALPHA = 0.2
        #omega
        self.filtered_ax = ALPHA * self.ax + (1.0-ALPHA) * self.prev_ax
        #if abs(self.filtered_omega) < 0.01: self.filtered_omega = 0.0
        self.prev_ax = self.filtered_ax
        #self.debug.data1 = self.filtered_ax
        #self.pub_debug.publish(self.debug)

def init_data():
    write_data(0,0,0,'w')

def write_data(t,data1,data2,op):
    f=open(DATA ,op)
    f.write(str(t) + "," + str(data1) + "," + str(data2) + "\n")
    f.close()

if __name__ == '__main__':
    rospy.init_node('mouse')
    init_data()
    m = Mouse()
    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
	m.get_ax()
	if m.flag == 0: m.get_offset()
	#self.pub4.publish(self.v_imu)
	#elif m.flag == 1: m.filter_ax()
	m.plot_a()
        rate.sleep()
