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

FREQ = 10 #Frequency[Hz]
CTR_P = 0.1 #Countrol Period[s]

PLOT_DATA = '/home/rairai/catkin_ws/src/pimouse_slam/scripts/plot_data.sh'
DATA = '/home/rairai/catkin_ws/src/pimouse_slam/scripts/data.csv'	
BUZZER = '/dev/rtbuzzer0'

class Mouse():
    def __init__(self):
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
	self.v = 0.0
	self.a = 0.0
	self.ax, self.filtered_ax,  self.prev_ax = 0.0, 0.0, 0.0
	self.omega, self.filtered_omega,  self.prev_omega = 0.0, 0.0, 0.0
	self.cnt= 0
	self.flag = 0
	self.sum, self.offset = 0.0, 0.0
	self.imu_data = Imu()

	self.info = Info()
        self.devfile = ['/dev/rtcounter_r1','/dev/rtcounter_l1']
        self.pub2 = rospy.Publisher('info', Info, queue_size = 1)
	self.pub3 = rospy.Publisher('data', Imu, queue_size = 1)
	self.sub = rospy.Subscriber('/imu/data_raw', Imu, self.callback_imu)

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time
	self.t = 0.0
	
    def plot_a(self):
	self.t += CTR_P
	self.th += self.omega * CTR_P
	#print(self.t, self.omega, self.th * 180.0/np.pi)
	write_data(self.t, self.omega,  self.filtered_omega, 'a')
	if self.cnt % 50 == 0:
	    #plot_data = Popen(PLOT_DATA, shell=True)
	    self.cnt = 0
	self.cnt += 1
	self.calc_r()

    def get_offset(self):
	if self.cnt >=  5:
	    self.sum += self.filtered_omega
	if self.cnt == 7:
	    self.offset = self.sum / -3.0  
	    self.cnt = -1
	    self.flag = 1
	    print("offset:" + str(self.offset))	
	    ready_buzzer()
	self.cnt += 1

    def calc_r(self):
        self.cur_time = rospy.Time.now()
        #dt = self.cur_time.to_sec() - self.last_time.to_sec()
	dt = CTR_P
	#self.v += self.a * d_t
	self.th += self.filtered_omega * dt
	self.x += self.vx * dt
	#print(self.th)
	#print(self.x * 100)
	self.info.x = self.x * 100 #m -> cm
	self.info.y = self.y * 100 #m -> cm
	self.info.th = self.th * 180.0/np.pi #rad -> deg
	self.pub2.publish(self.info)
	self.last_time = self.cur_time
	return 0

    def callback_imu(self, message):
	self.imu_data = message 
	self.pub3.publish(self.imu_data)
	self.omega = self.imu_data.angular_velocity.z + self.offset
	#self.ax = self.imu_data.linear_acceleration.x + self.offset
	self.filter_imu_data()
	#if abs(self.omega) < 0.02: self.omega = 0.0 

    def filter_imu_data(self):
	ALPHA = 0.2
	#ax
	self.filtered_ax = ALPHA * self.ax + (1.0-ALPHA) * self.prev_ax
	if abs(self.filtered_ax) < 0.05: self.filtered_ax = 0.0
	self.prev_ax = self.filtered_ax 

	#omega
	self.filtered_omega = ALPHA * self.omega + (1.0-ALPHA) * self.prev_omega
	#if abs(self.filtered_omega) < 0.05: self.filtered_omega = 0.0
	self.prev_omega = self.filtered_omega 
	

def init_data():
    write_data(0,0,0,'w')

def write_data(t,data1,data2,op):
    f=open(DATA ,op)
    f.write(str(t) + "," + str(data1) + "," + str(data2) + "\n")
    f.close()

def ready_buzzer():
    f = open(BUZZER, 'w')
    f.write("1760")
    f.close()
    time.sleep(0.2)
    f = open(BUZZER, 'w')
    f.write("0")
    f.close() 	

if __name__ == '__main__':
    rospy.init_node('mouse')
    init_data()
    m = Mouse()
    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
	if m.flag == 0: m.get_offset()
        elif m.flag == 1: m.plot_a()
        rate.sleep()
