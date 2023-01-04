#!/usr/bin/env python
#encoding: utf8
import sys, rospy
from pimouse_ros.msg import LightSensorValues, SwitchState, PulseCount
from std_srvs.srv import Trigger, TriggerResponse

class PulseCounter():
    def __init__(self):
        self.count = PulseCount()
	self.devfile = ['/dev/rtcounter_r1','/dev/rtcounter_l1']

    def read_count(self):
        r = open(self.devfile[0], 'r')
        self.count.right = int(r.readline())
	r.close()
	l = open(self.devfile[1], 'r')
        self.count.left = int(l.readline())
	l.close()
	#print(self.count)

    def reset_count(self):
        r = open(self.devfile[0], 'w')
        r.write("0")
	r.close()
	l = open(self.devfile[1], 'w')
        l.write("0")
	l.close()

#def callback(message):
   # print(message)

if __name__ == '__main__':
    devfile = ['/dev/rtcounter_l1','/dev/rtcounter_r1']
    rospy.init_node('pulse_check')
    p = PulseCounter()
    freq = 5
    rate = rospy.Rate(freq)
    pub = rospy.Publisher('pulsecounter', PulseCount, queue_size = 1)
    #sub = rospy.Subscriber('/pulsecounter2', PulseCount, callback)
    while not rospy.is_shutdown():
	p.read_count()
	pub.publish(p.count)
	rate.sleep()
    else:
	p.reset_count()
