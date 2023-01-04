#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse

rospy.wait_for_service('/motor_on')
rospy.wait_for_service('/motor_off')
rospy.on_shutdown(rospy.ServiceProxy('motor_off', Trigger).call)
rospy.ServiceProxy('/motor_on', Trigger).call()


while not rospy.is_shutdown():
    f = open("/dev/rtmotor0", 'w')
    direction = raw_input('w : forward, s:backward, a:left, d:right, return: stop > ')
    if 'w' in direction: f.write("553 553 1000")#f.write("450 450 1000")
    if 's' in direction: f.write("-450 -450 1000") 
    if 'a' in direction: f.write("-195 195 1060") #f.write("-245 245 400")
    if 'd' in direction: f.write("196 196 1082")#f.write("245 -245 400")
    f.close()
