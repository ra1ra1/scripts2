#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

rospy.wait_for_service('/motor_on')
rospy.wait_for_service('/motor_off')
rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
rospy.ServiceProxy('/motor_on',Trigger).call()

rospy.init_node('keyboard_cmd_vel')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel = Twist()
vel.linear.x = 0.0

while not rospy.is_shutdown():
    vel = Twist()
    direction = raw_input('w: forward, s: backward, a: left, d: right, return: stop > ')
    if 'w' in direction: vel.linear.x = 0.15#0.20
    if 's' in direction: vel.linear.x = -0.15#-0.20
    if 'a' in direction: vel.angular.z = np.pi * 0.5
    if 'd' in direction: vel.angular.z = -np.pi * 0.5
    pub.publish(vel)
