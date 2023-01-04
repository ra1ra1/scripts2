#!/usr/bin/env python
import rospy,copy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallStop():
    def __init__(self):
	self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	self.sensor_values = LightSensorValues()
	self.key_vel = Twist()
	rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_light)
	rospy.Subscriber('key_vel', Twist, self.callback_key)

    def callback_light(self, messages):
	self.sensor_values = messages

    def callback_key(self, messages):
	self.key_vel = messages


    def run(self):
	rate = rospy.Rate(10)
	data = Twist()

	accel = 0.004
	data.linear.x = 0.0
	while not rospy.is_shutdown():
	    if self.key_vel.linear.x != 0.0:
	    	data.linear.x += accel
	    	if self.sensor_values.sum_all >= 50:         data.linear.x = 0.0
	    	elif data.linear.x <= 0.1:		     data.linear.x = 0.1
	    	elif data.linear.x >= self.key_vel.linear.x: data.linear.x = self.key_vel.linear.x
	    else: data = self.key_vel

	    self.cmd_vel.publish(data)
            rate.sleep()

	##while not rospy.is_shutdown():
	   ##data.linear.x = 0.2 if self.sensor_values.sum_all < 500 else 0.0
	   ##self.cmd_vel.publish(data)
	   ##rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_stop')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    #rospy.ServiceProxy('/motor_off', Trigger).call()
    WallStop().run()
