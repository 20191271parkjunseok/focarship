#!/usr/bin/python

import rospy, math
from std_msgs.msg import Int32MultiArray

fr = 0

def callback(msg):
	print(msg)
	global fr

	fr = msg.data[2]

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

angle = 0
speed = 25
setPoint = 150

while not rospy.is_shutdown():
	xycar_msg.data = [angle, speed]
	motor_pub.publish(xycar_msg)

	kp = 0.1
	error = setPoint - fr


	if(fr < setPoint):
		kp = 0.1
	else:
		kp = 0.05
	
	p = kp * error
	angle = - p * 180 / math.pi

	print(angle)
