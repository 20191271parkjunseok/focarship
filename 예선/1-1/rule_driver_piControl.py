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
speed = 100
setPoint = 114
error = 0

while not rospy.is_shutdown():
	xycar_msg.data = [angle, speed]
	motor_pub.publish(xycar_msg)

	beforeError = error
	error = setPoint - fr

	if(fr > setPoint):
		kp = 0.1
	else:
		kp = 0.05

	ki = 1
	dError = error - beforeError
	p = kp * error
	i = ki * error * 30

	pid = p + i
	angle = -pid * 180 / math.pi

	print "pid: ", 
	print pid
	print "angle: ",
	print angle
