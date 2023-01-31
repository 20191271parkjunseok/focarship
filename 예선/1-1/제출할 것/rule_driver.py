#!/usr/bin/python

import rospy, math, time
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray

fr = 0.0

def callback(msg):
	global fr
	fr = msg.data[2]
	print(msg)

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

angle = 0.0
speed = 25
setPoint = 112
error = 0.0

pTerm = 0.0
iTerm = 0.0
dTerm = 0.0

beforeT = 0.0
afterT = 0.0

x=[]
y=[]
start=time.time()

while not rospy.is_shutdown():
	xycar_msg.data = [angle, speed]
	motor_pub.publish(xycar_msg)

	data = fr

	beforeErr = error
	error = setPoint - data

	kp = 0.05
	pTerm = kp * error

	#ki = 0.001
	#afterT = time.time()
	#dt = afterT - beforeT
	#beforeT = afterT
	#iTerm += ki * error * dt

	#kd = 0.000000001
	#dError = error - beforeErr
	#if dt != 0:
	#	dTerm = -kd * (dError / dt)

	aftertime=time.time()-start
	x.append(aftertime)
	y.append(data)

	if int(aftertime) == 20:
		plt.plot(x,y)
		plt.xlabel('time')
		plt.ylabel('fr')
		plt.title('Graph')
		plt.show()

	pid = pTerm
	angle = -pid * (180.0 / math.pi)
