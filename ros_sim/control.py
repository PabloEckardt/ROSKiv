#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import pid_input
from sensor_msgs.msg import LaserScan

kp = 14.0
kd = 0.09
servo_offset = 18.5	# zero correction offset in case servo is misaligned. 
prev_error = 0.0 
vel_input = 25.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

def calc_angle(error):
	if error < 0:
		return -0.20
	else:
		return 0.20



def control(data):
	global prev_error
	global vel_input
	global kp
	global kd

	## Your code goes here
	# 1. Scale the error
	# 2. Apply the PID equation on error
	# 3. Make sure the error is within bounds

	## END
	angle = calc_angle(float(data.pid_error))
	print str(angle) + " " + str(data.pid_error)
	msg = drive_param();
	msg.velocity = vel_input	
	msg.angle = angle
	pub.publish(msg)

if __name__ == '__main__':
	global kp
	global kd
	global vel_input
	print("Listening for error input")
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#vel_input = input("Enter Velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
