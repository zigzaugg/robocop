#!/usr/bin/env python

import roslib
import rospy
import signal
import sys
import numpy as np
import matplotlib.pyplot as plt
import pickle
from fw_wrapper.srv import *
from eecs301_grp_L.srv import *
import math
from sklearn import linear_model

#LEFT = 5
#RIGHT = 6
# from eecs301_grp_L.srv import *
# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed
# Zero
# Stand

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
    	resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


### OUR CODE ###
def shutdown(sig, stackframe):
	print("\nShutdown")
	stop()
	sys.exit(0)

def stop():
	setMotorWheelSpeed(5,0)
	setMotorWheelSpeed(6,0)
	rospy.sleep(.3)

def turn(speed, direction):
	if direction == 0:
		setMotorWheelSpeed(5, 1024+speed)
		setMotorWheelSpeed(6, 1024+speed)
	elif direction == 1:
		setMotorWheelSpeed(6, speed)
		setMotorWheelSpeed(5, speed)
		
def turnSpecific(duration, speed):
	turn(speed, 1)
	rospy.sleep(duration)
	stop()
		

def turnAngle(angle):
#angle is degrees/90
	while angle < -2:
		angle += 4
	while angle > 2:
		angle -= 4
		
	duration = turnTime #recent: 1.13
	if angle == 0:
		return
	elif angle == -2:
		rospy.loginfo("Turn Left Twice")
		turn(speed, 0)
		rospy.sleep(duration)
		stop()
		turn(speed, 0)
		rospy.sleep(duration)
		stop()
	elif angle == -1:
		rospy.loginfo("Turn Left")
		turn(speed, 0)
		rospy.sleep(duration)
		stop()
	elif angle == 1:
		rospy.loginfo("Turn Right")
		turn(speed, 1)
		rospy.sleep(duration)
		stop()
	elif angle == 2:
		rospy.loginfo("Turn Right Twice")
		turn(speed, 1)
		rospy.sleep(duration)
		stop()
		turn(speed, 1)
		rospy.sleep(duration)
		stop()
		
def distance(x1, y1, x2, y2):
	weight = 1
	return max(np.sqrt(np.power((x1-x2), 2) + np.power(weight*(y1-y2), 2)), 0.001)


def N_nearest(N, s, ang, data):
	return sorted(data, key=lambda datum: distance(x, y, datum[1], datum[2]))[:N]


def regression(s, ang, data):
	tot_weight = 0
	accumulator = 0
	
	for [point_tt, point_s, point_angle] in data:
		point_weight = 1/np.power(distance(s, ang, point_s, point_angle), 2)
		tot_weight += point_weight
		accumulator += point_weight * point_tt
		
	#return accumulator / tot_weight
	nn = N_nearest(2, s, ang, data)
	return (nn[0][0] + nn[1][0]) / 2
	
def plotReg(s, data):
	angs = np.arange(0, 360, 1)
	plt.plot(angs, [regression(s, ang, data) for ang in angs], 'k')
	plt.show()
	
def linReg(data):
	x = []
	y = []
	z = []
	for [tt, s, a] in data:
		z.append(tt)
		x.append(s)
		y.append(a)
	reg = linear_model.LinearRegression()
	reg.fit([x, y], z)


# Main function
if __name__ == "__main__":
	rospy.init_node('example_node', anonymous = True)
	rospy.loginfo("Starting Group L Control Node...")
	#signal.signal(signal.SIGINT, shutdown)
	r = rospy.Rate(2) # 10hz

	global turnTime
	global speed
	
	
	command = int(sys.argv[1])
	
	if command == 0:
		#Gather Data
		turnTime = float(sys.argv[2])
		speed = int(sys.argv[3])
		turnAngle(1)
		angle = int(raw_input("Enter Angels: "))
		data=pickle.load(open("machine.p", "rb"))
		data.append([turnTime, speed, angle])
		pickle.dump(data, open("machine.p", "wb"))
		
	elif command == 1:
		#Edit Data
		data=pickle.load(open("machine.p", "rb"))
		print data
		#edits
	elif command == 2:
		data=pickle.load(open("machine.p", "rb"))
		
		reg_s = int(sys.argv[2])
		reg_ang = int(sys.argv[3])
		
		print regression(reg_s, reg_ang, data)
		
	elif command == 3:
		data=pickle.load(open("machine.p", "rb"))
		plotReg(1000, data)
	elif command == 10:
		#Clear Data
		
		sure = raw_input("Nuclear Launch Codes: ")
		if sure == "yes":
			data = []
			pickle.dump(data, open("machine.p", "wb"))

