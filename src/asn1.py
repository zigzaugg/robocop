#!/usr/bin/env python

import roslib
import rospy
from fw_wrapper.srv import *
from eecs301_grp_L.srv import *
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
# Reset all joints
def zero():
	response = setMotorTargetPositionCommand(9, 512)
	response = setMotorTargetPositionCommand(2, 512)
	response = setMotorTargetPositionCommand(3, 512)
	response = setMotorTargetPositionCommand(4, 512)
	response = setMotorTargetPositionCommand(5, 512)
	response = setMotorTargetPositionCommand(6, 512)
	response = setMotorTargetPositionCommand(7, 612)
	response = setMotorTargetPositionCommand(8, 412)
	wait()

def raiseFoot(which):
	upper = 200
	lower = 75
	
	fastSpeed = 300
	slowSpeed = 150
	
	if which == 'left':
		rightAnkle = 512 - lower
		leftAnkle = 512 - upper
		rightSpeed = slowSpeed
		leftSpeed = fastSpeed
	else:
		rightAnkle = 512 + upper
		leftAnkle = 512 + lower
		rightSpeed = fastSpeed
		leftSpeed = slowSpeed
	
	setMotorTargetSpeed(3, rightSpeed)
	setMotorTargetSpeed(4, leftSpeed)
	setMotorTargetPositionCommand(3, rightAnkle)
	setMotorTargetPositionCommand(4, leftAnkle)
	rospy.sleep(.5)

def lowerFoot():
	setMotorTargetPositionCommand(3, 512)
	setMotorTargetPositionCommand(4, 512)
	wait()

def takeStep(which):
	raiseFoot(which)
	stepSize = 100
	swingSize = 200
	if which == 'left':
		motorVal = 512 - stepSize
		armVal = 512 + swingSize
	else:
		motorVal = 512 + stepSize
		armVal = 512 - swingSize
	setMotorTargetPositionCommand(9, motorVal)
	setMotorTargetPositionCommand(2, motorVal)
	setMotorTargetPositionCommand(5, armVal)
	setMotorTargetPositionCommand(6, armVal)
	wait()
	lowerFoot()
	
#Move arms out of the way	
def armsOutOfTheWay():
	setMotorTargetPositionCommand(7, 712)
	setMotorTargetPositionCommand(8, 312)
	wait()
	
def walk():
	takeStep('left')
	takeStep('right')
		
def closeFeet():
	#Closing step
	raiseFoot('left')
	
	#Move hips
	setMotorTargetPositionCommand(9, 512)
	setMotorTargetPositionCommand(2, 512)
	setMotorTargetPositionCommand(5, 512)
	setMotorTargetPositionCommand(6, 512)
	wait()
	lowerFoot()

def other(direction):
	if direction == 'left':
		return 'right'
	else:
		return 'left'
		
def wait():
	rospy.sleep(.4)
	
def setWalkSpeed(sp):
	setMotorTargetSpeed(9, sp)
	setMotorTargetSpeed(2, sp)
	setMotorTargetSpeed(3, sp)
	setMotorTargetSpeed(4, sp)
	setMotorTargetSpeed(5, 2*sp)
	setMotorTargetSpeed(6, 2*sp)
	
def setAllSpeed(sp):
	for i in range (2,10):
		setMotorTargetSpeed(i, sp)

def turn(which):
	setAllSpeed(250)
	if which == 0:
		motor = [7, 8, 4, 3, 9]
		position = [768, 1, 312, 422, 812, 850, 502, 512]
	elif which == 1:
		motor = [8, 7, 3, 4, 2]
		position = [256, 1023, 712, 602, 212, 264, 522, 512]		
	setMotorTargetPositionCommand(motor[0], position[0])
	setMotorTargetPositionCommand(motor[1], position[1])
	rospy.sleep(.6)
	setMotorTargetPositionCommand(motor[2], position[2])
	setMotorTargetPositionCommand(motor[3], position[3])
	rospy.sleep(.6)
	setMotorTargetPositionCommand(motor[2], position[4])
	setMotorTargetSpeed(motor[4], 150)
	setMotorTargetSpeed(motor[3], 54)
	setMotorTargetPositionCommand(motor[4], position[5])
	rospy.sleep(.3)
	setMotorTargetPositionCommand(motor[3], position[6])
	rospy.sleep(1)
	setMotorTargetPositionCommand(motor[2], position[7])
	rospy.sleep(1)
	setMotorTargetPositionCommand(motor[4], position[7])
	rospy.sleep(.8)

def turnAround():
	turn(0)
	turn(0)
	

def followWallRight():
	while True:
		setWalkSpeed(250)
		dist = getSensorValue(1)
		DMS = getSensorValue(3)
		if DMS > 1000:
			turn(0)
			rospy.sleep(.5)
			zero()
			armsOutOfTheWay()
		elif dist < 20:
			correctRight()
		elif dist > 70 or DMS > 800:
			correctLeft()
		
		takeStep("right")
		walk()
		closeFeet()

def followWallLeft():
	while True:
		setWalkSpeed(250)
		dist = getSensorValue(5)
		DMS = getSensorValue(3)
		if DMS > 1000:
			turn(1)
			rospy.sleep(.5)
			zero()
			armsOutOfTheWay()
		elif dist < 20:
			correctLeft()
		elif dist > 70 or DMS > 800:
			correctRight()
		
		takeStep("right")
		walk()
		closeFeet()
		
def correctRight():
	setMotorTargetPositionCommand(7, 762)
	setMotorTargetPositionCommand(4, 412)
	setMotorTargetPositionCommand(3, 412)
	rospy.sleep(.6)
	setMotorTargetPositionCommand(9, 442)
	wait()
	setMotorTargetSpeed(3,120)
	setMotorTargetPositionCommand(4, 512)
	setMotorTargetPositionCommand(3, 512)
	wait()
	setMotorTargetPositionCommand(9, 512)
	setMotorTargetPositionCommand(7, 712)
	setMotorTargetSpeed(3,250)
	wait()
	
def correctLeft():
	setMotorTargetPositionCommand(8, 262)
	setMotorTargetPositionCommand(3, 612)
	setMotorTargetPositionCommand(4, 612)
	rospy.sleep(.6)
	setMotorTargetPositionCommand(2, 582)
	wait()
	setMotorTargetSpeed(4,120)
	setMotorTargetPositionCommand(4, 512)
	setMotorTargetPositionCommand(3, 512)
	wait()
	setMotorTargetPositionCommand(2, 512)
	setMotorTargetPositionCommand(8, 312)
	setMotorTargetSpeed(4,250)
	wait()

def turnWhenBlocked():
	while True:
		rb = getSensorValue(1) > 40
		lb = getSensorValue(5) > 40
		fb = getSensorValue(3) > 2000
		if rb and lb and fb:
			turnAround()
			rospy.sleep(.5)
			zero()
			rospy.sleep(.5)
		elif rb and fb:
			turn(0)
			rospy.sleep(.5)
			zero()
			rospy.sleep(.5)
		elif lb and fb:
			turn(1)
			rospy.sleep(.5)
			zero()
			rospy.sleep(.5)


def serverStart():
	rospy.init_node('asn1')
	r = rospy.Rate(10)
	s = rospy.Service('turn_when_blocked', Listen, decideFunction)
	rospy.spin()

def decideFunction(funcNumber):
	rospy.loginfo(funcNumber.funcNumber)
	response = 0
	if funcNumber.funcNumber == 0:
		turnWhenBlocked()
		response = 1
	elif funcNumber.funcNumber == 1:
		setWalkSpeed(250)
		zero()
		armsOutOfTheWay()
		followWallRight()
		response = 1
	elif funcNumber.funcNumber == 2:
		setWalkSpeed(250)
		zero()
		armsOutOfTheWay()
		followWallLeft()
		response = 1
	return response

# Main function
if __name__ == "__main__":
	rospy.init_node('example_node', anonymous = True)
	rospy.loginfo("Starting Group L Control Node...")
	
	# control loop running at 10hz
	r = rospy.Rate(10) # 10hz
	
	setWalkSpeed(250)
	zero()
	turn(1)

	while not rospy.is_shutdown():
		# call function to get sensor value
		r.sleep()
