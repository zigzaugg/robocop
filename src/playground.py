#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *

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

# Reset all joints
def zero():
	response = setMotorTargetPositionCommand(1, 512)
	response = setMotorTargetPositionCommand(2, 512)
	response = setMotorTargetPositionCommand(3, 512)
	response = setMotorTargetPositionCommand(4, 512)
	response = setMotorTargetPositionCommand(5, 512)
	response = setMotorTargetPositionCommand(6, 512)
	response = setMotorTargetPositionCommand(7, 612)
	response = setMotorTargetPositionCommand(8, 412)
	
def stand():
	zero()
	while(getIsMotorMovingCommand(1) or getIsMotorMovingCommand(2) or getIsMotorMovingCommand(3) or getIsMotorMovingCommand(4) or getIsMotorMovingCommand(5) or getIsMotorMovingCommand(6) or getIsMotorMovingCommand(7) or getIsMotorMovingCommand(8)):
		q = 0
	rospy.loginfo("Standing up...")

	#raise arms
	setMotorTargetPositionCommand(7, 812)
	setMotorTargetPositionCommand(8, 212)
	while(getIsMotorMovingCommand(7) or getIsMotorMovingCommand(8)):
		q = 0

	#rotate shoulders
	setMotorTargetPositionCommand(5, 1023)
	setMotorTargetPositionCommand(6, 1)
	while(getIsMotorMovingCommand(5) or getIsMotorMovingCommand(6)):
		q = 0
		
	#lower arms
	setMotorTargetPositionCommand(7, 512)
	setMotorTargetPositionCommand(8, 512)
	while(getIsMotorMovingCommand(7) or getIsMotorMovingCommand(8)):
		q = 0
		
	#bend feet
	setMotorTargetPositionCommand(3, 156)
	setMotorTargetPositionCommand(4, 868)
	while(getIsMotorMovingCommand(3) or getIsMotorMovingCommand(4)):
		q = 0
		
	#Partially anti-rotate shoulders
	setMotorTargetPositionCommand(5, 462)
	setMotorTargetPositionCommand(6, 562)
	while(getIsMotorMovingCommand(5) or getIsMotorMovingCommand(6)):
		q = 0
	
	#level feet
	setMotorTargetPositionCommand(3, 512)
	setMotorTargetPositionCommand(4, 512)
	while(getIsMotorMovingCommand(3) or getIsMotorMovingCommand(4)):
		q = 0

	zero()
	rospy.loginfo("Upright")
	#standing = True

	
# Main function
if __name__ == "__main__":
	rospy.init_node('example_node', anonymous=True)
	rospy.loginfo("Starting Group L Control Node...")
	#tell robot it should be standing
	standing = True
#	zero()
	
	#stand()
	
#	setMotorTargetPositionCommand(motor_id, target_val)

	# control loop running at 10hz
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		# call function to get sensor value
		port = 1
		sensor_reading = getSensorValue(port)
		rospy.loginfo("Sensor value at port %d: %f", 1, sensor_reading)
		if sensor_reading > 1000:
			stand()
 		'''if standing:
			#reset all limbs if it just stood up
 			if not is_zero:
 				zero()
 				is_zero = True
 				
			# behavior 1: move arms based on sensor reading
			target_val = min(1024, 612 + sensor_reading / 2)
			response = setMotorTargetPositionCommand(7, target_val)
			target2_val = max(0, 412 - sensor_reading / 2)
			response = setMotorTargetPositionCommand(8, target2_val)
			
			# check if it's ACTUALLY standing
			if sensor_reading > 1000:
				standing = False
				stand()
		'''
		# sleep to enforce loop rate
		r.sleep()
		
