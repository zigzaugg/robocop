#!/usr/bin/env python

import roslib
import rospy
import signal
import sys
import numpy
from fw_wrapper.srv import *
from eecs301_grp_L.srv import *
from map import *
import threading
import time

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

def turn(speed, direction):
	if direction == 0:
		setMotorWheelSpeed(5, 1024+speed)
		setMotorWheelSpeed(6, 1024+speed)
	elif direction == 1:
		setMotorWheelSpeed(6, speed)
		setMotorWheelSpeed(5, speed)
'''
#motion combination 1
def forward(speed):
	LWheelCorrection = 0
	RWheelCorrection = 0

	if getSensorValue(RSENSOR) > 600:
		RWheelCorrection = 50
		print("correct r")
	if getSensorValue(LSENSOR) > 600:
		LWheelCorrection = 50
		print("correct l")

	
	setMotorWheelSpeed(5, 1024+speed+RWheelCorrection )
	setMotorWheelSpeed(6, speed+LWheelCorrection)

def forwardOne():
#don't call outside of "moveOne"
	rospy.loginfo("Taking One Step")
	forward(800)
	rospy.sleep(2)
	stop()
		
'''
#motion combination2
def forward(speed):
	setMotorWheelSpeed(5, 1024+speed )
	setMotorWheelSpeed(6, speed)

def forwardOne():
#don't call outside of "moveOne"
	rospy.loginfo("Taking One Step")
	r = rospy.Rate(5)
	thresh = 300
	#Lthresh = 150
	correct = .2
	n=0
	while n<11:
		n +=1
		sensorL = getSensorValue(LSENSOR)
		sensorR = getSensorValue(RSENSOR)
		dms = getSensorValue(DMS)
		if dms > 1450:
			stop()
			return
		if sensorL>thresh:
			setMotorWheelSpeed(6, 800+min(223, correct*sensorL))
			setMotorWheelSpeed(5, 1824)
		elif sensorR>thresh:
			setMotorWheelSpeed(5, 1824+min(223, correct*sensorR))
			setMotorWheelSpeed(6, 800)
		else:
			forward(800)
		r.sleep()
	stop()

def turnAngle(angle):
#angle is degrees/90
	while angle < -2:
		angle += 4
	while angle > 2:
		angle -= 4
		
	duration = turnTime #Times we've used: 1.134, 1.180, 1.175, 1.135
	if angle == 0:
		return
	elif angle == -2:
		rospy.loginfo("Turn Left Twice")
		turn(400, 0)
		rospy.sleep(duration)
		stop()
		turn(400, 0)
		rospy.sleep(duration)
		stop()
	elif angle == -1:
		rospy.loginfo("Turn Left")
		turn(400, 0)
		rospy.sleep(duration)
		stop()
	elif angle == 1:
		rospy.loginfo("Turn Right Once")
		turn(400, 1)
		rospy.sleep(duration)
		stop()
	elif angle == 2:
		rospy.loginfo("Turn Right Twice")
		turn(400, 1)
		rospy.sleep(duration)
		stop()
		turn(400, 1)
		rospy.sleep(duration)
		stop()
	
def moveDir(direction, map):
	global Dir
	
	turnAngle(direction-Dir)
	
	Dir = direction
	if Dir == 1:
		Loc[0] -= 1
	elif Dir == 2:
		Loc[1] += 1
	elif Dir == 3:
		Loc[0] += 1
	elif Dir == 4:
		Loc[1] -= 1
		
	if map.getNeighborObstacle(Loc[0], Loc[1], Dir):
		goToWall()
	else:
		forwardOne()
	rospy.loginfo("Loc: %d \t %d", Loc[0], Loc[1])
	rospy.loginfo("Dir: %d", Dir)

#Wow I made some correction thingies
def goToWall():
	rospy.loginfo("Walking Up to wall")
	forward(800)
	sense = getSensorValue(DMS)
	while sense < 1300:
		sense = getSensorValue(DMS)
		print(sense)
	print("stopping")
	stop()
	
###########Cost
def getNeighborX(x, y, dir):
	if dir == 1:
		return x-1
	elif dir == 2:
		return x
	elif dir == 3:
		return x+1
	elif dir == 4:
		return x
		
def getNeighborY(x, y, dir):

	if dir == 1:
		return y
	elif dir == 2:
		return y+1
	elif dir == 3:
		return y
	elif dir == 4:
		return y-1

def setAllCosts(map, x, y, cost):
	if not known[x][y]:
		return
	for dir in range(1, 5):
		if not map.getNeighborObstacle(x, y, dir):
			if map.getNeighborCost(x, y, dir) > cost + 1 or map.getNeighborCost(x, y, dir) == 1000:
				map.setNeighborCost(x, y, dir, cost + 1)
				setAllCosts(map, getNeighborX(x, y, dir), getNeighborY(x, y, dir), cost + 1)



def fillCostGrid(map, x, y):
	for i in xrange(8):
		for j in xrange(8):
			map.setCost(i, j, 1000)
	map.setCost(x, y, 0)
	setAllCosts(map, x, y, 0)
#######Cost

def walkToGoal(map, x, y, d):
	fillCostGrid(map, x, y)
	
	map.printObstacleMap()
	print known
	while True:
		currentCost = map.getCost(Loc[0], Loc[1])
		cost = [100,100,100,100]
		for ii in range(0,4):
			if not map.getNeighborObstacle(Loc[0], Loc[1], ii+1):
				cost[ii] = map.getNeighborCost(Loc[0], Loc[1], ii+1)
		if min(cost) >= currentCost:
			if not d==0:
				turnAngle(d-Dir)
			return currentCost
		else:
			moveDir(cost.index(min(cost))+1, map)

def getPath(map, x, y, d):
	fillCostGrid(map, x, y)
	return getPathSegment(map, Loc[0], Loc[1], x, y)

def getPathSegment(map, sx, sy, gx, gy):
	for neighbor in range(4):
		if not map.getNeighborObstacle(sx, sy, neighbor+1) and map.getNeighborCost(sx, sy, neighbor+1) < map.getCost(sx, sy):
			tail = getPathSegment(map, getNeighborX(sx, sy, neighbor+1), getNeighborY(sx, sy, neighbor+1), gx, gy)
			tail.insert(0, (sx,sy))
			return tail
	return [(sx, sy)]

#Finds if a block should never be returned to
	
	
def wrap(i):
	while i < 1:
		i += 4
	while i > 4:
		i -= 4
	return i
	
def surrounded(arr, x, y):
	if x < 0 or y < 0 or x > 7 or y > 7:
		return False
	toReturn = True
	print x
	print y
	toReturn = toReturn and (x<=0 or arr[x-1][y])
	toReturn = toReturn and (y<=0 or arr[x][y-1])
	toReturn = toReturn and (x>=7 or arr[x+1][y])
	toReturn = toReturn and (y>=7 or arr[x][y+1])
	return toReturn
#	return (x<=0 or arr[x-1][y]) and (y<=0 or arr[x][y-1]) and (x>=7 or arr[x+1][y]) and (y>=7 or arr[x][y+1])

def setBoundWalls(map):
	for i in range(8):
		map.setObstacle(i, 0, 1, 4)
		map.setObstacle(i, 7, 1, 2)
		map.setObstacle(0, i, 1, 1)
		map.setObstacle(7, i, 1, 3)


def mapBuild():
	global known
	coolMap = EECSMap()
	coolMap.clearObstacleMap()
	setBoundWalls(coolMap)
	IR_threshold = 32
	DMS_threshold = 1000

	known = numpy.zeros((8,8))

	while True:
		print Loc
		known[Loc[0]][Loc[1]] = 1
		print known
		if getSensorValue(LSENSOR) > IR_threshold:
			coolMap.setObstacle(Loc[0], Loc[1], 1, wrap(Dir-1))
		if getSensorValue(RSENSOR) > IR_threshold:
			coolMap.setObstacle(Loc[0], Loc[1], 1, wrap(Dir+1))
		if getSensorValue(DMS) > DMS_threshold:
			coolMap.setObstacle(Loc[0], Loc[1], 1, Dir)
		
		if surrounded(known, Loc[0]-1, Loc[1]):
			known[Loc[0]-1][Loc[1]] = 1
		
		if surrounded(known, Loc[0]+1, Loc[1]):
			known[Loc[0]+1][Loc[1]] = 1
		
		if surrounded(known, Loc[0], Loc[1]-1):
			known[Loc[0]][Loc[1]-1] = 1
		
		if surrounded(known, Loc[0], Loc[1]+1):
			known[Loc[0]][Loc[1]+1] = 1
		
		fillCostGrid(coolMap, Loc[0], Loc[1])
		closestX = Loc[0]
		closestY = Loc[1]
		closestDist = 1000000
		for x in range(8):
			for y in range(8):
				if (not known[x][y]) and coolMap.getCost(x, y) < closestDist:
					closestX = x
					closestY = y
					closestDist = coolMap.getCost(x, y)
		if Loc[0] == closestX and Loc[1] == closestY:
			break
		
		print Loc
		print [closestX, closestY]
		known[closestX][closestY] = 1
		walkToGoal(coolMap, closestX, closestY, 0)
		known[closestX][closestY] = 0

	coolMap.printObstaclemap()

def findAllWalls(map):
	pass
	

# Main function
if __name__ == "__main__":
	rospy.init_node('example_node', anonymous = True)
	rospy.loginfo("Starting Group L Control Node...")
	signal.signal(signal.SIGINT, shutdown)
	r = rospy.Rate(2) # 10hz

	global Loc 
	global Dir
	global turnTime
	global LSENSOR
	global RSENSOR
	global DMS
	LSENSOR = 1
	RSENSOR = 4
	DMS = 3
	
	Loc = [int(sys.argv[1]), int(sys.argv[2])]
	Dir = int(sys.argv[3])
	'''
	goal = [int(sys.argv[4]), int(sys.argv[5])]
	goalDir = int(sys.argv[6])
	'''
	turnTime = float(sys.argv[4])
	
	
	newMap = EECSMap()
	'''
	print("Path:")
	print(getPath(newMap, goal[0], goal[1], goalDir))
	'''
	
	#turnAngle(2)
	#turnAngle(2)
	
	mapBuild()
	#forwardOne()
	
	while not rospy.is_shutdown():
		# call function to get sensor value
		rospy.loginfo("Sensor value at port L: %f port R: %f",getSensorValue(LSENSOR),getSensorValue(RSENSOR))
		r.sleep()


