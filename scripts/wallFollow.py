#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan

FORWARD_SPEED = 0.5
ROTATE_SPEED = 0.5
STOP_DISTANCE = 0.5
REVERSE_DISTANCE = STOP_DISTANCE - 0.07
SCAN_THRESHOLD = 75
SCAN_SENSITIVITY = 0.1
TURN_SPEED= 1
WALL_DISTANCE = 0.55
WALL_VISION = 2.0
VISION_ANGLE = 0.7

rotating = False;
storedAngle = 0.0
wallFound = 0

publishMotor = rospy.Publisher('cmd_vel', Twist)
rospy.init_node('wander')

def laserCall(scan):

	global rotating
	global storedAngle
	global SCAN_THESHOLD
	global SCAN_SENSITIVITY	
	global VISION_ANGLE
	global wallFound

	ranges = scan.ranges
	angle = scan.angle_min
	minAngle = angle;
	maxAngle = angle;
	minDistance = ranges[0]
	maxDistance = ranges[0]
	maxPositionArray = 0
	minPosiitonArray = 0
	forwardDistance1 = 0.0
	forwardDistance2 = 0.0
	forwardDistance  = 0.0

	VISION_ANGLE = scan.angle_max

	for i in range (0,len(ranges)):
		if angle > -0.004 and angle < 0:
			forwardDistance1 = ranges[i]
		elif angle < 0.004 and angle > 0:
			forwardDistance2 = ranges[i]				
		if ranges[i] < minDistance and angle >= -VISION_ANGLE and angle <= VISION_ANGLE and ranges[i] >= scan.range_min:
			minDistance = ranges[i]
			minAngle = angle
			minPositionArray = i
		if ranges[i] >= maxDistance and angle >= -VISION_ANGLE and angle <= VISION_ANGLE and ranges[i] <= scan.range_max:
			maxDistance = ranges[i]
			maxAngle = angle
			maxPositionArray = i
		if angle == 0.0:
			forwardDistance = ranges[i]

		angle += scan.angle_increment

	if maxAngle == scan.angle_min:
		maxAngle = 0.0

	forwardDistance = (forwardDistance1 + forwardDistance2) / 2
	
	forwardSpeed = 0.0
	rotateSpeed = 0.0

	minAngle = -minAngle
	maxAngle = -maxAngle


	if minDistance < STOP_DISTANCE:
		if not rotating:
			storedAngle = minAngle
			rotating = True
		else:
			minAngle = storedAngle

		if wallFound == -1:
			rotateSpeed = -ROTATE_SPEED
		else:
			rotateSpeed = ROTATE_SPEED
	else:
		rotating = False;
		rotateSpeed = 0.0
		
		if minDistance < WALL_VISION or (wallFound is not 0):

			rs = (WALL_DISTANCE - minDistance) * TURN_SPEED

			if (minAngle < 0 and wallFound == 0) or wallFound == -1:
				wallFound = -1
				rotateSpeed = -rs

			elif (minAngle > 0 and wallFound == 0) or wallFound == 1:
				wallFound = 1
				rotateSpeed = rs
	
		forwardSpeed = FORWARD_SPEED

	move = Twist(Vector3(forwardSpeed,0,0), Vector3(0,0,rotateSpeed))
	publishMotor.publish(move)

subcribeLaser = rospy.Subscriber('base_scan', LaserScan, laserCall)
rospy.spin()  

