#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan

FORWARD_SPEED = 1
ROTATE_SPEED = 0.5
STOP_DISTANCE = 0.5
SCAN_THRESHOLD = 75
SCAN_SENSITIVITY = 0.1
VISION_ANGLE = 1.2
TURN_SPEED= 1

rotating = False;
storedAngle = 0.0

publishMotor = rospy.Publisher('cmd_vel', Twist, queue_size = 100)
rospy.init_node('wander')

def laserCall(scan):

	global rotating
	global storedAngle
	global SCAN_THESHOLD
	global SCAN_SENSITIVITY	
	global VISION_ANGLE

	ranges = scan.ranges
	angle = scan.angle_min
	minAngle = angle;
	maxAngle = angle;
	minDistance = ranges[0]
	maxDistance = ranges[0]
	maxPositionArray = 0
	minPosiitonArray = 0
	
	distances = []

	for i in range (0,len(ranges)):
		angle += scan.angle_increment
		if ranges[i] > scan.range_max:
			distances.append(scan.range_max)
		else:	
			distances.append(ranges[i])
		
		if distances[i] < minDistance and angle >= -VISION_ANGLE and angle <= VISION_ANGLE and ranges[i] >= scan.range_min:
			minDistance = distances[i]
			minAngle = angle
			minPositionArray = i
		if distances[i] >= maxDistance and angle >= -VISION_ANGLE and angle <= VISION_ANGLE and ranges[i] <= scan.range_max:
			maxDistance = distances[i]
			maxAngle = angle
			maxPositionArray = i

	if maxAngle == scan.angle_min:
		maxAngle = 0.0

	minAngle = -minAngle
	maxAngle = -maxAngle

	forwardSpeed = 0.0
	rotateSpeed = 0.0	

	if minDistance < STOP_DISTANCE:
		if not rotating:
			storedAngle = minAngle 
			rotating = True
		else:
			minAngle = storedAngle

		if minAngle <= 0.0:
			rotateSpeed = -ROTATE_SPEED
		else:
			rotateSpeed = ROTATE_SPEED
	else:
		rotating = False;

		rotateSpeed = math.sin(-maxAngle) * TURN_SPEED
		
		forwardSpeed = FORWARD_SPEED

	move = Twist(Vector3(forwardSpeed,0,0), Vector3(0,0,rotateSpeed))
	publishMotor.publish(move)

subcribeLaser = rospy.Subscriber('base_scan', LaserScan, laserCall)
rospy.spin()  

