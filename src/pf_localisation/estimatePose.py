#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from util import rotateQuaternion, getHeading
from dbscan import DBScan

class EstimatePose():

	# Finds the average estimated pose
	def avg_estimate(self, pf):
		sumX = 0
		sumY = 0
		sumOrientation = 0
		particles = pf.particlecloud

		for i in range (0, len(particles.poses)):
			sumX += particles.poses[i].position.x
			sumY += particles.poses[i].position.y
			heading = getHeading(particles.poses[i].orientation)
			sumOrientation += getHeading(particles.poses[i].orientation)

		pose = Pose()
		pose.position.x = sumX/len(particles.poses)
		pose.position.y = sumY/len(particles.poses)
		orientation = pf.estimatedpose.pose.pose.orientation
		pose.orientation = rotateQuaternion(orientation, sumOrientation/len(particles.poses) - getHeading(orientation))

		rospy.loginfo("X: " + str(pose.position.x) +", Y: " + str(pose.position.y))
		return pose
		
	# Estimate the pose based upon the largest cluster
	def dbscan_estimate(self, pf):
		eps = 0.5
		minposes = 4
		
		# Run DBSCAN
		db = DBScan(pf.particlecloud, eps, minposes)
		db.run()

        # Get the estimate
		guess = db.getguess()
		self.largestClusterSize = db.largestClusterSize
		
		return guess
	
	# Get the size of the currently largest cluster
	def dbscan_largestclustersize(self):
		return self.largestClusterSize
