#!/usr/bin/python
import math
import rospy
import random

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from util import rotateQuaternion, getHeading

class InitialiseCloud():

	HEIGHT = 0
	WIDTH = 0

	def set_map_dim(self, pf):
		global HEIGHT
		global WIDTH

		WIDTH = pf.occupancy_map.info.width * pf.occupancy_map.info.resolution
		HEIGHT = pf.occupancy_map.info.height * pf.occupancy_map.info.resolution

	def gauss_initialise(self, initialpose, numParticles, ratio, rand, pf, test):
		noisePose = 0.4
		self.set_map_dim(pf)		

		# Initialisation of Gaussian Parameters
		meanX = initialpose.position.x
		meanY = initialpose.position.y
		sigmaX = ratio # Test Value
		sigmaY = ratio  # Test Value

		poseArray = PoseArray()
		poseArray.header.stamp = rospy.Time.now()
		
		#Generation of Poses
		for m in range(0, numParticles):
			xNewPose = -1
			yNewPose = -1

			xNewPose = meanX + (random.gauss(0,sigmaX)*noisePose) 
			yNewPose = meanY + (random.gauss(0,sigmaY)*noisePose)

			
			#newPose Parameters
			newPose  = Pose()
			newPose.position.x = xNewPose
			newPose.position.y = yNewPose
			newPose.orientation = None

			#Different orientations assigned for different tests/situations
			if test:
				newPose.orientation = initialpose.orientation
			elif rand:
				newPose.orientation = rotateQuaternion(initialpose.orientation, random.uniform(-math.pi, math.pi))
			else:
				newPose.orientation = rotateQuaternion(initialpose.orientation, random.vonmisesvariate(0, 4))

			poseArray.poses.append(newPose)

		return poseArray

	def uniform_initialise(self, initialpose, numParticles, pf):
		self.set_map_dim(pf)		
		poseArray = PoseArray()
		poseArray.header.stamp = rospy.Time.now()
		listFreePoints = pf.listFreePoints

		for m in range(0, numParticles):

			randUninform = int(random.uniform(0,len(listFreePoints)-1))
			coordinates = listFreePoints[randUninform]
			xNewPose = coordinates.x * pf.occupancy_map.info.resolution
			yNewPose = coordinates.y * pf.occupancy_map.info.resolution
			
			#newPose Parameters
			newPose  = Pose()
			newPose.position.x = xNewPose
			newPose.position.y = yNewPose
			newPose.orientation = rotateQuaternion(initialpose.orientation, random.uniform(-math.pi, math.pi))
			poseArray.poses.append(newPose)
		return poseArray

	def equal_initialise(self, initialpose, numParticles, pf):
		self.set_map_dim(pf)
		poseArray = PoseArray()
		poseArray.header.stamp = rospy.Time.now()
		listFreePoints = pf.listFreePoints

		pixelGap = 18

		for m in range(0, len(listFreePoints), pixelGap):
			coordinates = listFreePoints[m]
			if coordinates.y % pixelGap == 0.0:
				xNewPose = coordinates.x * pf.occupancy_map.info.resolution
				yNewPose = coordinates.y * pf.occupancy_map.info.resolution
			
				#newPose Parameters
				newPose  = Pose()
				newPose.position.x = xNewPose
				newPose.position.y = yNewPose
				newPose.orientation = rotateQuaternion(initialpose.orientation, random.uniform(-math.pi, math.pi))
				poseArray.poses.append(newPose)
		return poseArray



