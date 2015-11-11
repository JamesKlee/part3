#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
from util import rotateQuaternion, getHeading

import random

class UpdateParticleCloud():
	
	totalWeight = 0.0
	maxWeight = 0.0
	WIDTH = 0.0
	HEIGHT = 0.0
	particleWeights = []
	storedPose = None
	listLocation = 0
	
	#Weights all of the particles in the particle cloud
	def weight_particles(self, scan, pf):

		global maxWeight
		global totalWeight
		global HEIGHT
		global WIDTH
		global particleWeights
		global exploded
		global listLocation

		maxWeight = 1.0
		totalWeight = 1.0

		HEIGHT = pf.occupancy_map.info.height * pf.occupancy_map.info.resolution
		WIDTH = pf.occupancy_map.info.width * pf.occupancy_map.info.resolution
		
		particleWeights = []
		
		#Calls the function to weight particles and record max weight and total weight
		for i in range(0, len(pf.particlecloud.poses)):
			weight = pf.sensor_model.get_weight(scan, pf.particlecloud.poses[i])
			particleWeights.append(weight)
			totalWeight += weight
			
			if weight > maxWeight:
				maxWeight = weight
				listLocation = i

	#Updates particles according to MCL
	def update_non_amcl(self, scan, pf):

		self.weight_particles(scan, pf)
		
		resampledPoses = []
		index = 0
		notAccepted = True
		numParticles = len(pf.particlecloud.poses)

		#Resamples the poses
		for i in range(0,numParticles):
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				posX = pf.particlecloud.poses[index].position.x
				posY = pf.particlecloud.poses[index].position.y
				if (random.uniform(0,1) < particleWeights[index]/totalWeight):
					notAccepted = False
			resampledPoses.append(pf.particlecloud.poses[index])

		cont = True
		pArray = PoseArray()
		temp = []
		val = Pose()
		count = 0

		#Smudges the poses
		while cont:
			temp = []
			val = resampledPoses[0]
			count = 0

			#Removes the duplicate poses from the list
			for i in range(0, len(resampledPoses)):
				if (resampledPoses[i] == val):
					count = count + 1
				else:
					temp.append(resampledPoses[i])

			resampledPoses = temp

			#Checks that we have allocated all particles to be smudged
			if (len(resampledPoses) == 0):
				cont = False
				
			#Apply smuding to all but one of the same resampled particle
			for i in range(0, count):
				if i > 0:
					newPose = Pose()
					newPose.position.x = random.gauss(val.position.x, 0.3) #TEST THIS
					newPose.position.y = random.gauss(val.position.y, 0.3)
					newPose.orientation = rotateQuaternion(val.orientation, random.vonmisesvariate(0, 4))
					 #MAKE SURE TO TEST
					pArray.poses.append(newPose)
					
				else:
					pArray.poses.append(val)
			
		return pArray

	#Updates particles according to AMCL
	def update_amcl(self, scan, pf):
		global listLocation

		self.weight_particles(scan, pf)

		numParticles = len(pf.particlecloud.poses)

		rospy.loginfo(maxWeight)

		#if the maximum weighted particle has a weight below 10 reinitialise the particles
		if maxWeight < 7:
			exploded = True

			pf.particlecloud = pf.reinitialise_cloud(pf.estimatedpose.pose.pose, 3.0, True)
			self.weight_particles(scan, pf)
			numParticles = len(pf.particlecloud.poses)

			
		resampledPoses = []
		index = 0
		notAccepted = True
	
		#Resample the poses
		for i in range(0,numParticles):
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				posX = pf.particlecloud.poses[index].position.x
				posY = pf.particlecloud.poses[index].position.y
				if (random.uniform(0,1) < particleWeights[index]/totalWeight):
					notAccepted = False
			resampledPoses.append(pf.particlecloud.poses[index])
	
		cont = True
		pArray = PoseArray()
		temp = []
		val = Pose()
		count = 0
			
		# TEST this value, rounding scalar
		scale = 0.66
	
		while cont:
			temp = []
			val = resampledPoses[0]
			count = 0
	
			for i in range(0, len(resampledPoses)):
				if (resampledPoses[i] == val):
					count = count + 1
				else:
					temp.append(resampledPoses[i])
	
			resampledPoses = temp
			if (len(resampledPoses) == 0):
				cont = False
						
			# THIS NEEDS TESTS, look at scalar above
			if (count > 4) and len(resampledPoses) >= 50: #TEST
				#count = count - 2
				count = int(count * scale)
						
			for i in range(0, count):
				if i > 0:
					newPose = Pose()
					newPose.position.x = random.gauss(val.position.x, 0.3) #TEST THIS
					newPose.position.y = random.gauss(val.position.y, 0.3) #TEST THIS
					newPose.orientation = rotateQuaternion(val.orientation, random.vonmisesvariate(0, 4)) #TEST THIS
					pArray.poses.append(newPose)
						
				else:
					pArray.poses.append(val)

		return pArray
