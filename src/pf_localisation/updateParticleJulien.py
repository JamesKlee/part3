#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from util import rotateQuaternion, getHeading, createQuaternion

import random
from weightParticle import weightParticle

class UpdateParticleCloud():
	
	totalWeight = 0.0
	maxWeight = 0.0
	particleWeights = []

	def __init__(self):
		self.pose = Pose()
		self.pose.position.x = 0
		self.pose.position.y = 0
		self.pose.orientation = createQuaternion(0)
	
	#Weights all of the particles in the particle cloud
	def weight_particles(self, scan, pf):
		global maxWeight
		global totalWeight
		global particleWeights
		maxWeight = 0.0
		totalWeight = 0.0
		particleWeights = []
		#Calls the function to weight particles and record max weight and total weight
		for i in range(0, len(pf.particlecloud.poses)):
			weight = pf.sensor_model.get_weight(scan, pf.particlecloud.poses[i])
			particleWeights.append(weight)
			totalWeight += weight
			if weight > maxWeight:
				maxWeight = weight

		pf.weights = particleWeights
		pf.maxWeight = maxWeight
		pf.totalWeight = totalWeight
		#if the maximum weighted particle has a weight below 7 reinitialise the particles
		if pf.maxWeight < 8:
			pf.exploded = True
		else:
			pf.exploded = False

	#Updates particles according to MCL
	def update_non_amcl(self, scan, pf):

		self.weight_particles(scan, pf)
		
		resampledPoses = []
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

		self.weight_particles(scan, pf)
		
		temp = []
		for i in range(0, len(pf.particlecloud.poses)):
			temp.append((pf.floorName, pf.particlecloud.poses[i], pf.weights[i]))
		
		resampledPoses = self.resample_amcl(temp, pf.totalWeight)
		
		temp = []
		for i in range(0, len(resampledPoses)):
			temp.append(resampledPoses[i][1])
	
		return self.smudge_amcl(temp)

	#Updates particles according to KLD-AMCL
	def update_kld_amcl(self, scan, pf):

		self.weight_particles(scan, pf)
		
		temp = []
		for i in range(0, len(pf.particlecloud.poses)):
			temp.append((pf.floorName, pf.particlecloud.poses[i], pf.weights[i]))
		
		resampledPoses = self.resample_kld(temp, pf.totalWeight)
		
		temp = []
		for i in range(0, len(resampledPoses)):
			temp.append(resampledPoses[i][1])
	
		return self.smudge_amcl(temp)

	def smudge_amcl(self, resampledPoses):
				
		cont = True
		pArray = PoseArray()

		if len(resampledPoses):
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
		else:
			pArray.poses = []

		return pArray

	def resample_amcl(self, particleWT, tWeight):
		#particleWT[i][0] is the map_topic associated with the particle
		#particleWT[i][1] is the particle
		#particleWT[i][2] is the weight associated with the particle 
		numParticles = len(particleWT)
			
		resampledPoses = []
		index = 0
		notAccepted = True
	
		#Resample the poses
		for i in range(0,numParticles):
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				particle = particleWT[index]
				if (random.uniform(0,1) < particle[2]/tWeight):
					notAccepted = False
			resampledPose = (particle[0], particle[1])
			resampledPoses.append(resampledPose)
		return resampledPoses

	#Updates particles according to KLD-AMCL
	def resample_kld(self, particleWT, tWeight):
		#particleWT[i][0] is the map_topic associated with the particle
		#particleWT[i][1] is the particle
		#particleWT[i][2] is the weight associated with the particle 
		#self.mapInfo[i][0] is the map_topic associated with the listFreePoints	
		#self.mapInfo[i][1] is the listFreePoints 
		#self.mapInfo[i][2] is the resolution of the map associated with listFreePoints
		numParticles = len(particleWT)
		index = 0
		notAccepted = True
		listFreePoints = []
		resampledPoses = []
		numberSamplesMap = {}

		#Initialize KLD Sampling 	
		zvalue = 2.1
		binsDict = {}
		binsSize = 0
		k = 0 #Number of Bins not empty
		epsilon = 0.05
		Mmin = 50
		#Mmin = int(0.9 * len(particleWT))
		M = 0
		Mx = 0

		#Initialising the bins
		for i in range(0, len(self.mapInfo)):
			listFreePoints = self.mapInfo[i][1]

			for j in range(0,len(listFreePoints), 5):
				currentCell = listFreePoints[j]
				topic = self.mapInfo[i][0]				
				cellX = currentCell.x
				cellY = currentCell.y
				valueBin = False
				binsDict[(topic, cellX, cellY)] = False			
				binsSize = binsSize + 1

		#Initialising the numberSamplesMap
		for n in range(0, len(self.mapInfo)):
			numberSamplesMap[self.mapInfo[n][0]] = 0 
		"""#Resample the poses
		for m in range(0, len(self.mapInfo)):
			M = 0
			Mx = 0
			mapName = self.mapInfo[m][0]

			while (M < Mx or M < Mmin) :
				#Get Sample 
				notAccepted = True
				while (notAccepted):
					index = random.randint(0,numParticles-1)
					particle = particleWT[index]
					if (random.uniform(0,1) < particle[2]/tWeight) and (particle[0] == mapName):
						notAccepted = False

				curr_sample = (particle[0], particle[1])
				resampledPoses.append(curr_sample)
				M = M + 1

				#Convert Coodinates of the Pose to know if the bin is Empty or not
				for i in range(0,len(self.mapInfo)) :
					if (particle[0] == self.mapInfo[i][0]):
						mapResolution = self.mapInfo[i][2]
						xBin = int(curr_sample[1].position.x / mapResolution)
						yBin = int(curr_sample[1].position.y / mapResolution)
				

				for l in range(0, binsSize):
					if ((curr_sample[0], xBin, yBin) in binsDict):
						#rospy.loginfo("Hello")
						if (binsDict[(curr_sample[0], xBin, yBin)] == False):
							binsDict[(curr_sample[0], xBin, yBin)] = True
							k = k + 1
							rospy.loginfo("k is %s"%k)
							if (k > 1):
								Mx = ((k-1)/(2*epsilon)) * math.pow(1 - (2/(9*(k-1))) + (math.sqrt(2/(9*(k-1)))*zvalue),3)
							break
				rospy.loginfo("Mx is %s"%Mx)
		
		"""
		while (M < Mx or M < Mmin) :
			#Get Sample 
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				particle = particleWT[index]
				if (random.uniform(0,1) < particle[2]/tWeight):
					notAccepted = False

			curr_sample = (particle[0], particle[1])
			resampledPoses.append(curr_sample)
			M = M + 1

			#Count Number Samples
			numberSamplesMap[curr_sample[0]] = numberSamplesMap[curr_sample[0]] + 1
			#Convert Coodinates of the Pose to know if the bin is Empty or not
			xBin = -1
			yBin = -1

			for i in range(0,len(self.mapInfo)) :
				if (particle[0] == self.mapInfo[i][0]):
					mapResolution = self.mapInfo[i][2]
					xBin = int(curr_sample[1].position.x / mapResolution)
					yBin = int(curr_sample[1].position.y / mapResolution)
				break
			

			for l in range(0, binsSize):
				if ((curr_sample[0], xBin, yBin) in binsDict):
					#rospy.loginfo("Hello")
					if (binsDict[(curr_sample[0], xBin, yBin)] == False):
						binsDict[(curr_sample[0], xBin, yBin)] = True
						k = k + 1
						if (k > 1):
							Mx = ((k-1)/(2*epsilon)) * math.pow(1 - (2/(9*(k-1))) + (math.sqrt(2/(9*(k-1)))*zvalue),3)
						break

		#Add particles if one map have no particles
		nameMapAdd = None
		mapResolution = 0
		minNumberParticles = -1 #Must be different of 0

		for key in numberSamplesMap:
			rospy.loginfo("Map = %s"%key)
			rospy.loginfo("Value = %s"%numberSamplesMap[key])
			value = numberSamplesMap[key]
			if value == 0 :
				nameMapAdd = key
			if (value != 0) and ((value < minNumberParticles) or minNumberParticles == -1):
				minNumberParticles = value

		if nameMapAdd != None:
			rospy.loginfo("Map to re-Initialize = %s"%nameMapAdd)
			#Finding Correct listFreePoints
			listFreePoints = []
			rospy.loginfo("minNumberParticles = %s"%minNumberParticles)
			for p in range(0, len(self.mapInfo)):
				if self.mapInfo[p][0] == nameMapAdd:
					listFreePoints = self.mapInfo[p][1] 
					mapResolution = self.mapInfo[p][2]  				
			#Generate Particles
			for m in range(0, minNumberParticles):
				rospy.loginfo("ListFree = %s"%len(listFreePoints))
				randUninform = int(random.uniform(0,len(listFreePoints)-1))
				coordinates = listFreePoints[randUninform]
				xNewPose = coordinates.x * mapResolution
				yNewPose = coordinates.y * mapResolution
			
				#newPose Parameters
				newPose  = Pose()
				newPose.position.x = xNewPose
				newPose.position.y = yNewPose
				newPose.orientation = rotateQuaternion(self.pose.orientation, random.uniform(-math.pi, math.pi))
				curr_sample = (nameMapAdd, newPose)
				resampledPoses.append(curr_sample)

		
		rospy.loginfo("number of particles resampled %s"%len(resampledPoses))
		return resampledPoses
