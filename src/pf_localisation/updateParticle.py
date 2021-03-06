#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from util import *

import random
from weightParticle import weightParticle

class UpdateParticleCloud():
	
	totalWeight = 0.0
	maxWeight = 0.0
	particleWeights = []

	def __init__(self):
		self.reinit = False
		self.reachedMax = False
	
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
		#if the maximum weighted particle has a weight below 3 reinitialise the particles
		rospy.loginfo("MaxWeight: " + str(maxWeight))
		if pf.maxWeight < 3.5:
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
					newPose.position.x = random.gauss(val.position.x, 0.35) #TEST THIS
					newPose.position.y = random.gauss(val.position.y, 0.35)
					newPose.orientation = rotateQuaternion(val.orientation, random.vonmisesvariate(0, 7))
					 #MAKE SURE TO TEST
					pArray.poses.append(newPose)
					
				else:
					pArray.poses.append(val)
			
		return pArray

	#Updates particles according to AMCL
	def update_amcl(self, scan, pf):
		self.reinit = False
		self.weight_particles(scan, pf)
		
		temp = []
		for i in range(0, len(pf.particlecloud.poses)):
			temp.append((pf.floorName, pf.particlecloud.poses[i], pf.weights[i]))

		if pf.exploded:
			return pf.reinitialise_cloud(pf.estimatedpose.pose.pose, 0, False)
		else:
			resampledPoses = self.resample_amcl(temp, pf.totalWeight)
		
			temp = []
			for i in range(0, len(resampledPoses)):
				temp.append(resampledPoses[i][1])
	
			return self.smudge_amcl(temp)

	#Updates particles according to KLD-AMCL
	def update_kld(self, scan, pf):

		self.weight_particles(scan, pf)
		
		temp = []
		for i in range(0, len(pf.particlecloud.poses)):
			temp.append((pf.floorName, pf.particlecloud.poses[i], pf.weights[i]))
		
		resampledPoses = self.resample_kld(temp, pf.totalWeight)

		if self.reinit:
			return pf.reinitialise_cloud(pf.estimatedpose.pose.pose, 0, False)

		else:
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
			scale = 0.75
	
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
						newPose.position.x = random.gauss(val.position.x, 0.15) #TEST THIS
						newPose.position.y = random.gauss(val.position.y, 0.15) #TEST THIS
						newPose.orientation = rotateQuaternion(val.orientation, random.vonmisesvariate(0, 7)) #TEST THIS
						pArray.poses.append(newPose)
						
					else:
						pArray.poses.append(val)
		else:
			pArray.poses = []

		return pArray

	def resample_amcl(self, particleWT, tWeight):
		
		resampledPoses = self.resample_roulette(particleWT, tWeight)

		return resampledPoses

	def resample_roulette(self, particleWT, tWeight):
		#particleWT[i][0] is the map_topic associated with the particle
		#particleWT[i][1] is the particle
		#particleWT[i][2] is the weight associated with the particle 
		numParticles = len(particleWT)
			
		resampledPoses = []

		for i in range(0,numParticles):
			value = random.random() * tWeight
			total = 0.0
			for j in range(0, numParticles):
				particle = particleWT[j]
				total = total + particle[2]
				if value <= total:
					resampledPose = (particle[0], particle[1])
					resampledPoses.append(resampledPose)
					break
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

		#Initialize KLD Sampling
		zvalue = 1.9
		binsDict = {}
		mapsDict = {}
		maxSizeBin = {}
		binsSize = 0
		k = 0 #Number of Bins not empty
		epsilon = 0.1
		Mmin = 50
		Mmax = 200
		M = 0
		Mx = 0

		if numParticles > 0:

			#Initialising the bins
			for i in range(0, len(self.mapInfo)):
				listFreePoints = self.mapInfo[i][1]
				mapsDict[self.mapInfo[i][0]] = [0, False]

				pixelGap = 2

				for j in range(0,len(listFreePoints), pixelGap):
					currentCell = listFreePoints[j]
					cellY = currentCell.y

					if cellY % pixelGap == 0:			
						cellX = currentCell.x
						topic = self.mapInfo[i][0]
						valueBin = False
						binsDict[(topic, cellX, cellY)] = [False,0]
						maxSizeBin[topic] = 0	
						binsSize = binsSize + 1

			self.reachedMax = False

			while (((M < Mx or M < (Mmin * len(self.mapInfo))) and (not self.reachedMax)) and (not self.reinit)) :
				#Get Sample
				notAccepted = True
				while notAccepted:
					value = random.random() * tWeight
					total = 0.0
					for j in range(0, numParticles):
						particle = particleWT[j]
						total = total + particle[2]
						if value <= total:
							nm = particle[0]
							mapsDict[nm][0] += 1
							#print(mapsDict[nm][0])
							if mapsDict[nm][0] <= Mmax:
								notAccepted = False

								if mapsDict[nm][0] == Mmax:
									mapsDict[nm][1] = True
								break						
			
				curr_sample = (particle[0], particle[1])
				resampledPoses.append(curr_sample)
				M = M + 1

				#Convert Coodinates of the Pose to know if the bin is Empty or not
				xBin = -1
				yBin = -1
				for i in range(0,len(self.mapInfo)) :
					if (particle[0] == self.mapInfo[i][0]):
						mapResolution = self.mapInfo[i][2]
						xBin = int(curr_sample[1].position.x / mapResolution)
						xBin = self.base_round(xBin, pixelGap)
						yBin = int(curr_sample[1].position.y / mapResolution)
						yBin = self.base_round(yBin, pixelGap)
						break

				

				if ((curr_sample[0], xBin, yBin) in binsDict):
					#rospy.loginfo("Hello")
					binsDict[(curr_sample[0], xBin, yBin)][1] = binsDict[(curr_sample[0], xBin, yBin)][1] + 1
					#Define the largest bin for each map
					if binsDict[(curr_sample[0], xBin, yBin)][1] > maxSizeBin[curr_sample[0]]:
						maxSizeBin[curr_sample[0]] = binsDict[(curr_sample[0], xBin, yBin)][1]
					if (binsDict[(curr_sample[0], xBin, yBin)][0] == False):
						binsDict[(curr_sample[0], xBin, yBin)][0] = True

						k = k + 1
						if (k > 1):
							Mx = ((k-1)/(2*epsilon)) * math.pow(1 - (2/(9*(k-1))) + (math.sqrt(2/(9*(k-1)))*zvalue),3)
							#print("Mx: " + str(Mx))
							if Mx > 350 + 100 * len(self.mapInfo):
								self.reinit = False
							if Mx > 200 + 100 * len(self.mapInfo):
								Mx = 200 + 100 * len(self.mapInfo)
				
				self.reachedMax = True
				for key in mapsDict:
					if not mapsDict[key][1]:
						self.reachedMax = False				
						
	
			if not self.reinit:
				for i in range(0, len(self.mapInfo)):
					listFreePoints = self.mapInfo[i][1]
					for j in range(0,int(Mx), 8):
						randUninform = int(random.uniform(0,len(listFreePoints)-1))
						coordinates = listFreePoints[randUninform]
						xNewPose = coordinates.x * self.mapInfo[i][2]
						yNewPose = coordinates.y * self.mapInfo[i][2]
			
						#newPose Parameters
						newPose  = Pose()
						newPose.position.x = xNewPose
						newPose.position.y = yNewPose
						newPose.orientation = rotateQuaternion(createQuaternion(0), random.uniform(-math.pi, math.pi))
						resampledPoses.append((self.mapInfo[i][0], newPose))

		rospy.loginfo("BROKEN FROM LOOP")
		return resampledPoses

	def base_round(self, val, base):
    		return int(base * round(float(val)/base))
