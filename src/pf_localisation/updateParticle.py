#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from util import rotateQuaternion, getHeading

import random

class UpdateParticleCloud():
	
	totalWeight = 0.0
	maxWeight = 0.0
	particleWeights = []
	
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

		self.weight_amcl(scan, pf)

		resampledPoses = self.resample(pf.particlecloud.poses, pf.weights, pf.totalWeight)
	
		return self.smudge_amcl(resampledPoses)

	def weight_amcl(self, scan, pf):
		self.weight_particles(scan, pf)

		#if the maximum weighted particle has a weight below 7 reinitialise the particles
		if pf.maxWeight < 6:
			pf.particlecloud = pf.reinitialise_cloud(pf.estimatedpose.pose.pose, 3.0, True)
			self.weight_particles(scan, pf)

	def smudge_amcl(self, resampledPoses):
				
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

	def resample(self, particles, particleWeights, tWeight):
		numParticles = len(particles)
			
		resampledPoses = []
		index = 0
		notAccepted = True
	
		#Resample the poses
		for i in range(0,numParticles):
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				particle = particles[index]
				posX = particle.position.x
				posY = particle.position.y
				if (random.uniform(0,1) < particleWeights[index]/tWeight):
					notAccepted = False
			resampledPoses.append(particle)
		return resampledPoses

	#Updates particles according to KLD-AMCL
	def update_kld_amcl(self, scan, pf):

		self.weight_particles(scan, pf)

		numParticles = len(pf.particlecloud.poses)

		rospy.loginfo(maxWeight)

		#if the maximum weighted particle has a weight below 10 reinitialise the particles
		if maxWeight < 7:
			pf.particlecloud = pf.reinitialise_cloud(pf.estimatedpose.pose.pose, 3.0, True)
			self.weight_particles(scan, pf)
			numParticles = len(pf.particlecloud.poses)


		resampledPoses = []
		index = 0
		notAccepted = True

		#Initialize KLD Sampling 	
		ztable = [i/maxWeight for i in particleWeights]
		support_samples=0
		num_samples=0
		quantile=0.5
		kld_error = 0.1
		bin_size = 0.1
		min_samples=10
		seed=-1
		kld_samples = 0

		if (min_samples < self.ABSOLUTE_MIN):
			kld_samples=self.ABSOLUTE_MIN
		else:
			kld_samples=min_samples

		bins = []

		confidence=quantile-0.5; # ztable is from right side of mean
		confidence=min(0.49998,max(0,confidence))

		max_error = kld_error;
		bin_size = bin_size; # list of lists

		zvalue=4.1;
		for i in range(len(ztable)):
			if(ztable[i] >= confidence):
				zvalue=i/100.0
				break
	
		#Resample the poses
		samples = []

		while (num_samples < min_samples and num_samples < 250) :
			#Get Sample
			notAccepted = True
			while (notAccepted):
				index = random.randint(0,numParticles-1)
				posX = pf.particlecloud.poses[index].position.x
				posY = pf.particlecloud.poses[index].position.y

				if (random.uniform(0,1) < particleWeights[index]/totalWeight):
					notAccepted = False

			curr_sample = pf.particlecloud.poses[index]
			samples.append(curr_sample)
			num_samples = num_samples+1
			curr_bin = curr_sample

			if len(bins)==0 or curr_bin not in bins:
				bins.append(curr_bin);
				support_samples = support_samples+1
			if support_samples>=2:
				k = support_samples-1
				k=math.ceil(k/(2*max_error)*pow(1-2/(9.0*k)+math.sqrt(2/(9.0*k))*zvalue,3))
				if k>kld_samples:
				    kld_samples = k

			min_samples = kld_samples
			
	
		resampledPoses = samples
		rospy.loginfo("Size Samples = %s"%len(samples))
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

