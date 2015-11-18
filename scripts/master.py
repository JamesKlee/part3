#!/usr/bin/python

import rospy
from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from pf_localisation.msg import WeightedParticles, Registration
from pf_localisation.updateParticle import UpdateParticleCloud

import sys
import time
from copy import deepcopy

class Node(object):

	def __init__(self):
		self.registered = []
		self.particleWT = []
		self.particlesAdded = []
		self.totalWeight = 0
		self.updater = UpdateParticleCloud()

		self._cloud_publisher = rospy.Publisher("/updatedCloud", PoseArray)
		self._weighted_particle_subscriber = rospy.Subscriber("/weightedParticles", WeightedParticles, self.addParticles, queue_size=1)
		self._register_subscriber = rospy.Subscriber("/regNode", Registration, self.register, queue_size=1)
		rospy.loginfo("RUNNING")
		
	def register(self, reg):
		nFound = True
		pos = 0
		for i in range (0, len(self.registered)):
			if reg.frame_id == self.registered[i]:
				nFound = False
				pos = i

		if nFound and reg.toAdd:
			self.registered.append(reg.frame_id)
			rospy.loginfo("\tREGISTERED: " + reg.frame_id)
		elif not nFound and not reg.toAdd:
			del self.registered[pos]
			rospy.loginfo("\tDEREGISTERED: " + reg.frame_id)

	def addParticles(self, wParticles):
		name = wParticles.poseArray.header.frame_id
		rospy.loginfo("\tRECEIVED: " + name)
		toAdd = False
		for i in range(0, len(self.registered)):
			if self.registered[i] == name:
				toAdd == True
		if toAdd:
			return

		toAdd = True
		for i in range(0, len(self.particlesAdded)):
			if self.particlesAdded[i] == name:
				toAdd = False
		if not toAdd:
			return

		for i in range(0, len(wParticles.poseArray.poses)):
			newWT = (name, wParticles.poseArray.poses[i], wParticles.array[i])
			self.particleWT.append(newWT)

		self.totalWeight = wParticles.totalWeight
		self.particlesAdded.append(name)

		if len(self.particlesAdded) == len(self.registered):
			self.resample()
			
		

	def resample(self):
		print("ENTERED RESAMPLE")
		particles = self.updater.resample(self.particleWT, self.totalWeight)
		toSend = []
		for i in range(0, len(self.registered)):
			toAdd = []
			toAdd.append(self.registered[i])
			toSend.append(toAdd)

		for i in range (0, len(particles)):
			particle = particles[i]
			for j in range(0, len(toSend)):
				if particle[0] == toSend[j][0]:
					toSend[j].append(particle[1])

		for i in range(0, len(toSend)):
			name = toSend[i][0]
			if len(toSend[i]) == 0:
				list = []
			else:
				list = toSend[i]
				del list[0]
			self.send(name,list)

	def send(self, map_topic, particles):
		rospy.loginfo("\tSENDING TO: " + map_topic)
		pArray = PoseArray()
		pArray.header.seq = 1
		pArray.header.stamp = rospy.get_rostime()
		pArray.header.frame_id = map_topic
		pArray.poses = particles
		self._cloud_publisher.publish(pArray)
		
		self.particleWT = []
		self.particlesAdded = []
		self.totalWeight = 0

rospy.init_node("master")
Node()
rospy.spin()
