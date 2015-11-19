#!/usr/bin/python

import rospy
from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from pf_localisation.msg import WeightedParticles, Registration
from pf_localisation.updateParticle import UpdateParticleCloud
import thread

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
		self.lock = thread.allocate_lock()

		self._cloud_publisher = rospy.Publisher("/updatedCloud", PoseArray)
		self._weighted_particle_subscriber = rospy.Subscriber("/weightedParticles", WeightedParticles, self.addParticles, queue_size=1)
		self._register_subscriber = rospy.Subscriber("/regNode", Registration, self.register, queue_size=1)
		rospy.loginfo("RUNNING")
		
	def register(self, reg):
		self.lock.acquire()
		nFound = True
		pos = 0
		for i in range (0, len(self.registered)):
			if reg.frame_id == self.registered[i][0]:
				nFound = False
				pos = i

		if nFound and reg.toAdd:
			self.registered.append((reg.frame_id, reg.freePoints, reg.resolution))
			rospy.loginfo("\tREGISTERED: " + reg.frame_id)
			self.updater.mapInfo = self.registered
		elif not nFound and not reg.toAdd:
			del self.registered[pos]
			rospy.loginfo("\tDEREGISTERED: " + reg.frame_id)
		self.lock.release()

	def addParticles(self, wParticles):
		self.lock.acquire()
		name = wParticles.poseArray.header.frame_id
		toAdd = False
		posReg = None
		for i in range(0, len(self.registered)):			
			if self.registered[i][0] == name:
				toAdd = True
				posReg = i
		if not toAdd:
			print("NOT FOUND IN REGISTERED")
			self.lock.release()
			return
		print("FOUND IN REGISTERED")
		toAdd = True
		for i in range(0, len(self.particlesAdded)):
			if self.particlesAdded[i] == name:
				toAdd = False
		if not toAdd:
			self.lock.release()
			return
		rospy.loginfo("\tRECEIVED: " + name)

		for i in range(0, len(wParticles.poseArray.poses)):
			newWT = (name, wParticles.poseArray.poses[i], wParticles.array[i])
			self.particleWT.append(newWT)

		self.totalWeight = self.totalWeight + wParticles.totalWeight
		self.particlesAdded.append(name)

		if len(self.particlesAdded) == len(self.registered):
			self.resample()
		else:
			self.lock.release()
			
		

	def resample(self):
		particles = self.updater.resample_kld(self.particleWT, self.totalWeight, self.registered)
		toSend = []
		for i in range(0, len(self.registered)):
			toAdd = []
			toAdd.append(self.registered[i][0])
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
			time.sleep(0.075)

		self.particleWT = []
		self.particlesAdded = []
		self.totalWeight = 0

		self.lock.release()

	def send(self, map_topic, particles):
		rospy.loginfo("\tSENDING TO: " + map_topic)
		pArray = PoseArray()
		pArray.header.seq = 1
		pArray.header.stamp = rospy.get_rostime()
		pArray.header.frame_id = map_topic
		pArray.poses = particles
		self._cloud_publisher.publish(pArray)

rospy.init_node("master")
Node()
rospy.spin()
