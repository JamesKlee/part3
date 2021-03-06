#!/usr/bin/python

import rospy
from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from pf_localisation.msg import WeightedParticles, Registration, Particles
from pf_localisation.updateParticle import UpdateParticleCloud
import thread

import sys
import time
from copy import deepcopy

class Node(object):

	def __init__(self, ftype):
		if ftype == "amcl" or ftype == "kld":
			self.ftype = ftype
		else:
			rospy.loginfo("ERROR: Filter '" + str(ftype) + "' not accepted")
			sys.exit(1)
		self.registered = []
		self.particleWT = []
		self.particlesAdded = []
		self.totalWeight = 0
		self.updater = UpdateParticleCloud()
		self.lock = thread.allocate_lock()
		self.reinitList = []
		self.reinit = False
		self.mapAdded = False

		self._cloud_publisher = rospy.Publisher("/updatedCloud", Particles)
		self._weighted_particle_subscriber = rospy.Subscriber("/weightedParticles", WeightedParticles, self.addParticles, queue_size=100)
		self._register_subscriber = rospy.Subscriber("/regNode", Registration, self.register, queue_size=10)
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
			self.mapAdded = True
			rospy.loginfo("\tREGISTERED: " + reg.frame_id)
		elif not nFound and not reg.toAdd:
			del self.registered[pos]
			rospy.loginfo("\tDEREGISTERED: " + reg.frame_id)
			self.mapAdded = True
			
			if len(self.particlesAdded) == len(self.registered):
				self.resample()

		self.updater.mapInfo = self.registered
		self.lock.release()

	def addParticles(self, wParticles):
		self.lock.acquire()
		name = wParticles.poseArray.header.frame_id
		#print(name)
		toAdd = False
		for i in range(0, len(self.registered)):			
			if self.registered[i][0] == name:
				toAdd = True
		if not toAdd:
			print("NOT FOUND IN REG")
			self.lock.release()
			return
		toAdd = True
		for i in range(0, len(self.particlesAdded)):
			if self.particlesAdded[i] == name:
				toAdd = False
		if not toAdd:
			self.lock.release()
			return

		for i in range(0, len(wParticles.poseArray.poses)):
			newWT = (name, wParticles.poseArray.poses[i], wParticles.array[i])
			self.particleWT.append(newWT)

		self.totalWeight = self.totalWeight + wParticles.totalWeight
		self.particlesAdded.append(name)

		self.reinitList.append(wParticles.reinit)

		if len(self.particlesAdded) == len(self.registered):
			self.resample()
		else:
			self.lock.release()
			
		

	def resample(self):

		#rospy.loginfo("PARTICLES RECIEVED: " + str(len(self.particleWT)))
		
		particles = None
		if self.ftype == "kld":
			particles = self.updater.resample_kld(self.particleWT, self.totalWeight)
		elif self.ftype == "amcl":
			particles = self.updater.resample_amcl(self.particleWT, self.totalWeight)
		else:
			rospy.logError("ERROR IN TYPE OF RESAMPLE")
		
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

		self.reinit = True

		if not self.mapAdded:
			for i in range (0, len(self.reinitList)):
				if self.reinitList[i] == False:
					self.reinit = False
				break

		self.count = 0
		

		for i in range(0, len(toSend)):
			time.sleep(0.1 * len(self.registered))
			name = toSend[i][0]
			if len(toSend[i]) <= 1:
				list = []
			else:
				list = toSend[i]
				del list[0]
			self.send(name,list)

		self.particleWT = []
		self.particlesAdded = []
		self.totalWeight = 0
		self.reinitList = []
		self.mapAdded = False
		self.updater.reinit = False

		self.lock.release()

	def send(self, map_topic, plist):
		particles = Particles()
		particles.particles.header.seq = 1
		particles.particles.header.stamp = rospy.get_rostime()
		particles.particles.header.frame_id = map_topic
		particles.particles.poses = plist
		self.count = self.count + len(plist)
		if self.ftype == "amcl" or self.mapAdded:
			particles.reinit = self.reinit
		else:
			particles.reinit = self.updater.reinit
		self._cloud_publisher.publish(particles)
		#print("SENT")

rospy.init_node("master")
if len(sys.argv) != 2:
	print("\tUSAGE: master.py <filterType>")
	sys.exit(1)
ftype = str(sys.argv[1])
Node(ftype)
rospy.spin()
