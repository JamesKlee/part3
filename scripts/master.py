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
		self.updater = UpdateParticleCloud()

		self._cloud_publisher = rospy.Publisher("/updatedCloud", PoseArray)
		self._weighted_particle_subscriber = rospy.Subscriber("/weightedParticles", WeightedParticles, self.resample, queue_size=1)
		self._register_subscriber = rospy.Subscriber("/regNode", Registration, self.register, queue_size=10)
		rospy.loginfo("RUNNING")
		
	def register(self, reg):
		nFound = True
		pos = 0
		for i in range (0, len(self.registered)):
			if reg.frame_id == self.registered[i].frame_id:
				nFound = False
				pos = i

		if nFound and reg.toAdd:
			self.registered.append(reg)
			rospy.loginfo("\tREGISTERED: " + reg.frame_id)
		elif not nFound and not reg.toAdd:
			del self.registered[pos]
			rospy.loginfo("\tDEREGISTERED: " + reg.frame_id)

	def resample(self, wParticles):
		rospy.loginfo("\tRECEIVED: " + wParticles.poseArray.header.frame_id)
		particles = self.updater.resample(wParticles.poseArray.poses, wParticles.array, wParticles.totalWeight)
		self.send(wParticles.poseArray.header.frame_id, particles)

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
