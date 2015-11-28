#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
#from DBSCAN_for_ParticleFilter import DBSCAN_for_ParticleFilter
from util import rotateQuaternion, getHeading

import random
import time

class PFLocaliser(PFLocaliserBase):
       
	def __init__(self, num, mapTopic, algorithmName):
			
		# Call the superclass constructor
		super(PFLocaliser, self).__init__(num, mapTopic, algorithmName)

		# Set motion model parameters
		self.ODOM_ROTATION_NOISE = 0#0.064
		self.ODOM_TRANSLATION_NOISE = 0#1.006
		self.ODOM_DRIFT_NOISE = 0#0.086

	#Updates the particle 
	def update_particle_cloud(self, scan):
		self.cloud.reinit = False
		self.particlecloud = self.cloud.update_kld(scan, self)

	#Initialise particle cloud
	def initialise_particle_cloud(self, initialpose):			
		#return init.gauss_initialise(initialpose.pose.pose, M, 6, True, self, False)
		return self.init.uniform_initialise(initialpose.pose.pose, self.num, self)

	#Estimate the pose
	def estimate_pose(self):
		return self.estimate.dbscan_estimate(self)

	#Reinitialise the cloud in AMCL
	def reinitialise_cloud(self, initialpose, ratio, random):
		#return init.gauss_initialise(initialpose, M, ratio, random, self, False)
		return self.init.uniform_initialise(initialpose, self.num, self)
