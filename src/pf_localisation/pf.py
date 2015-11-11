#!/usr/bin/python
import math
import rospy

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
#from DBSCAN_for_ParticleFilter import DBSCAN_for_ParticleFilter
from util import rotateQuaternion, getHeading
from updateParticle import UpdateParticleCloud
from estimatePose import EstimatePose
from initialise import InitialiseCloud

import random
import time


class PFLocaliser(PFLocaliserBase):

	M = 0
	cloud = None
	estimate = None
	init = None
       
	def __init__(self):
		global M
		global cloud
		global estimate
		global init
		global odoTst
			
		# Call the superclass constructor
		super(PFLocaliser, self).__init__()

		# Initialise objects
		cloud = UpdateParticleCloud()
		estimate = EstimatePose()
		init = InitialiseCloud()

		# Set motion model parameters
		self.ODOM_ROTATION_NOISE = 0.064
		self.ODOM_TRANSLATION_NOISE = 1.006
		self.ODOM_DRIFT_NOISE = 0.086

		M = 200 #NON_AMCL
		#M = 250 #AMCL

	#Updates the particle 
	def update_particle_cloud(self, scan):
		self.particlecloud = cloud.update_amcl(scan, self)

	#Initialise particle cloud
	def initialise_particle_cloud(self, initialpose):			
		return init.gauss_initialise(initialpose.pose.pose, M, 6, True, self, False)

	#Estimate the pose
	def estimate_pose(self):
		return estimate.dbscan_estimate(self)

	#Reinitialise the cloud in AMCL
	def reinitialise_cloud(self, initialpose, ratio, random):
		return init.gauss_initialise(initialpose, M, ratio, random, self, False)
		
		
