#!/usr/bin/python

import sys
import math
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster

# Publishes the estimated pose
# to the cluster channel
class ClusterTask:
	def __init__(self):
		self.pub = rospy.Publisher("cluster", Cluster, queue_size=5)
		self.seq = 0
	
	# Generate the ROS message header
	def gen_header(self):
		h = Header()
		h.stamp = rospy.Time.now()
		
		h.seq = self.seq
		self.seq += 1
	
		return h
    
    # Publish the cluster message
	def publish(self, floorName, pose, pointsInCluster, totalPoints):
		c = Cluster()
		c.header = self.gen_header()
		c.floorName = floorName
		c.cluster = pose
		c.pointsInCluster = pointsInCluster
		c.totalPoints = totalPoints
		
		self.pub.publish(c)
