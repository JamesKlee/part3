#!/usr/bin/python

import sys
import math
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster

class ClusterTask:
	def __init__(self):
		self.pub = rospy.Publisher("cluster", Cluster, queue_size=5)
		self.seq = 0
	
	def gen_header(self):
		h = Header()
		h.stamp = rospy.Time.now()
		
		h.seq = self.seq
		self.seq += 1
	
		return h

	def publish(self, pose, pointsInCluster):
		c = Cluster()
		c.header = self.gen_header()
		c.floorName = "basement"
		c.cluster = pose
		c.pointsInCluster = pointsInCluster
		
		# ...
		
		self.pub.publish(c)

def main(argv):
	task = ClusterTask()
	
	rospy.init_node("cluster_task" , anonymous=True) # multiple tasks
	
	for i in range(1, 23):
		task.publish(Pose())
	
if __name__ == "__main__":
	main(sys.argv)
