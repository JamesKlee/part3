#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster
from collections import Counter

likelihood = {}
posterior = {}

def cluster_callback(c):
	floor = c.floorName
	
	likelihood[floor] = c.pointsInCluster / float(c.totalPoints)
	
	if not posterior.has_key(floor):
		posterior[floor] = 0.5

def main():
	rospy.init_node("cluster_decider")
	rospy.Subscriber("cluster", Cluster, cluster_callback)
	
	rate = rospy.Rate(2) # 2hz
	
	while not rospy.is_shutdown():
		total = 0
		
		for (floor, val) in posterior.items():
			# multiply by the previous probability
			posterior[floor] *= likelihood[floor]
			total += posterior[floor]
		
		# normalise the posterior
		for (floor, val) in posterior.items():
			posterior[floor] /= total
		
		# get the floor with the largest probability
		if len(posterior):
			print("")
			print(posterior)
			guess = max(posterior, key=(lambda k: posterior[k]))
			rospy.loginfo("I'm probably on the " + guess)
		
		rate.sleep()

if __name__ == "__main__":
	main()
