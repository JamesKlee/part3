#!/usr/bin/python

import math
import sys
import rospy
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster
from collections import Counter

likelihood = {}
posterior = {}

def cluster_callback(c):
	floor = c.floorName
	likelihood[floor] = c.pointsInCluster / float(c.totalPoints)

def decider(floorCount):
	rospy.init_node("cluster_decider")
	rospy.Subscriber("cluster", Cluster, cluster_callback)
	
	rate = rospy.Rate(2) # 2hz
	
	rospy.loginfo("waiting for " + str(floorCount) + " floor(s)")
	
	while len(likelihood) != floorCount and not rospy.is_shutdown():
		rospy.sleep(0.1)
	
	rospy.loginfo("received messages from all floors")
	
	# initally an equal probability for all floors
	for (floor, _) in likelihood.items():
		posterior[floor] = 1.0 / floorCount
	
	while not rospy.is_shutdown():
		total = 0.0
		
		# multiply by the previous probability
		for (floor, val) in posterior.items():
			posterior[floor] *= likelihood[floor]
			total += posterior[floor]
		
		# normalise the posterior
		for (floor, val) in posterior.items():
			posterior[floor] /= total
		
		# find the floor with the largest probability
		print("")
		print(posterior)
		guess = max(posterior, key=(lambda k: posterior[k]))
		rospy.loginfo("I'm probably on the " + guess)
		
		rate.sleep()

def main(argv):
	if len(argv) != 2:
		print("invalid number of arguments")
		print("usage: " + argv[0] + " <floor count>")
		sys.exit(0)
	
	floorCount = int(argv[1])
	
	decider(floorCount)

if __name__ == "__main__":
	main(sys.argv)
