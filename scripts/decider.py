#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster
from collections import Counter

estimates = {}

def cluster_callback(c):
	#rospy.loginfo("Received cluster from " + c.floorName + " #" + str(c.header.seq))
	
	# add into result set
	estimates[c.floorName] = c.pointsInCluster / float(c.totalPoints)

# get the name of the floor with the largest cluster
def estimate_floor():
	return max(estimates, key=(lambda k: estimates[k]))

def main():
	rospy.init_node("cluster_decider")
	rospy.Subscriber("cluster", Cluster, cluster_callback)
	
	rate = rospy.Rate(2) # 2hz
	
	history = []
	hlen = 5
	
	while not rospy.is_shutdown():
		rospy.loginfo("clusters = " + str(estimates))
		rospy.loginfo("history = " + str(history))
		
		# add the largest cluster to history
		if len(estimates) > 0:
			history.insert(0, estimate_floor())
			
			if len(history) > hlen:
				history = history[:hlen]
		
		# take a guess 
		if len(history) >= hlen:
			guess = Counter(history[:hlen]).most_common(1)[0][0]
		
			rospy.loginfo("I'm probably on the " + guess)
		
		rate.sleep()

if __name__ == "__main__":
	main()
