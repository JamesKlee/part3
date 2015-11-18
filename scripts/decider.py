#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster

estimates = {}

def cluster_callback(c):
	rospy.loginfo("Received cluster from " + c.floorName + " #" + str(c.header.seq))
	
	# add into result set
	estimates[c.floorName] = c.pointsInCluster

def estimate_floor():
	return max(estimates, key=(lambda k: estimates[k]))

def main():
	rospy.init_node("cluster_decider")
	rospy.Subscriber("cluster", Cluster, cluster_callback)
	
	rate = rospy.Rate(1) # 1hz
	
	while not rospy.is_shutdown():
		print("clusters = " + str(estimates))
		
		if len(estimates) > 0:
			print("I'm probably on the " + estimate_floor())
		print("")
		
		rate.sleep()

if __name__ == "__main__":
	main()
