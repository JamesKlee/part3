#!/usr/bin/python

import math
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from pf_localisation.msg import Cluster
from collections import Counter

likelihood = {}
posterior = {}

def cluster_callback(c):
	floor = c.floorName
	likelihood[floor] = c.pointsInCluster / float(c.totalPoints)

# normalise a dictionary as so
# all values sum up to 1.0
def normalise(d):
	total = 0.0

	for k in d:
		total += d[k]

	for k in d:
		d[k] /= total

def decider(floorCount):
	rospy.init_node("cluster_decider")
	rospy.Subscriber("cluster", Cluster, cluster_callback)
	
	pub = rospy.Publisher("floor_likelihood", String, queue_size=10)
	
	rate = rospy.Rate(2) # 2hz
	
	# wait to hear from all floors before starting
	rospy.loginfo("waiting for " + str(floorCount) + " floor(s)")
	
	while len(likelihood) != floorCount and not rospy.is_shutdown():
		rospy.sleep(0.1)
	
	rospy.loginfo("received messages from all floors")
	
	# initally an equal probability for all floors
	for floor in likelihood:
		posterior[floor] = 1.0 / floorCount
	
	while not rospy.is_shutdown():
		# multiply by the previous probability
		for floor in posterior:
			posterior[floor] *= likelihood[floor]
		
		normalise(posterior)

		# limit the certainty
		for floor in posterior:
			posterior[floor] = max(posterior[floor], 0.01)

		# make sure to normalise again
		normalise(posterior)

		# print floor with the largest probability
		if len(posterior):
			print("likelihoods")
			print(likelihood)
			print("")
			
			print("posteriors")
			print(posterior)
			print("")
			
			guess = max(posterior, key=(lambda k: posterior[k]))
			rospy.loginfo("I'm probably on the " + guess)
			
			guess = max(likelihood, key=(lambda k: likelihood[k]))
			pub.publish(guess)
		
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
