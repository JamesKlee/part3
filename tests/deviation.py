#!/usr/bin/python

from __future__ import print_function

import rospy
import math
import sys
from time import sleep
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from pf_localisation.msg import Cluster
from pf_localisation.dbscan import DBScan
from copy import deepcopy

# The .yaml and .world files both operate in slightly different
# coordinate spaces, so it is necessary to convert between them

# Measurements derived by comparing landmarks on both maps
yamlToWorldScale = 1.161382441489639
yamlToWorldOffset = Point()
yamlToWorldOffset.x = -17.3009767893627
yamlToWorldOffset.y = -16.653441463569337

worldToYamlScale = 1.0 / yamlToWorldScale
worldToYamlOffset = Point()
worldToYamlOffset.x = -yamlToWorldOffset.x / worldToYamlScale
worldToYamlOffset.y = -yamlToWorldOffset.y / worldToYamlScale

# Convert from yaml to world coordinate space
def yamlToWorld(yPoint):
	wPoint = Point()
	wPoint.x = yamlToWorldOffset.x + yPoint.x * yamlToWorldScale
	wPoint.y = yamlToWorldOffset.y + yPoint.y * yamlToWorldScale
	return wPoint

# Convert from world to yaml coordinate space
def worldToYaml(wPoint):
	yPoint = Point()
	yPoint.x = worldToYamlOffsetX + wPoint.x * worldToYamlScale
	yPoint.y = worldToYamlOffsetY + wPoint.y * worldToYamlScale
	return yPoint

groundTruth = Point()
particleCloud = PoseArray()

def groundTruthCallback(p):
	global groundTruth
	groundTruth = p.pose.pose.position

def particleCloudCallback(p):
	global particleCloud
	particleCloud = p

# Calculate the Euclidean distance between 2 points
def getDistance(a, b):
	dx = a.x - b.x
	dy = a.y - b.y
	
	dist = math.sqrt(dx**2 + dy**2)
	return dist

def runDBScan(particles, eps, minPoints):
	db = DBScan(particles, eps, minPoints)
	db.run()
	
	guess = db.getguess()
	return guess

# Get the average position of a particle cloud
def averagePoints(particles):
	particles = particles.poses
	
	if len(particles) == 0:
		return Point()
	
	avg = Point()
	avg.x = 0.0
	avg.y = 0.0
	
	for p in particles:
		p = p.position
		
		avg.x += p.x
		avg.y += p.y
	
	avg.x /= len(particles)
	avg.y /= len(particles)
	
	return avg

#def groundTruthListener(odom):
#	global groundTruth
#	groundTruth = odom.pose.pose.position
#	
#	world = yamlToWorld(yamlPos.x, yamlPos.y)
#	return world


#dd
history = []

# Run a combination of DBSCAN algorithms on the data, offline
def applyAlgorithms():
	print("\n\n---")
	
	headers = ["time", "avg"]
	
	for minPoints in range(3, 32):
			for i in range(1, 25):
				eps = 0.1 * i
				headers.append("eps=" + str(eps) + " minPoints=" + str(minPoints))
	
	print(headers)
	
	curr = 0
	
	for t in history:
		time  = t[0]
		cloud = t[1]
		truth = t[2]
		
		readings = [str(time)]
		
		# Run average on the data
		avg = averagePoints(cloud)
		dist = getDistance(truth, yamlToWorld(avg))
		readings.append(dist)
		
		# Run various DBSCAN configurations on the data
		for minPoints in range(3, 32):
			for i in range(1, 25):
				eps = 0.1 * i
				
				estimate = runDBScan(cloud, eps, minPoints).position
				#print("eps=" + str(eps) + " minPts=" + str(minPoints))
				dist = getDistance(truth, yamlToWorld(estimate))
				readings.append(dist)
		
		print(readings)
		
		# State progress
		curr += 1
		print(curr, "/", len(history), file=sys.stderr)

class EstimateTest():
	def __init__(self, map_topic):
		self.map_topic = map_topic
		
	def run(self):
		rospy.init_node("deviation_test")
		# Get the particle cloud (which is in .yaml space)
		rospy.Subscriber("particlecloud_" + self.map_topic, PoseArray, particleCloudCallback)
		
		# Get the actual position of the robot (given in .world space)
		rospy.Subscriber("base_pose_ground_truth", Odometry, groundTruthCallback)
		
		rate = rospy.Rate(1) # 1 Hz
		
		# Run all of the algorithms once the data has been collected.
		rospy.on_shutdown(applyAlgorithms)
		
		while not rospy.is_shutdown():
			# Cache data so it isn't modified between algorithms
			cloud = deepcopy(particleCloud)
			truth = groundTruth
			
			time = rospy.get_time()
			
			# Keep a log of current data, so it can be revisited
			history.append((time, cloud, truth))
			
			print("time = " + str(time))
			
			avg = averagePoints(cloud)
			print("avg", getDistance(truth, yamlToWorld(avg)))
			
			
			print("\n")
			
			rate.sleep()
		print("YYYYYYYYYY")
		
def main(argv):
	if len(argv) != 2:
		print("usage: " + argv[0] + " <map topic>")
		sys.exit(1)
	
	map_topic = argv[1]
	
	test = EstimateTest(map_topic)
	test.run()
	
if __name__ == "__main__":
	main(sys.argv)
