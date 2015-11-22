#!/usr/bin/python

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import pf_localisation.pf
from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from pf_localisation.msg import WeightedParticles, Registration

from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation
from threading import Lock

import sys
import time
import signal
from copy import deepcopy

class ParticleFilterLocalisationNode(object):

	map_topic = None

	def intHandler(self, signum, frame):
		register = Registration()
		register.frame_id = map_topic
		register.toAdd = False
		register.resolution = 0.0
		register.freePoints = []
		self._registration_publisher.publish(register)
		raise KeyboardInterrupt()

    	def __init__(self, _map_topic):
		global map_topic
		map_topic = _map_topic
		signal.signal(signal.SIGINT, self.intHandler)

		#PARTICLES
		self.numParticles = 200


        	# Minimum change (m/radians) before publishing new particle cloud and pose
        	self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
       		self._particle_filter = pf_localisation.pf.PFLocaliser(self.numParticles, map_topic)

        	self._latest_scan = None
        	self._last_published_pose = None
        	self._initial_pose_received = False

		self._registration_publisher = rospy.Publisher("/regNode", Registration)
        	self._pose_publisher = rospy.Publisher("/estimatedpose_" + map_topic, PoseStamped)
        	self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
                                                    PoseWithCovarianceStamped)
        	self._cloud_publisher = rospy.Publisher("/particlecloud_" + map_topic, PoseArray)
        	self._tf_publisher = rospy.Publisher("/tf", tfMessage)
		self._init_pose_publisher = rospy.Publisher("/initialpose",
                                                    PoseWithCovarianceStamped)

        	rospy.loginfo("Waiting for a map...")
        	try:
            		ocuccupancy_map = rospy.wait_for_message(map_topic, OccupancyGrid, 20)
        	except:
            		rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            		sys.exit(1)
        	rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        	self._particle_filter.set_map(ocuccupancy_map)
        
        	self._laser_subscriber = rospy.Subscriber("/base_scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)
        	self._initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        	self._odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                     self._odometry_callback,
                                                     queue_size=1)

		register = Registration()
		register.frame_id = map_topic
		register.toAdd = True
		register.resolution = self._particle_filter.occupancy_map.info.resolution
		register.freePoints = self._particle_filter.listFreePoints
		self._registration_publisher.publish(register)


	def initialisePose(self):
		csPose = PoseWithCovarianceStamped()
		csPose.header.seq = 1
		csPose.header.stamp = rospy.get_rostime()
		csPose.header.frame_id = map_topic #TO CHANGE
		csPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
		
		csPose.pose.pose.position.x = 0
		csPose.pose.pose.position.y = 0
		csPose.pose.pose.orientation = createQuaternion(0)
		self._init_pose_publisher.publish(csPose)

    	def _initial_pose_callback(self, pose):
        	""" called when RViz sends a user supplied initial pose estimate """
		if pose.header.frame_id != map_topic:
			return
        	self._particle_filter.set_initial_pose(pose)
        	self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        	self._initial_pose_received = True
        	self._cloud_publisher.publish(self._particle_filter.particlecloud)

    	def _odometry_callback(self, odometry):
        	"""
        	Odometry received. If the filter is initialised then execute
        	a filter predict step with odeometry followed by an update step using
        	the latest laser.
        	"""
        	if self._initial_pose_received:
        	    t_odom = self._particle_filter.predict_from_odometry(odometry)
        	    t_filter = self._particle_filter.update_filter(self._latest_scan, map_topic, self.numParticles)
        	    #if t_odom + t_filter > 0.1:
        	     #   rospy.logwarn("Filter cycle overran timeslot")
        	      #  rospy.loginfo("Odometry update: %fs"%t_odom)
        	       # rospy.loginfo("Particle update: %fs"%t_filter)

	def _laser_callback(self, scan):
        	"""
        	Laser received. Store a ref to the latest scan. If robot has moved
        	much, republish the latest pose to update RViz
        	"""
        	self._latest_scan = scan
        	if self._initial_pose_received:
        	    if  self._sufficientMovementDetected(self._particle_filter.estimatedpose):
   	    	        # Publish the new pose
        	        self._amcl_pose_publisher.publish(self._particle_filter.estimatedpose)
        	        estimatedpose =  PoseStamped()
        	        estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
   			estimatedpose.header.frame_id = map_topic
			self._pose_publisher.publish(estimatedpose)

                	# Update record of previously-published pose
                	self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        
                	# Get updated particle cloud and publish it
			self._particle_filter.particlecloud.header.frame_id = map_topic
                	self._cloud_publisher.publish(self._particle_filter.particlecloud)
        
                	# Get updated transform and publish it
                	self._tf_publisher.publish(self._particle_filter.tf_message)

    	def _sufficientMovementDetected(self, latest_pose):
        	"""
        	Compares the last published pose to the current pose. Returns true
        	if movement is more the self._PUBLISH_DELTA
        	"""
        	# Check that minimum required amount of movement has occurred before re-publishing
        	latest_x = latest_pose.pose.pose.position.x
        	latest_y = latest_pose.pose.pose.position.y
        	prev_x = self._last_published_pose.pose.pose.position.x
        	prev_y = self._last_published_pose.pose.pose.position.y
        	location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        	# Also check for difference in orientation: Take a zero-quaternion,
        	# rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        	latest_rot = latest_pose.pose.pose.orientation
        	prev_rot = self._last_published_pose.pose.pose.orientation
	
        	q = rotateQuaternion(Quaternion(w=1.0),
      	                     getHeading(latest_rot))   # Rotate forward
      		q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        	heading_delta = abs(getHeading(q))
		return (location_delta > self._PUBLISH_DELTA or heading_delta > self._PUBLISH_DELTA)

# --- Main Program  ---
if len(sys.argv) <= 1 or len(sys.argv) >= 3:
	print("\tUSAGE: node.py <mapTopic>")
	sys.exit(1)
tmap = str(sys.argv[1])

rospy.init_node("pf_localisation_" + tmap)
node = ParticleFilterLocalisationNode(tmap)
time.sleep(2)
node.initialisePose()
rospy.spin()
