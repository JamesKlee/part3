"""
@author rowanms
An abstract Localiser which needs to be extended as PFLocaliser
before PFLocalisationNode will work.
@author burbrcjc
Converted to Python
"""

import rospy

from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray,
                               Quaternion,  Transform,  TransformStamped, Point )
from tf.msg import tfMessage
from tf import transformations
from nav_msgs.msg import OccupancyGrid
from pf_localisation.msg import WeightedParticles, Registration
from updateParticle import UpdateParticleCloud
from estimatePose import EstimatePose
from initialise import InitialiseCloud

import math
import random
import sys
import numpy as np
from util import rotateQuaternion, getHeading
import numpy as np
from threading import Lock
import time
import sensor_model
PI_OVER_TWO = math.pi/2 # For faster calculations

class PFLocaliserBase(object):

    INIT_X = 10 		# Initial x location of robot (metres)
    INIT_Y = 5			# Initial y location of robot (metres)
    INIT_Z = 0 			# Initial z location of robot (metres)
    INIT_HEADING = 0 	# Initial orientation of robot (radians)
    
    def __init__(self, _num):
        # Initialise fields
        self.estimatedpose =  PoseWithCovarianceStamped()
        self.occupancy_map = OccupancyGrid()
        self.particlecloud =  PoseArray()
        self.tf_message = tfMessage()
	self.weights = []
	self.maxWeight = 0
	self.totalWeight = 0
	self.num = _num

	# Initialise objects
	self.cloud = UpdateParticleCloud()
	self.estimate = EstimatePose()
	self.init = InitialiseCloud()

	self._weighted_particle_publisher = rospy.Publisher("/weightedParticles",
                                                    WeightedParticles)
	self._registration_publisher = rospy.Publisher("/regNode", Registration)
        
        self._update_lock =  Lock()
        
        # Parameters
        self.ODOM_ROTATION_NOISE = 0 		# Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 	# Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 			# Odometry y axis (side-side) noise
        self.NUMBER_PREDICTED_READINGS = 20
    
        # Set 'previous' translation to origin
        # All Transforms are given relative to 0,0,0, not in absolute coords.
        self.prev_odom_x  = 0.0 # Previous odometry translation from origin
        self.prev_odom_y = 0.0  # Previous odometry translation from origin
        self.prev_odom_heading = 0.0 # Previous heading from odometry data
        self.last_odom_pose = None
        
        # Request robot's initial odometry values to be recorded in prev_odom
        self.odom_initialised = False
        self.sensor_model_initialised = False

        # Set default initial pose to initial position and orientation.
        self.estimatedpose.pose.pose.position.x = self.INIT_X
        self.estimatedpose.pose.pose.position.y = self.INIT_Y
        self.estimatedpose.pose.pose.position.z = self.INIT_Z
        self.estimatedpose.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
                                                                    self.INIT_HEADING)
        # NOTE: Currently not making use of covariance matrix
        
        self.estimatedpose.header.frame_id = "/map"
        self.particlecloud.header.frame_id = "/map"
        
        # Sensor model
        self.sensor_model =  sensor_model.SensorModel()

	# Free Point where Robot can be
	self.listFreePoints = []

    def initialise_particle_cloud(self, initialpose):
        """
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        raise NotImplementedError()

    def update_filter(self, scan, map_topic, numParticles):
        """
        Called whenever there is a new LaserScan message.
        This calls update methods (implemented by subclass) to do actual
        particle filtering, given the map and the LaserScan, and then updates
        Transform tf appropriately.
        
        :Args:
            |  scan (sensor_msgs.msg.LaserScan) latest laser scan to resample
               the particle filter based on
        """
        if not self.sensor_model_initialised:
            self.sensor_model.set_laser_scan_parameters(self.NUMBER_PREDICTED_READINGS,
                                                        scan.range_max,
                                                        len(scan.ranges),
                                                        scan.angle_min,
                                                        scan.angle_max)
            self.sensor_model_initialised = True
        with self._update_lock:
            t = time.time()
            # Call user-implemented particle filter update method
            #self.update_particle_cloud(scan)
            
	    #SERVER CODE
            self.cloud.weight_amcl(scan, self)

	    # Get particle cloud and weights and publish it 
	    pWeights = WeightedParticles()
	    pWeights.poseArray.header.seq = 1
	    pWeights.poseArray.header.stamp = rospy.get_rostime()
	    pWeights.poseArray.header.frame_id = map_topic
	    pWeights.poseArray.poses = self.particlecloud.poses
	    pWeights.array = self.weights
	    pWeights.maxWeight = self.maxWeight
	    pWeights.totalWeight = self.totalWeight
	    rospy.loginfo("SENDING")
	    self._weighted_particle_publisher.publish(pWeights)

	    resampledParticles = []

	    loop = True
	    while loop:
		    try:
		   	pArray = rospy.wait_for_message("/updatedCloud", PoseArray, 2)
			rospy.loginfo("\tRECEIVED MESSAGE TO: " + pArray.header.frame_id)
			if pArray.header.frame_id == map_topic:
				loop = False
				resampledParticles = pArray.poses
		    except:
		    	self._weighted_particle_publisher.publish(pWeights)

	    self.particlecloud = self.cloud.smudge_amcl(resampledParticles)
            self.particlecloud.header.frame_id = map_topic
            self.estimatedpose.pose.pose = self.estimate_pose()
            currentTime = rospy.Time.now()
            
            # Given new estimated pose, now work out the new transform
            self.recalculate_transform(currentTime)
            # Insert correct timestamp in particlecloud and estimatedpose,
            # so extending subclasses don't need to worry about this, but can
            # just concentrate on updating actual particle and pose locations
            self.particlecloud.header.stamp = currentTime
            self.estimatedpose.header.stamp = currentTime

        return time.time()-t

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. I.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        raise NotImplementedError()


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        raise NotImplementedError()


    def recalculate_transform(self, currentTime):
        """
        Creates updated transform from /odom to /map given recent odometry and
        laser data.
        
        :Args:
            | currentTime (rospy.Time()): Time stamp for this update
         """
        
        transform = Transform()

        T_est = transformations.quaternion_matrix([self.estimatedpose.pose.pose.orientation.x,
                                                   self.estimatedpose.pose.pose.orientation.y,
                                                   self.estimatedpose.pose.pose.orientation.z,
                                                   self.estimatedpose.pose.pose.orientation.w])
        T_est[0, 3] = self.estimatedpose.pose.pose.position.x
        T_est[1, 3] = self.estimatedpose.pose.pose.position.y
        T_est[2, 3] = self.estimatedpose.pose.pose.position.z
        
        T_odom = transformations.quaternion_matrix([self.last_odom_pose.pose.pose.orientation.x,
                                                   self.last_odom_pose.pose.pose.orientation.y,
                                                   self.last_odom_pose.pose.pose.orientation.z,
                                                   self.last_odom_pose.pose.pose.orientation.w])
        T_odom[0, 3] = self.last_odom_pose.pose.pose.position.x
        T_odom[1, 3] = self.last_odom_pose.pose.pose.position.y
        T_odom[2, 3] = self.last_odom_pose.pose.pose.position.z
        T = np.dot(T_est, np.linalg.inv(T_odom))
        q = transformations.quaternion_from_matrix(T) #[:3, :3])

        transform.translation.x = T[0, 3] 
        transform.translation.y = T[1, 3] 
        transform.translation.z = T[2, 3] 
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        

        # Insert new Transform into a TransformStamped object and add to the
        # tf tree
        new_tfstamped = TransformStamped()
        new_tfstamped.child_frame_id = "/odom"
        new_tfstamped.header.frame_id = "/map"
        new_tfstamped.header.stamp = currentTime
        new_tfstamped.transform = transform

        # Add the transform to the list of all transforms
        self.tf_message = tfMessage(transforms=[new_tfstamped])
        

    def predict_from_odometry(self, odom):
        """
        Adds the estimated motion from odometry readings to each of the
        particles in particlecloud.
        
        :Args:
            | odom (nav_msgs.msg.Odometry): Recent Odometry data
        """
        with self._update_lock:
            
            t = time.time()
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            new_heading = getHeading(odom.pose.pose.orientation)
            
            # On our first run, the incoming translations may not be equal to 
            # zero, so set them appropriately
            if not self.odom_initialised:
                self.prev_odom_x = x
                self.prev_odom_y = y
                self.prev_odom_heading = new_heading
                self.odom_initialised = True
    
            # Find difference between current and previous translations
            dif_x = x - self.prev_odom_x
            dif_y = y - self.prev_odom_y
            dif_heading = new_heading - self.prev_odom_heading
            if dif_heading >  math.pi:
                dif_heading = (math.pi * 2) - dif_heading
            if dif_heading <  -math.pi:
                dif_heading = (math.pi * 2) + dif_heading
            
            # Update previous pure odometry location (i.e. excluding noise) 
            # with the new translation
            self.prev_odom_x = x
            self.prev_odom_y = y
            self.prev_odom_heading = new_heading
            self.last_odom_pose = odom
            
            # Find robot's linear forward/backward motion, given the dif_x and 
            # dif_y changes and its orientation
            distance_travelled = math.sqrt(dif_x*dif_x + dif_y*dif_y)
            direction_travelled = math.atan2(dif_y, dif_x)
            temp = abs(new_heading - direction_travelled)
    
            if temp < -PI_OVER_TWO or temp > PI_OVER_TWO:
                # We are going backwards
                distance_travelled = distance_travelled * -1
            
            # Update each particle with change in position (plus noise)
            for p in self.particlecloud.poses:
                
                rnd = random.normalvariate(0, 1)
                
                # Rotate particle according to odometry rotation, plus  noise
                p.orientation = (rotateQuaternion(p.orientation,
                                                  dif_heading + rnd * dif_heading * self.ODOM_ROTATION_NOISE))
                
                # Get particle's new orientation
                theta = getHeading(p.orientation)
                
                # Find translation in the direction of particle's orientation
                travel_x = distance_travelled * math.cos(theta)
                travel_y = distance_travelled * math.sin(theta)
                p.position.x = (p.position.x + travel_x +
                                (rnd * travel_x * self.ODOM_TRANSLATION_NOISE))
                p.position.y = (p.position.y + travel_y +
                                (rnd * travel_y * self.ODOM_DRIFT_NOISE))
    
        return time.time() - t
    
    def set_initial_pose(self, pose):
        """ Initialise filter with start pose """
        self.estimatedpose.pose = pose.pose
        # Estimated pose has been set, so we should now reinitialise the 
        # particle cloud around it
        rospy.loginfo("Got pose. Calling initialise_particle_cloud().")
        self.particlecloud = self.initialise_particle_cloud(self.estimatedpose)
        self.particlecloud.header.frame_id = "/map"
    
    def set_map(self, occupancy_map):
        """ Set the map for localisation """
        self.occupancy_map = occupancy_map
        self.sensor_model.set_map(occupancy_map)

	#Define Free Occupancy Points
	sizeRealMap = len(self.occupancy_map.data)
	sizeSupposedMap = self.occupancy_map.info.width * self.occupancy_map.info.height

	rospy.loginfo("Real Map = %s"%sizeRealMap)
	rospy.loginfo("Suppose Map = %s"%sizeSupposedMap)

	i = 0
	occupancyData = self.occupancy_map.data

	for h in range(0, self.occupancy_map.info.height):
		for w in range(0, self.occupancy_map.info.width):			
			if (occupancyData[i] >= 0.0) :
				point = Point()
				point.x = w
				point.y = h
				point.z = 0
				self.listFreePoints.append(point)
			i += 1
	rospy.loginfo("Len listFreePoints= %s"%len(self.listFreePoints))

        # Map has changed, so we should reinitialise the particle cloud
        rospy.loginfo("Particle filter got map. (Re)initialising.")
        self.particlecloud = self.initialise_particle_cloud(self.estimatedpose)
        self.particlecloud.header.frame_id = "/map"
