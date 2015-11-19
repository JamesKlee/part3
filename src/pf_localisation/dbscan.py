#!/usr/bin/python

import math
from geometry_msgs.msg import Pose, PoseArray
from util import rotateQuaternion, getHeading

# Calculates the standard deviation of a list of angles (given in radians)
def circularStddev(angles):
    s = 0
    c = 0

    for a in angles:
        s += math.cos(a)
        c += math.sin(a)
    
    s /= len(angles)
    c /= len(angles)

    stddev = math.sqrt(-math.log(s**2 + c**2))
    return stddev

class DBScan():
    NOT_VISITED, VISITED, NOISE = range(0, 3)

    def __init__(self, particlecloud, eps, minposes):
        self.points = particlecloud.poses
        self.eps = eps
        self.minposes = minposes
        self.clusterlist = []
        self.visited = []
        self.largestClusterSize = 0

    # arguments :
    #   eps : float => Max distance with the neighborhood of a Pose
    #   minposes : float => Minimal number of Poses in the neighborhood of a Pose
    def run(self):
        self.visited = [self.NOT_VISITED] * len(self.points)

        for i in range(0, len(self.points)):
            if self.visited[i] != self.NOT_VISITED:
                continue

            self.visited[i] = self.VISITED
            point = self.points[i]
            neighbours = self.regionQuery(point)

            if len(neighbours) < self.minposes:
                self.visited[i] = self.NOISE
            else:
                currentCluster = set()
                self.extendCluster(i, neighbours, currentCluster)
                self.clusterlist.append(currentCluster)

    def extendCluster(self, point, neighbours, cluster):
        cluster.add(point)

        while True:
            changed = False

            for i in neighbours:
                if self.visited[i] == self.NOT_VISITED:
                    self.visited[i] = self.VISITED

                    neighboursP = self.regionQuery(self.points[i])
            
                    if len(neighboursP) >= self.minposes:
                       neighbours = neighbours | neighboursP
                       changed = True
                       break
                
                isInAnyCluster = False

                for cl in self.clusterlist:
                    if i in cl:
                        isInAnyCluster = True
                        break
                
                if not isInAnyCluster:
                    cluster.add(i)
                    
            if not changed:
                break;
        
    
    # Get the indexes of all neighbouring points
    def regionQuery(self, referencePose):
        neighbours = set()

        for i in range(0, len(self.points)):
            dx = self.points[i].position.x - referencePose.position.x
            dy = self.points[i].position.y - referencePose.position.y

            dist = math.sqrt(math.pow(dx,2) + math.pow(dy,2))

            if dist < self.eps:
                neighbours.add(i)

        return neighbours

    def returnProbPositionRobot(self):
        # We take the cluster with the high density
        highDensityCluster = []

        for cl in self.listClusters:
            if len(cl) > len(highDensityCluster):
                highDensityCluster = cl

        sumX = 0
        sumY = 0
        sumOrientation = 0

        for i in range (0, len(highDensityCluster)):
            sumX += highDensityCluster[i].position.x
            sumY += highDensityCluster[i].position.y
            sumOrientation += getHeading(highDensityCluster[i].orientation)

        pose = Pose()
        pose.position.x = sumX/len(particles.poses)
        pose.position.y = sumY/len(particles.poses)
        pose.orientation = rotateQuaternion(pose.orientation, sumOrientation/len(highDensityCluster))

        return pose

    def getStddev(self, cluster):
        radianList = []

        for i in cluster:
            radianList.append(getHeading(self.points[i].orientation))
            
        return circularStddev(radianList)

    def getguess(self, pf):
        clusterInfo = []
        
        for c in self.clusterlist:
            avg = Pose()
            avgOrientation = 0
            
            for i in c:
                avg.position.x += self.points[i].position.x
                avg.position.y += self.points[i].position.y
                
                avgOrientation += getHeading(self.points[i].orientation)
            
            avg.position.x /= len(c)
            avg.position.y /= len(c)
            
            orientation = pf.estimatedpose.pose.pose.orientation
            avg.orientation = rotateQuaternion(orientation, avgOrientation/len(c) - getHeading(orientation))
            
            length = len(c)
            stddev = self.getStddev(c)
            
            clusterInfo.append((avg, length, stddev))
        
        # Select the largest cluster
        largestSize = 0
        
        for ci in clusterInfo:
            largestSize = max(largestSize, ci[1])
        
        # Keep only the largest clusters
        clusterInfo = filter(lambda x: x[1] == largestSize, clusterInfo)
        
        # Just return a point if there are no clusters
        if len(clusterInfo) == 0:
            self.largestClusterSize = 1
            
            if len(self.points) > 0:
	            return self.points[0]
	        else:
	            return Pose()
	            
        self.largestClusterSize = largestSize
        
        # A cluster is the largest
        if len(clusterInfo) == 1:
            return clusterInfo[0][0]
        
        # Find the cluster with the smallest standard deviation
        deviations = []
        minDeviation = float("inf")
        
        for ci in clusterInfo:
            minDeviation = min(minDeviation, ci[2])
        
        # Get the cluster
        clusterInfo = filter(lambda x: x[2] == minDeviation, clusterInfo)
        return clusterInfo[0][0]
