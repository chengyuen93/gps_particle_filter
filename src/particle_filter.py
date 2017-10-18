#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
import gpsmath
import math
import random
import time
import numpy as np
from copy import deepcopy

current_time = 0
last_time = 0
interval = 0

class Particle(object):
	""" Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

	def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):

		self.w = w
		self.theta = theta
		self.x = x
		self.y = y

class ParticleFilter:
	def __init__(self):
		self.initialized = False
		rospy.init_node('pf')
		self.n_particles = 1000
		self.sigma = 2.0 			#guess for how inaccurate the gps data can be in meters

		self.odom_pose_listener = rospy.Subscriber('pose_bef_pf', Vector3, self.odom_callback)
		self.gps_listener	=	rospy.Subscriber('fix', NavSatFix, self.gps_callback)
		self.pose_pub	=	rospy.Publisher('pose_aft_pf', Vector3, queue_size = 10)

		self.particle_cloud = []
		self.new_odom_xy_theta = []
		self.current_odom_xy_theta = []
		self.lonlat = []

		self.initialized = True
		self.pose_set = False
		self.odom_data = False

	def odom_callback(self, data):
		self.new_odom_xy_theta = [data.x, data.y, data.z] 	#lon, lat, bearing
		self.odom_data = True

	def gps_callback(self, data):
		global current_time, last_time, interval
		
		if data.status.status < 0:
			return
		elif data.status.status >= 0:
			self.lonlat = [data.longitude, data.latitude]
		if not (self.initialized):
			return
		if not self.pose_set:
			if self.odom_data:
				self.initialize_particle_cloud(self.new_odom_xy_theta)
				self.pose_set = True
				last_time = rospy.get_time()
				current_time = rospy.get_time()
				return
			else:
				return
		
		if not self.particle_cloud:
			self.initialize_particle_cloud(self.new_odom_xy_theta)
			self.current_odom_xy_theta = self.new_odom_xy_theta
		elif interval > 0:
			self.update_particles_with_odom()
			self.update_particles_with_gps()
				
			current_time = rospy.get_time()
			if current_time - last_time >= interval:
				# self.update_particles_with_odom()
				# self.update_particles_with_gps()
				self.update_robot_pose()
				self.resample_particles()
				last_time = rospy.get_time()
		elif interval <= 0:
			self.update_particles_with_odom()
			self.update_particles_with_gps()
			self.update_robot_pose()
			self.resample_particles()



	def update_robot_pose(self):
		""" Update the estimate of the robot's pose given the updated particles.
		    Computed by taking the weighted average of poses.
		"""
		# first make sure that the particle weights are normalized
		self.normalize_particles()
		x = 0
		y = 0
		theta = 0
		v = 0
		for particle in self.particle_cloud:
			x += particle.x * particle.w
			y += particle.y * particle.w
			v += math.radians(particle.theta) * particle.w 
		theta = math.degrees(v)
		if theta < 0.0:
			theta = theta + 360.0
		elif theta >= 360:
			theta = theta - 360.0
		pose = Vector3()
		pose.x = x
		pose.y = y
		pose.z = theta
		self.pose_pub.publish(pose)

	def normalize_particles(self):
		""" Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
		tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1
		for particle in self.particle_cloud:
			particle.w = particle.w/tot_weight


	def update_particles_with_odom(self):
	    	""" Update the particles using the newly given odometry pose.
	        	The function computes the value delta which is a tuple (x,y,theta)
	        	that indicates the change in position and angle between the odometry
	        	when the particles were last updated and the current odometry.
	        	msg: this is not really needed to implement this, but is here just in case.
	        """
	        # compute the change in x,y,theta since our last update
	        if self.current_odom_xy_theta:
	        	old_odom_xy_theta = self.current_odom_xy_theta
	        	dist = gpsmath.haversine(self.current_odom_xy_theta[0], self.current_odom_xy_theta[1], self.new_odom_xy_theta[0], self.new_odom_xy_theta[1]) #mm

	        	delta_angle = self.new_odom_xy_theta[2] - self.current_odom_xy_theta[2]
	        	if delta_angle > 180.0:
	        		delta_angle = delta_angle - 360.0
	        	elif delta_angle <= -180.0:
	        		delta_angle = delta_angle + 360.0

	        	self.current_odom_xy_theta = self.new_odom_xy_theta
	        else:
	        	self.current_odom_xy_theta = self.new_odom_xy_theta
	        	return

	        for particle in self.particle_cloud:
	        	particle.theta = particle.theta + delta_angle
	        	if particle.theta >= 360.0:
	        		particle.theta = particle.theta - 360.0
	        	elif particle.theta < 0.0:
	        		particle.theta = particle.theta + 360.0
	        	x_new, y_new = gpsmath.get_gps(particle.x, particle.y, dist, particle.theta) #mm
	        	particle.x = x_new
	        	particle.y = y_new
    	

	def resample_particles(self):
		self.normalize_particles()

		newParticles = []
		for i in range(len(self.particle_cloud)):
	    	    # resample the same # of particles
	    	    choice = np.random.random_sample()
	    	    # all the particle weights sum to 1
	    	    csum = 0 # cumulative sum
	    	    for particle in self.particle_cloud:
	    	        csum += particle.w
	    	        if csum >= choice:
	    	            # if the random choice fell within the particle's weight
	    	            newParticles.append(deepcopy(particle))
	    	            break
		self.particle_cloud = newParticles

	def update_particles_with_gps(self):
		""" Updates the particle weights in response to the gps location """
        	for particle in self.particle_cloud:
        	    d = gpsmath.haversine(particle.x, particle.y, self.lonlat[0], self.lonlat[1]) #mm
        	    d = d/1000.0 #m
        	    # calculate nearest distance to particle's position 
        	    prob = math.exp((-d**2)/(2*self.sigma**2)) #m
        	    particle.w = prob
			# assign particles weight

	def initialize_particle_cloud(self, xy_theta):
        	""" Initialize the particle cloud.
        	    Arguments
        	    xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """
        
        	rad = 2 # meters

        	self.particle_cloud = []
        	self.particle_cloud.append(Particle(xy_theta[0], xy_theta[1], xy_theta[2]))
        	for i in range(self.n_particles-1):
        	    # initial facing of the particle
        	    # theta = xy_theta[2] + random.random() * 5.0
        	    theta = random.random() * 360

        	    # compute params to generate x,y in a circle
        	    other_theta = random.random() * 360
        	    radius = random.random() * rad * 1000 #mm
        	    # x => straight ahead
        	    x, y = gpsmath.get_gps(xy_theta[0], xy_theta[1], radius, other_theta) #mm
        	    particle = Particle(x,y,theta)
        	    self.particle_cloud.append(particle)

        	self.normalize_particles()
		self.update_robot_pose()

if __name__ == "__main__":
	n = ParticleFilter()

	try:
		while not rospy.is_shutdown():
			pass
	except rospy.ROSInterruptException:
		pass
