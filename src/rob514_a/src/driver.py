#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys
import numpy as np

print("Thanks for making this work Zane")

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

from std_msgs.msg import Float64


class Driver:
	'''
	This class encapsulates the functionality that drives the robot forward, and stops it when
	any laser rangefinder reading is less that a specificied value.
	'''
	def __init__(self, distance=1.0):
		'''
		Inputs:
		  distance: the stopping distance, in meters
		'''
		self.distance = distance

		# Set up a publisher.  The default topic for Twist messages is cmd_vel.
		# Set up your publishers first
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
		self.subscriber = rospy.Subscriber('distance', Float64, self.callback, queue_size=10)

		self.subscribe_rotation = rospy.Subscriber('rotate', Float64, self.rotate, queue_size = 10)

		#proportional control
		self.k_p = 1.2
		self.k_p_rot = 1.8
		self.max_speed = 0.5
		self.rotation = 0.0

	# A callback to deal with the LaserScan messages.
	def callback(self, shortest):
		# Every time we get a laser scan, calculate the shortest scan distance, and set
		# the speed accordingly.

		# Create a twist and fill in all the fields.
		t = Twist()
		t.linear.x = 0.0
		t.linear.y = 0.0
		t.linear.z = 0.0
		t.angular.x = 0.0
		t.angular.y = 0.0
		t.angular.z = 0.0

		t.linear.x = self.k_p * (shortest.data - self.distance)

		if abs(t.linear.x) > self.max_speed:
			t.linear.x = np.sign(t.linear.x) * self.max_speed
		elif abs(t.linear.x) < 1.0E-2:
			t.linear.x = 0.0
			t.angular.z = self.k_p_rot*self.rotation

		# Send the command to the robot.
		self.publisher.publish(t)

		# Print out a log message to the INFO channel to let us know it's working.
		rospy.loginfo(f'Published speed {t.linear.x}')
		rospy.loginfo(f'Published rotation {t.angular.z}')
		rospy.loginfo(f'Shortest Distance {shortest}')
	
	def rotate(self, rotation):
		self.rotation = rotation.data


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver()

	# Now that everything is wired up, we just spin.
	rospy.spin()