#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


class Stopper:
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
		self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.callback, queue_size=10)

	# A callback to deal with the LaserScan messages.
	def callback(self, scan):
		# Every time we get a laser scan, calculate the shortest scan distance, and set
		# the speed accordingly.
		shortest = min(scan.ranges)

		# Create a twist and fill in all the fields.
		t = Twist()
		t.linear.x = 0.0
		t.linear.y = 0.0
		t.linear.z = 0.0
		t.angular.x = 0.0
		t.angular.y = 0.0
		t.angular.z = 0.0

		# Too close?
		if shortest < self.distance:
			t.linear.x = 0.0
		else:
			t.linear.x = 0.2

		# Send the command to the robot.
		self.publisher.publish(t)

		# Print out a log message to the INFO channel to let us know it's working.
		rospy.loginfo(f'Published {t.linear.x}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper2', argv=sys.argv)

	stopper = Stopper()

	# Now that everything is wired up, we just spin.
	rospy.spin()