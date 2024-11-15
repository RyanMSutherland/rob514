#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys
import numpy as np

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float64



class Distance:
	'''
	This class encapsulates the functionality that drives the robot forward, and stops it when
	any laser rangefinder reading is less that a specificied value.
	'''
	def __init__(self):

		# Set up a publisher.  The default topic for Twist messages is cmd_vel.
		# Set up your publishers first
		self.publisher = rospy.Publisher('distance', Float64, queue_size=10)

		# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
		self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.callback, queue_size=10)		

	# A callback to deal with the LaserScan messages.
	def callback(self, scan):
		# Every time we get a laser scan, calculate the shortest scan distance, and set
		# the speed accordingly.

		theta_adjustment = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
		theta_adjustment = np.where(abs(theta_adjustment) < np.pi/6, theta_adjustment, scan.angle_max)
		distances = abs(scan.ranges / np.cos(theta_adjustment))
		shortest_distance = min(distances)
		# Send the command to the robot.
		self.publisher.publish(shortest_distance)

		# Print out a log message to the INFO channel to let us know it's working.
		rospy.loginfo(f'Shortest distance {shortest_distance}')

if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('distance', argv=sys.argv)

	distance = Distance()

	# Now that everything is wired up, we just spin.
	rospy.spin()