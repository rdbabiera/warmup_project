#!/usr/bin/env python3
""" This script causes Turtlebot to follow a person"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(object):
    def __init__(self):
        #--- Initialize sub and pub
        rospy.init_node('person_follower')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #--- Initialize State Variables, Saved Variables from Scanner
        self.closest_index = 0
        self.closest_range = 4
        self.range_max = 3.5

        # Declared Values
        self.stop_distance = 0.4
        self.linear_vel = 0.15

        #--- Spinup Time
        rospy.sleep(2)

    # Change State Variables
    def process_scan(self, data):
        ranges = data.ranges

        closest_index = 0
        closest_range = 4
        for i, r in enumerate(ranges):
            if r < closest_range and r > 0:
                closest_range = r
                closest_index = i
        
        self.closest_range = closest_range
        self.closest_index = closest_index

    def get_directions(self):
        command = Twist()
        # Do Nothing
        if self.closest_range == 4:
            return command

        # Stopping Range
        if self.closest_range >= 0.025 and self.closest_range <= self.stop_distance:
            if self.closest_index >= 180 and self.closest_index < 330:
                command.angular.z = -0.785
            elif self.closest_index >= 29 and self.closest_index < 180:
                command.angular.z = 0.785
        # Moving Range
        elif self.closest_range > self.stop_distance and self.closest_range < 4:
            command.linear.x = self.linear_vel
            # Handle View
            if self.closest_index >= 180 and self.closest_index < 345:
                command.angular.z = (self.closest_index - 360) / 180
                command.linear.x = ((self.closest_index - 180) / 180) * self.linear_vel
            elif self.closest_index >= 14 and self.closest_index < 180:
                command.angular.z = self.closest_index / 180
                command.linear.x = ((-self.closest_index + 179) / 180) * self.linear_vel
            command.angular.z *= 1.5

        return command

    # Run in Loop until Exit
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Get Next Direction, Transmit for a short period
            command = self.get_directions()
            for i in range(0, 2):
                self.pub_move.publish(command)
                r.sleep()

if __name__ == '__main__':
    biteszadusto = PersonFollower()
    biteszadusto.run()