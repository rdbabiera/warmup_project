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
        self.closest_index = None
        self.closest_range = None

        # Static Values
        self.range_max = 4
        self.increment = None
        self.angle_min = None
        self.angle_max = None

        #--- Spinup Time
        rospy.sleep(2)

    # Change State Variables
    def process_scan(self, data):
        ranges = data.ranges
        if self.closest_index == None:
            self.increment = data.angle_increment
            self.angle_min = data.angle_min
            self.angle_max = data.angle_max
            self.range_max = min(self.range_max, data.range_max)

            self.closest_index = 0
            self.closest_range = 10

        self.closest_range = 4
        for i, r in enumerate(ranges):
            if r < self.closest_range and r > 0:
                self.closest_range = r
                self.closest_index = i
        # print(f"Closest Angle: {self.closest_index}, Closest Range: {self.closest_range}")

    def get_directions(self):
        command = Twist()
        if self.closest_range > 0.60 and self.closest_range < 10:
            command.linear.x = 0.15
            # Handle View
            if (self.closest_index <= 360 and self.closest_index > 345) or \
               (self.closest_index < 15 and self.closest_index >= 0):
                command.angular.z = 0
            elif (self.closest_index >= 180):
                command.angular.z = -(self.closest_index - 180) / 180
            else:
                command.angular.z = self.closest_index / 180
                #command.angular.z = min((self.closest_index / 360) * 6.28, 0.20)
        return command

    # Run in Loop until Exit
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Do Nothing while no Data is Provided
            if not self.closest_index and not self.closest_range:
                continue
            # Get Next Direction, Transmit for a short period
            command = self.get_directions()
            for i in range(0, 3):
                self.pub_move.publish(command)
                r.sleep()

if __name__ == '__main__':
    biteszadusto = PersonFollower()
    biteszadusto.run()