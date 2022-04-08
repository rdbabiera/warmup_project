#!/usr/bin/env python3
""" This script causes Turtlebot to follow the walls in a room"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower(object):
    def __init__(self):
        #--- Initialize sub and pub
        rospy.init_node('wall_follower')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #--- Static Variables
        self.closest_index = 0
        self.closest_range = 0
        self.range_max = 3.5

        # Declared Values
        self.safe_dist = 0.40
        self.buffer = 0.05
        self.linear_vel = 0.125

        #--- Spinup Time
        rospy.sleep(2)
    
    def update_directions(self, angles):
        closest_range = 4
        closest_index = 0
        increment = 2

        for i in range(0, 360):
            curr_range = 0
            count = 0
            for j in range(-increment, increment+1):
                ind = (i + j) % 360
                r = angles[ind]
                if r > 0:
                    curr_range += r
                    count += 1
            if count > 0:
                curr_range /= count
                if curr_range < closest_range:
                    closest_range = curr_range
                    closest_index = i

        self.closest_index = closest_index
        self.closest_range = closest_range       

    def update_control(self):
        command = Twist()
        if self.closest_range > self.range_max:
            command.linear.x = 0.125
            self.pub_move.publish(command)
            return
        angle = self.closest_index
        error = angle - 89
        
        command.linear.x = 0.125
        # Case 1: Cruise Control
        if ((self.closest_range <= self.safe_dist - self.buffer) or \
            (self.closest_range >= self.safe_dist + self.buffer)) and \
            (abs(error) < 4) :
            command.angular.z = (self.closest_range - self.safe_dist) * 0.03
        else:
            command.angular.z = error * 0.03
        self.pub_move.publish(command)


    def process_scan(self, data):
        angles = data.ranges
        self.update_directions(angles)
        self.update_control()

    # Run in Loop until Exit
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    sheep = WallFollower()
    sheep.run()