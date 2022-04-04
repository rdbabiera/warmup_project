#!/usr/bin/env python3
""" This script causes Turtlebot to follow a person"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(object):
    def __init__(self):
        #--- Initialize sub and pub, let robot sleep to warm up
        rospy.init_node('person_follower')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, queue_size=10)
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(2)

    def check_surroundings(self):
        pass

    def check_changes(self):
        pass

    # Run in Loop until Exit
    def run(self):

        rospy.spin()


if __name__ == '__main__':
    biteszadusto = PersonFollower()
    biteszadusto.run()