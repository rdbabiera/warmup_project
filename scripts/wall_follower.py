#!/usr/bin/env python3
""" This script causes Turtlebot to follow the walls in a room"""

class WallFollower(object):
    def __init__(self):
        #--- Initialize sub and pub
        rospy.init_node('wall_follower')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #--- Spinup Time
        rospy.sleep(2)

    # Change State Variables
    def process_scan(self, data):

        pass


    # Run in Loop until Exit
    def run(self):
        while not rospy.is_shutdown():
        


if __name__ == '__main__':
    sheep = WallFollower()
    sheep.run()