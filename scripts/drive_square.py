#!/usr/bin/env python3
""" This script causes Turtlebot to drive in a square """
import rospy
from geometry_msgs.msg import Twist

class DriveSquare(object):
    # Initialize Object
    #   - duration: seconds per turn
    #   - linear: linear speed (m/s)
    def __init__(self, duration, linear):
        rospy.init_node('drive_square')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        rospy.sleep(1)

        # Initialize Speed Properties
        self.turn_duration = duration
        self.turn_speed = 1.5708 / self.turn_duration
        self.linear_speed = linear

    # stop() - stops turtlebot
    def stop(self):
        stop = Twist()
        self.pub.publish(stop)

    # forward() - moves turtlebot forward at set linear speed
    def forward(self):
        forward = Twist()
        forward.linear.x = self.linear_speed
        forward.angular.z = 0
        self.pub.publish(forward)

    # turn() - turns turtlebot at speed determined by turn duration
    def turn(self):
        turn = Twist()
        turn.linear.x = 0
        turn.angular.z = self.turn_speed
        self.pub.publish(turn)

    # run() - drives turtlebot in a square
    def run(self):
        r = rospy.Rate(5)
        for i in range(0, 4):
            # Forward - 6 seconds
            for j in range(0, 30):
                self.forward()
                r.sleep()
            # Turning - 1 second
            for j in range(0, self.turn_duration * 5):
                self.turn()
                r.sleep()
        self.stop()


if __name__ == '__main__':
    linear = 0.20
    turn_duration = 1
    node = DriveSquare(turn_duration, linear)
    node.run()