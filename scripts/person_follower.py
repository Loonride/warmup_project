#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PersonFollower(object):
    """ This node makes Turtlebot follow a person
    """

    def __init__(self):
        rospy.init_node('person_follower')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_scan(self, data):
        range_index = 0
        range_dist = data.ranges[0]
        for i in range(len(data.ranges)):
            r = data.ranges[i]
            if r < range_dist:
                range_index = i
                range_dist = r

        if range_dist == math.inf:
            self.set_v(0, 0)
            return

        diff = range_dist - 0.5
        speed = 0
        if diff > 0:
            speed = diff * 0.5
        
        range_index_dist = range_index
        angular_speed = 0.015 * range_index
        angular_speed_sign = 1
        if range_index >= 180:
            range_index_dist = 360 - range_index
            angular_speed = 0.015 * (360 - range_index)
            angular_speed_sign = -1

        angular_speed = angular_speed_sign * angular_speed
        if range_index_dist > 60 and range_dist > 2:
            speed = 0

        self.set_v(speed, angular_speed)

    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)

if __name__ == '__main__':
    node = PersonFollower()
    rospy.spin()
