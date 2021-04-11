#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class WallFollower(object):
    """ This node makes Turtlebot follow a wall
    """

    def __init__(self):
        rospy.init_node('wall_follower')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

        self.angular_speed_sign = None

    def process_scan(self, data):
        range_index = 0
        range_dist = data.ranges[0]
        for i in range(len(data.ranges)):
            r = data.ranges[i]
            if r < range_dist:
                range_index = i
                range_dist = r

        if range_dist == math.inf or range_dist > 1:
            self.set_v(0.4, 0)
            return
        
        range_index_dist = range_index
        angular_speed_sign = -1
        if range_index >= 180:
            angular_speed_sign = 1
            range_index_dist = 360 - range_index

        if self.angular_speed_sign is None:
            self.angular_speed_sign = angular_speed_sign
        speed = range_dist * 0.8
        turn_rate = abs(90 - range_index_dist)

        final_angular_speed_sign = self.angular_speed_sign
        if range_index_dist - 90 > 0 and turn_rate < 20:
            final_angular_speed_sign = -1 * self.angular_speed_sign
            turn_rate *= 0.5
        elif range_index_dist - 90 > 0:
            turn_rate = 1

        angular_speed = 0.02 * turn_rate

        if data.ranges[0] < 0.8:
            speed = 0
            angular_speed = 0.5
            final_angular_speed_sign = self.angular_speed_sign

        angular_speed = final_angular_speed_sign * angular_speed

        self.set_v(speed, angular_speed)

    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)

if __name__ == '__main__':
    node = WallFollower()
    rospy.spin()
