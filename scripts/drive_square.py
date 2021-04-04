#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DriveSquare(object):
    """This node drives in a square"""

    def __init__(self):
        rospy.init_node('drive_square')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.process_odom)

        self.start_angle = None

        self.task_num = 0
        self.task_loop = [
            {
                'func': self.set_angle,
                'args': [0]
            },
            {
                'func': self.drive_until_x,
                'args': [1, True, 0]
            },
            {
                'func': self.set_angle,
                'args': [math.pi * 0.5]
            },
            {
                'func': self.drive_until_y,
                'args': [1, True, math.pi * 0.5]
            },
            {
                'func': self.set_angle,
                'args': [math.pi]
            },
            {
                'func': self.drive_until_x,
                'args': [0, False, math.pi]
            },
            {
                'func': self.set_angle,
                'args': [math.pi * -0.5]
            },
            {
                'func': self.drive_until_y,
                'args': [0, False, math.pi * -0.5]
            },
        ]

    def process_odom(self, odom):
        # print(odom)
        # print(self.task_num)
        task = self.task_loop[self.task_num]
        res = task['func'](odom, *task['args'])
        if res:
            self.start_angle = None
            self.task_num += 1
            if self.task_num >= len(self.task_loop):
                self.task_num = 0

    def set_v(self, velocity, angular_velocity):
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)

    def should_turn_right(self, target_angle):
        right = True
        if self.start_angle > 0:
            if target_angle > self.start_angle:
                right = False
            elif target_angle < 0 and abs(target_angle) > abs(self.start_angle - math.pi):
                right = False
        else:
            if target_angle < 0 and target_angle > self.start_angle:
                right = False
            elif target_angle >= 0 and target_angle < self.start_angle + math.pi:
                right = False
        return right

    def within_small_range(self, target_angle, min_angle, max_angle):
        if (min_angle < 0 and max_angle < 0) or (min_angle > 0 and max_angle > 0):
            if target_angle > min_angle and target_angle < max_angle:
                return True
        elif max_angle > min_angle + math.pi:
            if target_angle < min_angle or target_angle > max_angle:
                return True
        else:
            if target_angle > min_angle and target_angle < max_angle:
                return True
        return False

    def angle_mul(self, odom, target_angle):
        o = odom.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([o.x, o.y, o.z, o.w])
        if self.start_angle is None:
            self.start_angle = yaw
        
        right = self.should_turn_right(target_angle)
        right_mul = -1 if right else 1
        if not self.within_small_range(target_angle, min(self.start_angle, yaw), max(self.start_angle, yaw)):
            return right_mul
        return 0

    def set_angle(self, odom, target_angle):
        mul = self.angle_mul(odom, target_angle)
        if mul == 0:
            return True

        self.set_v(0.0, mul * 0.3)
        return False

    def drive_until_x(self, odom, x, lt, target_angle):
        if (lt and odom.pose.pose.position.x < x) or (not lt and odom.pose.pose.position.x > x):
            mul = self.angle_mul(odom, target_angle)
            self.set_v(0.2, mul * 0.05)
            return False
        return True

    def drive_until_y(self, odom, y, lt, target_angle):
        if (lt and odom.pose.pose.position.y < y) or (not lt and odom.pose.pose.position.y > y):
            mul = self.angle_mul(odom, target_angle)
            self.set_v(0.2, mul * 0.05)
            return False
        return True

    def stop(self, odom):
        self.set_v(0.0, 0.0)
        return False

if __name__ == '__main__':
    node = DriveSquare()
    rospy.spin()
