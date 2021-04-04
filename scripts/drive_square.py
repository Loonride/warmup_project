#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DriveSquare(object):
    """ This node makes Turtlebot drive in a square using odom readings
    """

    def __init__(self):
        rospy.init_node('drive_square')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.process_odom)

        self.start_angle = None

        # This is a list of tasks that the robot should perform sequentially in order to
        # drive in a square. The tasks are functions, along with the args that should
        # be passed into the functions.
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
        """ This is the callback for the odom subscription. It selects the task based on
        the current task number, then passes in odom and args. If the task function returns
        True, the task is complete and the task num is incremented.
        """
        task = self.task_loop[self.task_num]
        res = task['func'](odom, *task['args'])
        if res:
            self.start_angle = None
            self.task_num += 1
            if self.task_num >= len(self.task_loop):
                self.task_num = 0

    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)

    def should_turn_right(self, target_angle):
        """ This function determines if the robot should turn left or right to reach the
        target angle the fastest based on the start angle.
        """
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
        """ This function determines if the target angle is within the min and max angle
        range, where this range must be < 180 degrees
        """
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
        """ This helper takes the odom and target angle and determines which direction the
        robot should rotate, or else if the robot has reached the target angle so that it can
        stop spinning.
        """
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
        """ This is a task function that rotates the robot without any linear velocity.
        """
        mul = self.angle_mul(odom, target_angle)
        if mul == 0:
            return True

        self.set_v(0.0, mul * 0.4)
        return False

    def drive_until_x(self, odom, x, lt, target_angle):
        """ This tasks make the robot drive until it reaches a certain x value.
        """
        if (lt and odom.pose.pose.position.x < x) or (not lt and odom.pose.pose.position.x > x):
            # note that while the robot is driving it does a bit of angle correction
            # in order to make the square better.
            mul = self.angle_mul(odom, target_angle)
            self.set_v(0.2, mul * 0.05)
            return False
        return True

    def drive_until_y(self, odom, y, lt, target_angle):
        """ This tasks make the robot drive until it reaches a certain y value.
        """
        if (lt and odom.pose.pose.position.y < y) or (not lt and odom.pose.pose.position.y > y):
            # note that while the robot is driving it does a bit of angle correction
            # in order to make the square better.
            mul = self.angle_mul(odom, target_angle)
            self.set_v(0.2, mul * 0.05)
            return False
        return True

    def stop(self, odom):
        """ This tasks make the robot stop.
        """
        self.set_v(0.0, 0.0)
        return False

if __name__ == '__main__':
    node = DriveSquare()
    rospy.spin()
