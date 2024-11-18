#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        self.odom_subscriber = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 100  # QoS history depth
        )

        # Subscribe to the 'drive' topic
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            100  # QoS history depth
        )

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # TODO: set PID gains
        self.kp = 0.01
        self.kd = 0.002
        self.ki = 0

        # TODO: store history
        self.integral = 0
        self.prev_error = 0 
        self.error = 0

        # TODO: store any necessary values you think you'll need

        self.theta = np.pi / 3.6                  # 50 degrees
        self.L = 2   
        self.D_REF = 1.3

        self.start_t = -1.0
        self.curr_t = 0.0
        self.prev_t = 0.0

    def get_range(self, msg, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        angle_diff = abs(msg.angle_min - angle)                 
        step_angle_deg = angle_diff / msg.angle_increment       
        return msg.ranges[len(msg.ranges) - int(step_angle_deg)]

    def get_error(self, msg, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """ 

        #################   Step 1 : Calculate a and b   #################

        # assuming msg.ranges[-1] is at -135 degrees    
        # 90 degrees to the left
        b = self.get_range(msg, -np.pi / 2)
        a = self.get_range(msg, -np.pi / 2 + self.theta)

        #################   Step 2 : Calculate alpha   #################

        alpha = np.arctan((a * np.cos(self.theta) - b) / (a * np.sin(self.theta)))

        #################   Step 3 : Calculate D_t   #################

        D_t = max(b * np.cos(alpha), 0)
        D_t1 = D_t + self.L * np.sin(alpha)
        print("{:.2f} {:.2f} {:.2f}".format(D_t, D_t1, dist - D_t1))

        self.prev_error = self.error
        self.error = dist - D_t1
        self.integral += self.error

        self.prev_t = self.curr_t
        self.curr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_t < 0.0:
            self.start_t = self.curr_t

        return dist - D_t1

    def to_radians(self, theta):
        """
        Convert degrees to radians

        Args:
            theta: angle in degrees

        Returns:
            angle in radians
        """
        return np.pi * theta / 180.0

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        #################   Step 4 : Get steering angle from controllers   #################

        if self.prev_t == 0.0:
            return
        
        angle = (self.kp * error +
                 self.ki * self.integral * (self.curr_t - self.start_t) +
                 self.kd * (error - self.prev_error) / (self.curr_t - self.prev_t))
        
        print("Angle: {:.2f}".format(angle))
        print("Error: {:.2f}".format(error))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle

        if abs(angle) < self.to_radians(5):
            drive_msg.drive.speed = 2.5
        elif abs(angle) < self.to_radians(10):
            drive_msg.drive.speed = 1.5
        elif abs(angle) < self.to_radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        error = self.get_error(msg, self.D_REF) # TODO: replace with error calculated by get_error()
        self.pid_control(-error)                 # TODO: actuate the car with PID

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()