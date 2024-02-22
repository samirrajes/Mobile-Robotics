#!/usr/bin/env python3

# sudo apt install ros-foxy-tf-transformations
# sudo pip3 install transforms3d

# chmod +x ~/ros2_ws/src/pd_control/pd_control/pd_control.py

# colcon build --packages-select pd_control
# source install/local_setup.bash

# ros2 launch turtlebot3_gazebo empty_world.launch.py
# ros2 run pd_control pd_control

# setup file
# 'pd_control = pd_control.pd_control:main'

from math import pi, sqrt, atan2, cos, sin
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # Calculate the error
        error = (self.set_point - current_value + pi) % (2 * pi) - pi

        # Calculate P_term (Proportional term)
        P_term = self.Kp * error

        # Calculate D_term (Derivative term)
        D_term = self.Kd * (error - self.previous_error)

        # Update the previous error for the next iteration
        self.previous_error = error

        # The control output is the sum of all terms
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0  # Reset previous error on set point change
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot3(Node):
    def __init__(self):
        super().__init__('turtlebot3_move_square')
        self.get_logger().info("Press Ctrl + C to terminate")
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rate = self.create_rate(10)

        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.controller = Controller(P=0.7, D=0.6)  # Initialize PD controller with example gains
        self.waypoints = [[4, 0], [4, 4], [0, 4], [0, 0]]
        self.current_waypoint_index = 0

        self.get_logger().info("Turtlebot3 PD Control Node has been started.")

    def odom_callback(self, msg):
        # Extract pose from odometry message
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Log and save trajectory occasionally
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])
            self.get_logger().info(f"odom: x={self.pose.x}; y={self.pose.y}; theta={yaw}")

    def run(self):
        while rclpy.ok():
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_robot()
                break  # Stop the robot after the last waypoint

            target_x, target_y = self.waypoints[self.current_waypoint_index]
            desired_theta = atan2(target_y - self.pose.y, target_x - self.pose.x)
            # error_theta = (desired_theta - current_theta + pi) % (2 * pi) - pi
            
            # Update the PD controller's set point to the desired orientation
            current_theta = self.pose.theta
            self.controller.setPoint(desired_theta)

            # PD control to adjust robot's orientation
            control_effort = self.controller.update(current_theta)
            twist_msg = Twist()
            twist_msg.angular.z = control_effort
            
            # Move forward if the orientation is correct within a tolerance
            if abs(desired_theta - current_theta) < 0.05:
                twist_msg.linear.x = 0.3  # Move forward
                if self.distance_to_waypoint(target_x, target_y) < 0.05:
                    self.current_waypoint_index += 1
                    twist_msg.linear.x = 0.0  # Stop moving forward
            
            self.vel_pub.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0)
    
    def stop_robot(self):
        twist_msg = Twist()
        self.vel_pub.publish(twist_msg)
        self.get_logger().info("Finished trajectory.")
        # Optionally, save trajectory to a file
        np.savetxt('trajectory.csv', np.array(self.trajectory), delimiter=',')
    
    def distance_to_waypoint(self, x, y):
        return sqrt((x - self.pose.x) ** 2 + (y - self.pose.y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    turtlebot = Turtlebot3()
    turtlebot.run()
    turtlebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
