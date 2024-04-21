import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0.0

    def update(self, current_value):
        # calculate the error
        error = self.set_point - current_value

        # calculate P_term (Proportional term)
        P_term = self.Kp * error

        # calculate D_term (Derivative term)
        D_term = self.Kd * (error - self.previous_error)

        # update the previous error for the next iteration
        self.previous_error = error

        # the control output is the sum of all terms
        return P_term + D_term

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)

        
        self.use_closed_loop = True  # flag to switch between open and closed loop
        self.controller = Controller(P=-0.2, D=0.0)
        self.safe_distance = 0.3  # safe distance from the obstacle

    def laser_scan_callback(self, msg):
        # get closest distance in the front 60 degree arc
        front_distances = msg.ranges[:30] + msg.ranges[-30:]
        front_distance = min(front_distances)
        print(f"Front distance: {front_distance}")
        

        if self.use_closed_loop:
            # closed-loop control: PD controller
            control_effort = self.controller.update(front_distance)
            print(f"Error: {self.controller.set_point - front_distance}")
            print(f"Control effort: {control_effort}")

            if front_distance < self.safe_distance:
                self.stop_robot()
            else:
                self.move_robot(max(0.0, min(control_effort, 0.25)))

        else:
            # open-loop control: stop if obstacle is closer than safe distance
            if front_distance < self.safe_distance:
                self.stop_robot()
            else:
                self.move_robot(0.25)  # no obstacle, move at max speed

    def stop_robot(self):
        self.move_robot(0.0)

    def move_robot(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()