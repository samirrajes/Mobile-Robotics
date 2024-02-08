#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info("Press CTRL + C to terminate")

    def timer_callback(self):
        msg = Twist()
        # Set the velocity here: using Twist.linear.x/y/z and/or Twist.angular.x/y/z
        # TODO
        msg.linear.x = 2.0
        msg.angular.z = 1.5

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
