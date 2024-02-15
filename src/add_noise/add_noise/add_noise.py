import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AddMotionNoise(Node):
    def __init__(self):
        super().__init__('add_motion_noise')

        self.declare_parameter('linear_noise', value=0.001)
        self.declare_parameter('angular_noise', value=0.001)
        self.declare_parameter('topic_name', value='cmd_vel')

        self.linear_noise = self.get_parameter('linear_noise').get_parameter_value().double_value
        self.angular_noise = self.get_parameter('angular_noise').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        self.get_logger().info(f'linear_noise: {self.linear_noise}')
        self.get_logger().info(f'angular_noise: {self.angular_noise}')
        self.get_logger().info(f'topic_name: {self.topic_name}')
        
        self.publisher_ = self.create_publisher(Twist, self.topic_name, 10)
        self.subscription_ = self.create_subscription(Twist, self.topic_name, self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        # Adding noise to the velocities
        linear_noise = np.random.normal(0, self.linear_noise)
        angular_noise = np.random.normal(0, self.angular_noise)

        msg.linear.x += linear_noise
        msg.angular.z += angular_noise
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published modified cmd_vel with noise: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    add_motion_noise = AddMotionNoise()
    rclpy.spin(add_motion_noise)
    add_motion_noise.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()