# ros2 pkg create --build-type ament_python obstacle_avoidance

# 'console_scripts': [
#   'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
# ],

# colcon build --packages-select obstacle_avoidance
# source install/local_setup.bash 

# ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
# ros2 run obstacle_avoidance obstacle_avoidance_node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10)
        self.safe_distance = 0.3  # Safe distance from the obstacle
        # For closed-loop control
        self.kp = 0.5  # Proportional gain

    def laser_scan_callback(self, msg):
        front_distance = min(msg.ranges[0:30] + msg.ranges[330:360])  # Get the closest distance in front
        if front_distance < self.safe_distance:
            # Open-loop control: Just stop
            self.stop_robot()
        else:
            # Closed-loop control: Adjust speed based on distance
            error = front_distance - self.safe_distance
            linear_speed = self.calculate_speed(error)
            self.move_robot(linear_speed)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def move_robot(self, speed):
        twist = Twist()
        twist.linear.x = min(max(speed, 0), 0.22)  # Clamp the speed to a safe range
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def calculate_speed(self, error):
        return self.kp * error

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
