# ros2 pkg create --build-type ament_python laser_scanner --dependencies rclpy sensor_msgs

# entry_points={
#     'console_scripts': [
#         'laser_scan_node = laser_scanner.laser_scan_node:main',
#     ],
# },

# colcon build --packages-select laser_scanner
# install local/setup.bash

# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ros2 run turtlebot3_gazebo turtlebot3_drive
# ros2 run laser_scanner laser_scan_node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanNode(Node):
    def __init__(self):
        super().__init__('laser_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def laser_scan_callback(self, msg):
        # Ensure that the indices do not exceed the length of the ranges array
        num_ranges = len(msg.ranges)
        front_index = 0
        left_index = min(89, num_ranges - 1)
        back_index = min(179, num_ranges - 1)
        right_index = min(269, num_ranges - 1)

        front = msg.ranges[front_index]
        left = msg.ranges[left_index]
        back = msg.ranges[back_index]
        right = msg.ranges[right_index]
        
        self.get_logger().info(f'Front: {front}, Left: {left}, Back: {back}, Right: {right}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_node = LaserScanNode()
    rclpy.spin(laser_scan_node)
    laser_scan_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()



