# ros2 pkg create --build-type ament_python wall_following

# 'console_scripts': [
#   'wall_following_node = wall_following.wall_following_node:main',
# ],

# colcon build --packages-select wall_following
# source install local_setup.bash 

# ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
# ros2 run wall_following wall_following_node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')
        # Publisher to control the robot's linear and angular velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscription to the laser scan to get distance measurements around the robot
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10)
        # Initial state set to approach the wall
        self.state = "approach_wall"
        # Safe distance to maintain from the wall
        self.safe_distance = 0.3
        # Proportional gain for distance control
        self.kp_distance = 0.5
        # Proportional gain for angle control
        self.kp_angle = 0.4

    def laser_scan_callback(self, msg):
        # Handling the robot's behavior based on the current state
        if self.state == "approach_wall":
            self.approach_wall(msg)
        elif self.state == "align_with_wall":
            self.align_with_wall(msg)
        elif self.state == "follow_wall":
            self.follow_wall(msg)

    def approach_wall(self, msg):
        # Calculate the minimum distance in front to detect the wall
        front_distance = min(min(msg.ranges[0:30]), min(msg.ranges[330:360]))
        # If the front distance is less than the safe distance, stop and change state
        if front_distance < self.safe_distance:
            self.publish_velocity(0.0, 0.0)
            self.state = "align_with_wall"
        else:
            # Else, move forward to approach the wall
            self.publish_velocity(0.1, 0.0)

    def align_with_wall(self, msg):
        right_distance = msg.ranges[270]  # Adjust index based on your robot's configuration
        # Check if the robot is approximately aligned by being within a threshold
        if 0.25 < right_distance < 0.35:
            self.publish_velocity(0.0, 0.0)
            self.state = "follow_wall"
        else:
            # Calculate the error between current and desired distance for alignment
            distance_error = right_distance - self.safe_distance
            # Apply proportional control to determine the rotation speed for alignment
            rotation_speed = self.kp_angle * distance_error
            self.publish_velocity(0.0, rotation_speed)

    def follow_wall(self, msg):
        right_distance = msg.ranges[270]
        distance_error = right_distance - self.safe_distance
        # Apply proportional control to adjust the robot's orientation
        angular_speed = -self.kp_angle * distance_error
        # Dynamic adjustment of linear speed based on distance error for smoother control
        linear_speed = max(0.1 - abs(distance_error), 0.05)  # Ensure a minimum speed
        self.publish_velocity(linear_speed, angular_speed)

    def publish_velocity(self, linear, angular):
        # Utility method to publish linear and angular velocities
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    # Cleanup and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

