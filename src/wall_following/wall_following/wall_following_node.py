import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Controller:
    def __init__(self, P=0.8, D=0.1):
        self.Kp = P
        self.Kd = D
        self.set_point = 0.3  # Target distance from the wall
        self.previous_error = 0.0

    def update(self, current_distance):
        error = self.set_point - current_distance
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_controller = Controller()
        self.state = 'approach'  # Initial state
        self.get_logger().info("Wall Follower Node has been started.")

    def laser_callback(self, msg):
        front_distance = msg.ranges[0]    # Front
        right_distance = msg.ranges[269]  # Right

        self.get_logger().info(f"State: {self.state}, Front: {front_distance:.2f}, Right: {right_distance:.2f}")

        # Process the state and distances
        self.process_state(front_distance, right_distance)

    def process_state(self, front, right):
        msg = Twist()
        if self.state == 'approach':
            if front > 0.32:
                msg.linear.x = 0.1  # Move forward
            else:
                msg.linear.x = 0.0  # Stop
                self.state = 'rotate'  # Change state to rotate
                self.get_logger().info("Reached wall, starting to rotate.")
# <
        elif self.state == 'rotate':
            rightNotInRange = (right <= 0.28) or (right >= 0.32)
            if (rightNotInRange):
                msg.angular.z = 0.1  # Rotate slowly to find the wall on the right
            elif (front > 1.0):
                msg.angular.z = 0.0  # Stop rotation
                self.state = 'follow'  # Change state to follow wall
                self.get_logger().info("Aligned with wall, starting to follow.")

        elif self.state == 'follow':
            angular_z_correction = self.distance_controller.update(right)
            msg.linear.x = 0.01  # Move forward at a constant speed
            msg.angular.z = angular_z_correction  # Apply PD control correction
            print(angular_z_correction)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()