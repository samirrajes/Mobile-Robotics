import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = None
        self.state_ = 0  # 0 - forward, 1 - rotate
        self.count_ = 0

        self.forward_distance = 4.0  # meters
        self.rotation_angle = pi / 2  # radians (90 degrees)

        self.forward_speed = 0.5  # meters per second
        self.rotation_speed = 0.1  # radians per second

        self.forward_duration = self.forward_distance / self.forward_speed
        self.rotation_duration = self.rotation_angle / self.rotation_speed

        self.init_timer()

    def init_timer(self):
        self.timer_ = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        msg = Twist()
        if self.state_ == 0:  # move forward
            if self.count_ < self.forward_duration * 10:
                msg.linear.x = self.forward_speed
                self.count_ += 1
            else:
                msg.linear.x = 0.0
                self.state_ = 1  # switch to rotating
                self.count_ = 0  # reset count for rotation
        elif self.state_ == 1:  # rotate
            if self.count_ < self.rotation_duration * 10:
                msg.angular.z = self.rotation_speed
                self.count_ += 1
            else:
                msg.angular.z = 0.0
                self.state_ = 0  
                self.count_ = 0  
                if hasattr(self, "lap_count_"):
                    self.lap_count_ += 1
                else:
                    self.lap_count_ = 1
                if self.lap_count_ >= 4:  # square complete
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.timer_.cancel()  
                    self.destroy_node()  
                    rclpy.shutdown() 
                    return
            # for x in range(1):
            #     msg.angular.z = self.rotation_speed
            # self.state_ = 1  # switch to rotating
            # self.count_ = 0  # reset count for rotation


        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    square_mover = SquareMover()
    try:
        rclpy.spin(square_mover)
    except KeyboardInterrupt:
        square_mover.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        square_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()