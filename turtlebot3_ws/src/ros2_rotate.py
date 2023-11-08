import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class Rotate(Node):
    def __init__(self):
        super().__init__('Robot_Rotator')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_msg = Twist()

    def command_rotation_input(self):
        print("Let's rotate your robot")
        self.speed = float(input("Input your speed (degrees/sec): "))
        self.distance = math.radians(float(input("Type your distance (degrees): ")))  # Ensure radians for the distance
        rotate_input = input("Clockwise? ").lower()
        self.clockwise = rotate_input == 'yes'  # Convert to boolean

    def rotate(self):
        angular_speed = math.radians(self.speed)
        relative_angle = math.radians(self.distance)

        vel_msg = Twist()
        vel_msg.linear.x = 1.0

        if self.clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)

        t0 = self.get_clock().now().to_msg().sec
        current_angle = 0.0

        while current_angle < relative_angle:
            self.publisher_.publish(vel_msg)
            t1 = self.get_clock().now().to_msg().sec
            elapsed_time = t1 - t0

            current_angle = abs(angular_speed) * elapsed_time

            self.get_logger().info(f"Rotating... Current angle (rad): {current_angle}")
            self.get_logger().info(f"Target angle (rad): {relative_angle}")
            self.get_logger().info("")

        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    rotate_node = Rotate()
    rotate_node.command_rotation_input()
    rotate_node.rotate()
    rclpy.spin_once(rotate_node, timeout_sec=0.1)  # Process a single iteration to allow clean shutdown
    rotate_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     
