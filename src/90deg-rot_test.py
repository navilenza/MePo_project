import rclpy
from geometry_msgs.msg import Twist
import math

def rotate_robot(node):
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()

    # Assuming the robot moves in the z-axis (angular z for rotation)
    angular_speed = 0.5  # Adjust the speed as needed
    relative_angle = math.radians(90)  # 90 degrees

    twist.angular.z = angular_speed

    t0 = rclpy.clock.Clock().now().nanoseconds / 1e9  # Get current time
    current_angle = 0

    while current_angle < relative_angle:
        publisher.publish(twist)
        t1 = rclpy.clock.Clock().now().nanoseconds / 1e9  # Get updated time
        current_angle = angular_speed * (t1 - t0)
        node.get_logger().info(f"Rotating... Current angle: {math.degrees(current_angle)}")
        node.get_logger().info(f"Target angle: {math.degrees(relative_angle)}")
        node.get_logger().info("")

    twist.angular.z = 0.0
    publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rotate_robot_node')

    try:
        rotate_robot(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
