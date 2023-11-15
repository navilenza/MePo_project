import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class Rotate:
    def __init__(self):
        self.node = rclpy.create_node('rotate_robot_node')
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()

    
    def set_params(self, speed, angle, clockwise):
        self.speed = speed
        self.angle = angle
        self.clockwise = clockwise
    
    def isEqual(self, current_angle, relative_angle): 
        return abs(current_angle - relative_angle) < 0.015
    


    def rotate_robot(self):
        angular_speed = math.radians(self.speed)
        relative_angle = math.radians(self.angle)

        self.vel_msg.angular.z = angular_speed

        if self.clockwise:
            self.vel_msg.angular.z = -abs(angular_speed)
        else:
            self.vel_msg.angular.z = abs(angular_speed)

        t0 = rclpy.clock.Clock().now().nanoseconds / 1e9
        current_angle = 0.0

        while not self.isEqual(current_angle, relative_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rclpy.clock.Clock().now().nanoseconds / 1e9
            current_angle = angular_speed * (t1 - t0)
            self.node.get_logger().info(f"Rotating... Current angle: {math.degrees(current_angle)}, Target angle: {math.degrees(relative_angle)}")

        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    rotate = Rotate()
    rotate.set_params(30, 90, True)  # Set the parameters here
    rotate.rotate_robot()
    rclpy.spin(rotate.node)  # Spin the node here
    rclpy.shutdown()

if __name__ == '__main__':
    main()
