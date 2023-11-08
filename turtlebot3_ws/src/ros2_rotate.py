import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

def rotate_robot(node):
    velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    vel_msg = Twist()


    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = float(input("Input your speed (degrees/sec):"))
    angle = float(input("Type your distance (degrees):"))
    rotate_input = input("Clockwise? yes or no ").lower()
    clockwise = rotate_input == 'yes' 

    # Conversion to radians
    angular_speed = math.radians(speed)
    relative_angle = math.radians(angle)  

    vel_msg.angular.z = angular_speed


    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    #Initialisation of the initial time and current angle
    t0 = rclpy.clock.Clock().now().nanoseconds / 1e9 
    current_angle = 0.0
       
    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rclpy.clock.Clock().now().nanoseconds / 1e9  # Get updated time
        current_angle = angular_speed * (t1 - t0)
        node.get_logger().info(f"Rotating... Current angle: {math.degrees(current_angle)}")
        node.get_logger().info(f"Target angle: {math.degrees(relative_angle)}")
        node.get_logger().info("")

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

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
