import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# class GoForward is the ROS2 version of move from the ROS wiki
class GoForward(Node):

    def __init__(self):
        super().__init__('GoforwardCmd_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.speed = 0.0
        self.distance = 0.0
        self.isForward = True

    def command_input(self):
        print("Let's move your robot")
        self.speed = float(input("Input your speed:"))
        self.distance = float(input("Type your distance:"))
        forward_input = input("Forward? (yes or no): ").lower()
        self.isForward = forward_input in ['yes']

    def set_params(self, speed, distance, isForward):
        self.speed = speed
        self.distance = distance
        self.isForward = isForward
    
    def timer_callback(self):
        move_cmd = Twist()

        if self.isForward:
            move_cmd.linear.x = abs(self.speed)
        else:
            move_cmd.linear.x = -abs(self.speed)

        self.publisher_.publish(move_cmd)
        self.get_logger().info('Publishing cmd_vel')
        self.update_distance()

    def update_distance(self):
        current_distance = self.speed * 0.5  # Assuming constant speed
        self.distance -= current_distance

        if self.distance <= 0:
            self.stop_turtlebot()

    def stop_turtlebot(self):
        self.get_logger().info('Stopping turtlebot')
        self.get_logger().info('Robot stopped')
        self.publisher_.publish(Twist())
        self.timer.cancel()

    def run(self):
        self.command_input()  # Get user input before spinning the node
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.stop_turtlebot()
        finally:
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        cmd_publisher = GoForward()
        cmd_publisher.command_input()  # Get user input
        rclpy.spin(cmd_publisher)
    except KeyboardInterrupt:
        cmd_publisher.stop_turtlebot()
        cmd_publisher.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()