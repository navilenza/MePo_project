import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Rotate:
    def __init__(self):
        self.node = rclpy.create_node('rotate_robot_node')
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()

    def run(self):
        self.rotate_robot()

    def set_params(self, speed, angle, clockwise):
        self.speed = speed
        self.angle = angle
        self.clockwise = clockwise
    
    def isEqual(self, current_angle, relative_angle): 
        return abs(current_angle - relative_angle) < 0.01
    
    def rotate_robot(self):
        angular_speed = math.radians(self.speed)
        relative_angle = math.radians(self.angle)

        self.vel_msg.angular.z = angular_speed

        if self.clockwise:
            self.vel_msg.angular.z = -abs(angular_speed)
        else:
            self.vel_msg.angular.z = abs(angular_speed)

        # Initial time and angle
        t0 = rclpy.clock.Clock().now().nanoseconds / 1e9
        current_angle = 0.0

        while not self.isEqual(current_angle, relative_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rclpy.clock.Clock().now().nanoseconds / 1e9
            # Calculate elapsed time
            elapsed_time = (t1 - t0)
            # Update the current angle based on the angular speed and elapsed time
            current_angle = angular_speed * elapsed_time

            # Log the rotation progress
            self.node.get_logger().info(f"Rotating... Current angle: {math.degrees(current_angle)}, Target angle: {math.degrees(relative_angle)}")

            # Terminating condition with a small margin for precision
            if abs(current_angle - relative_angle) < 0.01:
                break
        print(Rotate().isEqual(current_angle, relative_angle))
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        
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
        self.get_logger().info(f"Moving Forward... Current distance: {self.distance}")
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
        try:
            rclpy.spin(self)
            
        except KeyboardInterrupt:
            self.stop_turtlebot()
        finally:
            self.destroy_node()

class GoToGoal(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pose = Pose()
        self.rate = 10  # Rate in Hz
        self.goal_pose = Pose()
        self.distance_tolerance = 0.0  # Initialize with zero for now
        self.go_forward = GoForward()
        self.rotate = Rotate()

    def user_goal(self):
        self.goal_pose = Pose()
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))
        self.distance_tolerance = float(input("Set your tolerance: "))

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def run(self):
        self.get_logger().info(f"Position: {self.pose}, Goal: {self.goal_pose}")

        rotate = Rotate()
        delta_angle = self.steering_angle(self.goal_pose)
        is_CW = delta_angle > 0  # Determine the direction of rotation
        speed_angular = 0.5 #1.0 * abs(delta_angle)
        rotate.set_params(speed_angular, abs(delta_angle), is_CW)
        rotate.run()

        distance = self.euclidean_distance(self.goal_pose)
        speed = 0.5 #* distance
        forward_command = GoForward()
        forward_command.set_params(speed, distance, isForward=True)
        forward_command.run()


def main(args=None):
    rclpy.init(args=args)

    node = GoToGoal()  # Creating an instance of GoToGoal
    node.user_goal()  # Setting the goal
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


