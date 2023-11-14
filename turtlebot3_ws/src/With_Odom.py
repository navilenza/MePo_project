import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math
import numpy as np

class Rotate:
    def __init__(self, node):
        self.node = node
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()

    def isEqual(self, current_angle, relative_angle):
        return abs(current_angle - relative_angle) < 0.00015

    def rotate_robot(self, speed, angle, clockwise):
        angular_speed = math.radians(speed)
        relative_angle = math.radians(angle)

        self.vel_msg.angular.z = -abs(angular_speed) if clockwise else abs(angular_speed)

        t0 = self.node.get_clock().now().nanoseconds / 1e9
        current_angle = 0.0
        while not self.isEqual(abs(current_angle), abs(relative_angle)):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.node.get_clock().now().nanoseconds / 1e9
            current_angle = angular_speed * (t1 - t0) * (-1 if clockwise else 1)
            current_angle = current_angle % 360
            self.node.get_logger().info(f"Rotating... Current angle: {math.degrees(current_angle)}, Target angle: {angle}")

        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)

class GoForward(Node):
    def __init__(self):
        super().__init__('GoforwardCmd_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = 0.0
        self.distance = 0.0
        self.isForward = True
        self.current_distance = 0.0

    def set_params(self, speed, distance, isForward):
        self.speed = speed
        self.distance = distance
        self.isForward = isForward
        self.current_distance = 0.0

    def timer_callback(self):
        move_cmd = Twist()

        if self.isForward:
            move_cmd.linear.x = abs(self.speed)
        else:
            move_cmd.linear.x = -abs(self.speed)

        self.publisher_.publish(move_cmd)
        self.get_logger().info(f"Moving Forward... Current distance: {self.current_distance}")
        self.update_distance()

    def update_distance(self):
        self.current_distance += abs(self.speed) * self.timer_period

        if self.current_distance >= self.distance:
            self.stop_turtlebot()

    def stop_turtlebot(self):
        self.get_logger().info('Stopping turtlebot')
        self.get_logger().info('Robot stopped')
        self.publisher_.publish(Twist())
        self.timer.cancel()

    def run(self):
        self.get_logger().info(f"Moving Forward... Goal distance: {self.distance}")

        while rclpy.ok() and self.current_distance < self.distance:
            self.timer_callback()
            rclpy.spin_once(self)

        self.stop_turtlebot()

class Odom(Node):
    def euler_from_quaternion(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

class GoToGoal(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.rotate = Rotate(self)
        self.go_forward = GoForward()

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = Odometry()
        self.goal_pose = Pose()
        self.distance_tolerance = 0.0

    def user_goal(self):
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))
        self.theta = math.radians(float(input("Set your rotation angle (in degrees): "))) 
        self.distance_tolerance = float(input("Set your tolerance: "))

    def update_pose(self, data):
        pose, orientation = data.pose.pose.position, data.pose.pose.orientation
        roll, pitch, yaw = Odom.euler_from_quaternion(orientation)

        self.pose = data.pose.pose
        self.pose.theta = yaw

    def steering_angle(self, goal_pose):
        goal_x, goal_y = goal_pose.x, goal_pose.y
        current_x, current_y = self.pose.pose.pose.position.x, self.pose.pose.pose.position.y

        return math.atan2(goal_y - current_y, goal_x - current_x)


    # Pas besoin de ça
    def euclidean_distance(self, goal_pose):
        goal_x, goal_y = goal_pose.x, goal_pose.y
        current_x, current_y = self.pose.pose.pose.position.x, self.pose.pose.pose.position.y

        return math.sqrt(pow((goal_x - current_x), 2) + pow((goal_y - current_y), 2))

    def run(self):
        while rclpy.ok():
            self.user_goal()
            self.get_logger().info(f"Position: {self.pose}, Goal: {self.goal_pose}")

            delta_angle = self.steering_angle(self.goal_pose)
            is_CW = delta_angle > 0
            speed_angular = 15

            self.rotate.rotate_robot(speed_angular, abs(math.degrees(delta_angle)), is_CW)

            distance = self.euclidean_distance(self.goal_pose)
            speed = 0.25
            self.go_forward.set_params(speed, distance, True)
            self.go_forward.run()
            self.rotate.rotate_robot(speed_angular, self.theta, is_CW)
            
            self.go_forward.stop_turtlebot()
            # Ask the user if they want to continue
            # doesn't appear // The code is blocked at self.go_forward.stop_turtlebot()

            user_input = input("New imput?  (y/n): ")
            if user_input.lower() != 'y':
                break

#class Predefined_path:
#    def square(self):



def main(args=None):

    rclpy.init(args=args)
    node = GoToGoal()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
