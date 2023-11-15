import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math



class Rotate:
    # Initiate the node for the robot rotation
    def __init__(self, node):
        self.node = node
        self.velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()
    
    # Bool to determine if we need to rotate or not
    def isEqual(self, current_angle, relative_angle):
        return abs(current_angle - relative_angle) < 0.00015

    # Function to make the rotation
    def rotate_robot(self, speed, angle, clockwise):

        # Conversion in radians
        angular_speed = math.radians(speed)
        relative_angle = math.radians(angle)

        # Adjust the direction of rotation
        self.vel_msg.angular.z = -abs(angular_speed) if clockwise else abs(angular_speed)

        #t0, t1 are used for approx the angle
        t0 = self.node.get_clock().now().nanoseconds / 1e9
        current_angle = 0.0


        while not self.isEqual(abs(current_angle), abs(relative_angle)):
            # Give the instruction to rotate through velocity_publisher
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.node.get_clock().now().nanoseconds / 1e9
            # Angle approximation calculus
            current_angle = angular_speed * (t1 - t0) * (-1 if clockwise else 1)
            # print a msg in the terminal just to see the current angle
            self.node.get_logger().info(f"Rotating... Current angle: {abs(math.degrees(current_angle))}, Target angle: {angle}")

        # Stop the rotation
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)

class GoForward(Node):
    # Initiate the node for going forward and some variable
    def __init__(self):
        super().__init__('GoforwardCmd_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = 0.0
        self.distance = 0.0
        self.isForward = True
        self.current_distance = 0.0
    
    # Data parameters 
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

    # Update of the information of the distance
    def update_distance(self):
        self.current_distance += abs(self.speed) * self.timer_period
        # if the movement is over
        if self.current_distance >= self.distance:
            self.stop_turtlebot()
    # Instruction to stop the robot
    def stop_turtlebot(self):
        self.get_logger().info('Stopping turtlebot')
        self.get_logger().info('Robot stopped')
        self.publisher_.publish(Twist())
        self.timer.cancel()

    # The actual instruction to move forward
    def run(self):
        while self.current_distance < self.distance:
            self.timer_callback()
            rclpy.spin_once(self)
            time.sleep(1)
        self.stop_turtlebot()

class GoToGoal(Node):

    # Initiate the Node for GoToGoal
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.rotate = Rotate(self)
        self.go_forward = GoForward()

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = Pose()
        self.goal_pose = Pose()
        self.distance_tolerance = 0.0

    # Get the user input
    def user_goal(self):
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))
        self.theta = float(input("Set your rotation angle (in degrees): ")) 
        self.distance_tolerance = float(input("Set your tolerance: "))
    
    # Get the actual pose of the robot 
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # Angle between goal pose and self pose
    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    # Distance approximation to know when we are close enough
    def euclidean_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    # Instruction run for the user to put the goal for the robot
    def run(self):
        while rclpy.ok():
            # Get user input
            self.reset_go_forward()  # Reset GoForward instance before each run
            self.user_goal()
            self.get_logger().info(f"Position: {self.pose}, Goal: {self.goal_pose}")

            # Initiate the variable for the rotation
            delta_angle = self.steering_angle(self.goal_pose)
            is_CW = delta_angle > 0
            speed_angular = 15

            # Rotate
            self.rotate.rotate_robot(speed_angular, abs(math.degrees(delta_angle)), is_CW)
            
            # Initiate the variable for going forward
            distance = self.euclidean_distance(self.goal_pose)
            speed = 0.25
            self.go_forward.set_params(speed, distance, True)
            
            # Actual movement
            self.go_forward.run()
            time.sleep(2)

            # The theta rotation for the final pose/ final rotation
            is_CW = self.theta > 0
            self.rotate.rotate_robot(speed_angular, abs(self.theta), is_CW)   
            # We got to the goal yeaaah
            self.go_forward.stop_turtlebot()



    # Used for the pré-defined path
    def reset_go_forward(self):
        self.go_forward = GoForward()
    # Same as the run but without user input and the theta angle
    def run_no_user_input(self, distance, angle):
        self.get_logger().info(f"Position: {self.pose}, Goal: {self.goal_pose}")
        self.reset_go_forward()  # Reset GoForward instance before each run
        self.go_forward.set_params(0.25, distance, True)
        self.go_forward.run()
        self.rotate.rotate_robot(15, angle, True)
        self.go_forward.stop_turtlebot()

    # Generate mini Goal for the circle
    def generate_circle_waypoints(self, radius, num_waypoints=36):
        waypoints = []

        for i in range(num_waypoints):
            theta = i * (2 * math.pi) / num_waypoints
            x = self.goal_pose.x + radius * math.cos(theta)
            y = self.goal_pose.y + radius * math.sin(theta)
            waypoints.append((x, y))
        return waypoints

    def follow_circle(self, radius):
        waypoints = self.generate_circle_waypoints(radius)
        # loop through all the waypoints
        for waypoint in waypoints:
            self.goal_pose.x, self.goal_pose.y = waypoint
            self.run_no_user_input(0.25, 15)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    # Differents output but the predefined path will just exit the code when the figure is drawn
    # Only go to goal loop
    # I was too lazy to do a loop with the action input :D
    action = input("Enter '1' to go to a certain goal, '2' to do a square, and '3' to do a circle \n")
    
    # Go to a goal
    if action == '1':
        node.run()
        rclpy.shutdown()

    # Draw the square
    elif action == '2':
        # side_length = 2 follow the grid :)
        side_length = float(input("Enter the side length of the square: "))
        # call run without user input to do the square
        for i in range(4):
            node.run_no_user_input(side_length, 90)
    
    # Draw the circle
    elif action == '3':
        radius = float(input("Enter the radius of the circle: "))
        node.follow_circle(radius)

    # Nothing
    else:
        print("Invalid choice.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
