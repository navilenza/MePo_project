import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt

# Import your implementations (GoForward and Rotate)
from ros2_goforward import GoForward
from ros2_rotate2 import Rotate

class Gotogoal:
    def __init__(self):
        self.node = rclpy.create_node('turtlebot_controller')
        self.velocity_publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.node.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        self.pose = Pose()
        self.rate = 10  # Rate in Hz

        # Initialize GoForward and Rotate objects
        self.go_forward = GoForward()
        self.rotate = Rotate()
        self.max_speed = 30

    def user_goal(self):
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))
        return goal_pose, distance_tolerance

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)

    def move2goal(self, goal_pose, distance_tolerance):
        vel_msg = Twist()

        delta_angle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        is_CW = delta_angle > 0  # Determine the direction of rotation
        speed_angular = 4.0 * abs(delta_angle)
        self.rotate.set_params(speed_angular, abs(delta_angle), is_CW)  # Set rotation

        # Rotate robot to align with the goal pose
        self.rotate.rotate_robot()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            distance = self.euclidean_distance(goal_pose)
            speed = 1.5 * distance
            self.go_forward.set_params(speed, distance, True)  # Set forward movement

            # Update forward movement in a separate loop after rotation
            self.go_forward.run()

        # Stop the robot after reaching the goal
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Gotogoal()
    goal_pose, distance_tolerance = controller.user_goal()
    controller.move2goal(goal_pose, distance_tolerance)
    rclpy.spin(controller.node)  # Use the node's spin function
    controller.node.destroy_node()  # Clean up the node

if __name__ == '__main__':
    main()

