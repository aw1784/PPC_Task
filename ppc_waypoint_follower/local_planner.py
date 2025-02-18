import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from point_follower_interfaces.action import Navigate
from rclpy.action import ActionServer
import math

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        # Action Server for navigation
        self._action_server = ActionServer(self, Navigate, '/navigate', self.execute_callback)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Local Planner Node Started...")

    def execute_callback(self, goal_handle):
        """Executes the navigation action by following the path."""
        self.get_logger().info("Navigation started...")
        path = goal_handle.request.path.poses

        if not path:
            self.get_logger().error("Received empty path!")
            goal_handle.succeed()
            return Navigate.Result(success=False)

        # Follow each point in the path
        for pose_stamped in path:
            goal_x = pose_stamped.pose.position.x
            goal_y = pose_stamped.pose.position.y

            if not self.rotate_to_goal(goal_x, goal_y):
                return Navigate.Result(success=False)

            if not self.move_to_goal(goal_x, goal_y):
                return Navigate.Result(success=False)

            # Provide feedback (remaining distance)
            remaining_distance = int(math.sqrt(goal_x**2 + goal_y**2))
            goal_handle.publish_feedback(Navigate.Feedback(remaining_distance=remaining_distance))

        self.get_logger().info("Navigation complete.")
        goal_handle.succeed()
        return Navigate.Result(success=True)

    def rotate_to_goal(self, goal_x, goal_y):
        """Rotates the robot to face the goal."""
        twist = Twist()
        angle_to_goal = math.atan2(goal_y, goal_x)

        # Simple rotation logic
        while abs(angle_to_goal) > 0.05:
            twist.angular.z = 0.5 if angle_to_goal > 0 else -0.5
            self.cmd_pub.publish(twist)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        return True

    def move_to_goal(self, goal_x, goal_y):
        """Moves the robot forward until the goal is reached."""
        twist = Twist()
        distance = math.sqrt(goal_x**2 + goal_y**2)

        while distance > 0.1:
            twist.linear.x = 0.2
            self.cmd_pub.publish(twist)
            distance -= 0.2  # Simulating movement (update distance)

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
