import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from point_follower_interfaces.srv import CreatePlan

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        # Service to generate paths
        self.srv = self.create_service(CreatePlan, '/create_plan', self.handle_create_plan)

        # Publisher for visualization
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)

        self.get_logger().info("Global Planner Node Started...")

    def handle_create_plan(self, request, response):
        """Handles path planning requests by creating a straight-line path."""
        self.get_logger().info("Received path request...")

        # Create a path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # Create start and goal PoseStamped messages
        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose.position.x = 0.0  # Assume starting at (0,0)
        start_pose.pose.position.y = 0.0
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header = path.header
        goal_pose.pose = request.goal_pose  # Target pose

        # Interpolating straight-line path (simplified)
        steps = 10
        for i in range(steps + 1):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_pose.pose.position.x + (goal_pose.pose.position.x - start_pose.pose.position.x) * (i / steps)
            pose.pose.position.y = start_pose.pose.position.y + (goal_pose.pose.position.y - start_pose.pose.position.y) * (i / steps)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # Publish path for visualization
        self.path_pub.publish(path)

        response.path = path
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
