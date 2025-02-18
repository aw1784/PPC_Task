import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from point_follower_interfaces.srv import StartMission
from point_follower_interfaces.srv import CreatePlan
from point_follower_interfaces.action import Navigate
from rclpy.action import ActionClient

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        
        # State machine states
        self.state = 'idle'
        self.valid_states = ['idle', 'create path', 'navigate']

        # Publisher for state updates
        self.state_pub = self.create_publisher(String, '/state', 10)

        # Subscriber for mission updates
        self.mission_sub = self.create_subscription(String, '/mission', self.mission_callback, 10)

        # Service client for path planning
        self.plan_client = self.create_client(CreatePlan, '/create_plan')

        # Action client for navigation
        self.nav_client = ActionClient(self, Navigate, '/navigate')

        self.get_logger().info("Behavior Node Started. Waiting for missions...")

    def publish_state(self):
        """Publish the current FSM state."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Current State: {self.state}")

    def mission_callback(self, msg):
        """Handle incoming mission commands."""
        if msg.data == "Stop":
            self.state = 'idle'
            self.publish_state()
            return
        
        if msg.data == "GoTo":
            self.state = 'create path'
            self.publish_state()
            self.request_path()

    def request_path(self):
        """Call the /create_plan service to get a path."""
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Path planner service not available!")
            return

        request = CreatePlan.Request()
        request.goal_pose = Pose()  # Placeholder goal pose

        future = self.plan_client.call_async(request)
        future.add_done_callback(self.handle_plan_response)

    def handle_plan_response(self, future):
        """Handle the response from /create_plan."""
        try:
            response = future.result()
            self.get_logger().info("Received path. Transitioning to navigate state.")
            self.state = 'navigate'
            self.publish_state()
            self.start_navigation(response.path)
        except Exception as e:
            self.get_logger().error(f"Failed to get path: {str(e)}")
            self.state = 'idle'
            self.publish_state()

    def start_navigation(self, path):
        """Send the received path to the navigation action server."""
        goal_msg = Navigate.Goal()
        goal_msg.path = path

        self.get_logger().info("Sending navigation goal...")
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.handle_navigation_response)

    def handle_navigation_response(self, future):
        """Handle the navigation action response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            self.state = 'idle'
            self.publish_state()
            return

        self.get_logger().info("Navigation in progress...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.handle_navigation_result)

    def handle_navigation_result(self, future):
        """Handle completion of navigation."""
        result = future.result().result
        if result.success:
            self.get_logger().info("Navigation completed successfully.")
        else:
            self.get_logger().error("Navigation failed.")
        self.state = 'idle'
        self.publish_state()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
