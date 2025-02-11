import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ppc_waypoint_follower.srv import StartMission # Custom service
class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # Create service server
        self.srv = self.create_service(StartMission, '/start_mission', self.handle_mission_request)
        # Create publisher for valid missions
        self.publisher = self.create_publisher(String, '/mission', 10)
        
        self.get_logger().info("Mission Node Started... Waiting for requests.")
    def handle_mission_request(self, request, response):
        valid_missions = ["GoTo", "Stop"]
        if request.mission_name in valid_missions:
            response.accepted = True
            self.get_logger().info(f"Mission '{request.mission_name}' accepted. Publishing...")
 
 # Publish mission name to /mission topic
            msg = String()
            msg.data = request.mission_name
            self.publisher.publish(msg)
        else:
            response.accepted = False
            self.get_logger().warn(f"Invalid mission: '{request.mission_name}'. Request rejected.")
        return response
def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
 main()
