import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from backseat_msgs.action import DoMission
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# For high-rate, non-critical sensor data
best_effort_volatile_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Send ony once without retry
    durability=QoSDurabilityPolicy.VOLATILE,        # Do not store old messages
    depth=1
)

class DubinsActionServerClient:
    def __init__(self, node):
        self.action_client = ActionClient(node, DoMission, 'do_mission')
        self.node = node
        #self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.goal_handler = None

    def send_goal(self, goal: DoMission.Goal):
        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(
            goal, 
            feedback_callback=self.process_feedback
        )

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, response):
        new_goal_handler = response.result()
        if not new_goal_handler.accepted:
            return
        
        if self.goal_handler is not None:
            self.goal_handler.cancel_goal_async().add_done_callback(self.goal_cancelled)
        
        self.goal_handler = new_goal_handler
        self.goal_handler.get_result_async().add_done_callback(self.process_result)

    def goal_cancelled(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Goal successfully canceled')
        else:
            self.node.get_logger().info('Goal failed to cancel')


    def process_feedback(self, feedback_msg):
        self.node.get_logger().info(f'xt_error: {feedback_msg.feedback.xt_error}, %: {feedback_msg.feedback.mission_completion_perc}')
        self.node.get_logger().info(f'goal_id: {feedback_msg.goal_id}')
        pass

    def process_result(self, future):
        result = future.result().result # DoMission.Result
        self.node.get_logger().info(f'mission_complete: {result.mission_complete}')
        #rclpy.shutdown()

class ActionClientInterface(Node):
    """ A node that triggers and action client on a topic callback """
    def __init__(self):
        super().__init__('action_client_interface')
        self.action_client = DubinsActionServerClient(self)
        self.create_subscription(Bool, '/auto_docking/trigger', self.call_action_server, best_effort_volatile_qos)

    def call_action_server(self, msg):
        """ Create a mission and send it"""
        goal_msg = DoMission.Goal()
        goal_msg.filename = 'a_mission_to_run.file'
        
        self.action_client.send_goal(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionClientInterface()
    #node.call_action_server()
    rclpy.spin(node)
    rclpy.shutdown()