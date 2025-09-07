#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from backseat_msgs.action import DoMission


class DubinsActionServer(Node):
    def __init__(self):
        super().__init__('dubins_action_server_node')
        #self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.goal_handle = None

        self._feedback = DoMission.Feedback()
        self._result = DoMission.Result()

        self.action_server = ActionServer(
            self,
            DoMission,
            'do_mission',
            execute_callback=self.__run,
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback)
        
        self.get_logger().info('Server initialized. Waiting for new mission...')

    def __run(self, goal_handle):
        # goal = DoMission.Result(goal_handle.request)
        # goal_handle.request.filename
        # goal_handle.request.mission
        # goal_handle.request.id
        

        mission_complete = True
        i=0
        self.get_logger().info("Doing long running things (busy)")
        import time
        
        while i < 100:
            #self.get_logger().info('Goal aborted')
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return DoMission.Result()
             
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                mission_complete = False
                self.get_logger().warn('Cancel requested')
                return DoMission.Result()

            self._feedback.xt_error = float(i)
            i += 1
            goal_handle.publish_feedback(self._feedback)
            time.sleep(0.1)
        goal_handle.succeed()
        self.get_logger().info("Goal succeed")
        return DoMission.Result(mission_complete=mission_complete)

    def __goal_callback(self, goal_request):
        self.get_logger().info('Received new goal request')
        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel callback executed')
        
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    node = DubinsActionServer()
    
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
