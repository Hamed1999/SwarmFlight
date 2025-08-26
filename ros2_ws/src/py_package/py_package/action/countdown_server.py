#!/usr/bin python3
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionServer
from my_interfaces.action import Countdown
import time
from rclpy.action import CancelResponse


class CountdownActionServer(Node):
    def __init__(self):
        super().__init__("countdown_server")
        
        self.action_server = ActionServer(self, Countdown, "countdown",self.execute_callback, cancel_callback=self.cancel_callback)
        self.get_logger().info("Countdown Action Server is ready : )")

    def cancel_callback(self, cancel_request):
        # Accept all cancellation requests
        self.get_logger().info("Canceling in progress ...")
        return CancelResponse.ACCEPT    

    async def execute_callback(self, goal_handle):
        starting_number = goal_handle.request.starting_number
        self.get_logger().info(f"Executing countdown from {starting_number}")
        
        feedback_msg = Countdown.Feedback()
        result = Countdown.Result()

        for i in range(starting_number, 0, -1):
            feedback_msg.current_number = i

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.ending_number = i
                self.get_logger().info('Countdown canceled')
                return result
            
            self.get_logger().info(f"Sending feedback: {feedback_msg.current_number}")
            time.sleep(1.0)

            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result.ending_number = 0
        return result


def main(args = None):
    rclpy.init(args=args)
    action_server = CountdownActionServer() 
    try:
        rclpy.spin(node=action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info('Shutting down action server...')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()   