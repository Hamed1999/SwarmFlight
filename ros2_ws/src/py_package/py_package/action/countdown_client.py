#!/usr/bin python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_interfaces.action import Countdown
import sys
import time

class CountdownActionClient(Node):
    def __init__(self):
        super().__init__("countdown_client")
       
        self.action_client = ActionClient(self, Countdown, "countdown")
        self.goal_handle = None
        self.get_logger().info("Countdown Action Client is ready : )")

    def send_goal(self, starting_number):
        self.get_logger().info(f"Sending goal with starting number {starting_number}")
        goal_msg = Countdown.Goal()
        goal_msg.starting_number = starting_number

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if self.goal_handle.accepted:
            self.get_logger().info("Goal was accepted.")
            self.goal_handle.get_result_async().add_done_callback(self.result_callback)
        else:
            self.get_logger().info("Goal was rejected.")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.current_number}")


    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result ending number: {result.ending_number}")

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            #self.action_client._cancel_goal()
            cancel_future.add_done_callback(self._cancel_done)
            
        else:
            self.get_logger().info('No active goal to cancel')

    def _cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


def main(args = None):
    rclpy.init(args=args)
    

    try:
        starting_number = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    except ValueError:
        print('Please provide a valid integer')
        return

    action_client = CountdownActionClient()
    action_client.send_goal(starting_number)
    
    try:
        cancel_timer = 3  # seconds
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)
            if time.time() - start_time > cancel_timer:
                action_client.cancel_goal()
                break
        input()

    except KeyboardInterrupt:
        action_client.get_logger().info('Canceling goal...')

    action_client.destroy_node()    
    rclpy.shutdown()

if __name__ == "__main__":
    main()   
