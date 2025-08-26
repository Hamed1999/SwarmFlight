#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVelNode(Node):
    def __init__(self):
        super().__init__("cmd_vel")
        
       
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info("Publishing to /turtle1/cmd_vel : )")

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 1.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0
        self.publisher.publish(msg)
        print(f"Linear Vel: {msg.linear}\nAngular Vel: {msg.angular}")

def main(args = None):
    rclpy.init(args=args)
    node = CmdVelNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
