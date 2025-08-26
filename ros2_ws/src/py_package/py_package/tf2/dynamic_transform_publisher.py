#!/usr/bin python3
import math
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DynamicTransformPublisher(Node):
    def __init__(self):
        super().__init__("dynamic_transform_publisher")
       
        self.transform_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_dynamic_transform)
        self.start_time = self.get_clock().now()
        self.get_logger().info("Dynamic transform broadcaster is running : )")

    def broadcast_dynamic_transform(self):
       now = self.get_clock().now()
       elapsed_time = (now - self.start_time).nanoseconds / 1e9
       dynamic_transform = TransformStamped()

       dynamic_transform.header.stamp = now.to_msg()
       dynamic_transform.header.frame_id = "world"
       dynamic_transform.child_frame_id = "turtle1"

       dynamic_transform.transform.translation.x = 2 * math.cos(elapsed_time)
       dynamic_transform.transform.translation.y = 2 * math.sin(elapsed_time)
       dynamic_transform.transform.translation.z = 1 * math.sin(elapsed_time) * math.cos(elapsed_time)

       yaw = elapsed_time
       dynamic_transform.transform.rotation.x = 0.0
       dynamic_transform.transform.rotation.y = 0.0
       dynamic_transform.transform.rotation.z = math.sin(yaw/2.0)
       dynamic_transform.transform.rotation.w = math.cos(yaw/2.0)

       self.transform_broadcaster.sendTransform(dynamic_transform)

       self.get_logger().info("BraodCasting transform: "
                              f"translation: ({dynamic_transform.transform.translation.x:.2f}, " 
                              f"{dynamic_transform.transform.translation.y:.2f}, "
                              f"{dynamic_transform.transform.translation.z:.2f})  "
                              f"rotation: ({dynamic_transform.transform.rotation.x:.2f}, "
                              f"{dynamic_transform.transform.rotation.y:.2f}, "
                              f"{dynamic_transform.transform.rotation.z:.2f}, "
                              f"{dynamic_transform.transform.rotation.w:.2f})")

def main(args = None):
    rclpy.init(args=args)

    transform_publisher_node = DynamicTransformPublisher()
    
    try:
       
        rclpy.spin(transform_publisher_node)
    except KeyboardInterrupt:
        transform_publisher_node.get_logger().info('Canceling dynamic transform publishing...')

    transform_publisher_node.destroy_node()    
    rclpy.shutdown()

if __name__ == "__main__":
    main()   