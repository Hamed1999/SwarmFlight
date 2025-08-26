#!/usr/bin python3
import math
import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class DynamicTransformListener(Node):
    def __init__(self):
        super().__init__("dynamic_transform_listener")
       
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.lookup_transform)
        self.start_time = self.get_clock().now()
        self.get_logger().info("Dynamic transform listener is running : )")

    def lookup_transform(self):
       try:
           now = self.get_clock().now()
           tf_transform : TransformStamped = self.tf_buffer.lookup_transform(
               "world", "turtle1", rclpy.time.Time()) # rclpy.time.Time() is historical data 
           
           self.get_logger().info("Received transform: "
                              f"translation: ({tf_transform.transform.translation.x:.2f}, " 
                              f"{tf_transform.transform.translation.y:.2f}, "
                              f"{tf_transform.transform.translation.z:.2f})  "
                              f"rotation: ({tf_transform.transform.rotation.x:.2f}, "
                              f"{tf_transform.transform.rotation.y:.2f}, "
                              f"{tf_transform.transform.rotation.z:.2f}, "
                              f"{tf_transform.transform.rotation.w:.2f})")
           
           q = tf_transform.transform.rotation
           magnitude = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
           self.get_logger().info(f"\nQuaternion magnitude: {magnitude:.6f}")

       except Exception as e:
           self.get_logger().warn(f"Trasform not available: {e}")    

def main(args = None):
    rclpy.init(args=args)

    transform_listener_node = DynamicTransformListener()
    
    try:
       
        rclpy.spin(transform_listener_node)
    except KeyboardInterrupt:
        transform_listener_node.get_logger().info('Canceling dynamic transform listenning...')

    transform_listener_node.destroy_node()    
    rclpy.shutdown()

if __name__ == "__main__":
    main()   