#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy #TODO
from px4_msgs.msg import VehicleCommand as VehicleCommandMsg
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, GotoSetpoint 

import time
import math

class OffboardController(Node):
    def __init__(self, drone_ID=1):
        super().__init__("offboard_takeoff_node")
        self.id = drone_ID
        self.namespace = '/px4_' + str(drone_ID)
    
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1 
        )
        
        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f'{self.namespace}/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint,
            f'{self.namespace}/fmu/in/goto_setpoint',
            qos_profile
        )
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f'{self.namespace}/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommandMsg,
            f'{self.namespace}/fmu/in/vehicle_command',
            qos_profile
        )
        
        #Create subscriber
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            f'{self.namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        #Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0 # set to negative for NED frame (Up)
        self.current_state = "INIT" # all states: INIT, OFFBOARD_REQUESTED, ARMED, TAKEOFF
        
        # Create a timer to publish commands more than 2HZ (e.g. 10HZ)
        self.get_logger().info(f"Starting offboard control for {self.namespace} drone.")
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    # Functions   
    def vehicle_status_callback(self, msg):
        """ Callback function for vehicle_status topic subscriber """
        self.vehicle_status = msg
        self.get_logger().info(f"Nav State: {self.vehicle_status.nav_state}")
        self.get_logger().info(f"Arming State: {self.vehicle_status.arming_state}")
        
    def publish_offboard_control_mode_heartbeat(self):
        """Publish the offboard control mode & heatbeat."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.offboard_control_mode_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self, pose = [math.nan, math.nan, math.nan], yaw=0.0):
        """Publish the set trajectory setpoint command."""
        msg = TrajectorySetpoint()
        msg.position = pose
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publish trajectory setpoint: (X:{pose[0]}, Y:{pose[1]}, Z:{pose[2]})")
        
        self.trajectory_setpoint_publisher.publish(msg)
        
    def publish_goto_setpoint(self, pose = [math.nan, math.nan, math.nan]):
        """Publish the Go to setpoint command."""
        msg = GotoSetpoint()
        msg.position = pose
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publish goto setpoint: (X:{pose[0]}, Y:{pose[1]}, Z:{pose[2]})")
        
        self.goto_setpoint_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommandMsg()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        #msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.confirmation = 1
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publish VehicleCommand: {command}")
        
        self.vehicle_command_publisher.publish(msg)
    
    def publish_set_offboard_mode(self):
        """Send the command to switch to Offboard mode."""
        self.publish_vehicle_command(VehicleCommandMsg.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"Drone_{self.id}: Requesting Offboard Mode ...")
        self.current_state = "OFBOARD_REQUESTED"
        
    def arm(self):
        """Send the arming command."""
        self.publish_vehicle_command(VehicleCommandMsg.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f"Drone_{self.id}: Sending Arming Command ...")
        self.current_state = "ARMED"        
    
    def disarm(self):
        """Send the disarming command."""
        self.publish_vehicle_command(VehicleCommandMsg.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f"Drone_{self.id}: Sending Disarming Command ...")
        self.current_state = "DISARMED"        

    def takeoff(self):
        self.publish_trajectory_setpoint([math.nan, math.nan, self.takeoff_height])
        #self.publish_goto_setpoint([math.nan, math.nan, self.takeoff_height])
        self.current_state = "TAKEOFF"            
        self.get_logger().info(f"Drone_{self.id}: Takeoff location is (nan, nan, {self.takeoff_height})")
        
    def timer_callback(self):
        """Main control loop executed by the timer."""
        # Always publish it 
        self.publish_offboard_control_mode_heartbeat()
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
           self.get_logger().info(f"Drone_{self.id}:Takeoff is in Progress ...")
        
        # State Machine
        if self.current_state == "INIT":
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter >= 10: # Send 10 setpoints (1 sec) before switching mode
                self.publish_set_offboard_mode()
        
        elif self.current_state == "OFBOARD_REQUESTED":
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info(f"Drone_{self.id}: Offboard mode confirmed. Arming in progress...")
                self.publish_vehicle_command(VehicleCommandMsg.VEHICLE_CMD_RUN_PREARM_CHECKS)
                self.arm()
            else:
                self.publish_set_offboard_mode()   
        
        elif self.current_state == "ARMED":
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info(f"Drone_{self.id} Armed . Takeoff in progress...")
                self.takeoff()
            else:
                self.arm()
                #pass
                
        elif self.current_state == "TAKEOFF":
            self.takeoff()
            
        # Implement Other logics for formations ...   
         
                
        
def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('offboard_takeoff_launcher')
    
    # Declare the 'drone_ID' parameter with default value of 1
    node.declare_parameter('drone_ID', 1)
    
    # Retrieve the 'drone_ID' parameter value
    drone_ID = node.get_parameter('drone_ID').get_parameter_value().integer_value
    
    # Destory temporary node used parameter handling
    node.destroy_node()
    
    # e.g. in terminal:
    # ros2 run swarm_control offboard_takeoff --ros-args -p drone_ID:=2
     
    offboard_node = OffboardController(drone_ID=drone_ID)
    rclpy.spin(offboard_node)
    
    offboard_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()              