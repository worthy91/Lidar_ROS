#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class BatteryDockingNode(Node):
    def __init__(self):
        super().__init__('battery_docking_node')
        
        # Configuration
        self.dock_x = -4.38
        self.dock_y = 1.28
        self.test_interval = 60.0 # Seconds (1 Minute) for testing
        
        # State variables
        self.last_dock_time = time.time()
        self.is_docking = False

        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer to check battery/time status every 1 second
        self.timer = self.create_timer(1.0, self.check_battery_status)
        
        self.get_logger().info("Battery Node Started. Will dock every 60 seconds.")

    def check_battery_status(self):
        # Check if we are already docking or charging to prevent spamming
        if self.is_docking:
            return

        current_time = time.time()
        elapsed_time = current_time - self.last_dock_time

        # Logic: If 1 minute has passed, trigger docking
        if elapsed_time > self.test_interval:
            self.get_logger().warn(f"LOW BATTERY SIMULATION ({int(elapsed_time)}s passed). Triggering Docking!")
            self.send_docking_goal()

    def send_docking_goal(self):
        self.is_docking = True
        
        # Wait for Nav2 to be active
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            self.is_docking = False
            return

        # Create the goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Docking Coordinates
        goal_msg.pose.pose.position.x = self.dock_x
        goal_msg.pose.pose.position.y = self.dock_y
        goal_msg.pose.pose.orientation.w = 1.0  # Simple orientation, facing forward
        
        self.get_logger().info(f"Sending robot to Dock at x:{self.dock_x}, y:{self.dock_y}...")

        # Send the goal
        # This naturally PREEMPTS (cancels) any current waypoint following
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Docking goal rejected :(')
            self.is_docking = False
            return

        self.get_logger().info('Docking goal accepted. Moving to dock...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Robot has Arrived at Dock!')
        
        # Simulate Charging Time (e.g., wait 10 seconds before allowing next cycle)
        # In a real scenario, here you would trigger the "Resume Waypoints" logic
        time.sleep(5.0) 
        
        # Reset timer for the next loop
        self.last_dock_time = time.time()
        self.is_docking = False
        self.get_logger().info('Battery Recharged (Simulated). Resume operations.')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryDockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()