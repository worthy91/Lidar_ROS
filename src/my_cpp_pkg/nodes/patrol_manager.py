#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tkinter as tk
from tkinter import font

# --- CONFIGURATION ---
DOCK_POSE = [-4.38, 1.28] 

# NEW WAYPOINTS
WAYPOINTS_LIST = [
    [7.9, 11.1],
    [12.1, -11.1],
    [-10.8, 9.9],
    [-9.7, -11.2]
]

# TIMING CONFIGURATION
BATTERY_TIMEOUT = 100.0  # Seconds 100% -> 0%
CHARGING_TIME = 10.0      # Seconds 0% -> 100%

# Rates
DISCHARGE_RATE = 100.0 / BATTERY_TIMEOUT 
CHARGE_RATE = 100.0 / CHARGING_TIME      

class RobotDashboard:
    """A simple Pop-up GUI to visualize robot state"""
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Security Robot Monitor")
        self.root.geometry("300x180")
        self.root.attributes('-topmost', True) # Keep window on top
        
        # Styling
        self.bg_color = "#2c3e50"
        self.text_color = "#ecf0f1"
        self.root.configure(bg=self.bg_color)
        self.custom_font = font.Font(family="Helvetica", size=12, weight="bold")
        self.big_font = font.Font(family="Helvetica", size=24, weight="bold")

        # Labels
        self.lbl_title = tk.Label(self.root, text="SYSTEM STATUS", bg=self.bg_color, fg="#bdc3c7", font=self.custom_font)
        self.lbl_title.pack(pady=10)

        self.lbl_state = tk.Label(self.root, text="INITIALIZING", bg=self.bg_color, fg="#3498db", font=self.custom_font)
        self.lbl_state.pack(pady=5)

        self.lbl_battery = tk.Label(self.root, text="100%", bg=self.bg_color, fg="#2ecc71", font=self.big_font)
        self.lbl_battery.pack(pady=20)

    def update(self, battery_pct, state_text):
        try:
            # Update Text
            self.lbl_state.config(text=state_text)
            self.lbl_battery.config(text=f"{int(battery_pct)}%")

            # Dynamic Colors
            if battery_pct > 50:
                batt_color = "#2ecc71" # Green
            elif battery_pct > 20:
                batt_color = "#f1c40f" # Yellow
            else:
                batt_color = "#e74c3c" # Red
            
            self.lbl_battery.config(fg=batt_color)
            
            # Refresh window
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass # Window closed

class SmartPatrol(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.navigator = BasicNavigator()
        
        # Battery State
        self.battery_level = 100.0
        self.state_description = "PATROLLING" 
        
        # Visualizer Publisher (Keep this for RVIZ backup)
        self.marker_pub = self.create_publisher(MarkerArray, '/battery_markers', 10)
        
        # Initialize GUI
        self.gui = RobotDashboard()

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def update_visuals(self):
        """Updates both the Popup Window and RViz Markers"""
        # 1. Update GUI
        self.gui.update(self.battery_level, self.state_description)

        # 2. Update RViz Marker (Optional but helpful backup)
        marker_array = MarkerArray()
        
        # Text Marker
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "battery_text"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.5 
        text.pose.position.z = 1.0 
        text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
        text.text = f"{int(self.battery_level)}%"
        text.lifetime.nanosec = 200000000

        marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

    def run(self):
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("--- Patrol Started ---")

        current_wp_index = 0
        dt = 0.1 

        while rclpy.ok():
            self.state_description = f"WAYPOINT {current_wp_index+1}"
            target = WAYPOINTS_LIST[current_wp_index]
            
            goal_pose = self.create_pose(target[0], target[1])
            self.navigator.goToPose(goal_pose)

            # Loop while moving
            while not self.navigator.isTaskComplete():
                self.battery_level -= DISCHARGE_RATE * dt
                self.update_visuals()

                # Check Low Battery
                if self.battery_level <= 25.0: 
                    self.get_logger().warn("LOW BATTERY! Docking...")
                    dock_success = self.perform_docking_sequence(dt)
                    
                    if dock_success:
                        self.get_logger().info(f"Resuming Waypoint {current_wp_index+1}...")
                        self.navigator.goToPose(goal_pose)
                    else:
                        return 

                time.sleep(dt)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                current_wp_index = (current_wp_index + 1) % len(WAYPOINTS_LIST)

    def perform_docking_sequence(self, dt):
        self.navigator.cancelTask()
        self.state_description = "STOPPING..."
        self.update_visuals()
        time.sleep(1.5) 

        # Go to Dock
        self.state_description = "GOING TO DOCK"
        dock_pose = self.create_pose(DOCK_POSE[0], DOCK_POSE[1])
        self.navigator.goToPose(dock_pose)
        time.sleep(1.0)

        # Wait for arrival
        while not self.navigator.isTaskComplete():
            self.battery_level -= DISCHARGE_RATE * dt
            self.update_visuals()
            time.sleep(dt)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.state_description = "CHARGING"
            
            # Charge Loop
            while self.battery_level < 100.0:
                self.battery_level += CHARGE_RATE * dt
                if self.battery_level > 100.0: self.battery_level = 100.0
                
                self.update_visuals()
                time.sleep(dt)
            
            return True
        else:
            self.state_description = "STRANDED / FAILED"
            self.update_visuals()
            return False

def main():
    rclpy.init()
    node = SmartPatrol()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()