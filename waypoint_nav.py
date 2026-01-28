import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # <--- Added Lidar import
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- NEW: Subscribe to LiDAR ---
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_distance = 10.0 # Default to clear path

        self.waypoints = [(1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)]
        self.current_idx = 0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.last_x, self.last_y = 0.0, 0.0
        self.stuck_counter = 0
        self.is_reversing = False
        self.reverse_timer = 0

        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info("Lidar-Enhanced Navigator Active.")

    def scan_callback(self, msg):
        # Look at the center 30 degrees of the scan
        # For a 360-sample lidar, indices 165 to 195 are directly in front
        front_ranges = msg.ranges[165:195]
        self.obstacle_distance = min(front_ranges) if front_ranges else 10.0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def navigate(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("MISSION SUCCESS!")
            self.cmd_pub.publish(Twist())
            return

        msg = Twist()

        # RECOVERY: Back up if we are in recovery mode
        if self.is_reversing:
            msg.linear.x = -0.3
            msg.angular.z = 0.6
            self.reverse_timer += 1
            if self.reverse_timer > 20:
                self.is_reversing = False
                self.reverse_timer = 0
            self.cmd_pub.publish(msg)
            return

        # --- LIDAR SAFETY: Stop if something is closer than 0.5 meters ---
        if self.obstacle_distance < 0.6:
            self.get_logger().warn(f"LIDAR Sensed Wall at {self.obstacle_distance:.2f}m! Reversing.")
            self.is_reversing = True
            return

        # NORMAL NAVIGATION (Same as before)
        target_x, target_y = self.waypoints[self.current_idx]
        dist = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        angle_to_target = math.atan2(target_y - self.y, target_x - self.x)
        angle_diff = angle_to_target - self.yaw
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        if abs(angle_diff) > 0.4:
            msg.angular.z = 0.8 if angle_diff > 0 else -0.8
            self.get_logger().info(f"Turning to Target {self.current_idx+1}")
        elif dist > 0.3:
            msg.linear.x = 0.4
            self.get_logger().info(f"Driving to Target {self.current_idx+1}")
        else:
            self.get_logger().info(f"Waypoint {self.current_idx+1} Reached!")
            self.current_idx += 1
            
        self.last_x, self.last_y = self.x, self.y
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(WaypointNavigator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()