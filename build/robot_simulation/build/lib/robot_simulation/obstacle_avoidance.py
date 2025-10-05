#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.safe_distance = 1.0
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        self.get_logger().info("🚀 Obstacle Avoidance Node Started! Safe distance: 1.0m")
        self.get_logger().info("Waiting for laser data...")
        
    def laser_callback(self, msg):
        # Debug: Print first few laser readings
        if len(msg.ranges) > 0:
            self.get_logger().info(f"Laser ranges[0:5]: {msg.ranges[0:5]}", throttle_duration_sec=2.0)
        
        twist = Twist()
        num_readings = len(msg.ranges)
        
        if num_readings == 0:
            self.get_logger().warn("No laser readings received!")
            return
        
        # Analyze regions - using 120 degree front arc
        front_start = num_readings * 120 // 360
        front_end = num_readings * 240 // 360
        front_region = msg.ranges[front_start:front_end]
        
        left_region = msg.ranges[:num_readings//4]  # Left 90 degrees
        right_region = msg.ranges[3*num_readings//4:]  # Right 90 degrees
        
        # Filter out infinity and zeros
        front_filtered = [r for r in front_region if not (math.isinf(r) or r == 0.0)]
        left_filtered = [r for r in left_region if not (math.isinf(r) or r == 0.0)]
        right_filtered = [r for r in right_region if not (math.isinf(r) or r == 0.0)]
        
        min_front = min(front_filtered) if front_filtered else 10.0
        min_left = min(left_filtered) if left_filtered else 10.0
        min_right = min(right_filtered) if right_filtered else 10.0
        
        self.get_logger().info(f"Distances - Front: {min_front:.2f}m, Left: {min_left:.2f}m, Right: {min_right:.2f}m")
        
        # Obstacle avoidance logic
        if min_front > self.safe_distance:
            # No obstacle in front - move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info("✅ Moving forward - Clear path")
        else:
            # Obstacle detected - turn
            twist.linear.x = 0.0  # Stop forward motion
            
            if min_left > min_right:
                # More space on left - turn left
                twist.angular.z = self.angular_speed
                self.get_logger().info(f"🔄 Turning LEFT - Front obstacle: {min_front:.2f}m")
            else:
                # More space on right - turn right
                twist.angular.z = -self.angular_speed
                self.get_logger().info(f"🔄 Turning RIGHT - Front obstacle: {min_front:.2f}m")
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
