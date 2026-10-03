#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generation')
        
        # --- TRAJECTORY PARAMETERS ---
        # Start at (0,0), via (6,0), via (6,3), finish at (0,3)
        self.waypoints = np.array([
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 3.0],
            [0.0, 3.0] 
        ])
        
        self.v_max = 0.5      # m/s
        self.v_corner = 0.2   # m/s
        self.v_min = 0.1      # m/s (Ensures it actually finishes)
        self.slow_down_dist = 0.4 # m
        # -----------------------------

        # Publishers for the Reference Trajectory
        self.path_pub = self.create_publisher(Path, 'sprint_review_path', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.target_twist_pub = self.create_publisher(Twist, 'target_velocity', 10)
        
        self.timer_period = 0.05 # 20Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.current_waypoint_idx = 0
        self.dist_traveled_in_segment = 0.0
        self.is_running = True
        
        # Current simulated state
        self.ref_x, self.ref_y = self.waypoints[0]
        
        # Pre-calculate segment lengths
        diffs = np.diff(self.waypoints, axis=0)
        self.segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        self.get_logger().info(f'Reference Trajectory Generator started. Path: {self.waypoints.tolist()}')
        self.publish_visual_path()

    def get_segment_velocity(self, s, seg_idx):
        seg_len = self.segment_lengths[seg_idx]
        v_target = self.v_max
        
        # Ramp up at the very start
        if seg_idx == 0 and s < self.slow_down_dist:
            v_target = (s / self.slow_down_dist) * self.v_max
            
        # Ramp down at the very end
        elif seg_idx == len(self.segment_lengths) - 1 and (seg_len - s) < self.slow_down_dist:
            dist_to_end = max(0.0, seg_len - s)
            v_target = (dist_to_end / self.slow_down_dist) * self.v_max
            
        # Slow down for intermediate corners
        elif (seg_len - s) < self.slow_down_dist:
            v_target = self.v_corner + (self.v_max - self.v_corner) * ((seg_len - s) / self.slow_down_dist)

        return max(self.v_min, v_target)

    def control_loop(self):
        if not self.is_running:
            return

        seg_idx = self.current_waypoint_idx
        if seg_idx >= len(self.segment_lengths):
            self.get_logger().info("Trajectory Reference Finished.")
            self.is_running = False
            self.publish_zero_velocity()
            return

        seg_len = self.segment_lengths[seg_idx]
        v = self.get_segment_velocity(self.dist_traveled_in_segment, seg_idx)
        
        # Direction of current segment
        p1 = self.waypoints[seg_idx]
        p2 = self.waypoints[seg_idx + 1]
        heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        
        # Update Reference Position
        self.ref_x = p1[0] + (self.dist_traveled_in_segment / seg_len) * (p2[0] - p1[0])
        self.ref_y = p1[1] + (self.dist_traveled_in_segment / seg_len) * (p2[1] - p1[1])
        
        # Publish Reference State
        self.publish_reference(self.ref_x, self.ref_y, heading, v)
        
        # Progress distance
        self.dist_traveled_in_segment += v * self.timer_period
        
        # Check for waypoint completion (with 1cm tolerance)
        if self.dist_traveled_in_segment >= (seg_len - 0.01):
            self.get_logger().info(f"Reference passed waypoint {seg_idx + 1}: {p2}")

            self.current_waypoint_idx += 1
            self.dist_traveled_in_segment = 0.0

    def publish_reference(self, x, y, yaw, v):
        # 1. Publish Pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.target_pose_pub.publish(pose)

        # 2. Publish Twist
        twist = Twist()
        twist.linear.x = float(v)
        # Note: Angular z is 0 because we assume the controller handles the rotation between segments
        self.target_twist_pub.publish(twist)

    def publish_zero_velocity(self):
        twist = Twist()
        self.target_twist_pub.publish(twist)

    def publish_visual_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for pt in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            path.poses.append(pose)
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()