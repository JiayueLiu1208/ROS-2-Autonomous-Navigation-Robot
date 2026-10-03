#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math


class MapServer(Node):
    def __init__(self):
        super().__init__('final_demo_map_server')

        # ==========================================
        # --- MAP / PLANNING PARAMETERS ---
        # Coordinate convention:
        #   ROS map x = document X_W
        #   ROS map y = document Y_W
        #
        # The final demo room is approximately:
        #   X_W range: 0.0 to 15.0 m
        #   Y_W range: 0.0 to 10.8 m
        # ==========================================
        self.resolution = 0.1          # 0.1 m per grid cell
        self.robot_radius = 0.15       # hard inflation radius
        self.decay_radius = 0.10       # extra soft cost radius

        self.width_m = 15.0            # X_W direction
        self.height_m = 10.8           # Y_W direction

        self.wall_thickness = 0.10
        self.plant_radius = 0.10       # requested dotted point radius

        # Set this to False if plants should only be visual markers,
        # not physical obstacles for path planning.
        self.PLANTS_AS_OBSTACLES = True

        # Example start and goal poses.
        # Change these depending on your testing task.
        self.start_x, self.start_y, self.start_yaw = 0.8, 0.8, 0.0
        self.goal_x, self.goal_y, self.goal_yaw = 13.2, 9.8, 0.0

        # ==========================================
        # --- FINAL DEMO PLANT LOCATIONS ---
        # From Table 2 of the provided document.
        # Format: (name, X_W, Y_W)
        # ==========================================
        self.plants = [
            ("P1", 1.00, 10.00),
            ("P2", 5.00, 4.00),
            ("P3", 8.40, 8.00),
            ("P4", 10.00, 5.00),
            ("P5", 12.40, 1.00),
            ("P6", 14.60, 8.00),
        ]

        # ==========================================
        # --- TABLE / OBSTACLE LAYOUT ---
        # Approximate table positions estimated from Figure 1.
        # Format: (name, centre_x, centre_y, size_x, size_y)
        #
        # Since the PDF only gives exact coordinates for plants
        # and ArUco markers, these table positions should be treated
        # as practical simulation approximations.
        # ==========================================
        table_sx = 1.45
        table_sy = 1.45

        self.tables = [
            # bottom row
            ("T1", 3.45, 8.25, table_sx, table_sy),
            ("T2", 3.45, 5.25, table_sx, table_sy),
            ("T3", 3.45, 2.25, table_sx, table_sy),

            # lower-middle row
            ("T4", 6.45, 8.25, table_sx, table_sy),
            ("T5", 6.45, 2.25, table_sx, table_sy),

            # upper-middle row
            ("T6", 9.85, 8.25, table_sx, table_sy),
            ("T7", 9.85, 2.25, table_sx, table_sy),

            # top row
            ("T8", 13.15, 8.25, table_sx, table_sy),
            ("T9", 13.15, 5.25, table_sx, table_sy),
            ("T10", 13.15, 2.25, table_sx, table_sy),
        ]

        # Approximate lectern position near the lower-left side of Figure 1.
        # Its back side is solid in +Y through to the room boundary.
        self.lecterns = [
            ("Lectern", 2.60, 10.2375, 1.45, 1.125),
        ]

        # ==========================================
        # --- ROS PUBLISHERS ---
        # ==========================================
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.wall_viz_pub = self.create_publisher(Marker, '/map_obstacles_viz', 10)
        self.plant_viz_pub = self.create_publisher(Marker, '/plant_points_viz', 10)
        self.plant_label_pub = self.create_publisher(Marker, '/plant_labels_viz', 10)
        self.start_pub = self.create_publisher(PoseStamped, '/initial_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.width = int(round(self.width_m / self.resolution))
        self.height = int(round(self.height_m / self.resolution))

        self.timer = self.create_timer(2.0, self.publish_all)

        self.get_logger().info(
            f'Final Demo Map Server started: {self.width_m:.1f} m x {self.height_m:.1f} m, '
            f'resolution = {self.resolution:.2f} m/cell'
        )

    # ==========================================================
    # Utility functions
    # ==========================================================

    def euler_to_quaternion(self, yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0),
        }

    def clamp_cell_range(self, start, end, max_size):
        start = max(0, min(max_size, start))
        end = max(0, min(max_size, end))
        return start, end

    # ==========================================================
    # Base map creation
    # ==========================================================

    def create_base_map(self):
        """
        Creates the binary obstacle map before C-space inflation.

        Occupancy values:
            0   = free space
            100 = occupied obstacle
        """
        grid = np.zeros((self.height, self.width), dtype=np.int8)

        def draw_rect(x, y, w, h, value=100):
            """
            Draw rectangle using lower-left corner (x, y), width w, height h.
            Coordinates are in metres.
            """
            c_start = int(math.floor(x / self.resolution))
            c_end = int(math.ceil((x + w) / self.resolution))
            r_start = int(math.floor(y / self.resolution))
            r_end = int(math.ceil((y + h) / self.resolution))

            r_start, r_end = self.clamp_cell_range(r_start, r_end, self.height)
            c_start, c_end = self.clamp_cell_range(c_start, c_end, self.width)

            grid[r_start:r_end, c_start:c_end] = value

        def draw_rect_center(cx, cy, sx, sy, value=100):
            """
            Draw rectangle using centre point and dimensions.
            """
            draw_rect(cx - sx / 2.0, cy - sy / 2.0, sx, sy, value)

        def draw_circle(cx, cy, radius, value=100):
            """
            Draw circular obstacle using centre and radius.
            Used for plant dotted points.
            """
            r_cells = int(math.ceil(radius / self.resolution))

            c_centre = int(round(cx / self.resolution))
            r_centre = int(round(cy / self.resolution))

            for dr in range(-r_cells, r_cells + 1):
                for dc in range(-r_cells, r_cells + 1):
                    rr = r_centre + dr
                    cc = c_centre + dc

                    if 0 <= rr < self.height and 0 <= cc < self.width:
                        dist = math.sqrt((dr * self.resolution) ** 2 +
                                         (dc * self.resolution) ** 2)
                        if dist <= radius:
                            grid[rr, cc] = value

        # --------------------------
        # Room boundary walls
        # --------------------------
        wt = self.wall_thickness

        draw_rect(0.0, 0.0, self.width_m, wt)                    # bottom wall
        draw_rect(0.0, self.height_m - wt, self.width_m, wt)      # top wall
        draw_rect(0.0, 0.0, wt, self.height_m)                    # left wall
        draw_rect(self.width_m - wt, 0.0, wt, self.height_m)      # right wall

        # --------------------------
        # Tables
        # --------------------------
        for _, cx, cy, sx, sy in self.tables:
            draw_rect_center(cx, cy, sx, sy, value=100)

        # --------------------------
        # Lectern
        # --------------------------
        for _, cx, cy, sx, sy in self.lecterns:
            draw_rect_center(cx, cy, sx, sy, value=100)

        # --------------------------
        # Plants as dotted circular points
        # --------------------------
        if self.PLANTS_AS_OBSTACLES:
            for _, px, py in self.plants:
                draw_circle(px, py, self.plant_radius, value=100)

        return grid

    # ==========================================================
    # Configuration-space inflation
    # ==========================================================

    def apply_c_space(self, grid):
        """
        Applies obstacle inflation and soft cost decay.

        Values:
            100 = real obstacle
            80  = hard inflated region within robot radius
            1-60 = soft proximity cost region
            0   = free space
        """
        total_radius = self.robot_radius + self.decay_radius
        total_cells = int(np.ceil(total_radius / self.resolution))

        result_grid = grid.copy().astype(np.float32)

        obstacle_rows, obstacle_cols = np.where(grid == 100)

        for r, c in zip(obstacle_rows, obstacle_cols):
            for dr in range(-total_cells, total_cells + 1):
                for dc in range(-total_cells, total_cells + 1):
                    nr = r + dr
                    nc = c + dc

                    if not (0 <= nr < self.height and 0 <= nc < self.width):
                        continue

                    if grid[nr, nc] == 100:
                        continue

                    dist = np.sqrt(dr ** 2 + dc ** 2) * self.resolution

                    if dist <= self.robot_radius:
                        result_grid[nr, nc] = max(result_grid[nr, nc], 80)

                    elif dist <= total_radius:
                        ratio = (dist - self.robot_radius) / self.decay_radius
                        cost = 60.0 * (1.0 - ratio)
                        result_grid[nr, nc] = max(result_grid[nr, nc], cost)

        return result_grid.astype(np.int8)

    # ==========================================================
    # Visualisation markers
    # ==========================================================

    def publish_viz_markers(self, base_grid, inflated_grid):
        """
        Publishes a CUBE_LIST marker for obstacles and C-space inflation.
        Black cells are real obstacles.
        Blue cells are inflated / cost regions.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "final_demo_c_space"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.08

        obstacle_color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)
        inflated_color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.35)

        for r in range(self.height):
            for c in range(self.width):
                if inflated_grid[r, c] > 0:
                    p = Point()
                    p.x = c * self.resolution + self.resolution / 2.0
                    p.y = r * self.resolution + self.resolution / 2.0
                    p.z = 0.04

                    marker.points.append(p)

                    if base_grid[r, c] == 100:
                        marker.colors.append(obstacle_color)
                    else:
                        marker.colors.append(inflated_color)

        self.wall_viz_pub.publish(marker)

    def publish_plant_markers(self):
        """
        Publishes the known plant locations as green dotted points.
        Each point has diameter 0.2 m, giving radius 0.1 m.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "plant_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        diameter = 2.0 * self.plant_radius
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.05

        plant_color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=1.0)

        for _, px, py in self.plants:
            p = Point()
            p.x = float(px)
            p.y = float(py)
            p.z = 0.10

            marker.points.append(p)
            marker.colors.append(plant_color)

        self.plant_viz_pub.publish(marker)

        # Publish text labels P1-P6
        for i, (name, px, py) in enumerate(self.plants):
            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "plant_labels"
            label.id = i + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = float(px)
            label.pose.position.y = float(py)
            label.pose.position.z = 0.35

            label.scale.z = 0.25
            label.color = ColorRGBA(r=0.0, g=0.6, b=0.0, a=1.0)
            label.text = name

            self.plant_label_pub.publish(label)

    # ==========================================================
    # Main publisher
    # ==========================================================

    def publish_all(self):
        base_grid = self.create_base_map()
        c_grid = self.apply_c_space(base_grid)

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height

        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = c_grid.flatten().tolist()

        self.map_pub.publish(map_msg)
        self.publish_viz_markers(base_grid, c_grid)
        self.publish_plant_markers()

        self.start_pub.publish(
            self.create_pose_msg(self.start_x, self.start_y, self.start_yaw, map_msg.header)
        )

        self.goal_pub.publish(
            self.create_pose_msg(self.goal_x, self.goal_y, self.goal_yaw, map_msg.header)
        )

    def create_pose_msg(self, x, y, yaw, header):
        ps = PoseStamped()
        ps.header = header

        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0

        q = self.euler_to_quaternion(yaw)
        ps.pose.orientation.x = q['x']
        ps.pose.orientation.y = q['y']
        ps.pose.orientation.z = q['z']
        ps.pose.orientation.w = q['w']

        return ps


def main(args=None):
    rclpy.init(args=args)
    node = MapServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
