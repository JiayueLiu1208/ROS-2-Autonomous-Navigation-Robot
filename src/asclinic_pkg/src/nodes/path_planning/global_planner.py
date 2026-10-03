#!/usr/bin/env python3

import heapq
import math
import time
from dataclasses import dataclass
from itertools import permutations

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from asclinic_pkg.msg import MissionProgress
from final_demo_layout import COVERAGE_STOPS, PLANTS, ROUTE_WAYPOINTS, STOP_POINTS


START_PREFERRED_STOP_ID = 'P5'


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def set_yaw(pose: Pose, yaw: float) -> None:
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


def distance_point_to_segment(
    px: float,
    py: float,
    sx: float,
    sy: float,
    ex: float,
    ey: float,
) -> float:
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(px - sx, py - sy)
    ratio = ((px - sx) * dx + (py - sy) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    cx = sx + ratio * dx
    cy = sy + ratio * dy
    return math.hypot(px - cx, py - cy)


def distance_to_polyline(x: float, y: float, points) -> float:
    if len(points) < 2:
        return float('inf')
    best = float('inf')
    for start, end in zip(points, points[1:]):
        best = min(
            best,
            distance_point_to_segment(
                x,
                y,
                float(start[0]),
                float(start[1]),
                float(end[0]),
                float(end[1]),
            ),
        )
    return best


@dataclass
class AStarResult:
    points: list[tuple[float, float, float]]
    cost: float


class HeadingAStar:
    """8-connected grid A* with heading/turn cost and optional path bias."""

    DIRECTIONS = (
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
        (-1, 0),
        (-1, 1),
    )

    def __init__(
        self,
        map_info,
        static_grid: np.ndarray,
        *,
        explored_grid: np.ndarray | None,
        fog_path_preference: float,
        dynamic_mask: np.ndarray | None,
        blocked_cost_threshold: int,
        rotation_cost_per_90deg_m: float,
        turn_start_cost_m: float,
        soft_cost_weight_m: float,
        obstacle_clearance_radius_m: float,
        obstacle_clearance_weight_m: float,
        hard_obstacle_clearance_m: float,
        path_step_m: float,
        current_path_bias_m: float,
        current_path_bias_radius_m: float,
        start_heading_bias_m: float,
        start_heading_bias_distance_m: float,
        max_nearest_free_radius_m: float,
    ) -> None:
        self.info = map_info
        self.static_grid = static_grid
        self.explored_grid = (
            explored_grid
            if explored_grid is not None and explored_grid.shape == static_grid.shape
            else None
        )
        self.fog_path_preference = max(1.0, float(fog_path_preference))
        self.unexplored_step_cost_scale = 1.0 / self.fog_path_preference
        self.dynamic_mask = dynamic_mask
        self.blocked_cost_threshold = int(blocked_cost_threshold)
        self.rotation_cost_per_rad = float(rotation_cost_per_90deg_m) / (0.5 * math.pi)
        self.turn_start_cost_m = max(0.0, float(turn_start_cost_m))
        self.soft_cost_weight_m = max(0.0, float(soft_cost_weight_m))
        self.obstacle_clearance_radius_m = max(
            0.0,
            float(obstacle_clearance_radius_m),
        )
        self.obstacle_clearance_weight_m = max(
            0.0,
            float(obstacle_clearance_weight_m),
        )
        self.hard_obstacle_clearance_m = max(0.0, float(hard_obstacle_clearance_m))
        self.path_step_m = max(0.02, float(path_step_m))
        self.current_path_bias_m = max(0.0, float(current_path_bias_m))
        self.current_path_bias_radius_m = max(0.0, float(current_path_bias_radius_m))
        self.start_heading_bias_m = max(0.0, float(start_heading_bias_m))
        self.start_heading_bias_distance_m = max(
            0.0,
            float(start_heading_bias_distance_m),
        )
        self.max_nearest_free_radius_m = max(0.0, float(max_nearest_free_radius_m))
        self.height, self.width = static_grid.shape
        self.resolution = float(map_info.resolution)
        self.origin_x = float(map_info.origin.position.x)
        self.origin_y = float(map_info.origin.position.y)
        self.heading_yaws = tuple(math.atan2(dr, dc) for dr, dc in self.DIRECTIONS)
        self.blocked_mask = self.build_blocked_mask()
        self.clearance_cost_grid = self.build_clearance_cost_grid()

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        *,
        start_yaw: float | None = None,
        goal_yaw: float | None = None,
        path_bias_points: list[tuple[float, float, float]] | None = None,
    ) -> AStarResult | None:
        start_cell = self.nearest_free_cell(start[0], start[1])
        goal_cell = self.nearest_free_cell(goal[0], goal[1])
        if start_cell is None or goal_cell is None:
            return None

        if start_yaw is None:
            start_yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        start_heading = self.heading_index(start_yaw)
        bias_corridor = self.build_bias_corridor(path_bias_points)

        distances = np.full((self.height, self.width, 8), np.inf, dtype=np.float64)
        parents: dict[tuple[int, int, int], tuple[int, int, int]] = {}

        sr, sc = start_cell
        gr, gc = goal_cell
        start_state = (sr, sc, start_heading)
        distances[sr, sc, start_heading] = 0.0
        heap: list[tuple[float, float, int, int, int]] = []
        heapq.heappush(
            heap,
            (self.heuristic(sr, sc, gr, gc), 0.0, sr, sc, start_heading),
        )

        best_goal_state: tuple[int, int, int] | None = None
        best_goal_cost = float('inf')
        max_iterations = max(1000, self.height * self.width * 16)
        iterations = 0

        while heap and iterations < max_iterations:
            estimated_total, cost_so_far, row, col, heading = heapq.heappop(heap)
            iterations += 1
            if cost_so_far > distances[row, col, heading] + 1.0e-9:
                continue
            if estimated_total >= best_goal_cost:
                break

            if row == gr and col == gc:
                total_cost = cost_so_far
                if goal_yaw is not None:
                    total_cost += self.turn_cost(
                        goal_yaw,
                        self.heading_yaws[heading],
                    )
                if total_cost < best_goal_cost:
                    best_goal_cost = total_cost
                    best_goal_state = (row, col, heading)
                continue

            for next_heading, (dr, dc) in enumerate(self.DIRECTIONS):
                nr = row + dr
                nc = col + dc
                if not self.in_bounds(nr, nc) or self.is_blocked(nr, nc):
                    continue

                base_step_cost = math.hypot(dr, dc) * self.resolution
                step_cost = base_step_cost * self.fog_step_cost_scale(nr, nc)
                rotation_cost = self.turn_cost(
                    self.heading_yaws[next_heading],
                    self.heading_yaws[heading],
                )
                map_cost = max(0, int(self.static_grid[nr, nc]))
                soft_cost = (map_cost / 100.0) * self.soft_cost_weight_m * base_step_cost
                clearance_cost = self.clearance_cost_grid[nr, nc] * base_step_cost
                bias_cost = 0.0
                if bias_corridor is not None and not bias_corridor[nr, nc]:
                    bias_cost = self.current_path_bias_m
                start_heading_bias_cost = self.start_heading_bias_cost(
                    sr,
                    sc,
                    nr,
                    nc,
                    next_heading,
                    start_yaw,
                )

                new_cost = (
                    cost_so_far
                    + step_cost
                    + rotation_cost
                    + soft_cost
                    + clearance_cost
                    + bias_cost
                    + start_heading_bias_cost
                )
                if new_cost + 1.0e-9 < distances[nr, nc, next_heading]:
                    distances[nr, nc, next_heading] = new_cost
                    parents[(nr, nc, next_heading)] = (row, col, heading)
                    heapq.heappush(
                        heap,
                        (
                            new_cost + self.heuristic(nr, nc, gr, gc),
                            new_cost,
                            nr,
                            nc,
                            next_heading,
                        ),
                    )

        if best_goal_state is None:
            return None

        cells = self.reconstruct_cells(best_goal_state, parents)
        points = self.cells_to_path(cells, start, goal, start_yaw, goal_yaw)
        return AStarResult(points=points, cost=best_goal_cost)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if not self.in_bounds(row, col):
            return None
        return row, col

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def is_blocked(self, row: int, col: int) -> bool:
        return bool(self.blocked_mask[row, col])

    def fog_step_cost_scale(self, row: int, col: int) -> float:
        if self.explored_grid is None or self.fog_path_preference <= 1.0:
            return 1.0
        if (
            self.static_grid[row, col] < 0
            or self.static_grid[row, col] >= self.blocked_cost_threshold
        ):
            return 1.0
        return self.unexplored_step_cost_scale if self.explored_grid[row, col] < 0 else 1.0

    def build_blocked_mask(self) -> np.ndarray:
        blocked = (self.static_grid < 0) | (self.static_grid >= self.blocked_cost_threshold)
        if self.dynamic_mask is not None:
            blocked = blocked | self.dynamic_mask

        if self.hard_obstacle_clearance_m <= 0.0:
            return blocked

        try:
            from scipy.ndimage import binary_dilation
        except ImportError:
            return blocked

        radius_cells = int(math.ceil(self.hard_obstacle_clearance_m / self.resolution))
        if radius_cells <= 0:
            return blocked

        yy, xx = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
        structure = (xx * xx + yy * yy) <= radius_cells * radius_cells
        return binary_dilation(blocked, structure=structure)

    def build_clearance_cost_grid(self) -> np.ndarray:
        if (
            self.obstacle_clearance_radius_m <= 0.0
            or self.obstacle_clearance_weight_m <= 0.0
        ):
            return np.zeros((self.height, self.width), dtype=np.float32)

        try:
            from scipy.ndimage import distance_transform_edt
        except ImportError:
            return np.zeros((self.height, self.width), dtype=np.float32)

        free_mask = ~self.blocked_mask
        clearance_m = distance_transform_edt(free_mask, sampling=self.resolution)
        normalized = 1.0 - np.clip(clearance_m / self.obstacle_clearance_radius_m, 0.0, 1.0)
        penalty = self.obstacle_clearance_weight_m * normalized * normalized
        penalty[self.blocked_mask] = 0.0
        return penalty.astype(np.float32)

    def nearest_free_cell(self, x: float, y: float) -> tuple[int, int] | None:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        row, col = cell
        if not self.is_blocked(row, col):
            return cell

        max_cells = int(math.ceil(self.max_nearest_free_radius_m / self.resolution))
        for radius in range(1, max_cells + 1):
            best_cell = None
            best_distance = float('inf')
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dr), abs(dc)) != radius:
                        continue
                    nr = row + dr
                    nc = col + dc
                    if not self.in_bounds(nr, nc) or self.is_blocked(nr, nc):
                        continue
                    wx, wy = self.cell_center(nr, nc)
                    distance = math.hypot(wx - x, wy - y)
                    if distance < best_distance:
                        best_distance = distance
                        best_cell = (nr, nc)
            if best_cell is not None:
                return best_cell
        return None

    def heading_index(self, yaw: float) -> int:
        best_index = 0
        best_error = float('inf')
        for index, heading_yaw in enumerate(self.heading_yaws):
            error = abs(wrap_to_pi(yaw - heading_yaw))
            if error < best_error:
                best_index = index
                best_error = error
        return best_index

    def heuristic(self, row: int, col: int, goal_row: int, goal_col: int) -> float:
        return (
            math.hypot(goal_row - row, goal_col - col)
            * self.resolution
            * self.unexplored_step_cost_scale
        )

    def turn_cost(self, new_yaw: float, previous_yaw: float) -> float:
        yaw_delta = abs(wrap_to_pi(new_yaw - previous_yaw))
        if yaw_delta <= 1.0e-6:
            return 0.0
        return self.rotation_cost_per_rad * yaw_delta + self.turn_start_cost_m

    def start_heading_bias_cost(
        self,
        start_row: int,
        start_col: int,
        row: int,
        col: int,
        heading_index: int,
        start_yaw: float,
    ) -> float:
        if (
            self.start_heading_bias_m <= 0.0
            or self.start_heading_bias_distance_m <= 0.0
        ):
            return 0.0

        distance_from_start = (
            math.hypot(row - start_row, col - start_col) * self.resolution
        )
        if distance_from_start > self.start_heading_bias_distance_m:
            return 0.0

        heading_error = abs(wrap_to_pi(self.heading_yaws[heading_index] - start_yaw))
        if heading_error <= 1.0e-6:
            return 0.0

        fade = 1.0 - (distance_from_start / self.start_heading_bias_distance_m)
        normalized_error = min(1.0, heading_error / (0.5 * math.pi))
        return self.start_heading_bias_m * fade * normalized_error

    def build_bias_corridor(
        self,
        path_bias_points: list[tuple[float, float, float]] | None,
    ) -> np.ndarray | None:
        if (
            not path_bias_points
            or self.current_path_bias_m <= 0.0
            or self.current_path_bias_radius_m <= 0.0
        ):
            return None

        corridor = np.zeros((self.height, self.width), dtype=bool)
        radius_cells = int(math.ceil(self.current_path_bias_radius_m / self.resolution))
        for point in path_bias_points:
            cell = self.world_to_cell(float(point[0]), float(point[1]))
            if cell is None:
                continue
            row, col = cell
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    nr = row + dr
                    nc = col + dc
                    if not self.in_bounds(nr, nc):
                        continue
                    if math.hypot(dr, dc) * self.resolution <= self.current_path_bias_radius_m:
                        corridor[nr, nc] = True
        return corridor

    def reconstruct_cells(
        self,
        goal_state: tuple[int, int, int],
        parents: dict[tuple[int, int, int], tuple[int, int, int]],
    ) -> list[tuple[int, int, int]]:
        cells = [goal_state]
        state = goal_state
        while state in parents:
            state = parents[state]
            cells.append(state)
        cells.reverse()
        return cells

    def cells_to_path(
        self,
        cells: list[tuple[int, int, int]],
        start: tuple[float, float],
        goal: tuple[float, float],
        start_yaw: float,
        goal_yaw: float | None,
    ) -> list[tuple[float, float, float]]:
        if not cells:
            final_yaw = goal_yaw if goal_yaw is not None else start_yaw
            return [(start[0], start[1], start_yaw), (goal[0], goal[1], final_yaw)]

        key_points: list[tuple[float, float]] = [(float(start[0]), float(start[1]))]
        previous_direction: tuple[int, int] | None = None
        for index in range(1, len(cells)):
            prev_row, prev_col, _ = cells[index - 1]
            row, col, _ = cells[index]
            direction = (row - prev_row, col - prev_col)
            if previous_direction is not None and direction != previous_direction:
                wx, wy = self.cell_center(prev_row, prev_col)
                key_points.append((wx, wy))
            previous_direction = direction
        key_points.append((float(goal[0]), float(goal[1])))

        dense_points = self.densify_key_points(key_points)
        return self.add_yaws(dense_points, start_yaw, goal_yaw)

    def densify_key_points(
        self,
        key_points: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        if len(key_points) <= 1:
            return key_points

        dense = [key_points[0]]
        for start, end in zip(key_points, key_points[1:]):
            sx, sy = start
            ex, ey = end
            distance = math.hypot(ex - sx, ey - sy)
            steps = max(1, int(math.ceil(distance / self.path_step_m)))
            for step in range(1, steps + 1):
                ratio = step / steps
                dense.append((sx + ratio * (ex - sx), sy + ratio * (ey - sy)))
        return dense

    def add_yaws(
        self,
        points: list[tuple[float, float]],
        start_yaw: float,
        goal_yaw: float | None,
    ) -> list[tuple[float, float, float]]:
        yawed: list[tuple[float, float, float]] = []
        for index, (x, y) in enumerate(points):
            if index + 1 < len(points):
                nx, ny = points[index + 1]
                if math.hypot(nx - x, ny - y) > 1.0e-6:
                    yaw = math.atan2(ny - y, nx - x)
                else:
                    yaw = start_yaw
            elif goal_yaw is not None:
                yaw = goal_yaw
            elif index > 0:
                px, py = points[index - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = start_yaw
            yawed.append((x, y, yaw))
        return yawed


class FastGridPlanner:
    """Static 2D grid search used for fast mission-order cost estimates."""

    DIRECTIONS = HeadingAStar.DIRECTIONS

    def __init__(
        self,
        map_info,
        static_grid: np.ndarray,
        *,
        explored_grid: np.ndarray | None,
        fog_path_preference: float,
        blocked_cost_threshold: int,
        rotation_cost_per_90deg_m: float,
        turn_start_cost_m: float,
        soft_cost_weight_m: float,
        obstacle_clearance_radius_m: float,
        obstacle_clearance_weight_m: float,
        hard_obstacle_clearance_m: float,
        path_step_m: float,
        max_nearest_free_radius_m: float,
    ) -> None:
        self.info = map_info
        self.static_grid = static_grid
        self.explored_grid = (
            explored_grid
            if explored_grid is not None and explored_grid.shape == static_grid.shape
            else None
        )
        self.fog_path_preference = max(1.0, float(fog_path_preference))
        self.unexplored_step_cost_scale = 1.0 / self.fog_path_preference
        self.blocked_cost_threshold = int(blocked_cost_threshold)
        self.rotation_cost_per_rad = float(rotation_cost_per_90deg_m) / (0.5 * math.pi)
        self.turn_start_cost_m = max(0.0, float(turn_start_cost_m))
        self.soft_cost_weight_m = max(0.0, float(soft_cost_weight_m))
        self.obstacle_clearance_radius_m = max(
            0.0,
            float(obstacle_clearance_radius_m),
        )
        self.obstacle_clearance_weight_m = max(
            0.0,
            float(obstacle_clearance_weight_m),
        )
        self.hard_obstacle_clearance_m = max(0.0, float(hard_obstacle_clearance_m))
        self.path_step_m = max(0.02, float(path_step_m))
        self.max_nearest_free_radius_m = max(0.0, float(max_nearest_free_radius_m))
        self.height, self.width = static_grid.shape
        self.resolution = float(map_info.resolution)
        self.origin_x = float(map_info.origin.position.x)
        self.origin_y = float(map_info.origin.position.y)
        self.blocked_mask = self.build_blocked_mask()
        self.clearance_cost_grid = self.build_clearance_cost_grid()

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        *,
        start_yaw: float | None = None,
        goal_yaw: float | None = None,
    ) -> AStarResult | None:
        results = self.plan_many(
            start,
            {'goal': goal},
            start_yaw=start_yaw,
            goal_yaws={'goal': goal_yaw} if goal_yaw is not None else None,
        )
        return results.get('goal')

    def plan_many(
        self,
        start: tuple[float, float],
        goals: dict[str, tuple[float, float]],
        *,
        start_yaw: float | None = None,
        goal_yaws: dict[str, float] | None = None,
    ) -> dict[str, AStarResult | None]:
        results: dict[str, AStarResult | None] = {
            goal_id: None for goal_id in goals
        }
        if not goals:
            return results

        start_cell = self.nearest_free_cell(start[0], start[1])
        if start_cell is None:
            return results

        if start_yaw is None:
            first_goal = next(iter(goals.values()))
            start_yaw = math.atan2(first_goal[1] - start[1], first_goal[0] - start[0])

        target_cells: dict[str, tuple[int, int]] = {}
        target_ids_by_cell: dict[tuple[int, int], list[str]] = {}
        for goal_id, goal_xy in goals.items():
            goal_cell = self.nearest_free_cell(goal_xy[0], goal_xy[1])
            if goal_cell is None:
                continue
            target_cells[goal_id] = goal_cell
            target_ids_by_cell.setdefault(goal_cell, []).append(goal_id)

        remaining = set(target_cells.keys())
        if not remaining:
            return results

        distances = np.full((self.height, self.width), np.inf, dtype=np.float64)
        parents: dict[tuple[int, int], tuple[int, int]] = {}

        sr, sc = start_cell
        distances[sr, sc] = 0.0
        heap: list[tuple[float, int, int]] = [(0.0, sr, sc)]
        max_iterations = max(1000, self.height * self.width * 4)
        iterations = 0

        while heap and remaining and iterations < max_iterations:
            cost_so_far, row, col = heapq.heappop(heap)
            iterations += 1
            if cost_so_far > distances[row, col] + 1.0e-9:
                continue

            for goal_id in target_ids_by_cell.get((row, col), []):
                if goal_id not in remaining:
                    continue
                cells = self.reconstruct_cells((row, col), parents)
                goal_yaw = goal_yaws.get(goal_id) if goal_yaws else None
                total_cost = cost_so_far + self.turn_cost_for_cells(
                    cells,
                    start_yaw,
                    goal_yaw,
                )
                results[goal_id] = AStarResult(
                    points=self.cells_to_path(
                        cells,
                        start,
                        goals[goal_id],
                        start_yaw,
                        goal_yaw,
                    ),
                    cost=total_cost,
                )
                remaining.remove(goal_id)
            if not remaining:
                break

            for dr, dc in self.DIRECTIONS:
                nr = row + dr
                nc = col + dc
                if not self.in_bounds(nr, nc) or self.is_blocked(nr, nc):
                    continue
                base_step_cost = math.hypot(dr, dc) * self.resolution
                step_cost = base_step_cost * self.fog_step_cost_scale(nr, nc)
                map_cost = max(0, int(self.static_grid[nr, nc]))
                soft_cost = (map_cost / 100.0) * self.soft_cost_weight_m * base_step_cost
                clearance_cost = self.clearance_cost_grid[nr, nc] * base_step_cost
                new_cost = cost_so_far + step_cost + soft_cost + clearance_cost
                if new_cost + 1.0e-9 < distances[nr, nc]:
                    distances[nr, nc] = new_cost
                    parents[(nr, nc)] = (row, col)
                    heapq.heappush(heap, (new_cost, nr, nc))

        return results

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if not self.in_bounds(row, col):
            return None
        return row, col

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def is_blocked(self, row: int, col: int) -> bool:
        return bool(self.blocked_mask[row, col])

    def fog_step_cost_scale(self, row: int, col: int) -> float:
        if self.explored_grid is None or self.fog_path_preference <= 1.0:
            return 1.0
        if (
            self.static_grid[row, col] < 0
            or self.static_grid[row, col] >= self.blocked_cost_threshold
        ):
            return 1.0
        return self.unexplored_step_cost_scale if self.explored_grid[row, col] < 0 else 1.0

    def build_blocked_mask(self) -> np.ndarray:
        blocked = (self.static_grid < 0) | (self.static_grid >= self.blocked_cost_threshold)
        if self.hard_obstacle_clearance_m <= 0.0:
            return blocked

        try:
            from scipy.ndimage import binary_dilation
        except ImportError:
            return blocked

        radius_cells = int(math.ceil(self.hard_obstacle_clearance_m / self.resolution))
        if radius_cells <= 0:
            return blocked

        yy, xx = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
        structure = (xx * xx + yy * yy) <= radius_cells * radius_cells
        return binary_dilation(blocked, structure=structure)

    def build_clearance_cost_grid(self) -> np.ndarray:
        if (
            self.obstacle_clearance_radius_m <= 0.0
            or self.obstacle_clearance_weight_m <= 0.0
        ):
            return np.zeros((self.height, self.width), dtype=np.float32)

        try:
            from scipy.ndimage import distance_transform_edt
        except ImportError:
            return np.zeros((self.height, self.width), dtype=np.float32)

        free_mask = ~self.blocked_mask
        clearance_m = distance_transform_edt(free_mask, sampling=self.resolution)
        normalized = 1.0 - np.clip(clearance_m / self.obstacle_clearance_radius_m, 0.0, 1.0)
        penalty = self.obstacle_clearance_weight_m * normalized * normalized
        penalty[self.blocked_mask] = 0.0
        return penalty.astype(np.float32)

    def nearest_free_cell(self, x: float, y: float) -> tuple[int, int] | None:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        row, col = cell
        if not self.is_blocked(row, col):
            return cell

        max_cells = int(math.ceil(self.max_nearest_free_radius_m / self.resolution))
        for radius in range(1, max_cells + 1):
            best_cell = None
            best_distance = float('inf')
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dr), abs(dc)) != radius:
                        continue
                    nr = row + dr
                    nc = col + dc
                    if not self.in_bounds(nr, nc) or self.is_blocked(nr, nc):
                        continue
                    wx, wy = self.cell_center(nr, nc)
                    distance = math.hypot(wx - x, wy - y)
                    if distance < best_distance:
                        best_distance = distance
                        best_cell = (nr, nc)
            if best_cell is not None:
                return best_cell
        return None

    def turn_cost_for_cells(
        self,
        cells: list[tuple[int, int]],
        start_yaw: float,
        goal_yaw: float | None,
    ) -> float:
        if len(cells) < 2:
            if goal_yaw is None:
                return 0.0
            return self.turn_cost(goal_yaw, start_yaw)

        total = 0.0
        previous_yaw = start_yaw
        for previous_cell, cell in zip(cells, cells[1:]):
            prev_row, prev_col = previous_cell
            row, col = cell
            direction_yaw = math.atan2(row - prev_row, col - prev_col)
            total += self.turn_cost(direction_yaw, previous_yaw)
            previous_yaw = direction_yaw

        if goal_yaw is not None:
            total += self.turn_cost(goal_yaw, previous_yaw)
        return total

    def turn_cost(self, new_yaw: float, previous_yaw: float) -> float:
        yaw_delta = abs(wrap_to_pi(new_yaw - previous_yaw))
        if yaw_delta <= 1.0e-6:
            return 0.0
        return self.rotation_cost_per_rad * yaw_delta + self.turn_start_cost_m

    def reconstruct_cells(
        self,
        goal_cell: tuple[int, int],
        parents: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[int, int]]:
        cells = [goal_cell]
        cell = goal_cell
        while cell in parents:
            cell = parents[cell]
            cells.append(cell)
        cells.reverse()
        return cells

    def cells_to_path(
        self,
        cells: list[tuple[int, int]],
        start: tuple[float, float],
        goal: tuple[float, float],
        start_yaw: float,
        goal_yaw: float | None,
    ) -> list[tuple[float, float, float]]:
        if not cells:
            final_yaw = goal_yaw if goal_yaw is not None else start_yaw
            return [(start[0], start[1], start_yaw), (goal[0], goal[1], final_yaw)]

        key_points: list[tuple[float, float]] = [(float(start[0]), float(start[1]))]
        previous_direction: tuple[int, int] | None = None
        for index in range(1, len(cells)):
            prev_row, prev_col = cells[index - 1]
            row, col = cells[index]
            direction = (row - prev_row, col - prev_col)
            if previous_direction is not None and direction != previous_direction:
                wx, wy = self.cell_center(prev_row, prev_col)
                key_points.append((wx, wy))
            previous_direction = direction
        key_points.append((float(goal[0]), float(goal[1])))

        dense_points = self.densify_key_points(key_points)
        return self.add_yaws(dense_points, start_yaw, goal_yaw)

    def densify_key_points(
        self,
        key_points: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        if len(key_points) <= 1:
            return key_points

        dense = [key_points[0]]
        for start, end in zip(key_points, key_points[1:]):
            sx, sy = start
            ex, ey = end
            distance = math.hypot(ex - sx, ey - sy)
            steps = max(1, int(math.ceil(distance / self.path_step_m)))
            for step in range(1, steps + 1):
                ratio = step / steps
                dense.append((sx + ratio * (ex - sx), sy + ratio * (ey - sy)))
        return dense

    def add_yaws(
        self,
        points: list[tuple[float, float]],
        start_yaw: float,
        goal_yaw: float | None,
    ) -> list[tuple[float, float, float]]:
        yawed: list[tuple[float, float, float]] = []
        for index, (x, y) in enumerate(points):
            if index + 1 < len(points):
                nx, ny = points[index + 1]
                if math.hypot(nx - x, ny - y) > 1.0e-6:
                    yaw = math.atan2(ny - y, nx - x)
                else:
                    yaw = start_yaw
            elif goal_yaw is not None:
                yaw = goal_yaw
            elif index > 0:
                px, py = points[index - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = start_yaw
            yawed.append((x, y, yaw))
        return yawed


class DynamicKnownMapMissionPlanner(Node):
    """
    Dynamic known-map mission planner.

    The planner keeps the public interface used by the previous fixed planner:
    it publishes nav_msgs/Path on reference_path and a PoseArray of plant stop
    poses. The route order is optimized with A* pairwise costs and then kept
    stable until the robot leaves the planned mission path by a large margin.
    """

    def __init__(self):
        super().__init__('known_map_path_planner')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('stop_poses_topic', 'plant_stop_poses')
        self.declare_parameter('path_marker_topic', 'known_map_path_marker')
        self.declare_parameter('stop_marker_topic', 'plant_stop_pose_marker')
        self.declare_parameter('dynamic_obstacle_marker_topic', 'dynamic_obstacle_marker')
        self.declare_parameter('explored_map_topic', 'explored_map')
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mission_progress_topic', 'mission_progress')
        self.declare_parameter('planner_period_sec', 0.50)
        self.declare_parameter('path_step_m', 0.08)
        self.declare_parameter('plant_goal_standoff_m', 1.0)
        self.declare_parameter('blocked_cost_threshold', 80)
        self.declare_parameter('soft_cost_weight_m', 2.0)
        self.declare_parameter('obstacle_clearance_radius_m', 0.30)
        self.declare_parameter('obstacle_clearance_weight_m', 3.0)
        self.declare_parameter('hard_obstacle_clearance_m', 0.0)
        self.declare_parameter('fog_path_preference', 1.0)
        self.declare_parameter('fast_global_ordering', True)
        self.declare_parameter('global_order_grid_stride', 1)
        self.declare_parameter('include_coverage_stops', True)
        self.declare_parameter('fog_cleanup_enabled', False)
        self.declare_parameter('fog_cleanup_trigger_ratio', 0.50)
        self.declare_parameter('fog_cleanup_min_cluster_ratio', 0.01)
        self.declare_parameter('fog_cleanup_max_cluster_ratio', 0.10)
        self.declare_parameter('fog_cleanup_max_goals', 1)
        self.declare_parameter('fog_cleanup_goal_prefix', 'F')
        self.declare_parameter('planned_coverage_enabled', False)
        self.declare_parameter('planned_coverage_radius_m', 2.0)
        self.declare_parameter('planned_coverage_use_line_of_sight', True)
        self.declare_parameter('planned_coverage_sample_step_m', 0.25)
        self.declare_parameter('planned_coverage_ray_angle_step_deg', 6.0)
        self.declare_parameter('planned_coverage_min_cluster_area_m2', 0.60)
        self.declare_parameter('planned_coverage_max_goals', 2)
        self.declare_parameter('planned_coverage_goal_tolerance_m', 0.25)
        self.declare_parameter('planned_coverage_goal_prefix', 'X')
        self.declare_parameter('max_exact_order_stops', 6)
        self.declare_parameter('order_deviation_threshold_m', 5.0)
        self.declare_parameter('active_path_deviation_threshold_m', 0.75)
        self.declare_parameter('rotation_cost_per_90deg_m', 0.3)
        self.declare_parameter('turn_start_cost_m', 0.25)
        self.declare_parameter('current_path_bias_m', 0.02)
        self.declare_parameter('current_path_bias_radius_m', 0.30)
        self.declare_parameter('start_heading_bias_m', 0.15)
        self.declare_parameter('start_heading_bias_distance_m', 0.70)
        self.declare_parameter('dynamic_obstacle_ttl_sec', 1.0)
        self.declare_parameter('dynamic_obstacle_inflation_m', 0.20)
        self.declare_parameter('dynamic_obstacle_max_range_m', 4.0)
        self.declare_parameter('dynamic_obstacle_path_margin_m', 0.20)
        self.declare_parameter('dynamic_obstacle_static_filter_m', 0.0)
        self.declare_parameter('dynamic_obstacle_beam_stride', 4)
        self.declare_parameter('dynamic_obstacle_max_points', 1500)
        self.declare_parameter('dynamic_obstacle_marker_latest_only', True)
        self.declare_parameter('replan_cooldown_sec', 2.0)
        self.declare_parameter('max_nearest_free_radius_m', 0.60)
        self.declare_parameter('lidar_in_base_x', 0.0)
        self.declare_parameter('lidar_in_base_y', 0.0)
        self.declare_parameter('lidar_in_base_yaw', 0.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)
        self.declare_parameter('lidar_scan_angle_multiplier', 1.0)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.stop_poses_topic = str(self.get_parameter('stop_poses_topic').value)
        self.path_marker_topic = str(self.get_parameter('path_marker_topic').value)
        self.stop_marker_topic = str(self.get_parameter('stop_marker_topic').value)
        self.dynamic_obstacle_marker_topic = str(
            self.get_parameter('dynamic_obstacle_marker_topic').value
        )
        self.explored_map_topic = str(
            self.get_parameter('explored_map_topic').value
        ).strip()
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.mission_progress_topic = str(
            self.get_parameter('mission_progress_topic').value
        )

        self.planner_period_sec = max(
            0.10,
            float(self.get_parameter('planner_period_sec').value),
        )
        self.path_step_m = max(0.02, float(self.get_parameter('path_step_m').value))
        self.plant_goal_standoff_m = max(
            0.0,
            float(self.get_parameter('plant_goal_standoff_m').value),
        )
        self.blocked_cost_threshold = int(self.get_parameter('blocked_cost_threshold').value)
        self.soft_cost_weight_m = max(
            0.0,
            float(self.get_parameter('soft_cost_weight_m').value),
        )
        self.obstacle_clearance_radius_m = max(
            0.0,
            float(self.get_parameter('obstacle_clearance_radius_m').value),
        )
        self.obstacle_clearance_weight_m = max(
            0.0,
            float(self.get_parameter('obstacle_clearance_weight_m').value),
        )
        self.hard_obstacle_clearance_m = max(
            0.0,
            float(self.get_parameter('hard_obstacle_clearance_m').value),
        )
        self.fog_path_preference = max(
            1.0,
            float(self.get_parameter('fog_path_preference').value),
        )
        self.fast_global_ordering = as_bool(
            self.get_parameter('fast_global_ordering').value
        )
        self.global_order_grid_stride = max(
            1,
            int(self.get_parameter('global_order_grid_stride').value),
        )
        self.include_coverage_stops = as_bool(
            self.get_parameter('include_coverage_stops').value
        )
        self.fog_cleanup_enabled = as_bool(
            self.get_parameter('fog_cleanup_enabled').value
        )
        self.fog_cleanup_trigger_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('fog_cleanup_trigger_ratio').value)),
        )
        self.fog_cleanup_min_cluster_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('fog_cleanup_min_cluster_ratio').value)),
        )
        self.fog_cleanup_max_cluster_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('fog_cleanup_max_cluster_ratio').value)),
        )
        if self.fog_cleanup_max_cluster_ratio < self.fog_cleanup_min_cluster_ratio:
            self.fog_cleanup_max_cluster_ratio = self.fog_cleanup_min_cluster_ratio
        self.fog_cleanup_max_goals = max(
            0,
            int(self.get_parameter('fog_cleanup_max_goals').value),
        )
        self.fog_cleanup_goal_prefix = (
            str(self.get_parameter('fog_cleanup_goal_prefix').value).strip() or 'F'
        )
        self.planned_coverage_enabled = as_bool(
            self.get_parameter('planned_coverage_enabled').value
        )
        self.planned_coverage_radius_m = max(
            0.0,
            float(self.get_parameter('planned_coverage_radius_m').value),
        )
        self.planned_coverage_use_line_of_sight = as_bool(
            self.get_parameter('planned_coverage_use_line_of_sight').value
        )
        self.planned_coverage_sample_step_m = max(
            0.05,
            float(self.get_parameter('planned_coverage_sample_step_m').value),
        )
        self.planned_coverage_ray_angle_step = math.radians(
            max(
                1.0,
                float(self.get_parameter('planned_coverage_ray_angle_step_deg').value),
            )
        )
        self.planned_coverage_min_cluster_area_m2 = max(
            0.0,
            float(self.get_parameter('planned_coverage_min_cluster_area_m2').value),
        )
        self.planned_coverage_max_goals = max(
            0,
            int(self.get_parameter('planned_coverage_max_goals').value),
        )
        self.planned_coverage_goal_tolerance_m = max(
            0.05,
            float(self.get_parameter('planned_coverage_goal_tolerance_m').value),
        )
        self.planned_coverage_goal_prefix = (
            str(self.get_parameter('planned_coverage_goal_prefix').value).strip() or 'X'
        )
        self.max_exact_order_stops = max(
            2,
            int(self.get_parameter('max_exact_order_stops').value),
        )
        self.order_deviation_threshold_m = max(
            0.0,
            float(self.get_parameter('order_deviation_threshold_m').value),
        )
        self.active_path_deviation_threshold_m = max(
            0.0,
            float(self.get_parameter('active_path_deviation_threshold_m').value),
        )
        self.rotation_cost_per_90deg_m = max(
            0.0,
            float(self.get_parameter('rotation_cost_per_90deg_m').value),
        )
        self.turn_start_cost_m = max(
            0.0,
            float(self.get_parameter('turn_start_cost_m').value),
        )
        self.current_path_bias_m = max(
            0.0,
            float(self.get_parameter('current_path_bias_m').value),
        )
        self.current_path_bias_radius_m = max(
            0.0,
            float(self.get_parameter('current_path_bias_radius_m').value),
        )
        self.start_heading_bias_m = max(
            0.0,
            float(self.get_parameter('start_heading_bias_m').value),
        )
        self.start_heading_bias_distance_m = max(
            0.0,
            float(self.get_parameter('start_heading_bias_distance_m').value),
        )
        self.dynamic_obstacle_ttl_sec = max(
            0.0,
            float(self.get_parameter('dynamic_obstacle_ttl_sec').value),
        )
        self.dynamic_obstacle_inflation_m = max(
            0.0,
            float(self.get_parameter('dynamic_obstacle_inflation_m').value),
        )
        self.dynamic_obstacle_max_range_m = max(
            0.0,
            float(self.get_parameter('dynamic_obstacle_max_range_m').value),
        )
        self.dynamic_obstacle_path_margin_m = max(
            0.0,
            float(self.get_parameter('dynamic_obstacle_path_margin_m').value),
        )
        self.dynamic_obstacle_static_filter_m = max(
            0.0,
            float(self.get_parameter('dynamic_obstacle_static_filter_m').value),
        )
        self.dynamic_obstacle_beam_stride = max(
            1,
            int(self.get_parameter('dynamic_obstacle_beam_stride').value),
        )
        self.dynamic_obstacle_max_points = max(
            100,
            int(self.get_parameter('dynamic_obstacle_max_points').value),
        )
        self.dynamic_obstacle_marker_latest_only = as_bool(
            self.get_parameter('dynamic_obstacle_marker_latest_only').value
        )
        self.replan_cooldown_sec = max(
            0.0,
            float(self.get_parameter('replan_cooldown_sec').value),
        )
        self.max_nearest_free_radius_m = max(
            0.0,
            float(self.get_parameter('max_nearest_free_radius_m').value),
        )
        self.lidar_in_base_x = float(self.get_parameter('lidar_in_base_x').value)
        self.lidar_in_base_y = float(self.get_parameter('lidar_in_base_y').value)
        self.lidar_in_base_yaw = (
            float(self.get_parameter('lidar_in_base_yaw').value)
            + math.radians(float(self.get_parameter('lidar_in_base_yaw_deg').value))
        )
        self.lidar_scan_angle_multiplier = float(
            self.get_parameter('lidar_scan_angle_multiplier').value
        )

        self.frame_id = 'map'
        self.plants = {plant_id: (x, y) for plant_id, x, y in PLANTS}
        self.coverage_stops = list(COVERAGE_STOPS) if self.include_coverage_stops else []
        self.base_stop_points = [
            (plant_id, float(x), float(y))
            for plant_id, x, y in STOP_POINTS
        ] + [
            (stop_id, float(x), float(y))
            for stop_id, x, y, _yaw_deg in self.coverage_stops
        ]
        self.base_stop_order_ids = [stop_id for stop_id, _, _ in self.base_stop_points]
        self.base_stop_yaw_by_id = {
            plant_id: self.yaw_towards_plant(plant_id, x, y)
            for plant_id, x, y in self.base_stop_points
            if plant_id in self.plants
        }
        self.base_stop_yaw_by_id.update(
            {
                stop_id: math.radians(float(yaw_deg))
                for stop_id, _x, _y, yaw_deg in self.coverage_stops
            }
        )
        self.generated_exploration_stop_points: list[
            tuple[str, float, float, float]
        ] = []
        self.generated_exploration_ids: set[str] = set()
        self.completed_exploration_stop_ids: set[str] = set()
        self.planned_coverage_signature: tuple | None = None
        self.fog_cleanup_signature: tuple | None = None
        self.stop_points: list[tuple[str, float, float]] = []
        self.stop_order_ids: list[str] = []
        self.stop_xy_by_id: dict[str, tuple[float, float]] = {}
        self.planner_goal_xy_by_id: dict[str, tuple[float, float]] = {}
        self.calculated_stop_xy_by_id: dict[str, tuple[float, float]] = {}
        self.stop_yaw_by_id: dict[str, float] = {}
        self.refresh_stop_points()
        _, home_x, home_y = ROUTE_WAYPOINTS[0]
        self.home_xy = (float(home_x), float(home_y))
        self.home_yaw = 0.0

        self.map_info = None
        self.map_data: np.ndarray | None = None
        self.map_revision = 0
        self.explored_map_info = None
        self.explored_map_data: np.ndarray | None = None
        self.explored_revision = 0
        self.latest_pose: tuple[float, float, float] | None = None
        self.completed_stop_ids: set[str] = set()
        self.progress_current_stop_id = ''
        self.progress_dwell_active = False
        self.progress_mission_complete = False

        self.dynamic_obstacle_points: list[tuple[float, float, float]] = []
        self.latest_scan_obstacle_points: list[tuple[float, float, float]] = []
        self.dynamic_obstacle_revision = 0
        self.planned_dynamic_revision = -1
        self.planned_map_revision = -1
        self.planned_explored_revision = -1
        self.last_replan_time = 0.0
        self.last_order_log_time = 0.0
        self.last_wait_log_time = 0.0

        self.mission_order: list[str] = []
        self.ordered_mission_path: list[tuple[float, float, float]] = []
        self.active_target_id = ''
        self.active_path: list[tuple[float, float, float]] = []
        self.last_accepted_path: list[tuple[float, float, float]] = []
        self.published_once = False

        self.static_pair_cache: dict[tuple[str, str], AStarResult | None] = {}
        self.static_home_cache: dict[str, AStarResult | None] = {}
        self.static_order_planner: FastGridPlanner | None = None
        self.static_order_planner_revision = -1

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )
        self.explored_map_sub = None
        if self.explored_map_topic:
            self.explored_map_sub = self.create_subscription(
                OccupancyGrid,
                self.explored_map_topic,
                self.explored_map_callback,
                10,
            )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            reliable_qos,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.progress_sub = self.create_subscription(
            MissionProgress,
            self.mission_progress_topic,
            self.mission_progress_callback,
            reliable_qos,
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.stop_pub = self.create_publisher(PoseArray, self.stop_poses_topic, 10)
        self.path_marker_pub = self.create_publisher(Marker, self.path_marker_topic, 10)
        self.stop_marker_pub = self.create_publisher(Marker, self.stop_marker_topic, 10)
        self.dynamic_obstacle_marker_pub = self.create_publisher(
            Marker,
            self.dynamic_obstacle_marker_topic,
            10,
        )

        self.publish_timer = self.create_timer(
            self.planner_period_sec,
            self.planner_tick,
        )

        self.get_logger().info(
            '[DYNAMIC PLANNER] map=%s explored=%s odom=%s scan=%s path=%s progress=%s'
            % (
                self.map_topic,
                self.explored_map_topic or '<disabled>',
                self.odom_topic,
                self.scan_topic,
                self.path_topic,
                self.mission_progress_topic,
            )
        )
        self.get_logger().info(
            '[DYNAMIC PLANNER] order freeze threshold=%.2fm turn cost=%.2fm/90deg start=%.2fm'
            % (
                self.order_deviation_threshold_m,
                self.rotation_cost_per_90deg_m,
                self.turn_start_cost_m,
            )
        )
        self.get_logger().info(
            '[DYNAMIC PLANNER] start heading bias=%.2fm over %.2fm'
            % (self.start_heading_bias_m, self.start_heading_bias_distance_m)
        )
        self.get_logger().info(
            '[DYNAMIC PLANNER] plant goals use plant locations; published paths stop %.2fm before each plant.'
            % self.plant_goal_standoff_m
        )
        self.get_logger().info(
            '[DYNAMIC PLANNER] obstacle clearance radius=%.2fm weight=%.2f hard=%.2fm'
            % (
                self.obstacle_clearance_radius_m,
                self.obstacle_clearance_weight_m,
                self.hard_obstacle_clearance_m,
            )
        )
        if self.fog_path_preference > 1.0 and self.explored_map_topic:
            self.get_logger().info(
                '[DYNAMIC PLANNER] fog path preference=%.2fx; unexplored free cells cost %.2fx normal step.'
                % (
                    self.fog_path_preference,
                    1.0 / self.fog_path_preference,
                )
            )
        self.get_logger().info(
            '[DYNAMIC PLANNER] global order solver=%s'
            % (
                'fast 2D static planner stride=%d' % self.global_order_grid_stride
                if self.fast_global_ordering
                else 'heading A*'
            )
        )
        if self.planned_coverage_enabled:
            self.get_logger().info(
                '[DYNAMIC PLANNER] planned coverage enabled radius=%.2fm los=%s max_goals=%d min_cluster=%.2fm^2'
                % (
                    self.planned_coverage_radius_m,
                    self.planned_coverage_use_line_of_sight,
                    self.planned_coverage_max_goals,
                    self.planned_coverage_min_cluster_area_m2,
                )
            )
        if self.fog_cleanup_enabled:
            self.get_logger().info(
                '[DYNAMIC PLANNER] fog cleanup waypoints enabled trigger<%.0f%% cluster=%.0f%%..%.0f%% max_goals=%d'
                % (
                    100.0 * self.fog_cleanup_trigger_ratio,
                    100.0 * self.fog_cleanup_min_cluster_ratio,
                    100.0 * self.fog_cleanup_max_cluster_ratio,
                    self.fog_cleanup_max_goals,
                )
            )

    def yaw_towards_plant(self, plant_id: str, stop_x: float, stop_y: float) -> float:
        plant_x, plant_y = self.plants[plant_id]
        return math.atan2(plant_y - stop_y, plant_x - stop_x)

    def refresh_stop_points(self) -> None:
        self.stop_points = list(self.base_stop_points) + [
            (stop_id, float(x), float(y))
            for stop_id, x, y, _yaw in self.generated_exploration_stop_points
        ]
        self.stop_order_ids = [stop_id for stop_id, _, _ in self.stop_points]
        self.stop_xy_by_id = {
            stop_id: (float(x), float(y)) for stop_id, x, y in self.stop_points
        }
        self.planner_goal_xy_by_id = {
            stop_id: self.plants.get(stop_id, (float(x), float(y)))
            for stop_id, x, y in self.stop_points
        }
        self.stop_yaw_by_id = dict(self.base_stop_yaw_by_id)
        self.stop_yaw_by_id.update(
            {
                stop_id: float(yaw)
                for stop_id, _x, _y, yaw in self.generated_exploration_stop_points
            }
        )
        self.calculated_stop_xy_by_id = {
            stop_id: xy
            for stop_id, xy in self.calculated_stop_xy_by_id.items()
            if stop_id in self.stop_order_ids
        }

    def planning_goal_xy(self, stop_id: str) -> tuple[float, float]:
        if stop_id in self.planner_goal_xy_by_id:
            return self.planner_goal_xy_by_id[stop_id]
        return self.stop_xy_by_id[stop_id]

    def planning_goal_yaw(self, stop_id: str) -> float | None:
        if stop_id in self.plants and self.plant_goal_standoff_m > 0.0:
            return None
        return self.stop_yaw_by_id.get(stop_id)

    def adjust_result_for_target(
        self,
        target_id: str,
        result: AStarResult | None,
        *,
        remember_stop: bool = False,
    ) -> AStarResult | None:
        if result is None:
            return None
        if target_id not in self.plants or self.plant_goal_standoff_m <= 0.0:
            if remember_stop and result.points:
                x, y, _yaw = result.points[-1]
                self.calculated_stop_xy_by_id[target_id] = (float(x), float(y))
            return result
        adjusted = AStarResult(
            points=self.trim_path_before_plant(target_id, result.points),
            cost=result.cost,
        )
        if remember_stop and adjusted.points:
            x, y, _yaw = adjusted.points[-1]
            self.calculated_stop_xy_by_id[target_id] = (float(x), float(y))
        return adjusted

    def display_stop_points(self) -> list[tuple[str, float, float]]:
        return [
            (
                stop_id,
                *self.calculated_stop_xy_by_id.get(stop_id, (float(x), float(y))),
            )
            for stop_id, x, y in self.stop_points
        ]

    def display_stop_yaw(self, stop_id: str, x: float, y: float) -> float:
        if stop_id in self.plants:
            plant_x, plant_y = self.plants[stop_id]
            return math.atan2(plant_y - y, plant_x - x)
        return self.stop_yaw_by_id[stop_id]

    def remember_calculated_stop(
        self,
        target_id: str,
        result: AStarResult | None,
    ) -> None:
        if result is None or not result.points or target_id not in self.stop_order_ids:
            return
        x, y, _yaw = result.points[-1]
        self.calculated_stop_xy_by_id[target_id] = (float(x), float(y))

    def trim_path_before_plant(
        self,
        plant_id: str,
        points: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        if not points:
            return []

        plant_x, plant_y = self.plants[plant_id]
        standoff = self.plant_goal_standoff_m
        if standoff <= 1.0e-6:
            final = list(points)
            x, y, _yaw = final[-1]
            final[-1] = (x, y, math.atan2(plant_y - y, plant_x - x))
            return final

        distance_from_goal = 0.0
        for index in range(len(points) - 1, 0, -1):
            upstream_x, upstream_y, _upstream_yaw = points[index - 1]
            downstream_x, downstream_y, _downstream_yaw = points[index]
            segment_length = math.hypot(
                downstream_x - upstream_x,
                downstream_y - upstream_y,
            )
            if segment_length <= 1.0e-9:
                continue

            if distance_from_goal + segment_length >= standoff:
                backtrack = standoff - distance_from_goal
                ratio = max(0.0, min(1.0, backtrack / segment_length))
                stop_x = downstream_x + ratio * (upstream_x - downstream_x)
                stop_y = downstream_y + ratio * (upstream_y - downstream_y)
                stop_yaw = math.atan2(plant_y - stop_y, plant_x - stop_x)

                trimmed = list(points[:index])
                if trimmed:
                    last_x, last_y, _last_yaw = trimmed[-1]
                    if math.hypot(stop_x - last_x, stop_y - last_y) <= 1.0e-6:
                        trimmed[-1] = (stop_x, stop_y, stop_yaw)
                    else:
                        trimmed.append((stop_x, stop_y, stop_yaw))
                else:
                    trimmed.append((stop_x, stop_y, stop_yaw))
                return trimmed

            distance_from_goal += segment_length

        start_x, start_y, _start_yaw = points[0]
        return [(start_x, start_y, math.atan2(plant_y - start_y, plant_x - start_x))]

    def completed_mission_stop_ids(self) -> set[str]:
        return set(self.completed_stop_ids) | set(self.completed_exploration_stop_ids)

    def map_callback(self, msg: OccupancyGrid) -> None:
        new_grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        map_changed = (
            self.map_info is None
            or self.map_data is None
            or msg.info.width != self.map_info.width
            or msg.info.height != self.map_info.height
            or abs(msg.info.resolution - self.map_info.resolution) > 1.0e-9
            or abs(msg.info.origin.position.x - self.map_info.origin.position.x) > 1.0e-9
            or abs(msg.info.origin.position.y - self.map_info.origin.position.y) > 1.0e-9
            or not np.array_equal(new_grid, self.map_data)
        )
        self.map_info = msg.info
        self.map_data = new_grid
        if map_changed:
            self.map_revision += 1
            self.static_pair_cache.clear()
            self.static_home_cache.clear()
            self.static_order_planner = None
            self.static_order_planner_revision = -1
            self.ordered_mission_path = []
            self.planned_coverage_signature = None
            self.fog_cleanup_signature = None
            if (
                (self.planned_coverage_enabled or self.fog_cleanup_enabled)
                and self.generated_exploration_ids
            ):
                self.set_generated_exploration_goals([])

    def explored_map_callback(self, msg: OccupancyGrid) -> None:
        new_grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        explored_changed = (
            self.explored_map_info is None
            or self.explored_map_data is None
            or msg.info.width != self.explored_map_info.width
            or msg.info.height != self.explored_map_info.height
            or abs(msg.info.resolution - self.explored_map_info.resolution) > 1.0e-9
            or abs(msg.info.origin.position.x - self.explored_map_info.origin.position.x) > 1.0e-9
            or abs(msg.info.origin.position.y - self.explored_map_info.origin.position.y) > 1.0e-9
            or not np.array_equal(new_grid, self.explored_map_data)
        )
        self.explored_map_info = msg.info
        self.explored_map_data = new_grid
        if explored_changed:
            self.explored_revision += 1
            self.static_pair_cache.clear()
            self.static_home_cache.clear()
            self.static_order_planner = None
            self.static_order_planner_revision = -1
            self.fog_cleanup_signature = None

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def mission_progress_callback(self, msg: MissionProgress) -> None:
        new_completed = set(str(stop_id) for stop_id in msg.completed_stop_ids)
        if new_completed != self.completed_stop_ids:
            completed_text = ','.join(sorted(new_completed)) or '<none>'
            self.get_logger().info(
                '[DYNAMIC PLANNER] Mission progress completed=[%s]' % completed_text
            )
            self.completed_stop_ids = new_completed
            completed = self.completed_mission_stop_ids()
            self.mission_order = [
                stop_id
                for stop_id in self.mission_order
                if stop_id not in completed
            ]
            if self.active_target_id in completed:
                self.active_target_id = ''
                self.active_path = []
        self.progress_current_stop_id = str(msg.current_stop_id)
        self.progress_dwell_active = bool(msg.dwell_active)
        self.progress_mission_complete = bool(msg.mission_complete)

    def scan_callback(self, msg: LaserScan) -> None:
        if self.latest_pose is None or self.dynamic_obstacle_ttl_sec <= 0.0:
            return

        robot_x, robot_y, robot_yaw = self.latest_pose
        now = time.monotonic()
        max_range = msg.range_max if msg.range_max > 0.0 else self.dynamic_obstacle_max_range_m
        if self.dynamic_obstacle_max_range_m > 0.0:
            max_range = min(max_range, self.dynamic_obstacle_max_range_m)

        new_points: list[tuple[float, float, float]] = []
        for index, distance in enumerate(msg.ranges):
            if index % self.dynamic_obstacle_beam_stride != 0:
                continue
            if not math.isfinite(distance):
                continue
            if distance <= max(0.0, msg.range_min) or distance > max_range:
                continue

            scan_angle = self.lidar_scan_angle_multiplier * (
                msg.angle_min + index * msg.angle_increment
            )
            # This LiDAR is mounted with its scan +x axis facing robot-backward.
            sensor_x = -distance * math.cos(scan_angle)
            sensor_y = distance * math.sin(scan_angle)
            local_x = (
                self.lidar_in_base_x
                + math.cos(self.lidar_in_base_yaw) * sensor_x
                - math.sin(self.lidar_in_base_yaw) * sensor_y
            )
            local_y = (
                self.lidar_in_base_y
                + math.sin(self.lidar_in_base_yaw) * sensor_x
                + math.cos(self.lidar_in_base_yaw) * sensor_y
            )
            world_x = robot_x + math.cos(robot_yaw) * local_x - math.sin(robot_yaw) * local_y
            world_y = robot_y + math.sin(robot_yaw) * local_x + math.cos(robot_yaw) * local_y
            if self.is_known_static_obstacle_hit(world_x, world_y):
                continue
            new_points.append((world_x, world_y, now))

        self.latest_scan_obstacle_points = new_points
        if new_points:
            self.prune_dynamic_obstacles(now)
            self.dynamic_obstacle_points.extend(new_points)
            if len(self.dynamic_obstacle_points) > self.dynamic_obstacle_max_points:
                self.dynamic_obstacle_points = self.dynamic_obstacle_points[
                    -self.dynamic_obstacle_max_points:
                ]
            self.dynamic_obstacle_revision += 1

    def is_known_static_obstacle_hit(self, x: float, y: float) -> bool:
        if self.dynamic_obstacle_static_filter_m <= 0.0:
            return False
        if self.map_info is None or self.map_data is None:
            return False

        resolution = float(self.map_info.resolution)
        origin_x = float(self.map_info.origin.position.x)
        origin_y = float(self.map_info.origin.position.y)
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        height, width = self.map_data.shape
        if not (0 <= row < height and 0 <= col < width):
            return True

        radius_cells = int(math.ceil(self.dynamic_obstacle_static_filter_m / resolution))
        row_start = max(0, row - radius_cells)
        row_end = min(height, row + radius_cells + 1)
        col_start = max(0, col - radius_cells)
        col_end = min(width, col + radius_cells + 1)
        window = self.map_data[row_start:row_end, col_start:col_end]
        return bool(np.any((window < 0) | (window >= self.blocked_cost_threshold)))

    def planner_tick(self) -> None:
        self.publish_stop_poses()
        self.prune_dynamic_obstacles(time.monotonic())
        self.publish_dynamic_obstacle_marker()

        if self.map_info is None or self.map_data is None or self.latest_pose is None:
            self.log_waiting_for_inputs()
            return

        if self.progress_mission_complete and self.active_target_id == 'HOME':
            return

        self.update_exploration_progress()

        if not self.ensure_mission_order():
            return

        target_id = self.next_target_id()
        if not target_id:
            return

        now = time.monotonic()
        should_replan, reason = self.should_replan_active_path(target_id, now)
        if should_replan:
            self.replan_active_path(target_id, reason, now)

        if self.active_path:
            self.publish_active_path()

    def log_waiting_for_inputs(self) -> None:
        now = time.monotonic()
        if now - self.last_wait_log_time < 2.0:
            return
        self.last_wait_log_time = now
        missing = []
        if self.map_info is None or self.map_data is None:
            missing.append(self.map_topic)
        if self.latest_pose is None:
            missing.append(self.odom_topic)
        self.get_logger().info(
            '[DYNAMIC PLANNER] Waiting for %s.' % ', '.join(missing)
        )

    def remaining_stop_ids(self) -> list[str]:
        completed = self.completed_mission_stop_ids()
        return [
            stop_id
            for stop_id in self.stop_order_ids
            if stop_id not in completed
        ]

    def ensure_mission_order(self) -> bool:
        if self.fog_cleanup_enabled:
            self.refresh_fog_cleanup_goals()
        elif self.planned_coverage_enabled:
            self.refresh_planned_coverage_goals()

        remaining = self.remaining_stop_ids()
        if not remaining:
            self.mission_order = []
            return True

        self.mission_order = [
            stop_id for stop_id in self.mission_order if stop_id in remaining
        ]

        need_reorder = not self.mission_order
        if self.ordered_mission_path and self.latest_pose is not None:
            rx, ry, _ = self.latest_pose
            deviation = distance_to_polyline(rx, ry, self.ordered_mission_path)
            if deviation > self.order_deviation_threshold_m:
                need_reorder = True
        elif not self.ordered_mission_path:
            need_reorder = True

        if not need_reorder:
            return True

        return self.compute_mission_order(remaining)

    def compute_mission_order(self, remaining: list[str]) -> bool:
        if self.latest_pose is None:
            return False

        start_time = time.monotonic()
        current_x, current_y, current_yaw = self.latest_pose
        current_to_stop = self.current_to_stop_costs(
            remaining,
            current_x,
            current_y,
            current_yaw,
        )

        if not current_to_stop:
            return False

        self.precompute_static_order_costs(remaining)

        preferred_start_id = self.start_preferred_stop_id(remaining, current_to_stop)
        best_order, best_cost = self.find_best_order(
            remaining,
            current_to_stop,
            preferred_start_id=preferred_start_id,
        )

        if best_order is None:
            self.get_logger().error('[DYNAMIC PLANNER] Could not find a feasible stop order.')
            return False

        self.mission_order = list(best_order)
        self.ordered_mission_path = self.build_ordered_mission_path(
            best_order,
            current_to_stop,
        )
        self.active_target_id = ''
        self.active_path = []

        now = time.monotonic()
        if now - self.last_order_log_time >= 1.0:
            self.last_order_log_time = now
            self.get_logger().info(
                '[DYNAMIC PLANNER] Mission order: %s -> HOME (cost %.2fm, %.0fms)'
                % (
                    ' -> '.join(self.mission_order),
                    best_cost,
                    (now - start_time) * 1000.0,
                )
            )
        return True

    def find_best_order(
        self,
        remaining: list[str],
        current_to_stop: dict[str, AStarResult],
        *,
        preferred_start_id: str = '',
    ) -> tuple[tuple[str, ...] | None, float]:
        if len(remaining) <= self.max_exact_order_stops:
            return self.find_exact_best_order(
                remaining,
                current_to_stop,
                preferred_start_id=preferred_start_id,
            )
        return self.find_greedy_order(
            remaining,
            current_to_stop,
            preferred_start_id=preferred_start_id,
        )

    def find_exact_best_order(
        self,
        remaining: list[str],
        current_to_stop: dict[str, AStarResult],
        *,
        preferred_start_id: str = '',
    ) -> tuple[tuple[str, ...] | None, float]:
        best_order: tuple[str, ...] | None = None
        best_cost = float('inf')

        for order in permutations(remaining):
            if preferred_start_id and order[0] != preferred_start_id:
                continue
            first = order[0]
            if first not in current_to_stop:
                continue
            total_cost = current_to_stop[first].cost
            feasible = True
            for start_id, end_id in zip(order, order[1:]):
                result = self.static_stop_to_stop(start_id, end_id)
                if result is None:
                    feasible = False
                    break
                total_cost += result.cost
            if not feasible:
                continue

            home_result = self.static_stop_to_home(order[-1])
            if home_result is None:
                continue
            total_cost += home_result.cost

            if total_cost < best_cost:
                best_cost = total_cost
                best_order = order

        return best_order, best_cost

    def find_greedy_order(
        self,
        remaining: list[str],
        current_to_stop: dict[str, AStarResult],
        *,
        preferred_start_id: str = '',
    ) -> tuple[tuple[str, ...] | None, float]:
        unvisited = set(remaining)
        order: list[str] = []
        total_cost = 0.0

        current_id: str | None = None
        if preferred_start_id:
            preferred_result = current_to_stop.get(preferred_start_id)
            if preferred_result is None or preferred_start_id not in unvisited:
                return None, float('inf')
            total_cost += preferred_result.cost
            order.append(preferred_start_id)
            unvisited.remove(preferred_start_id)
            current_id = preferred_start_id

        while unvisited:
            if current_id is None:
                candidates = [
                    (result.cost, stop_id)
                    for stop_id, result in current_to_stop.items()
                    if stop_id in unvisited
                ]
            else:
                candidates = []
                for stop_id in unvisited:
                    result = self.static_stop_to_stop(current_id, stop_id)
                    if result is not None:
                        candidates.append((result.cost, stop_id))

            if not candidates:
                return None, float('inf')

            cost, next_id = min(candidates)
            total_cost += cost
            order.append(next_id)
            unvisited.remove(next_id)
            current_id = next_id

        if current_id is None:
            return None, float('inf')
        home_result = self.static_stop_to_home(current_id)
        if home_result is None:
            return None, float('inf')
        total_cost += home_result.cost
        self.get_logger().warn(
            '[DYNAMIC PLANNER] Using greedy order for %d stops; raise max_exact_order_stops only if compute budget allows.'
            % len(remaining)
        )
        return tuple(order), total_cost

    def start_preferred_stop_id(
        self,
        remaining: list[str],
        current_to_stop: dict[str, AStarResult],
    ) -> str:
        if self.completed_mission_stop_ids():
            return ''
        if START_PREFERRED_STOP_ID not in remaining:
            return ''
        if START_PREFERRED_STOP_ID not in current_to_stop:
            self.get_logger().warn(
                '[DYNAMIC PLANNER] Startup preference requested %s first, but it is not reachable.'
                % START_PREFERRED_STOP_ID
            )
            return ''
        self.get_logger().info(
            '[DYNAMIC PLANNER] Startup preference active: forcing %s as first stop.'
            % START_PREFERRED_STOP_ID
        )
        return START_PREFERRED_STOP_ID

    def current_to_stop_costs(
        self,
        remaining: list[str],
        current_x: float,
        current_y: float,
        current_yaw: float,
    ) -> dict[str, AStarResult]:
        current_to_stop: dict[str, AStarResult] = {}
        if self.fast_global_ordering:
            planner = self.make_static_order_planner()
            if planner is None:
                return current_to_stop
            goals = {
                stop_id: self.planning_goal_xy(stop_id) for stop_id in remaining
            }
            goal_yaws = {}
            for stop_id in remaining:
                goal_yaw = self.planning_goal_yaw(stop_id)
                if goal_yaw is not None:
                    goal_yaws[stop_id] = goal_yaw
            results = planner.plan_many(
                (current_x, current_y),
                goals,
                start_yaw=current_yaw,
                goal_yaws=goal_yaws,
            )
            for stop_id, result in results.items():
                if result is None:
                    self.get_logger().warn(
                        '[DYNAMIC PLANNER] No route from current pose to %s.' % stop_id
                    )
                    continue
                current_to_stop[stop_id] = self.adjust_result_for_target(
                    stop_id,
                    result,
                )
            return current_to_stop

        static_planner = self.make_astar(use_dynamic=False, path_bias_points=None)
        for stop_id in remaining:
            result = static_planner.plan(
                (current_x, current_y),
                self.planning_goal_xy(stop_id),
                start_yaw=current_yaw,
                goal_yaw=self.planning_goal_yaw(stop_id),
            )
            if result is None:
                self.get_logger().warn(
                    '[DYNAMIC PLANNER] No route from current pose to %s.' % stop_id
                )
                continue
            current_to_stop[stop_id] = self.adjust_result_for_target(
                stop_id,
                result,
            )
        return current_to_stop

    def precompute_static_order_costs(self, remaining: list[str]) -> None:
        if not self.fast_global_ordering:
            return

        planner = self.make_static_order_planner()
        if planner is None:
            return

        for start_id in remaining:
            missing_stop_ids = [
                stop_id
                for stop_id in remaining
                if stop_id != start_id and (start_id, stop_id) not in self.static_pair_cache
            ]
            missing_home = start_id not in self.static_home_cache
            if not missing_stop_ids and not missing_home:
                continue

            goals = {
                stop_id: self.planning_goal_xy(stop_id)
                for stop_id in missing_stop_ids
            }
            if missing_home:
                goals['HOME'] = self.home_xy

            goal_yaws = {}
            for stop_id in missing_stop_ids:
                goal_yaw = self.planning_goal_yaw(stop_id)
                if goal_yaw is not None:
                    goal_yaws[stop_id] = goal_yaw
            if missing_home:
                goal_yaws['HOME'] = self.home_yaw

            results = planner.plan_many(
                self.stop_xy_by_id[start_id],
                goals,
                start_yaw=self.stop_yaw_by_id[start_id],
                goal_yaws=goal_yaws,
            )
            for stop_id in missing_stop_ids:
                self.static_pair_cache[(start_id, stop_id)] = (
                    self.adjust_result_for_target(stop_id, results.get(stop_id))
                )
            if missing_home:
                self.static_home_cache[start_id] = results.get('HOME')

    def static_stop_to_stop(self, start_id: str, end_id: str) -> AStarResult | None:
        key = (start_id, end_id)
        if key not in self.static_pair_cache:
            if self.fast_global_ordering:
                planner = self.make_static_order_planner()
                result = (
                    None
                    if planner is None
                    else planner.plan(
                        self.stop_xy_by_id[start_id],
                        self.planning_goal_xy(end_id),
                        start_yaw=self.stop_yaw_by_id[start_id],
                        goal_yaw=self.planning_goal_yaw(end_id),
                    )
                )
                self.static_pair_cache[key] = self.adjust_result_for_target(
                    end_id,
                    result,
                )
            else:
                planner = self.make_astar(use_dynamic=False, path_bias_points=None)
                result = planner.plan(
                    self.stop_xy_by_id[start_id],
                    self.planning_goal_xy(end_id),
                    start_yaw=self.stop_yaw_by_id[start_id],
                    goal_yaw=self.planning_goal_yaw(end_id),
                )
                self.static_pair_cache[key] = self.adjust_result_for_target(
                    end_id,
                    result,
                )
        return self.static_pair_cache[key]

    def static_stop_to_home(self, start_id: str) -> AStarResult | None:
        if start_id not in self.static_home_cache:
            if self.fast_global_ordering:
                planner = self.make_static_order_planner()
                self.static_home_cache[start_id] = (
                    None
                    if planner is None
                    else planner.plan(
                        self.stop_xy_by_id[start_id],
                        self.home_xy,
                        start_yaw=self.stop_yaw_by_id[start_id],
                        goal_yaw=self.home_yaw,
                    )
                )
            else:
                planner = self.make_astar(use_dynamic=False, path_bias_points=None)
                self.static_home_cache[start_id] = planner.plan(
                    self.stop_xy_by_id[start_id],
                    self.home_xy,
                    start_yaw=self.stop_yaw_by_id[start_id],
                    goal_yaw=self.home_yaw,
                )
        return self.static_home_cache[start_id]

    def build_ordered_mission_path(
        self,
        order: tuple[str, ...],
        current_to_stop: dict[str, AStarResult],
    ) -> list[tuple[float, float, float]]:
        if not order:
            return []

        path: list[tuple[float, float, float]] = []
        first_result = current_to_stop[order[0]]
        self.remember_calculated_stop(order[0], first_result)
        self.append_path(path, first_result.points)
        for start_id, end_id in zip(order, order[1:]):
            result = self.static_stop_to_stop(start_id, end_id)
            if result is not None:
                self.remember_calculated_stop(end_id, result)
                self.append_path(path, result.points)
        home_result = self.static_stop_to_home(order[-1])
        if home_result is not None:
            self.append_path(path, home_result.points)
        return path

    def refresh_fog_cleanup_goals(self) -> None:
        if (
            not self.fog_cleanup_enabled
            or self.map_info is None
            or self.map_data is None
            or self.latest_pose is None
            or self.fog_cleanup_max_goals <= 0
        ):
            return

        if self.active_target_id in self.generated_exploration_ids:
            return

        explored_grid = self.map_aligned_explored_grid()
        if explored_grid is None:
            return

        signature = (
            self.map_revision,
            self.explored_revision,
            tuple(sorted(self.completed_stop_ids)),
        )
        if signature == self.fog_cleanup_signature:
            return

        required_mask = self.coverage_required_mask()
        required_cells = int(np.count_nonzero(required_mask))
        if required_cells <= 0:
            self.set_generated_exploration_goals([])
            self.fog_cleanup_signature = signature
            return

        fog_mask = (explored_grid < 0) & required_mask
        fog_cells = int(np.count_nonzero(fog_mask))
        fog_ratio = float(fog_cells) / float(required_cells)
        if fog_ratio >= self.fog_cleanup_trigger_ratio:
            self.set_generated_exploration_goals([])
            self.fog_cleanup_signature = signature
            return

        clusters = self.connected_residual_clusters(fog_mask)
        candidates: list[tuple[list[tuple[int, int]], float]] = []
        for cluster in clusters:
            cluster_ratio = float(len(cluster)) / float(required_cells)
            if (
                cluster_ratio <= self.fog_cleanup_min_cluster_ratio
                or cluster_ratio >= self.fog_cleanup_max_cluster_ratio
            ):
                continue
            candidates.append((cluster, cluster_ratio))

        goals = self.generate_fog_cleanup_goals_from_clusters(candidates)
        self.set_generated_exploration_goals(goals)
        self.fog_cleanup_signature = signature

        if goals:
            goal_ids = ', '.join(goal[0] for goal in goals)
            self.get_logger().info(
                '[DYNAMIC PLANNER] Fog cleanup active: fog_left=%.1f%%, candidate_clusters=%d, goals=%s'
                % (100.0 * fog_ratio, len(candidates), goal_ids)
            )

    def refresh_planned_coverage_goals(self) -> None:
        if (
            not self.planned_coverage_enabled
            or self.map_info is None
            or self.map_data is None
            or self.latest_pose is None
            or self.planned_coverage_radius_m <= 0.0
            or self.planned_coverage_max_goals <= 0
        ):
            return

        if self.active_target_id in self.generated_exploration_ids:
            return

        plant_remaining = [
            stop_id
            for stop_id in self.base_stop_order_ids
            if stop_id in self.plants and stop_id not in self.completed_stop_ids
        ]
        signature = (
            self.map_revision,
            tuple(plant_remaining),
            tuple(sorted(self.completed_stop_ids)),
        )
        if signature == self.planned_coverage_signature:
            return

        if not plant_remaining:
            self.set_generated_exploration_goals([])
            self.planned_coverage_signature = signature
            return

        current_x, current_y, current_yaw = self.latest_pose
        current_to_stop = self.current_to_stop_costs(
            plant_remaining,
            current_x,
            current_y,
            current_yaw,
        )
        if not current_to_stop:
            return

        self.precompute_static_order_costs(plant_remaining)
        plant_order, _plant_cost = self.find_best_order(
            plant_remaining,
            current_to_stop,
        )
        if plant_order is None:
            return

        plant_path = self.build_ordered_mission_path(plant_order, current_to_stop)
        covered_mask = self.build_planned_coverage_mask(plant_path)
        required_mask = self.coverage_required_mask()
        residual_mask = required_mask & ~covered_mask
        goals = self.generate_exploration_goals_from_residual(residual_mask)
        self.set_generated_exploration_goals(goals)
        self.planned_coverage_signature = signature

        residual_cells = int(np.count_nonzero(residual_mask))
        covered_cells = int(np.count_nonzero(covered_mask & required_mask))
        self.get_logger().info(
            '[DYNAMIC PLANNER] Planned coverage first pass %s -> HOME covers %d cells; residual=%d cells; exploration goals=%s'
            % (
                ' -> '.join(plant_order),
                covered_cells,
                residual_cells,
                ', '.join(goal[0] for goal in goals) or '<none>',
            )
        )

    def set_generated_exploration_goals(
        self,
        goals: list[tuple[str, float, float, float]],
    ) -> None:
        rounded_new = [
            (stop_id, round(x, 3), round(y, 3), round(yaw, 3))
            for stop_id, x, y, yaw in goals
        ]
        rounded_old = [
            (stop_id, round(x, 3), round(y, 3), round(yaw, 3))
            for stop_id, x, y, yaw in self.generated_exploration_stop_points
        ]
        if rounded_new == rounded_old:
            return

        self.generated_exploration_stop_points = list(goals)
        self.generated_exploration_ids = {stop_id for stop_id, _x, _y, _yaw in goals}
        self.completed_exploration_stop_ids.clear()
        self.refresh_stop_points()
        self.static_pair_cache.clear()
        self.static_home_cache.clear()
        self.ordered_mission_path = []
        self.mission_order = [
            stop_id
            for stop_id in self.mission_order
            if stop_id in self.stop_order_ids
        ]
        if (
            self.active_target_id
            and self.active_target_id != 'HOME'
            and self.active_target_id not in self.stop_order_ids
        ):
            self.active_target_id = ''
            self.active_path = []

    def update_exploration_progress(self) -> None:
        if (
            self.latest_pose is None
            or self.active_target_id not in self.generated_exploration_ids
            or self.active_target_id in self.completed_exploration_stop_ids
        ):
            return
        goal_x, goal_y = self.stop_xy_by_id[self.active_target_id]
        robot_x, robot_y, _robot_yaw = self.latest_pose
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if distance > self.planned_coverage_goal_tolerance_m:
            return

        completed_id = self.active_target_id
        self.completed_exploration_stop_ids.add(completed_id)
        self.mission_order = [
            stop_id for stop_id in self.mission_order if stop_id != completed_id
        ]
        self.active_target_id = ''
        self.active_path = []
        self.ordered_mission_path = []
        self.get_logger().info(
            '[DYNAMIC PLANNER] Exploration goal %s reached (%.2fm); moving to next mission target.'
            % (completed_id, distance)
        )

    def coverage_required_mask(self) -> np.ndarray:
        if self.map_data is None:
            return np.zeros((0, 0), dtype=bool)
        return (self.map_data >= 0) & (self.map_data < self.blocked_cost_threshold)

    def build_planned_coverage_mask(
        self,
        path: list[tuple[float, float, float]],
    ) -> np.ndarray:
        required_mask = self.coverage_required_mask()
        covered = np.zeros(required_mask.shape, dtype=bool)
        if not path or self.map_info is None or self.map_data is None:
            return covered

        last_sample: tuple[float, float] | None = None
        for x, y, _yaw in path:
            if last_sample is not None:
                if math.hypot(x - last_sample[0], y - last_sample[1]) < self.planned_coverage_sample_step_m:
                    continue
            self.mark_covered_cells_from_pose(covered, required_mask, x, y)
            last_sample = (x, y)
        return covered

    def mark_covered_cells_from_pose(
        self,
        covered: np.ndarray,
        required_mask: np.ndarray,
        x: float,
        y: float,
    ) -> None:
        if self.planned_coverage_use_line_of_sight:
            self.mark_visible_cells_from_pose(covered, required_mask, x, y)
            return

        center = self.world_to_map_cell(x, y)
        if center is None:
            return
        row_center, col_center = center
        radius_cells = int(
            math.ceil(self.planned_coverage_radius_m / float(self.map_info.resolution))
        )
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = row_center + dr
                col = col_center + dc
                if not self.in_map_bounds(row, col) or not required_mask[row, col]:
                    continue
                cell_x, cell_y = self.map_cell_center(row, col)
                if math.hypot(cell_x - x, cell_y - y) > self.planned_coverage_radius_m:
                    continue
                covered[row, col] = True

    def mark_visible_cells_from_pose(
        self,
        covered: np.ndarray,
        required_mask: np.ndarray,
        x: float,
        y: float,
    ) -> None:
        center = self.world_to_map_cell(x, y)
        if center is None:
            return
        row_center, col_center = center
        if required_mask[row_center, col_center]:
            covered[row_center, col_center] = True

        resolution = float(self.map_info.resolution)
        range_step_m = max(0.5 * resolution, 0.04)
        range_steps = max(1, int(math.ceil(self.planned_coverage_radius_m / range_step_m)))
        angle_count = max(12, int(math.ceil((2.0 * math.pi) / self.planned_coverage_ray_angle_step)))
        for angle_index in range(angle_count):
            angle = (2.0 * math.pi * angle_index) / float(angle_count)
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            previous_cell = center
            for step in range(1, range_steps + 1):
                distance = min(self.planned_coverage_radius_m, step * range_step_m)
                sample_x = x + distance * cos_angle
                sample_y = y + distance * sin_angle
                cell = self.world_to_map_cell(sample_x, sample_y)
                if cell is None:
                    break
                if cell == previous_cell:
                    continue
                previous_cell = cell
                row, col = cell
                if self.map_cell_blocks_visibility(row, col):
                    break
                if required_mask[row, col]:
                    covered[row, col] = True

    def generate_exploration_goals_from_residual(
        self,
        residual_mask: np.ndarray,
    ) -> list[tuple[str, float, float, float]]:
        if (
            residual_mask.size == 0
            or self.map_info is None
            or self.map_data is None
            or self.latest_pose is None
        ):
            return []

        clusters = self.connected_residual_clusters(residual_mask)
        if not clusters:
            return []
        clusters.sort(key=len, reverse=True)

        goals: list[tuple[str, float, float, float]] = []
        current_x, current_y, current_yaw = self.latest_pose
        reachability_planner = self.make_static_order_planner()
        for cluster in clusters:
            if len(goals) >= self.planned_coverage_max_goals:
                break
            area_m2 = len(cluster) * float(self.map_info.resolution) ** 2
            if area_m2 < self.planned_coverage_min_cluster_area_m2:
                continue

            candidate = self.exploration_goal_for_cluster(cluster)
            if candidate is None:
                continue
            goal_x, goal_y = candidate
            goal_yaw = math.atan2(goal_y - current_y, goal_x - current_x)
            if reachability_planner is not None:
                result = reachability_planner.plan(
                    (current_x, current_y),
                    (goal_x, goal_y),
                    start_yaw=current_yaw,
                    goal_yaw=goal_yaw,
                )
                if result is None:
                    continue

            stop_id = '%s%d' % (self.planned_coverage_goal_prefix, len(goals) + 1)
            goals.append((stop_id, goal_x, goal_y, goal_yaw))
        return goals

    def generate_fog_cleanup_goals_from_clusters(
        self,
        candidates: list[tuple[list[tuple[int, int]], float]],
    ) -> list[tuple[str, float, float, float]]:
        if not candidates or self.latest_pose is None:
            return []

        current_x, current_y, current_yaw = self.latest_pose
        reachability_planner = self.make_static_order_planner()
        reachable: list[tuple[float, float, int, float, float, float]] = []

        for cluster_index, (cluster, cluster_ratio) in enumerate(candidates):
            candidate = self.exploration_goal_for_cluster(cluster)
            if candidate is None:
                continue
            goal_x, goal_y = candidate
            goal_yaw = math.atan2(goal_y - current_y, goal_x - current_x)
            route_cost = math.hypot(goal_x - current_x, goal_y - current_y)
            if reachability_planner is not None:
                result = reachability_planner.plan(
                    (current_x, current_y),
                    (goal_x, goal_y),
                    start_yaw=current_yaw,
                    goal_yaw=goal_yaw,
                )
                if result is None:
                    continue
                route_cost = result.cost
            reachable.append(
                (route_cost, -cluster_ratio, cluster_index, goal_x, goal_y, goal_yaw)
            )

        reachable.sort()
        goals: list[tuple[str, float, float, float]] = []
        for _route_cost, _neg_ratio, _cluster_index, goal_x, goal_y, goal_yaw in reachable:
            if len(goals) >= self.fog_cleanup_max_goals:
                break
            stop_id = '%s%d' % (self.fog_cleanup_goal_prefix, len(goals) + 1)
            goals.append((stop_id, goal_x, goal_y, goal_yaw))
        return goals

    def connected_residual_clusters(
        self,
        residual_mask: np.ndarray,
    ) -> list[list[tuple[int, int]]]:
        height, width = residual_mask.shape
        visited = np.zeros((height, width), dtype=bool)
        clusters: list[list[tuple[int, int]]] = []
        for row in range(height):
            for col in range(width):
                if visited[row, col] or not residual_mask[row, col]:
                    continue
                stack = [(row, col)]
                visited[row, col] = True
                cluster: list[tuple[int, int]] = []
                while stack:
                    current_row, current_col = stack.pop()
                    cluster.append((current_row, current_col))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr = current_row + dr
                            nc = current_col + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and not visited[nr, nc]
                                and residual_mask[nr, nc]
                            ):
                                visited[nr, nc] = True
                                stack.append((nr, nc))
                clusters.append(cluster)
        return clusters

    def exploration_goal_for_cluster(
        self,
        cluster: list[tuple[int, int]],
    ) -> tuple[float, float] | None:
        if not cluster:
            return None
        centroid_row = sum(row for row, _col in cluster) / float(len(cluster))
        centroid_col = sum(col for _row, col in cluster) / float(len(cluster))
        row, col = min(
            cluster,
            key=lambda cell: (cell[0] - centroid_row) ** 2 + (cell[1] - centroid_col) ** 2,
        )
        return self.map_cell_center(row, col)

    def world_to_map_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.map_info is None:
            return None
        resolution = float(self.map_info.resolution)
        origin_x = float(self.map_info.origin.position.x)
        origin_y = float(self.map_info.origin.position.y)
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        if not self.in_map_bounds(row, col):
            return None
        return row, col

    def map_cell_center(self, row: int, col: int) -> tuple[float, float]:
        resolution = float(self.map_info.resolution)
        origin_x = float(self.map_info.origin.position.x)
        origin_y = float(self.map_info.origin.position.y)
        return (
            origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution,
        )

    def in_map_bounds(self, row: int, col: int) -> bool:
        if self.map_data is None:
            return False
        height, width = self.map_data.shape
        return 0 <= row < height and 0 <= col < width

    def map_cell_blocks_visibility(self, row: int, col: int) -> bool:
        if self.map_data is None or not self.in_map_bounds(row, col):
            return True
        return bool(
            self.map_data[row, col] < 0
            or self.map_data[row, col] >= self.blocked_cost_threshold
        )

    def static_line_of_sight_clear(
        self,
        source_x: float,
        source_y: float,
        target_x: float,
        target_y: float,
    ) -> bool:
        target_cell = self.world_to_map_cell(target_x, target_y)
        if target_cell is None:
            return False
        source_cell = self.world_to_map_cell(source_x, source_y)

        distance = math.hypot(target_x - source_x, target_y - source_y)
        if distance <= 1.0e-6:
            return True

        step_m = max(0.02, 0.5 * float(self.map_info.resolution))
        steps = max(1, int(math.ceil(distance / step_m)))
        for step in range(1, steps + 1):
            ratio = min(1.0, step / steps)
            sample_x = source_x + ratio * (target_x - source_x)
            sample_y = source_y + ratio * (target_y - source_y)
            sample_cell = self.world_to_map_cell(sample_x, sample_y)
            if sample_cell is None:
                return False
            if sample_cell == target_cell or sample_cell == source_cell:
                continue
            row, col = sample_cell
            if self.map_cell_blocks_visibility(row, col):
                return False
        return True

    def next_target_id(self) -> str:
        if self.mission_order:
            return self.mission_order[0]
        if self.remaining_stop_ids():
            return ''
        return 'HOME'

    def should_replan_active_path(self, target_id: str, now: float) -> tuple[bool, str]:
        force_replan = (
            not self.active_path
            or target_id != self.active_target_id
            or self.planned_map_revision != self.map_revision
        )
        reason = 'initial path' if not self.active_path else 'target/map changed'

        if (
            self.active_path
            and self.fog_path_preference > 1.0
            and self.explored_map_data is not None
            and self.planned_explored_revision != self.explored_revision
        ):
            force_replan = True
            reason = 'fog-of-war cost map changed'

        if self.active_path and self.latest_pose is not None:
            rx, ry, _ = self.latest_pose
            deviation = distance_to_polyline(rx, ry, self.active_path)
            if deviation > self.active_path_deviation_threshold_m:
                force_replan = True
                reason = 'robot deviated %.2fm from active path' % deviation

        if (
            self.active_path
            and self.dynamic_obstacle_revision != self.planned_dynamic_revision
            and self.dynamic_obstacles_affect_path(self.active_path)
        ):
            force_replan = True
            reason = 'LiDAR obstacle intersects active path'

        if not force_replan:
            return False, ''

        if not self.active_path or target_id != self.active_target_id:
            return True, reason

        if now - self.last_replan_time >= self.replan_cooldown_sec:
            return True, reason
        return False, ''

    def replan_active_path(self, target_id: str, reason: str, now: float) -> bool:
        if self.latest_pose is None:
            return False

        start_x, start_y, start_yaw = self.latest_pose
        if target_id == 'HOME':
            goal_xy = self.home_xy
            goal_yaw = self.home_yaw
        else:
            goal_xy = self.planning_goal_xy(target_id)
            goal_yaw = self.planning_goal_yaw(target_id)

        bias_path = self.last_accepted_path if target_id == self.active_target_id else None
        planner = self.make_astar(use_dynamic=True, path_bias_points=bias_path)
        result = planner.plan(
            (start_x, start_y),
            goal_xy,
            start_yaw=start_yaw,
            goal_yaw=goal_yaw,
            path_bias_points=bias_path,
        )

        if result is None:
            self.get_logger().warn(
                '[DYNAMIC PLANNER] Dynamic A* failed for %s; keeping previous path.'
                % target_id
            )
            if self.active_path:
                return False
            fallback_planner = self.make_astar(use_dynamic=False, path_bias_points=None)
            result = fallback_planner.plan(
                (start_x, start_y),
                goal_xy,
                start_yaw=start_yaw,
                goal_yaw=goal_yaw,
            )
            if result is None:
                self.get_logger().error(
                    '[DYNAMIC PLANNER] Static fallback also failed for %s.' % target_id
                )
                return False

        if target_id != 'HOME':
            result = self.adjust_result_for_target(
                target_id,
                result,
                remember_stop=True,
            )
            if result is None:
                return False

        self.active_target_id = target_id
        self.active_path = result.points
        self.last_accepted_path = result.points
        self.planned_dynamic_revision = self.dynamic_obstacle_revision
        self.planned_map_revision = self.map_revision
        self.planned_explored_revision = self.explored_revision
        self.last_replan_time = now

        self.get_logger().info(
            '[DYNAMIC PLANNER] Replanned to %s (%d poses, cost %.2fm): %s'
            % (target_id, len(self.active_path), result.cost, reason)
        )
        return True

    def make_static_order_planner(self) -> FastGridPlanner | None:
        if self.map_info is None or self.map_data is None:
            return None
        if (
            self.static_order_planner is not None
            and self.static_order_planner_revision == self.map_revision
        ):
            return self.static_order_planner

        order_info, order_grid, order_explored_grid = self.build_static_order_grid()
        self.static_order_planner = FastGridPlanner(
            order_info,
            order_grid,
            explored_grid=order_explored_grid,
            fog_path_preference=self.fog_path_preference,
            blocked_cost_threshold=self.blocked_cost_threshold,
            rotation_cost_per_90deg_m=self.rotation_cost_per_90deg_m,
            turn_start_cost_m=self.turn_start_cost_m,
            soft_cost_weight_m=self.soft_cost_weight_m,
            obstacle_clearance_radius_m=self.obstacle_clearance_radius_m,
            obstacle_clearance_weight_m=self.obstacle_clearance_weight_m,
            hard_obstacle_clearance_m=self.hard_obstacle_clearance_m,
            path_step_m=self.path_step_m,
            max_nearest_free_radius_m=self.max_nearest_free_radius_m,
        )
        self.static_order_planner_revision = self.map_revision
        return self.static_order_planner

    def map_aligned_explored_grid(self) -> np.ndarray | None:
        if (
            self.explored_map_info is None
            or self.explored_map_data is None
            or self.map_info is None
            or self.map_data is None
        ):
            return None
        if (
            self.explored_map_info.width != self.map_info.width
            or self.explored_map_info.height != self.map_info.height
            or abs(self.explored_map_info.resolution - self.map_info.resolution) > 1.0e-9
            or abs(self.explored_map_info.origin.position.x - self.map_info.origin.position.x) > 1.0e-9
            or abs(self.explored_map_info.origin.position.y - self.map_info.origin.position.y) > 1.0e-9
        ):
            return None
        if self.explored_map_data.shape != self.map_data.shape:
            return None
        return self.explored_map_data

    def current_explored_grid(self) -> np.ndarray | None:
        if self.fog_path_preference <= 1.0:
            return None
        return self.map_aligned_explored_grid()

    def build_static_order_grid(
        self,
    ) -> tuple[MapMetaData, np.ndarray, np.ndarray | None]:
        explored_grid = self.current_explored_grid()
        if self.global_order_grid_stride <= 1:
            return self.map_info, self.map_data, explored_grid

        stride = self.global_order_grid_stride
        height, width = self.map_data.shape
        coarse_height = int(math.ceil(height / stride))
        coarse_width = int(math.ceil(width / stride))
        coarse_grid = np.zeros((coarse_height, coarse_width), dtype=np.int16)
        coarse_explored_grid = (
            np.zeros((coarse_height, coarse_width), dtype=np.int16)
            if explored_grid is not None
            else None
        )

        for row in range(coarse_height):
            row_start = row * stride
            row_end = min(height, row_start + stride)
            for col in range(coarse_width):
                col_start = col * stride
                col_end = min(width, col_start + stride)
                block = self.map_data[row_start:row_end, col_start:col_end]
                if np.any(block < 0):
                    coarse_grid[row, col] = -1
                elif np.any(block >= self.blocked_cost_threshold):
                    coarse_grid[row, col] = self.blocked_cost_threshold
                else:
                    coarse_grid[row, col] = int(np.max(block))
                if coarse_explored_grid is not None:
                    explored_block = explored_grid[row_start:row_end, col_start:col_end]
                    traversable = (block >= 0) & (block < self.blocked_cost_threshold)
                    coarse_explored_grid[row, col] = (
                        -1 if np.any(explored_block[traversable] < 0) else 0
                    )

        order_info = MapMetaData()
        order_info.map_load_time = self.map_info.map_load_time
        order_info.resolution = float(self.map_info.resolution) * stride
        order_info.width = coarse_width
        order_info.height = coarse_height
        order_info.origin = self.map_info.origin
        return order_info, coarse_grid, coarse_explored_grid

    def make_astar(
        self,
        *,
        use_dynamic: bool,
        path_bias_points: list[tuple[float, float, float]] | None,
    ) -> HeadingAStar:
        dynamic_mask = self.build_dynamic_mask() if use_dynamic else None
        return HeadingAStar(
            self.map_info,
            self.map_data,
            explored_grid=self.current_explored_grid(),
            fog_path_preference=self.fog_path_preference,
            dynamic_mask=dynamic_mask,
            blocked_cost_threshold=self.blocked_cost_threshold,
            rotation_cost_per_90deg_m=self.rotation_cost_per_90deg_m,
            turn_start_cost_m=self.turn_start_cost_m,
            soft_cost_weight_m=self.soft_cost_weight_m,
            obstacle_clearance_radius_m=self.obstacle_clearance_radius_m,
            obstacle_clearance_weight_m=self.obstacle_clearance_weight_m,
            hard_obstacle_clearance_m=self.hard_obstacle_clearance_m,
            path_step_m=self.path_step_m,
            current_path_bias_m=(
                self.current_path_bias_m if path_bias_points else 0.0
            ),
            current_path_bias_radius_m=self.current_path_bias_radius_m,
            start_heading_bias_m=self.start_heading_bias_m,
            start_heading_bias_distance_m=self.start_heading_bias_distance_m,
            max_nearest_free_radius_m=self.max_nearest_free_radius_m,
        )

    def build_dynamic_mask(self) -> np.ndarray | None:
        if (
            self.map_info is None
            or self.map_data is None
            or not self.dynamic_obstacle_points
            or self.dynamic_obstacle_inflation_m <= 0.0
        ):
            return None

        height, width = self.map_data.shape
        mask = np.zeros((height, width), dtype=bool)
        res = float(self.map_info.resolution)
        origin_x = float(self.map_info.origin.position.x)
        origin_y = float(self.map_info.origin.position.y)
        radius_cells = int(math.ceil(self.dynamic_obstacle_inflation_m / res))

        for obstacle_x, obstacle_y, _ in self.dynamic_obstacle_points:
            col = int(math.floor((obstacle_x - origin_x) / res))
            row = int(math.floor((obstacle_y - origin_y) / res))
            if not (0 <= row < height and 0 <= col < width):
                continue
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    nr = row + dr
                    nc = col + dc
                    if not (0 <= nr < height and 0 <= nc < width):
                        continue
                    if math.hypot(dr, dc) * res <= self.dynamic_obstacle_inflation_m:
                        mask[nr, nc] = True
        return mask

    def dynamic_obstacles_affect_path(
        self,
        path: list[tuple[float, float, float]],
    ) -> bool:
        if len(path) < 2 or not self.dynamic_obstacle_points:
            return False

        margin = self.dynamic_obstacle_inflation_m + self.dynamic_obstacle_path_margin_m
        for obstacle_x, obstacle_y, _ in self.dynamic_obstacle_points:
            if distance_to_polyline(obstacle_x, obstacle_y, path) <= margin:
                return True
        return False

    def prune_dynamic_obstacles(self, now: float) -> None:
        if not self.dynamic_obstacle_points:
            return
        cutoff = now - self.dynamic_obstacle_ttl_sec
        original_count = len(self.dynamic_obstacle_points)
        self.dynamic_obstacle_points = [
            point for point in self.dynamic_obstacle_points if point[2] >= cutoff
        ]
        if len(self.dynamic_obstacle_points) != original_count:
            self.dynamic_obstacle_revision += 1

    def append_path(
        self,
        destination: list[tuple[float, float, float]],
        source: list[tuple[float, float, float]],
    ) -> None:
        if not source:
            return
        if not destination:
            destination.extend(source)
            return
        destination.extend(source[1:])

    def publish_active_path(self) -> None:
        path_msg = self.create_path_msg(self.active_path)
        self.path_pub.publish(path_msg)
        self.path_marker_pub.publish(self.create_path_marker(path_msg))
        self.stop_marker_pub.publish(self.create_stop_marker())

        if not self.published_once:
            self.published_once = True
            self.get_logger().info(
                '[DYNAMIC PLANNER] Publishing dynamic reference path on %s.'
                % self.path_topic
            )

    def create_path_msg(
        self,
        points: list[tuple[float, float, float]],
    ) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            set_yaw(pose_stamped.pose, yaw)
            path.poses.append(pose_stamped)
        return path

    def publish_stop_poses(self) -> None:
        self.stop_pub.publish(self.create_stop_pose_array())

    def publish_dynamic_obstacle_marker(self) -> None:
        self.dynamic_obstacle_marker_pub.publish(self.create_dynamic_obstacle_marker())

    def create_stop_pose_array(self) -> PoseArray:
        poses = PoseArray()
        poses.header.frame_id = self.frame_id
        poses.header.stamp = self.get_clock().now().to_msg()
        for plant_id, x, y in self.display_stop_points():
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            set_yaw(pose, self.display_stop_yaw(plant_id, x, y))
            poses.poses.append(pose)
        return poses

    def create_path_marker(self, path_msg: Path) -> Marker:
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = 'dynamic_known_map_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        for pose in path_msg.poses:
            marker.points.append(
                Point(
                    x=float(pose.pose.position.x),
                    y=float(pose.pose.position.y),
                    z=0.08,
                )
            )
        return marker

    def create_stop_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'plant_stop_poses'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 1.0
        for _, x, y in self.display_stop_points():
            marker.points.append(Point(x=float(x), y=float(y), z=0.12))
        return marker

    def create_dynamic_obstacle_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lidar_dynamic_obstacles'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        diameter = max(0.08, self.dynamic_obstacle_inflation_m * 2.0)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.10
        marker.color.r = 0.95
        marker.color.g = 0.0
        marker.color.b = 0.85
        marker.color.a = 0.65
        points = (
            self.latest_scan_obstacle_points
            if self.dynamic_obstacle_marker_latest_only
            else self.dynamic_obstacle_points
        )
        for x, y, _ in points:
            marker.points.append(Point(x=float(x), y=float(y), z=0.16))
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = DynamicKnownMapMissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
