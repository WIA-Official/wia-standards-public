#!/usr/bin/env python3
"""
WIA Smart Wheelchair Global Planner Node

This node provides global path planning using A* algorithm
on the occupancy grid map.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from std_msgs.msg import Header


@dataclass
class PlannerConfig:
    """Global planner configuration"""
    algorithm: str = 'astar'
    inflation_radius: float = 0.55
    robot_radius: float = 0.35
    cost_scaling_factor: float = 3.0
    lethal_cost: int = 253
    allow_unknown: bool = False
    diagonal_movement: bool = True


@dataclass(order=True)
class PriorityNode:
    """Node for priority queue in A*"""
    f_cost: float
    position: Tuple[int, int] = field(compare=False)
    g_cost: float = field(compare=False)
    parent: Optional['PriorityNode'] = field(default=None, compare=False)


class GlobalPlannerNode(Node):
    """
    Global path planner for WIA Smart Wheelchair.

    Uses A* algorithm to find optimal paths on occupancy grid.

    Subscribes:
        /wia_wheelchair/map (nav_msgs/OccupancyGrid)
        /wia_wheelchair/goal_pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/pose (geometry_msgs/PoseWithCovarianceStamped)

    Publishes:
        /wia_wheelchair/global_plan (nav_msgs/Path)
    """

    def __init__(self):
        super().__init__('global_planner')

        # Declare parameters
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('inflation_radius', 0.55)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('cost_scaling_factor', 3.0)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('diagonal_movement', True)

        # Load configuration
        self.config = PlannerConfig(
            algorithm=self.get_parameter('algorithm').value,
            inflation_radius=self.get_parameter('inflation_radius').value,
            robot_radius=self.get_parameter('robot_radius').value,
            cost_scaling_factor=self.get_parameter('cost_scaling_factor').value,
            allow_unknown=self.get_parameter('allow_unknown').value,
            diagonal_movement=self.get_parameter('diagonal_movement').value,
        )

        # State
        self.map: Optional[OccupancyGrid] = None
        self.costmap: Optional[np.ndarray] = None
        self.current_pose: Optional[PoseWithCovarianceStamped] = None

        # QoS for map
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/wia_wheelchair/map',
            self.map_callback,
            map_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/wia_wheelchair/goal_pose',
            self.goal_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/wia_wheelchair/pose',
            self.pose_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(
            Path,
            '/wia_wheelchair/global_plan',
            10
        )

        self.get_logger().info(
            f'Global planner started with algorithm: {self.config.algorithm}'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map and create costmap"""
        self.map = msg
        self._create_costmap()
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}'
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current pose"""
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped):
        """Plan path to goal"""
        if self.map is None or self.costmap is None:
            self.get_logger().warn('No map available for planning')
            return

        if self.current_pose is None:
            self.get_logger().warn('No current pose available')
            return

        # Get start position
        start = (
            self.current_pose.pose.pose.position.x,
            self.current_pose.pose.pose.position.y
        )

        # Get goal position
        goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.get_logger().info(
            f'Planning from {start} to {goal}'
        )

        # Plan path
        path = self._plan_path(start, goal)

        if path:
            # Publish path
            path_msg = self._create_path_message(path, msg.header.stamp)
            self.plan_pub.publish(path_msg)
            self.get_logger().info(f'Published path with {len(path)} waypoints')
        else:
            self.get_logger().warn('Failed to find path')

    def _create_costmap(self):
        """Create inflated costmap from occupancy grid"""
        if self.map is None:
            return

        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution

        # Convert to numpy array
        data = np.array(self.map.data, dtype=np.int16).reshape(height, width)

        # Create costmap
        self.costmap = np.zeros((height, width), dtype=np.uint8)

        # Calculate inflation in cells
        inflation_cells = int(self.config.inflation_radius / resolution)
        robot_cells = int(self.config.robot_radius / resolution)

        # Mark obstacles and inflate
        for y in range(height):
            for x in range(width):
                if data[y, x] >= 100:  # Occupied
                    self._inflate_obstacle(x, y, inflation_cells, robot_cells)
                elif data[y, x] == -1 and not self.config.allow_unknown:
                    self.costmap[y, x] = 255  # Unknown as obstacle

    def _inflate_obstacle(self, cx: int, cy: int, inflation: int, robot: int):
        """Inflate obstacle at (cx, cy)"""
        height, width = self.costmap.shape

        for dy in range(-inflation, inflation + 1):
            for dx in range(-inflation, inflation + 1):
                x = cx + dx
                y = cy + dy

                if 0 <= x < width and 0 <= y < height:
                    dist = math.sqrt(dx * dx + dy * dy)

                    if dist <= robot:
                        self.costmap[y, x] = 254  # Lethal
                    elif dist <= inflation:
                        # Exponential decay
                        factor = (inflation - dist) / (inflation - robot)
                        cost = int(252 * factor * factor)
                        self.costmap[y, x] = max(self.costmap[y, x], cost)

    def _plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """Plan path using A*"""
        if self.map is None or self.costmap is None:
            return None

        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        # Convert to grid coordinates
        start_grid = (
            int((start[0] - origin_x) / resolution),
            int((start[1] - origin_y) / resolution)
        )
        goal_grid = (
            int((goal[0] - origin_x) / resolution),
            int((goal[1] - origin_y) / resolution)
        )

        # Check bounds
        height, width = self.costmap.shape
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
            self.get_logger().error('Start position out of bounds')
            return None
        if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
            self.get_logger().error('Goal position out of bounds')
            return None

        # Check if start or goal is in obstacle
        if self.costmap[start_grid[1], start_grid[0]] >= 254:
            self.get_logger().error('Start position is in obstacle')
            return None
        if self.costmap[goal_grid[1], goal_grid[0]] >= 254:
            self.get_logger().error('Goal position is in obstacle')
            return None

        # Run A*
        path_grid = self._astar(start_grid, goal_grid)

        if path_grid is None:
            return None

        # Convert back to world coordinates
        path_world = [
            (
                p[0] * resolution + origin_x + resolution / 2,
                p[1] * resolution + origin_y + resolution / 2
            )
            for p in path_grid
        ]

        # Smooth path
        path_world = self._smooth_path(path_world)

        return path_world

    def _astar(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        height, width = self.costmap.shape

        # Movement directions
        if self.config.diagonal_movement:
            directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)
            ]
            costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]
        else:
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            costs = [1.0, 1.0, 1.0, 1.0]

        # Heuristic function
        def heuristic(pos):
            return math.sqrt(
                (pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2
            )

        # Initialize
        open_set = []
        start_node = PriorityNode(
            f_cost=heuristic(start),
            position=start,
            g_cost=0.0
        )
        heapq.heappush(open_set, start_node)

        visited = set()
        came_from = {}
        g_scores = {start: 0.0}

        while open_set:
            current = heapq.heappop(open_set)

            if current.position == goal:
                # Reconstruct path
                path = []
                pos = goal
                while pos in came_from:
                    path.append(pos)
                    pos = came_from[pos]
                path.append(start)
                return list(reversed(path))

            if current.position in visited:
                continue

            visited.add(current.position)

            for i, (dx, dy) in enumerate(directions):
                nx = current.position[0] + dx
                ny = current.position[1] + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    continue

                if (nx, ny) in visited:
                    continue

                cell_cost = self.costmap[ny, nx]
                if cell_cost >= 254:  # Lethal
                    continue

                # Calculate cost
                move_cost = costs[i]
                cell_factor = 1.0 + cell_cost / 50.0  # Scale cell cost
                new_g = current.g_cost + move_cost * cell_factor

                if (nx, ny) not in g_scores or new_g < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = new_g
                    came_from[(nx, ny)] = current.position

                    new_node = PriorityNode(
                        f_cost=new_g + heuristic((nx, ny)),
                        position=(nx, ny),
                        g_cost=new_g
                    )
                    heapq.heappush(open_set, new_node)

        return None  # No path found

    def _smooth_path(
        self,
        path: List[Tuple[float, float]],
        iterations: int = 50,
        weight_smooth: float = 0.5
    ) -> List[Tuple[float, float]]:
        """Smooth path using gradient descent"""
        if len(path) <= 2:
            return path

        smoothed = [list(p) for p in path]
        weight_data = 1.0 - weight_smooth

        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                for j in range(2):
                    smoothed[i][j] += weight_data * (path[i][j] - smoothed[i][j])
                    smoothed[i][j] += weight_smooth * (
                        smoothed[i - 1][j] + smoothed[i + 1][j] - 2 * smoothed[i][j]
                    )

        return [tuple(p) for p in smoothed]

    def _create_path_message(
        self,
        path: List[Tuple[float, float]],
        stamp
    ) -> Path:
        """Create ROS Path message"""
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        # Calculate orientations along path
        for i in range(len(msg.poses) - 1):
            dx = msg.poses[i + 1].pose.position.x - msg.poses[i].pose.position.x
            dy = msg.poses[i + 1].pose.position.y - msg.poses[i].pose.position.y
            yaw = math.atan2(dy, dx)

            msg.poses[i].pose.orientation.z = math.sin(yaw / 2)
            msg.poses[i].pose.orientation.w = math.cos(yaw / 2)

        # Last pose keeps final orientation
        if msg.poses:
            msg.poses[-1].pose.orientation = msg.poses[-2].pose.orientation

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
