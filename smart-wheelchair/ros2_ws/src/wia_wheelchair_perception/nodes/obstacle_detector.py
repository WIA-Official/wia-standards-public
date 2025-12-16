#!/usr/bin/env python3
"""
WIA Smart Wheelchair Obstacle Detector Node

This node processes laser scan data to detect obstacles
and publishes zone status for navigation safety.
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from wia_wheelchair_msgs.msg import (
    SafetyStatus,
    CollisionWarning,
)


class ZoneState(IntEnum):
    """Zone safety state"""
    CLEAR = 0
    WARNING = 1
    DANGER = 2


@dataclass
class Obstacle:
    """Detected obstacle"""
    id: int
    position: Tuple[float, float]
    size: Tuple[float, float]
    confidence: float
    last_seen: float


@dataclass
class ZoneStatus:
    """Status of all detection zones"""
    front: ZoneState = ZoneState.CLEAR
    front_left: ZoneState = ZoneState.CLEAR
    front_right: ZoneState = ZoneState.CLEAR
    left: ZoneState = ZoneState.CLEAR
    right: ZoneState = ZoneState.CLEAR
    rear: ZoneState = ZoneState.CLEAR


@dataclass
class ObstacleDetectorConfig:
    """Configuration for obstacle detection"""
    detection_range: float = 5.0
    safety_margin: float = 0.3
    danger_distance: float = 0.5
    warning_distance: float = 1.5
    cluster_tolerance: float = 0.1
    min_cluster_size: int = 3
    max_obstacles: int = 50


class ObstacleDetectorNode(Node):
    """
    Obstacle detection node for WIA Smart Wheelchair.

    Subscribes to:
        /wia_wheelchair/scan (sensor_msgs/LaserScan)

    Publishes:
        /wia_wheelchair/safety_status (wia_wheelchair_msgs/SafetyStatus)
        /wia_wheelchair/collision_warning (wia_wheelchair_msgs/CollisionWarning)
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # Declare parameters
        self.declare_parameter('detection_range', 5.0)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('danger_distance', 0.5)
        self.declare_parameter('warning_distance', 1.5)
        self.declare_parameter('cluster_tolerance', 0.1)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('robot_width', 0.7)

        # Load configuration
        self.config = ObstacleDetectorConfig(
            detection_range=self.get_parameter('detection_range').value,
            safety_margin=self.get_parameter('safety_margin').value,
            danger_distance=self.get_parameter('danger_distance').value,
            warning_distance=self.get_parameter('warning_distance').value,
            cluster_tolerance=self.get_parameter('cluster_tolerance').value,
            min_cluster_size=self.get_parameter('min_cluster_size').value,
        )
        self.robot_width = self.get_parameter('robot_width').value

        # State
        self.obstacles: List[Obstacle] = []
        self.zone_status = ZoneStatus()
        self.next_obstacle_id = 1

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/wia_wheelchair/scan',
            self.scan_callback,
            sensor_qos
        )

        # Publishers
        self.safety_pub = self.create_publisher(
            SafetyStatus,
            '/wia_wheelchair/safety_status',
            10
        )
        self.collision_pub = self.create_publisher(
            CollisionWarning,
            '/wia_wheelchair/collision_warning',
            10
        )

        self.get_logger().info('Obstacle detector node started')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan"""
        # Extract valid points
        points = self._extract_points(msg)

        # Cluster points
        clusters = self._cluster_points(points)

        # Convert to obstacles
        self.obstacles = [
            self._cluster_to_obstacle(cluster)
            for cluster in clusters[:self.config.max_obstacles]
        ]

        # Update zone status
        self._update_zone_status(msg)

        # Publish safety status
        self._publish_safety_status(msg.header)

        # Check for collision warning
        self._check_collision_warning(msg.header)

    def _extract_points(self, scan: LaserScan) -> List[Tuple[float, float]]:
        """Extract valid points from laser scan"""
        points = []

        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if not math.isfinite(range_val):
                continue
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            if range_val > self.config.detection_range:
                continue

            # Convert to Cartesian coordinates
            angle = scan.angle_min + i * scan.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append((x, y))

        return points

    def _cluster_points(
        self, points: List[Tuple[float, float]]
    ) -> List[List[Tuple[float, float]]]:
        """Simple Euclidean clustering"""
        clusters = []
        visited = [False] * len(points)

        for i, point in enumerate(points):
            if visited[i]:
                continue

            cluster = [point]
            visited[i] = True

            # Find neighbors
            for j in range(i + 1, len(points)):
                if visited[j]:
                    continue

                dist = math.sqrt(
                    (point[0] - points[j][0]) ** 2 +
                    (point[1] - points[j][1]) ** 2
                )

                if dist < self.config.cluster_tolerance:
                    cluster.append(points[j])
                    visited[j] = True

            if len(cluster) >= self.config.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def _cluster_to_obstacle(
        self, cluster: List[Tuple[float, float]]
    ) -> Obstacle:
        """Convert point cluster to obstacle"""
        # Calculate centroid and bounds
        sum_x = sum(p[0] for p in cluster)
        sum_y = sum(p[1] for p in cluster)
        min_x = min(p[0] for p in cluster)
        max_x = max(p[0] for p in cluster)
        min_y = min(p[1] for p in cluster)
        max_y = max(p[1] for p in cluster)

        obs_id = self.next_obstacle_id
        self.next_obstacle_id += 1

        return Obstacle(
            id=obs_id,
            position=(sum_x / len(cluster), sum_y / len(cluster)),
            size=(
                max_y - min_y + self.config.safety_margin,
                max_x - min_x + self.config.safety_margin
            ),
            confidence=min(1.0, len(cluster) / 20.0),
            last_seen=self.get_clock().now().nanoseconds / 1e9
        )

    def _update_zone_status(self, scan: LaserScan):
        """Update zone status from scan"""
        # Zone definitions (angle ranges in radians)
        zones = {
            'front': (-math.pi / 6, math.pi / 6),
            'front_left': (math.pi / 6, math.pi / 2),
            'front_right': (-math.pi / 2, -math.pi / 6),
            'left': (math.pi / 2, math.pi * 5 / 6),
            'right': (-math.pi * 5 / 6, -math.pi / 2),
            'rear': (math.pi * 5 / 6, math.pi),  # Also includes -pi to -5pi/6
        }

        for zone_name, (min_angle, max_angle) in zones.items():
            min_dist = float('inf')

            for i, range_val in enumerate(scan.ranges):
                if not math.isfinite(range_val):
                    continue
                if range_val < scan.range_min or range_val > scan.range_max:
                    continue

                angle = scan.angle_min + i * scan.angle_increment

                # Check if angle is in zone
                in_zone = min_angle <= angle <= max_angle

                # Special handling for rear zone (wraps around)
                if zone_name == 'rear':
                    in_zone = angle >= min_angle or angle <= -min_angle

                if in_zone:
                    min_dist = min(min_dist, range_val)

            # Determine zone state
            if min_dist <= self.config.danger_distance:
                state = ZoneState.DANGER
            elif min_dist <= self.config.warning_distance:
                state = ZoneState.WARNING
            else:
                state = ZoneState.CLEAR

            setattr(self.zone_status, zone_name, state)

    def _publish_safety_status(self, header: Header):
        """Publish safety status message"""
        msg = SafetyStatus()
        msg.header = header
        msg.header.frame_id = 'base_link'

        # Emergency stop (triggered by danger in front zone)
        msg.emergency_stop_active = (self.zone_status.front == ZoneState.DANGER)
        msg.emergency_stop_source = (
            SafetyStatus.ESTOP_SOURCE_SENSOR
            if msg.emergency_stop_active
            else SafetyStatus.ESTOP_SOURCE_NONE
        )

        # Obstacle detection flags
        msg.obstacle_front = (self.zone_status.front != ZoneState.CLEAR)
        msg.obstacle_left = (
            self.zone_status.left != ZoneState.CLEAR or
            self.zone_status.front_left != ZoneState.CLEAR
        )
        msg.obstacle_right = (
            self.zone_status.right != ZoneState.CLEAR or
            self.zone_status.front_right != ZoneState.CLEAR
        )
        msg.obstacle_rear = (self.zone_status.rear != ZoneState.CLEAR)

        # Minimum obstacle distance
        msg.min_obstacle_distance = self._get_min_obstacle_distance()

        # System health
        msg.system_healthy = True

        # Safety level
        if self.zone_status.front == ZoneState.DANGER:
            msg.safety_level = SafetyStatus.SAFETY_LEVEL_CRITICAL
        elif any(
            getattr(self.zone_status, z) == ZoneState.DANGER
            for z in ['front_left', 'front_right', 'left', 'right']
        ):
            msg.safety_level = SafetyStatus.SAFETY_LEVEL_WARNING
        elif any(
            getattr(self.zone_status, z) == ZoneState.WARNING
            for z in ['front', 'front_left', 'front_right']
        ):
            msg.safety_level = SafetyStatus.SAFETY_LEVEL_CAUTION
        else:
            msg.safety_level = SafetyStatus.SAFETY_LEVEL_NORMAL

        self.safety_pub.publish(msg)

    def _check_collision_warning(self, header: Header):
        """Check and publish collision warnings"""
        # Find closest obstacle in front
        closest = self._get_closest_front_obstacle()

        if closest is None:
            return

        dist = math.sqrt(closest.position[0] ** 2 + closest.position[1] ** 2)

        # Only warn if close enough
        if dist > self.config.warning_distance:
            return

        msg = CollisionWarning()
        msg.header = header
        msg.header.frame_id = 'base_link'

        msg.collision_imminent = (dist <= self.config.danger_distance)
        msg.distance_to_obstacle = dist

        # Estimate time to collision (assuming constant velocity)
        # This is simplified - real implementation would use velocity
        msg.time_to_collision = dist / 0.5 if dist > 0 else 0.0

        # Obstacle position
        msg.obstacle_position = Point(
            x=closest.position[0],
            y=closest.position[1],
            z=0.0
        )
        msg.obstacle_angle = math.atan2(closest.position[1], closest.position[0])

        # Recommended action
        if dist <= self.config.danger_distance:
            msg.recommended_action = CollisionWarning.ACTION_STOP
            msg.warning_level = CollisionWarning.LEVEL_CRITICAL
        elif dist <= self.config.danger_distance * 2:
            msg.recommended_action = CollisionWarning.ACTION_SLOW_DOWN
            msg.warning_level = CollisionWarning.LEVEL_DANGER
        else:
            msg.recommended_action = CollisionWarning.ACTION_SLOW_DOWN
            msg.warning_level = CollisionWarning.LEVEL_WARNING

        self.collision_pub.publish(msg)

    def _get_min_obstacle_distance(self) -> float:
        """Get minimum distance to any obstacle"""
        if not self.obstacles:
            return float('inf')

        return min(
            math.sqrt(obs.position[0] ** 2 + obs.position[1] ** 2)
            for obs in self.obstacles
        )

    def _get_closest_front_obstacle(self) -> Optional[Obstacle]:
        """Get closest obstacle in front half"""
        front_obstacles = [
            obs for obs in self.obstacles
            if obs.position[0] > 0  # In front of robot
            and abs(math.atan2(obs.position[1], obs.position[0])) < math.pi / 2
        ]

        if not front_obstacles:
            return None

        return min(
            front_obstacles,
            key=lambda obs: math.sqrt(obs.position[0] ** 2 + obs.position[1] ** 2)
        )


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
