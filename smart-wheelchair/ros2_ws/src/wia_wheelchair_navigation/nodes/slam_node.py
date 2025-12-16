#!/usr/bin/env python3
"""
WIA Smart Wheelchair SLAM Manager Node

This node manages SLAM operations including mapping,
map saving/loading, and localization mode switching.
"""

import os
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException


class SLAMMode(IntEnum):
    """SLAM operation mode"""
    IDLE = 0
    MAPPING = 1
    LOCALIZING = 2
    SAVING = 3


@dataclass
class SLAMConfig:
    """SLAM configuration"""
    algorithm: str = 'gmapping'
    map_resolution: float = 0.05
    map_update_interval: float = 1.0
    map_save_path: str = '/tmp/wia_wheelchair_maps'
    localization_quality_threshold: float = 0.5


@dataclass
class LocalizationQuality:
    """Localization quality metrics"""
    covariance_trace: float = 0.0
    particle_spread: float = 0.0
    quality_level: str = 'unknown'  # good, fair, poor, lost


class SLAMManagerNode(Node):
    """
    SLAM Manager node for WIA Smart Wheelchair.

    This node coordinates SLAM operations and provides
    services for map management and localization.

    Subscribes:
        /wia_wheelchair/map (nav_msgs/OccupancyGrid)
        /wia_wheelchair/pose (geometry_msgs/PoseWithCovarianceStamped)

    Publishes:
        /wia_wheelchair/slam_status (std_msgs/String)

    Services:
        /wia_wheelchair/slam/start_mapping
        /wia_wheelchair/slam/stop_mapping
        /wia_wheelchair/slam/save_map
        /wia_wheelchair/slam/load_map
        /wia_wheelchair/slam/start_localization
    """

    def __init__(self):
        super().__init__('slam_manager')

        # Declare parameters
        self.declare_parameter('algorithm', 'gmapping')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_update_interval', 1.0)
        self.declare_parameter('map_save_path', '/tmp/wia_wheelchair_maps')

        # Load configuration
        self.config = SLAMConfig(
            algorithm=self.get_parameter('algorithm').value,
            map_resolution=self.get_parameter('map_resolution').value,
            map_update_interval=self.get_parameter('map_update_interval').value,
            map_save_path=self.get_parameter('map_save_path').value,
        )

        # State
        self.mode = SLAMMode.IDLE
        self.current_map: Optional[OccupancyGrid] = None
        self.current_pose: Optional[PoseWithCovarianceStamped] = None
        self.localization_quality = LocalizationQuality()
        self.saved_locations: dict = {}

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for map (latched)
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
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/wia_wheelchair/pose',
            self.pose_callback,
            10
        )

        # Services
        self.start_mapping_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/slam/start_mapping',
            self.start_mapping_callback
        )
        self.stop_mapping_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/slam/stop_mapping',
            self.stop_mapping_callback
        )
        self.save_map_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/slam/save_map',
            self.save_map_callback
        )
        self.start_localization_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/slam/start_localization',
            self.start_localization_callback
        )

        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Create map directory
        os.makedirs(self.config.map_save_path, exist_ok=True)

        self.get_logger().info(
            f'SLAM Manager started with algorithm: {self.config.algorithm}'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map"""
        self.current_map = msg
        self.get_logger().debug(
            f'Received map: {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution}m/cell'
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Process incoming pose estimate"""
        self.current_pose = msg
        self._update_localization_quality(msg)

    def _update_localization_quality(self, pose: PoseWithCovarianceStamped):
        """Update localization quality metrics"""
        # Calculate covariance trace (x, y, yaw)
        cov = pose.pose.covariance
        trace = cov[0] + cov[7] + cov[35]  # xx, yy, yaw-yaw

        self.localization_quality.covariance_trace = trace

        # Determine quality level
        if trace < 0.1:
            self.localization_quality.quality_level = 'good'
        elif trace < 0.5:
            self.localization_quality.quality_level = 'fair'
        elif trace < 1.0:
            self.localization_quality.quality_level = 'poor'
        else:
            self.localization_quality.quality_level = 'lost'

    def start_mapping_callback(self, request, response):
        """Start mapping mode"""
        if self.mode == SLAMMode.MAPPING:
            response.success = False
            response.message = 'Already in mapping mode'
            return response

        self.mode = SLAMMode.MAPPING
        self.current_map = None

        self.get_logger().info('Started mapping mode')

        response.success = True
        response.message = 'Mapping started'
        return response

    def stop_mapping_callback(self, request, response):
        """Stop mapping mode"""
        if self.mode != SLAMMode.MAPPING:
            response.success = False
            response.message = 'Not in mapping mode'
            return response

        self.mode = SLAMMode.IDLE

        self.get_logger().info('Stopped mapping mode')

        response.success = True
        response.message = 'Mapping stopped'
        return response

    def save_map_callback(self, request, response):
        """Save current map"""
        if self.current_map is None:
            response.success = False
            response.message = 'No map available'
            return response

        self.mode = SLAMMode.SAVING

        try:
            map_name = f'map_{self.get_clock().now().nanoseconds}'
            map_path = os.path.join(self.config.map_save_path, map_name)
            os.makedirs(map_path, exist_ok=True)

            # Save map as PGM
            self._save_map_pgm(
                os.path.join(map_path, 'map.pgm'),
                self.current_map
            )

            # Save map metadata as YAML
            self._save_map_yaml(
                os.path.join(map_path, 'map.yaml'),
                self.current_map,
                'map.pgm'
            )

            # Save locations if any
            if self.saved_locations:
                self._save_locations(
                    os.path.join(map_path, 'locations.yaml')
                )

            self.get_logger().info(f'Map saved to: {map_path}')

            response.success = True
            response.message = f'Map saved to {map_path}'

        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            response.success = False
            response.message = f'Failed to save map: {e}'

        finally:
            self.mode = SLAMMode.IDLE

        return response

    def start_localization_callback(self, request, response):
        """Start localization mode"""
        if self.current_map is None:
            response.success = False
            response.message = 'No map loaded'
            return response

        self.mode = SLAMMode.LOCALIZING

        self.get_logger().info('Started localization mode')

        response.success = True
        response.message = 'Localization started'
        return response

    def _save_map_pgm(self, filepath: str, grid: OccupancyGrid):
        """Save occupancy grid as PGM image"""
        width = grid.info.width
        height = grid.info.height
        data = grid.data

        with open(filepath, 'wb') as f:
            # PGM header
            f.write(f'P5\n{width} {height}\n255\n'.encode())

            # Convert occupancy values to grayscale
            # -1 (unknown) -> 205
            # 0 (free) -> 254
            # 100 (occupied) -> 0
            pixels = bytearray(width * height)

            for i, value in enumerate(data):
                if value == -1:
                    pixels[i] = 205  # Unknown
                elif value == 0:
                    pixels[i] = 254  # Free
                elif value >= 100:
                    pixels[i] = 0    # Occupied
                else:
                    # Scale 1-99 to 1-253
                    pixels[i] = int(254 - (value * 254 / 100))

            # Flip vertically (PGM origin is top-left)
            for row in range(height - 1, -1, -1):
                f.write(pixels[row * width:(row + 1) * width])

    def _save_map_yaml(self, filepath: str, grid: OccupancyGrid, image_file: str):
        """Save map metadata as YAML"""
        origin = grid.info.origin

        metadata = {
            'image': image_file,
            'resolution': float(grid.info.resolution),
            'origin': [
                float(origin.position.x),
                float(origin.position.y),
                0.0  # yaw
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }

        with open(filepath, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)

    def _save_locations(self, filepath: str):
        """Save named locations"""
        locations = {
            'locations': [
                {
                    'name': name,
                    'pose': {
                        'x': pose['x'],
                        'y': pose['y'],
                        'theta': pose['theta']
                    }
                }
                for name, pose in self.saved_locations.items()
            ]
        }

        with open(filepath, 'w') as f:
            yaml.dump(locations, f, default_flow_style=False)

    def status_callback(self):
        """Periodic status update"""
        mode_names = {
            SLAMMode.IDLE: 'idle',
            SLAMMode.MAPPING: 'mapping',
            SLAMMode.LOCALIZING: 'localizing',
            SLAMMode.SAVING: 'saving',
        }

        self.get_logger().debug(
            f'SLAM Status: mode={mode_names[self.mode]}, '
            f'map_loaded={self.current_map is not None}, '
            f'localization_quality={self.localization_quality.quality_level}'
        )

    def get_current_pose(self) -> Optional[dict]:
        """Get current robot pose"""
        if self.current_pose is None:
            return None

        pose = self.current_pose.pose.pose
        return {
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': self._quaternion_to_yaw(pose.orientation),
        }

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Convert quaternion to yaw angle"""
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_location(self, name: str):
        """Save current location with a name"""
        pose = self.get_current_pose()
        if pose:
            self.saved_locations[name] = pose
            self.get_logger().info(f'Saved location "{name}": {pose}')
            return True
        return False

    def get_saved_locations(self) -> dict:
        """Get all saved locations"""
        return self.saved_locations.copy()


def main(args=None):
    rclpy.init(args=args)

    node = SLAMManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
