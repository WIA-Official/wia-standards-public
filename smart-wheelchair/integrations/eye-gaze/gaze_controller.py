#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Eye Gaze Controller

Enables gaze-based wheelchair control using eye tracking input.
Supports direction control, destination selection, and gaze+switch modes.
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class GazeControlMode(IntEnum):
    """Gaze control modes"""
    DISABLED = 0
    DIRECTION = 1      # Gaze controls direction/speed
    GOAL_SELECT = 2    # Gaze selects destination on map
    SWITCH_COMBO = 3   # Gaze + switch combination


@dataclass
class GazePoint:
    """Normalized gaze point (0-1 range)"""
    x: float  # 0=left, 0.5=center, 1=right
    y: float  # 0=top, 0.5=center, 1=bottom
    timestamp: float = 0.0
    confidence: float = 1.0


@dataclass
class GazeConfig:
    """Gaze control configuration"""
    # Sensitivity
    sensitivity: float = 1.0          # 0.1 - 2.0
    deadzone_radius: float = 0.15     # Screen center ignore zone

    # Velocity limits
    max_linear_speed: float = 0.5     # m/s
    max_angular_speed: float = 1.0    # rad/s

    # Dwell settings
    dwell_enabled: bool = True
    dwell_time: float = 1.0           # seconds to confirm
    dwell_radius: float = 0.1         # normalized radius

    # Smoothing
    smoothing_factor: float = 0.3     # Low-pass filter (0-1)

    # Safety
    require_confirmation: bool = True
    auto_stop_on_lost_gaze: bool = True
    lost_gaze_timeout: float = 0.5    # seconds


@dataclass
class GazeRegion:
    """Interactive region on screen"""
    name: str
    x: float
    y: float
    width: float
    height: float
    action: str
    data: Optional[dict] = None


@dataclass
class DwellState:
    """Dwell detection state"""
    target: Optional[GazeRegion] = None
    start_time: float = 0.0
    accumulated_time: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0


class GazeToVelocityConverter:
    """Converts gaze position to velocity commands"""

    def __init__(self, config: GazeConfig):
        self.config = config
        self._last_gaze: Optional[GazePoint] = None
        self._smoothed_x: float = 0.5
        self._smoothed_y: float = 0.5

    def convert(self, gaze: GazePoint) -> Twist:
        """Convert gaze point to velocity command"""
        # Apply smoothing
        alpha = self.config.smoothing_factor
        self._smoothed_x = alpha * gaze.x + (1 - alpha) * self._smoothed_x
        self._smoothed_y = alpha * gaze.y + (1 - alpha) * self._smoothed_y

        # Calculate offset from center
        offset_x = self._smoothed_x - 0.5
        offset_y = self._smoothed_y - 0.5

        # Check deadzone
        distance = math.sqrt(offset_x**2 + offset_y**2)
        if distance < self.config.deadzone_radius:
            return Twist()  # Zero velocity

        # Apply deadzone compensation
        effective_distance = (distance - self.config.deadzone_radius) / (0.5 - self.config.deadzone_radius)
        effective_distance = min(1.0, effective_distance)

        # Normalize direction
        if distance > 0:
            dir_x = offset_x / distance
            dir_y = offset_y / distance
        else:
            dir_x, dir_y = 0.0, 0.0

        # Apply sensitivity
        magnitude = effective_distance * self.config.sensitivity
        magnitude = min(1.0, magnitude)

        # Convert to velocity
        twist = Twist()

        # Y offset (up/down) controls linear velocity
        # Up = forward, Down = backward
        twist.linear.x = -dir_y * magnitude * self.config.max_linear_speed

        # X offset (left/right) controls angular velocity
        # Left = turn left (positive), Right = turn right (negative)
        twist.angular.z = -dir_x * magnitude * self.config.max_angular_speed

        self._last_gaze = gaze
        return twist

    def reset(self):
        """Reset smoothing state"""
        self._smoothed_x = 0.5
        self._smoothed_y = 0.5
        self._last_gaze = None


class DwellDetector:
    """Detects dwell (fixation) on targets"""

    def __init__(self, config: GazeConfig):
        self.config = config
        self.state = DwellState()
        self._regions: List[GazeRegion] = []

    def set_regions(self, regions: List[GazeRegion]):
        """Set interactive regions"""
        self._regions = regions

    def update(self, gaze: GazePoint, current_time: float) -> Optional[GazeRegion]:
        """
        Update dwell detection.
        Returns the region if dwell completed, None otherwise.
        """
        if not self.config.dwell_enabled:
            return None

        # Find region under gaze
        target_region = self._find_region(gaze.x, gaze.y)

        if target_region is None:
            # Reset if not looking at any region
            self.state = DwellState()
            return None

        if self.state.target != target_region:
            # Started looking at new region
            self.state = DwellState(
                target=target_region,
                start_time=current_time,
                center_x=gaze.x,
                center_y=gaze.y
            )
            return None

        # Check if gaze stayed within radius
        dx = gaze.x - self.state.center_x
        dy = gaze.y - self.state.center_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > self.config.dwell_radius:
            # Gaze moved too far, reset
            self.state.center_x = gaze.x
            self.state.center_y = gaze.y
            self.state.start_time = current_time
            return None

        # Accumulate dwell time
        self.state.accumulated_time = current_time - self.state.start_time

        # Check if dwell completed
        if self.state.accumulated_time >= self.config.dwell_time:
            completed_region = self.state.target
            self.state = DwellState()  # Reset
            return completed_region

        return None

    def get_progress(self) -> float:
        """Get current dwell progress (0-1)"""
        if self.state.target is None:
            return 0.0
        return min(1.0, self.state.accumulated_time / self.config.dwell_time)

    def _find_region(self, x: float, y: float) -> Optional[GazeRegion]:
        """Find region containing point"""
        for region in self._regions:
            if (region.x <= x <= region.x + region.width and
                region.y <= y <= region.y + region.height):
                return region
        return None


class GazeControllerNode(Node):
    """
    ROS2 node for gaze-based wheelchair control.

    Subscribes:
        /wia_wheelchair/gaze_point (geometry_msgs/Point)
        /wia_wheelchair/switch_input (std_msgs/Bool)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/goal_pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/gaze_status (std_msgs/String)

    Services:
        /wia_wheelchair/gaze/enable
        /wia_wheelchair/gaze/set_mode
    """

    def __init__(self):
        super().__init__('gaze_controller')

        # Declare parameters
        self.declare_parameter('sensitivity', 1.0)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('dwell_time', 1.0)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('control_rate', 20.0)

        # Load config
        self.config = GazeConfig(
            sensitivity=self.get_parameter('sensitivity').value,
            deadzone_radius=self.get_parameter('deadzone').value,
            max_linear_speed=self.get_parameter('max_linear_speed').value,
            max_angular_speed=self.get_parameter('max_angular_speed').value,
            dwell_time=self.get_parameter('dwell_time').value,
            smoothing_factor=self.get_parameter('smoothing').value,
        )

        # State
        self.enabled = False
        self.mode = GazeControlMode.DISABLED
        self.current_gaze: Optional[GazePoint] = None
        self.last_gaze_time = 0.0
        self.switch_pressed = False

        # Components
        self.velocity_converter = GazeToVelocityConverter(self.config)
        self.dwell_detector = DwellDetector(self.config)

        # Saved locations for goal selection
        self.locations: Dict[str, PoseStamped] = {}
        self._setup_default_regions()

        # Subscribers
        from geometry_msgs.msg import Point
        self.gaze_sub = self.create_subscription(
            Point,
            '/wia_wheelchair/gaze_point',
            self.gaze_callback,
            10
        )
        self.switch_sub = self.create_subscription(
            Bool,
            '/wia_wheelchair/switch_input',
            self.switch_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/wia_wheelchair/cmd_vel',
            10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/wia_wheelchair/goal_pose',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/gaze_status',
            10
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/gaze/enable',
            self.enable_callback
        )

        # Control timer
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info('Gaze controller initialized')

    def _setup_default_regions(self):
        """Setup default screen regions for goal selection"""
        # Create grid of location buttons
        regions = []

        # Navigation regions (example layout)
        nav_regions = [
            ('forward', 0.4, 0.1, 0.2, 0.2, 'velocity', {'linear': 0.5, 'angular': 0.0}),
            ('backward', 0.4, 0.7, 0.2, 0.2, 'velocity', {'linear': -0.3, 'angular': 0.0}),
            ('left', 0.1, 0.4, 0.2, 0.2, 'velocity', {'linear': 0.0, 'angular': 0.5}),
            ('right', 0.7, 0.4, 0.2, 0.2, 'velocity', {'linear': 0.0, 'angular': -0.5}),
            ('stop', 0.4, 0.4, 0.2, 0.2, 'stop', None),
        ]

        for name, x, y, w, h, action, data in nav_regions:
            regions.append(GazeRegion(name, x, y, w, h, action, data))

        self.dwell_detector.set_regions(regions)

    def gaze_callback(self, msg):
        """Receive gaze point from eye tracker"""
        self.current_gaze = GazePoint(
            x=msg.x,
            y=msg.y,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        self.last_gaze_time = self.current_gaze.timestamp

    def switch_callback(self, msg: Bool):
        """Receive switch input"""
        was_pressed = self.switch_pressed
        self.switch_pressed = msg.data

        # Detect rising edge (switch just pressed)
        if msg.data and not was_pressed:
            self._handle_switch_press()

    def _handle_switch_press(self):
        """Handle switch press event"""
        if self.mode == GazeControlMode.SWITCH_COMBO:
            # In combo mode, switch confirms gaze selection
            if self.dwell_detector.state.target:
                self._execute_region_action(self.dwell_detector.state.target)

    def enable_callback(self, request, response):
        """Enable/disable gaze control"""
        self.enabled = request.data
        if self.enabled:
            self.mode = GazeControlMode.DIRECTION
            response.message = 'Gaze control enabled'
        else:
            self.mode = GazeControlMode.DISABLED
            self._stop_wheelchair()
            response.message = 'Gaze control disabled'

        response.success = True
        return response

    def control_loop(self):
        """Main control loop"""
        if not self.enabled or self.mode == GazeControlMode.DISABLED:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Check for lost gaze
        if self.current_gaze is None:
            return

        gaze_age = current_time - self.last_gaze_time
        if gaze_age > self.config.lost_gaze_timeout:
            if self.config.auto_stop_on_lost_gaze:
                self._stop_wheelchair()
            self._publish_status('gaze_lost')
            return

        # Process based on mode
        if self.mode == GazeControlMode.DIRECTION:
            self._process_direction_mode()
        elif self.mode == GazeControlMode.GOAL_SELECT:
            self._process_goal_select_mode(current_time)
        elif self.mode == GazeControlMode.SWITCH_COMBO:
            self._process_switch_combo_mode(current_time)

    def _process_direction_mode(self):
        """Direct gaze-to-velocity control"""
        if self.current_gaze is None:
            return

        twist = self.velocity_converter.convert(self.current_gaze)
        self.cmd_vel_pub.publish(twist)

        self._publish_status(f'direction:linear={twist.linear.x:.2f},angular={twist.angular.z:.2f}')

    def _process_goal_select_mode(self, current_time: float):
        """Gaze-based destination selection with dwell"""
        if self.current_gaze is None:
            return

        # Check for dwell completion
        completed = self.dwell_detector.update(self.current_gaze, current_time)

        if completed:
            self._execute_region_action(completed)
        else:
            progress = self.dwell_detector.get_progress()
            target = self.dwell_detector.state.target
            if target:
                self._publish_status(f'dwell:{target.name}:{progress:.0%}')

    def _process_switch_combo_mode(self, current_time: float):
        """Gaze to select, switch to confirm"""
        if self.current_gaze is None:
            return

        # Update dwell detector to track what we're looking at
        # But don't auto-confirm - wait for switch
        self.dwell_detector.update(self.current_gaze, current_time)

        target = self.dwell_detector.state.target
        if target:
            self._publish_status(f'switch_combo:looking_at:{target.name}')

    def _execute_region_action(self, region: GazeRegion):
        """Execute action for selected region"""
        self.get_logger().info(f'Executing action: {region.action} for {region.name}')

        if region.action == 'velocity':
            # Direct velocity command
            twist = Twist()
            twist.linear.x = region.data.get('linear', 0.0)
            twist.angular.z = region.data.get('angular', 0.0)
            self.cmd_vel_pub.publish(twist)

        elif region.action == 'stop':
            self._stop_wheelchair()

        elif region.action == 'goal':
            # Navigate to saved location
            if region.name in self.locations:
                self.goal_pub.publish(self.locations[region.name])

        self._publish_status(f'executed:{region.name}')

    def _stop_wheelchair(self):
        """Send stop command"""
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str):
        """Publish gaze control status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def set_locations(self, locations: Dict[str, PoseStamped]):
        """Set available navigation locations"""
        self.locations = locations

        # Update regions for goal selection
        regions = []
        cols = 3
        for i, (name, pose) in enumerate(locations.items()):
            row = i // cols
            col = i % cols
            x = 0.1 + col * 0.3
            y = 0.1 + row * 0.2
            regions.append(GazeRegion(name, x, y, 0.25, 0.15, 'goal', None))

        self.dwell_detector.set_regions(regions)


def main(args=None):
    rclpy.init(args=args)

    node = GazeControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
