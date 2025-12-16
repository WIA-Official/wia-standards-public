#!/usr/bin/env python3
"""
WIA Smart Wheelchair Goal Manager Node

This node manages navigation goals, saved locations,
and provides high-level navigation control.
"""

import math
import os
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose2D
from std_msgs.msg import String
from std_srvs.srv import Trigger

from wia_wheelchair_msgs.msg import NavigationStatus
from wia_wheelchair_msgs.action import GoToPose


class NavState(IntEnum):
    """Navigation state"""
    IDLE = 0
    PLANNING = 1
    NAVIGATING = 2
    RECOVERING = 3
    ARRIVED = 4
    FAILED = 5
    CANCELLED = 6


@dataclass
class SavedLocation:
    """Saved location data"""
    name: str
    x: float
    y: float
    theta: float
    description: str = ''


@dataclass
class NavigationProgress:
    """Navigation progress"""
    state: NavState = NavState.IDLE
    distance_remaining: float = 0.0
    estimated_time: float = 0.0
    percent_complete: float = 0.0
    current_waypoint: int = 0
    total_waypoints: int = 0


class GoalManagerNode(Node):
    """
    Goal manager for WIA Smart Wheelchair.

    Manages navigation goals and saved locations.

    Subscribes:
        /wia_wheelchair/odom (nav_msgs/Odometry)
        /wia_wheelchair/global_plan (nav_msgs/Path)

    Publishes:
        /wia_wheelchair/goal_pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/navigation_status (wia_wheelchair_msgs/NavigationStatus)

    Services:
        /wia_wheelchair/nav/cancel
        /wia_wheelchair/nav/save_location
        /wia_wheelchair/nav/list_locations

    Actions:
        /wia_wheelchair/navigate_to_pose
    """

    def __init__(self):
        super().__init__('goal_manager')

        # Declare parameters
        self.declare_parameter('locations_file', '')
        self.declare_parameter('default_speed', 0.5)
        self.declare_parameter('status_rate', 5.0)

        # Load configuration
        locations_file = self.get_parameter('locations_file').value
        self.default_speed = self.get_parameter('default_speed').value
        status_rate = self.get_parameter('status_rate').value

        # State
        self.state = NavState.IDLE
        self.current_goal: Optional[PoseStamped] = None
        self.current_plan: Optional[Path] = None
        self.current_odom: Optional[Odometry] = None
        self.progress = NavigationProgress()
        self.saved_locations: Dict[str, SavedLocation] = {}

        # Load saved locations
        if locations_file and os.path.exists(locations_file):
            self._load_locations(locations_file)

        # Callback group for actions
        self.cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wia_wheelchair/odom',
            self.odom_callback,
            10
        )
        self.plan_sub = self.create_subscription(
            Path,
            '/wia_wheelchair/global_plan',
            self.plan_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/wia_wheelchair/goal_pose',
            10
        )
        self.status_pub = self.create_publisher(
            NavigationStatus,
            '/wia_wheelchair/navigation_status',
            10
        )

        # Services
        self.cancel_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/nav/cancel',
            self.cancel_callback
        )

        # Action server
        self.nav_action_server = ActionServer(
            self,
            GoToPose,
            '/wia_wheelchair/navigate_to_pose',
            execute_callback=self.execute_nav_callback,
            goal_callback=self.goal_accept_callback,
            cancel_callback=self.cancel_accept_callback,
            callback_group=self.cb_group
        )

        # Status timer
        self.status_timer = self.create_timer(
            1.0 / status_rate,
            self.status_callback
        )

        self.get_logger().info('Goal manager started')

    def odom_callback(self, msg: Odometry):
        """Update odometry"""
        self.current_odom = msg
        self._update_progress()

    def plan_callback(self, msg: Path):
        """Receive global plan"""
        self.current_plan = msg
        self.progress.total_waypoints = len(msg.poses)

        if self.state == NavState.PLANNING:
            self.state = NavState.NAVIGATING

    def goal_accept_callback(self, goal_request):
        """Accept or reject navigation goal"""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def cancel_accept_callback(self, goal_handle):
        """Accept or reject cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_nav_callback(self, goal_handle):
        """Execute navigation action"""
        self.get_logger().info('Executing navigation goal')

        # Get goal
        goal = goal_handle.request
        target_pose = goal.target_pose
        speed_factor = goal.speed_factor if goal.speed_factor > 0 else 1.0

        # Set state
        self.state = NavState.PLANNING
        self.current_goal = target_pose

        # Publish goal to planner
        self.goal_pub.publish(target_pose)

        # Wait for plan
        rate = self.create_rate(10)
        timeout = 10.0
        elapsed = 0.0

        while self.state == NavState.PLANNING and elapsed < timeout:
            if goal_handle.is_cancel_requested:
                self._cancel_navigation()
                goal_handle.canceled()
                return GoToPose.Result(
                    success=False,
                    message='Navigation cancelled'
                )
            rate.sleep()
            elapsed += 0.1

        if self.state != NavState.NAVIGATING:
            self.state = NavState.FAILED
            goal_handle.abort()
            return GoToPose.Result(
                success=False,
                message='Failed to plan path'
            )

        # Navigate
        while self.state == NavState.NAVIGATING:
            if goal_handle.is_cancel_requested:
                self._cancel_navigation()
                goal_handle.canceled()
                return GoToPose.Result(
                    success=False,
                    message='Navigation cancelled'
                )

            # Check if arrived
            if self._is_at_goal(target_pose):
                self.state = NavState.ARRIVED
                break

            # Publish feedback
            feedback = GoToPose.Feedback()
            if self.current_odom:
                feedback.current_pose = PoseStamped()
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback.current_pose.header.frame_id = 'map'
                feedback.current_pose.pose = self.current_odom.pose.pose

            feedback.distance_remaining = self.progress.distance_remaining
            feedback.estimated_time_remaining = self.progress.estimated_time
            feedback.status = self.state

            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Complete
        self.state = NavState.IDLE
        self.current_goal = None
        self.current_plan = None

        goal_handle.succeed()

        result = GoToPose.Result()
        result.success = True
        result.message = 'Goal reached'
        if self.current_odom:
            result.final_pose = PoseStamped()
            result.final_pose.pose = self.current_odom.pose.pose

        return result

    def cancel_callback(self, request, response):
        """Cancel current navigation"""
        self._cancel_navigation()
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    def _cancel_navigation(self):
        """Cancel navigation"""
        self.state = NavState.CANCELLED
        self.current_goal = None
        self.current_plan = None
        self.get_logger().info('Navigation cancelled')

    def _is_at_goal(self, goal: PoseStamped) -> bool:
        """Check if robot is at goal"""
        if self.current_odom is None:
            return False

        pose = self.current_odom.pose.pose
        dx = goal.pose.position.x - pose.position.x
        dy = goal.pose.position.y - pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        return dist < 0.15  # xy tolerance

    def _update_progress(self):
        """Update navigation progress"""
        if self.current_plan is None or self.current_odom is None:
            return

        pose = self.current_odom.pose.pose
        poses = self.current_plan.poses

        if not poses:
            return

        # Find closest waypoint
        min_dist = float('inf')
        closest_idx = 0

        for i, wp in enumerate(poses):
            dist = math.sqrt(
                (wp.pose.position.x - pose.position.x) ** 2 +
                (wp.pose.position.y - pose.position.y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Calculate remaining distance
        remaining = 0.0
        for i in range(closest_idx, len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            remaining += math.sqrt(
                (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2
            )

        self.progress.distance_remaining = remaining
        self.progress.current_waypoint = closest_idx
        self.progress.total_waypoints = len(poses)
        self.progress.percent_complete = (
            100.0 * closest_idx / len(poses) if poses else 0.0
        )

        # Estimate time
        speed = self.default_speed
        if self.current_odom:
            speed = max(0.1, abs(self.current_odom.twist.twist.linear.x))
        self.progress.estimated_time = remaining / speed

    def status_callback(self):
        """Publish navigation status"""
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Map state to status
        state_map = {
            NavState.IDLE: NavigationStatus.STATUS_IDLE,
            NavState.PLANNING: NavigationStatus.STATUS_IDLE,
            NavState.NAVIGATING: NavigationStatus.STATUS_NAVIGATING,
            NavState.RECOVERING: NavigationStatus.STATUS_RECOVERING,
            NavState.ARRIVED: NavigationStatus.STATUS_GOAL_REACHED,
            NavState.FAILED: NavigationStatus.STATUS_STUCK,
            NavState.CANCELLED: NavigationStatus.STATUS_PAUSED,
        }
        msg.status = state_map.get(self.state, NavigationStatus.STATUS_IDLE)

        if self.current_goal:
            msg.current_goal = self.current_goal

        msg.distance_to_goal = self.progress.distance_remaining
        msg.estimated_time = self.progress.estimated_time
        msg.waypoints_total = self.progress.total_waypoints
        msg.waypoints_completed = self.progress.current_waypoint
        msg.progress = self.progress.percent_complete / 100.0

        self.status_pub.publish(msg)

    # Location management

    def save_location(self, name: str, description: str = '') -> bool:
        """Save current location"""
        if self.current_odom is None:
            return False

        pose = self.current_odom.pose.pose
        theta = self._quaternion_to_yaw(pose.orientation)

        location = SavedLocation(
            name=name,
            x=pose.position.x,
            y=pose.position.y,
            theta=theta,
            description=description
        )

        self.saved_locations[name] = location
        self.get_logger().info(f'Saved location: {name}')
        return True

    def get_location(self, name: str) -> Optional[SavedLocation]:
        """Get saved location"""
        return self.saved_locations.get(name)

    def list_locations(self) -> list:
        """List all saved locations"""
        return list(self.saved_locations.keys())

    def navigate_to_location(self, name: str) -> bool:
        """Navigate to saved location"""
        location = self.get_location(name)
        if location is None:
            self.get_logger().warn(f'Unknown location: {name}')
            return False

        # Create goal
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = location.x
        goal.pose.position.y = location.y
        goal.pose.orientation.z = math.sin(location.theta / 2)
        goal.pose.orientation.w = math.cos(location.theta / 2)

        # Publish goal
        self.state = NavState.PLANNING
        self.current_goal = goal
        self.goal_pub.publish(goal)

        self.get_logger().info(f'Navigating to: {name}')
        return True

    def _load_locations(self, filepath: str):
        """Load saved locations from file"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            for loc in data.get('locations', []):
                pose = loc.get('pose', {})
                location = SavedLocation(
                    name=loc['name'],
                    x=pose.get('x', 0.0),
                    y=pose.get('y', 0.0),
                    theta=pose.get('theta', 0.0),
                    description=loc.get('description', '')
                )
                self.saved_locations[location.name] = location

            self.get_logger().info(
                f'Loaded {len(self.saved_locations)} locations'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load locations: {e}')

    def save_locations_to_file(self, filepath: str):
        """Save locations to file"""
        data = {
            'locations': [
                {
                    'name': loc.name,
                    'pose': {
                        'x': loc.x,
                        'y': loc.y,
                        'theta': loc.theta
                    },
                    'description': loc.description
                }
                for loc in self.saved_locations.values()
            ]
        }

        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Convert quaternion to yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = GoalManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
