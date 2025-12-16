#!/usr/bin/env python3
"""
WIA Smart Wheelchair Local Planner Node

This node provides local path planning and trajectory tracking
using Dynamic Window Approach (DWA).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan


@dataclass
class DWAConfig:
    """DWA configuration"""
    # Velocity limits
    max_vel_x: float = 1.0       # m/s
    min_vel_x: float = 0.0       # m/s
    max_vel_theta: float = 0.8   # rad/s
    min_vel_theta: float = -0.8  # rad/s

    # Acceleration limits
    acc_lim_x: float = 0.5       # m/s^2
    acc_lim_theta: float = 1.0   # rad/s^2

    # Simulation
    sim_time: float = 1.5        # seconds
    sim_granularity: float = 0.1 # seconds
    vx_samples: int = 20
    vtheta_samples: int = 20

    # Goal tolerance
    xy_goal_tolerance: float = 0.15   # meters
    yaw_goal_tolerance: float = 0.1   # radians

    # Cost weights
    path_distance_bias: float = 32.0
    goal_distance_bias: float = 24.0
    obstacle_cost_bias: float = 0.01

    # Safety
    min_obstacle_dist: float = 0.3    # meters
    robot_radius: float = 0.35        # meters


@dataclass
class Trajectory:
    """Simulated trajectory"""
    velocities: Tuple[float, float]  # (vx, vtheta)
    poses: List[Tuple[float, float, float]]  # [(x, y, theta), ...]
    cost: float = float('inf')


class LocalPlannerNode(Node):
    """
    Local planner for WIA Smart Wheelchair using DWA.

    Subscribes:
        /wia_wheelchair/global_plan (nav_msgs/Path)
        /wia_wheelchair/odom (nav_msgs/Odometry)
        /wia_wheelchair/scan (sensor_msgs/LaserScan)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/local_plan (nav_msgs/Path)
    """

    def __init__(self):
        super().__init__('local_planner')

        # Declare parameters
        self.declare_parameter('max_vel_x', 1.0)
        self.declare_parameter('min_vel_x', 0.0)
        self.declare_parameter('max_vel_theta', 0.8)
        self.declare_parameter('acc_lim_x', 0.5)
        self.declare_parameter('acc_lim_theta', 1.0)
        self.declare_parameter('sim_time', 1.5)
        self.declare_parameter('xy_goal_tolerance', 0.15)
        self.declare_parameter('yaw_goal_tolerance', 0.1)
        self.declare_parameter('min_obstacle_dist', 0.3)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('control_rate', 20.0)

        # Load configuration
        self.config = DWAConfig(
            max_vel_x=self.get_parameter('max_vel_x').value,
            min_vel_x=self.get_parameter('min_vel_x').value,
            max_vel_theta=self.get_parameter('max_vel_theta').value,
            acc_lim_x=self.get_parameter('acc_lim_x').value,
            acc_lim_theta=self.get_parameter('acc_lim_theta').value,
            sim_time=self.get_parameter('sim_time').value,
            xy_goal_tolerance=self.get_parameter('xy_goal_tolerance').value,
            yaw_goal_tolerance=self.get_parameter('yaw_goal_tolerance').value,
            min_obstacle_dist=self.get_parameter('min_obstacle_dist').value,
            robot_radius=self.get_parameter('robot_radius').value,
        )
        control_rate = self.get_parameter('control_rate').value

        # State
        self.global_plan: Optional[Path] = None
        self.current_odom: Optional[Odometry] = None
        self.current_scan: Optional[LaserScan] = None
        self.current_vel = (0.0, 0.0)  # (vx, vtheta)
        self.goal_reached = True

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.plan_sub = self.create_subscription(
            Path,
            '/wia_wheelchair/global_plan',
            self.plan_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wia_wheelchair/odom',
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/wia_wheelchair/scan',
            self.scan_callback,
            sensor_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/wia_wheelchair/cmd_vel',
            10
        )
        self.local_plan_pub = self.create_publisher(
            Path,
            '/wia_wheelchair/local_plan',
            10
        )

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )

        self.get_logger().info('Local planner (DWA) started')

    def plan_callback(self, msg: Path):
        """Receive global plan"""
        self.global_plan = msg
        self.goal_reached = False
        self.get_logger().info(f'Received global plan with {len(msg.poses)} poses')

    def odom_callback(self, msg: Odometry):
        """Update odometry"""
        self.current_odom = msg
        self.current_vel = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

    def scan_callback(self, msg: LaserScan):
        """Update laser scan"""
        self.current_scan = msg

    def control_callback(self):
        """Main control loop"""
        if self.goal_reached:
            return

        if self.global_plan is None or self.current_odom is None:
            return

        # Get current pose
        pose = self.current_odom.pose.pose
        current_x = pose.position.x
        current_y = pose.position.y
        current_theta = self._quaternion_to_yaw(pose.orientation)

        # Find local goal on path
        local_goal = self._find_local_goal(current_x, current_y)

        if local_goal is None:
            self._publish_stop()
            return

        # Check if goal reached
        if self._is_goal_reached(current_x, current_y, current_theta):
            self.goal_reached = True
            self._publish_stop()
            self.get_logger().info('Goal reached!')
            return

        # Get obstacles from scan
        obstacles = self._get_obstacles_from_scan()

        # Run DWA
        best_trajectory = self._dwa(
            current_x, current_y, current_theta,
            local_goal,
            obstacles
        )

        if best_trajectory:
            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = best_trajectory.velocities[0]
            cmd.angular.z = best_trajectory.velocities[1]
            self.cmd_pub.publish(cmd)

            # Publish local plan
            self._publish_local_plan(best_trajectory, current_x, current_y, current_theta)
        else:
            # No valid trajectory, stop
            self._publish_stop()
            self.get_logger().warn('No valid trajectory found')

    def _find_local_goal(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Find local goal point on global path"""
        if not self.global_plan or not self.global_plan.poses:
            return None

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0

        for i, pose in enumerate(self.global_plan.poses):
            dist = math.sqrt(
                (pose.pose.position.x - x) ** 2 +
                (pose.pose.position.y - y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Look ahead on path
        lookahead_dist = self.config.max_vel_x * self.config.sim_time
        accumulated_dist = 0.0
        goal_idx = closest_idx

        for i in range(closest_idx, len(self.global_plan.poses) - 1):
            p1 = self.global_plan.poses[i].pose.position
            p2 = self.global_plan.poses[i + 1].pose.position
            dist = math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
            accumulated_dist += dist

            if accumulated_dist >= lookahead_dist:
                goal_idx = i + 1
                break
            goal_idx = i + 1

        goal_pose = self.global_plan.poses[min(goal_idx, len(self.global_plan.poses) - 1)]
        return (goal_pose.pose.position.x, goal_pose.pose.position.y)

    def _is_goal_reached(self, x: float, y: float, theta: float) -> bool:
        """Check if final goal is reached"""
        if not self.global_plan or not self.global_plan.poses:
            return True

        final_pose = self.global_plan.poses[-1].pose
        dist = math.sqrt(
            (final_pose.position.x - x) ** 2 +
            (final_pose.position.y - y) ** 2
        )

        if dist > self.config.xy_goal_tolerance:
            return False

        goal_theta = self._quaternion_to_yaw(final_pose.orientation)
        angle_diff = abs(self._normalize_angle(goal_theta - theta))

        return angle_diff <= self.config.yaw_goal_tolerance

    def _get_obstacles_from_scan(self) -> List[Tuple[float, float]]:
        """Extract obstacle points from laser scan"""
        obstacles = []

        if self.current_scan is None:
            return obstacles

        scan = self.current_scan
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            if r > 5.0:  # Limit range
                continue

            angle = scan.angle_min + i * scan.angle_increment
            obstacles.append((
                r * math.cos(angle),
                r * math.sin(angle)
            ))

        return obstacles

    def _dwa(
        self,
        x: float, y: float, theta: float,
        goal: Tuple[float, float],
        obstacles: List[Tuple[float, float]]
    ) -> Optional[Trajectory]:
        """Dynamic Window Approach"""
        # Calculate dynamic window
        vx_min = max(self.config.min_vel_x,
                     self.current_vel[0] - self.config.acc_lim_x * 0.1)
        vx_max = min(self.config.max_vel_x,
                     self.current_vel[0] + self.config.acc_lim_x * 0.1)
        vth_min = max(self.config.min_vel_theta,
                      self.current_vel[1] - self.config.acc_lim_theta * 0.1)
        vth_max = min(self.config.max_vel_theta,
                      self.current_vel[1] + self.config.acc_lim_theta * 0.1)

        # Sample velocities
        vx_samples = np.linspace(vx_min, vx_max, self.config.vx_samples)
        vth_samples = np.linspace(vth_min, vth_max, self.config.vtheta_samples)

        best_trajectory = None
        best_cost = float('inf')

        for vx in vx_samples:
            for vth in vth_samples:
                # Simulate trajectory
                traj = self._simulate_trajectory(x, y, theta, vx, vth)

                # Check for collisions
                if self._check_collision(traj, obstacles):
                    continue

                # Calculate cost
                cost = self._calculate_cost(traj, goal)
                traj.cost = cost

                if cost < best_cost:
                    best_cost = cost
                    best_trajectory = traj

        return best_trajectory

    def _simulate_trajectory(
        self,
        x: float, y: float, theta: float,
        vx: float, vth: float
    ) -> Trajectory:
        """Simulate trajectory for given velocities"""
        poses = [(x, y, theta)]
        t = 0.0
        dt = self.config.sim_granularity

        while t < self.config.sim_time:
            theta = theta + vth * dt
            x = x + vx * math.cos(theta) * dt
            y = y + vx * math.sin(theta) * dt
            poses.append((x, y, theta))
            t += dt

        return Trajectory(
            velocities=(vx, vth),
            poses=poses
        )

    def _check_collision(
        self,
        traj: Trajectory,
        obstacles: List[Tuple[float, float]]
    ) -> bool:
        """Check if trajectory collides with obstacles"""
        for pose in traj.poses:
            for obs in obstacles:
                # Transform obstacle to trajectory frame
                dx = obs[0] - pose[0]
                dy = obs[1] - pose[1]
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < self.config.robot_radius + self.config.min_obstacle_dist:
                    return True

        return False

    def _calculate_cost(
        self,
        traj: Trajectory,
        goal: Tuple[float, float]
    ) -> float:
        """Calculate trajectory cost"""
        final_pose = traj.poses[-1]

        # Goal distance cost
        goal_dist = math.sqrt(
            (final_pose[0] - goal[0]) ** 2 +
            (final_pose[1] - goal[1]) ** 2
        )
        goal_cost = goal_dist * self.config.goal_distance_bias

        # Heading cost (align with goal)
        goal_angle = math.atan2(goal[1] - final_pose[1], goal[0] - final_pose[0])
        heading_cost = abs(self._normalize_angle(goal_angle - final_pose[2])) * 10.0

        # Speed cost (prefer higher speed)
        speed_cost = (self.config.max_vel_x - traj.velocities[0]) * 5.0

        return goal_cost + heading_cost + speed_cost

    def _publish_stop(self):
        """Publish zero velocity"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _publish_local_plan(
        self,
        traj: Trajectory,
        x: float, y: float, theta: float
    ):
        """Publish local plan visualization"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        for pose in traj.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pose[0]
            ps.pose.position.y = pose[1]
            ps.pose.orientation.z = math.sin(pose[2] / 2)
            ps.pose.orientation.w = math.cos(pose[2] / 2)
            msg.poses.append(ps)

        self.local_plan_pub.publish(msg)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Convert quaternion to yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
