#!/usr/bin/env python3
"""
WIA Smart Wheelchair Sensor Fusion Node

This node fuses IMU and wheel encoder data to produce
accurate odometry estimates using an Extended Kalman Filter.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Quaternion,
    TransformStamped,
    Twist,
    Pose,
    Point,
    Vector3,
)
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

from wia_wheelchair_msgs.msg import WheelchairState


@dataclass
class DifferentialDriveParams:
    """Parameters for differential drive kinematics"""
    wheel_base: float = 0.55      # meters (distance between wheels)
    wheel_radius: float = 0.15    # meters


@dataclass
class EKFState:
    """Extended Kalman Filter state"""
    x: float = 0.0          # position x (meters)
    y: float = 0.0          # position y (meters)
    theta: float = 0.0      # heading (radians)
    vx: float = 0.0         # linear velocity (m/s)
    omega: float = 0.0      # angular velocity (rad/s)

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta, self.vx, self.omega])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'EKFState':
        return cls(x=arr[0], y=arr[1], theta=arr[2], vx=arr[3], omega=arr[4])


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for wheelchair localization.

    State vector: [x, y, theta, vx, omega]
    """

    def __init__(
        self,
        process_noise: np.ndarray,
        measurement_noise_imu: np.ndarray,
        measurement_noise_encoder: np.ndarray,
    ):
        self.state = np.zeros(5)
        self.covariance = np.eye(5) * 0.1

        self.Q = process_noise
        self.R_imu = measurement_noise_imu
        self.R_encoder = measurement_noise_encoder

        self.last_update_time: Optional[float] = None

    def predict(self, dt: float):
        """Prediction step using motion model"""
        if dt <= 0:
            return

        x, y, theta, vx, omega = self.state

        # Motion model (constant velocity)
        x_new = x + vx * math.cos(theta) * dt
        y_new = y + vx * math.sin(theta) * dt
        theta_new = self._normalize_angle(theta + omega * dt)
        vx_new = vx
        omega_new = omega

        self.state = np.array([x_new, y_new, theta_new, vx_new, omega_new])

        # Jacobian of motion model
        F = np.array([
            [1, 0, -vx * math.sin(theta) * dt, math.cos(theta) * dt, 0],
            [0, 1, vx * math.cos(theta) * dt, math.sin(theta) * dt, 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
        ])

        # Update covariance
        self.covariance = F @ self.covariance @ F.T + self.Q * dt

    def update_imu(self, angular_velocity: float, linear_accel: float):
        """Update with IMU measurement"""
        # Measurement: [omega, ax]
        z = np.array([angular_velocity, linear_accel])

        # Measurement model: H @ state = [omega, 0] (simplified)
        # We use omega directly, and acceleration is used to correct vx drift
        H = np.array([
            [0, 0, 0, 0, 1],  # omega
            [0, 0, 0, 0, 0],  # ax (not directly measured)
        ])

        # Predicted measurement
        z_pred = np.array([self.state[4], 0])

        # Innovation
        y = z - z_pred

        # Kalman gain (simplified for angular velocity only)
        S = H @ self.covariance @ H.T + self.R_imu
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y
        self.state[2] = self._normalize_angle(self.state[2])

        # Update covariance
        I = np.eye(5)
        self.covariance = (I - K @ H) @ self.covariance

    def update_encoders(self, left_vel: float, right_vel: float, params: DifferentialDriveParams):
        """Update with wheel encoder measurement"""
        # Convert wheel velocities to robot velocities
        v_measured = (left_vel + right_vel) / 2.0 * params.wheel_radius
        omega_measured = (right_vel - left_vel) / params.wheel_base * params.wheel_radius

        # Measurement
        z = np.array([v_measured, omega_measured])

        # Measurement model
        H = np.array([
            [0, 0, 0, 1, 0],  # vx
            [0, 0, 0, 0, 1],  # omega
        ])

        # Predicted measurement
        z_pred = H @ self.state

        # Innovation
        y = z - z_pred

        # Kalman gain
        S = H @ self.covariance @ H.T + self.R_encoder
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y
        self.state[2] = self._normalize_angle(self.state[2])

        # Update covariance
        I = np.eye(5)
        self.covariance = (I - K @ H) @ self.covariance

    def get_state(self) -> EKFState:
        """Get current state estimate"""
        return EKFState.from_array(self.state)

    def get_covariance_6x6(self) -> np.ndarray:
        """Get 6x6 pose covariance for ROS message"""
        cov = np.zeros((6, 6))
        # Map [x, y, theta] to [x, y, z, roll, pitch, yaw]
        cov[0, 0] = self.covariance[0, 0]  # x variance
        cov[1, 1] = self.covariance[1, 1]  # y variance
        cov[5, 5] = self.covariance[2, 2]  # yaw variance
        cov[0, 1] = cov[1, 0] = self.covariance[0, 1]  # x-y covariance
        cov[0, 5] = cov[5, 0] = self.covariance[0, 2]  # x-yaw covariance
        cov[1, 5] = cov[5, 1] = self.covariance[1, 2]  # y-yaw covariance
        return cov

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset filter state"""
        self.state = np.array([x, y, theta, 0.0, 0.0])
        self.covariance = np.eye(5) * 0.1
        self.last_update_time = None

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class SensorFusionNode(Node):
    """
    Sensor fusion node for WIA Smart Wheelchair.

    Subscribes to:
        /wia_wheelchair/imu (sensor_msgs/Imu)
        /wia_wheelchair/joint_states (sensor_msgs/JointState)

    Publishes:
        /wia_wheelchair/odom (nav_msgs/Odometry)
        /wia_wheelchair/state (wia_wheelchair_msgs/WheelchairState)
        TF: odom -> base_link
    """

    def __init__(self):
        super().__init__('sensor_fusion')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.55)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Load parameters
        self.drive_params = DifferentialDriveParams(
            wheel_base=self.get_parameter('wheel_base').value,
            wheel_radius=self.get_parameter('wheel_radius').value,
        )
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Initialize EKF
        process_noise = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        imu_noise = np.diag([0.01, 0.1])
        encoder_noise = np.diag([0.01, 0.01])

        self.ekf = ExtendedKalmanFilter(process_noise, imu_noise, encoder_noise)
        self.last_time: Optional[Time] = None

        # Encoder state
        self.last_left_pos: Optional[float] = None
        self.last_right_pos: Optional[float] = None
        self.left_velocity: float = 0.0
        self.right_velocity: float = 0.0

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/wia_wheelchair/imu',
            self.imu_callback,
            sensor_qos
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/wia_wheelchair/joint_states',
            self.joint_callback,
            sensor_qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/wia_wheelchair/odom',
            10
        )
        self.state_pub = self.create_publisher(
            WheelchairState,
            '/wia_wheelchair/state',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for prediction step (100 Hz)
        self.prediction_timer = self.create_timer(0.01, self.prediction_callback)

        self.get_logger().info('Sensor fusion node started')

    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        # Extract angular velocity (z-axis for 2D)
        omega = msg.angular_velocity.z

        # Extract linear acceleration (x-axis for forward motion)
        ax = msg.linear_acceleration.x

        # Update EKF with IMU
        self.ekf.update_imu(omega, ax)

    def joint_callback(self, msg: JointState):
        """Process wheel encoder data"""
        # Find wheel joint indices
        left_idx = None
        right_idx = None

        for i, name in enumerate(msg.name):
            if 'left' in name.lower() and 'wheel' in name.lower():
                left_idx = i
            elif 'right' in name.lower() and 'wheel' in name.lower():
                right_idx = i

        if left_idx is None or right_idx is None:
            return

        # Get velocities if available
        if len(msg.velocity) > max(left_idx, right_idx):
            self.left_velocity = msg.velocity[left_idx]
            self.right_velocity = msg.velocity[right_idx]

            # Update EKF with encoder velocities
            self.ekf.update_encoders(
                self.left_velocity,
                self.right_velocity,
                self.drive_params
            )

    def prediction_callback(self):
        """Timer callback for EKF prediction and publishing"""
        current_time = self.get_clock().now()

        # Calculate dt
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                self.ekf.predict(dt)
        self.last_time = current_time

        # Get state
        state = self.ekf.get_state()

        # Publish odometry
        self._publish_odometry(state, current_time)

        # Publish wheelchair state
        self._publish_wheelchair_state(state, current_time)

        # Publish TF
        if self.publish_tf:
            self._publish_tf(state, current_time)

    def _publish_odometry(self, state: EKFState, stamp: Time):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        # Pose
        msg.pose.pose.position = Point(x=state.x, y=state.y, z=0.0)
        msg.pose.pose.orientation = self._yaw_to_quaternion(state.theta)

        # Pose covariance
        cov = self.ekf.get_covariance_6x6()
        msg.pose.covariance = cov.flatten().tolist()

        # Twist
        msg.twist.twist.linear = Vector3(x=state.vx, y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=state.omega)

        # Twist covariance (simplified)
        twist_cov = np.zeros((6, 6))
        twist_cov[0, 0] = 0.01  # vx variance
        twist_cov[5, 5] = 0.01  # omega variance
        msg.twist.covariance = twist_cov.flatten().tolist()

        self.odom_pub.publish(msg)

    def _publish_wheelchair_state(self, state: EKFState, stamp: Time):
        """Publish wheelchair state message"""
        msg = WheelchairState()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.base_frame

        # Velocity
        msg.velocity.linear.x = state.vx
        msg.velocity.angular.z = state.omega

        # Pose
        msg.pose.position.x = state.x
        msg.pose.position.y = state.y
        msg.pose.orientation = self._yaw_to_quaternion(state.theta)

        # Motor speeds (rad/s)
        if self.drive_params.wheel_radius > 0:
            msg.left_motor_speed = self.left_velocity
            msg.right_motor_speed = self.right_velocity

        # Mode (default to manual)
        msg.mode = WheelchairState.MODE_MANUAL

        # Status
        msg.status = WheelchairState.STATUS_MOTORS_ENABLED

        self.state_pub.publish(msg)

    def _publish_tf(self, state: EKFState, stamp: Time):
        """Publish TF transform"""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = 0.0

        q = self._yaw_to_quaternion(state.theta)
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)

    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
