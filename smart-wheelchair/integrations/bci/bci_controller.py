#!/usr/bin/env python3
"""
WIA Smart Wheelchair - BCI (Brain-Computer Interface) Controller

Enables brain signal-based wheelchair control.
Supports motor imagery classification, intent detection, and hybrid control modes.
"""

import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String, Float32MultiArray
from std_srvs.srv import SetBool, Trigger


class BCIIntent(IntEnum):
    """BCI detected intents"""
    NONE = 0
    FORWARD = 1
    STOP = 2
    LEFT = 3
    RIGHT = 4
    SELECT = 5
    BACK = 6


class BCIControlMode(IntEnum):
    """BCI control modes"""
    DISABLED = 0
    DIRECT = 1          # Direct BCI to velocity
    DISCRETE = 2        # Discrete commands (forward/stop/turn)
    HYBRID = 3          # BCI intent + autonomous execution
    SELECTION = 4       # BCI for menu selection


@dataclass
class BCIPattern:
    """Brain signal pattern for intent detection"""
    name: str
    channel_weights: List[float]       # EEG channel weights
    frequency_bands: List[Tuple[float, float]]  # (low, high) Hz
    threshold: float = 0.7             # Detection threshold
    min_duration: float = 0.5          # Minimum signal duration


@dataclass
class BCIConfig:
    """BCI controller configuration"""
    # Safety settings
    max_speed: float = 0.3             # m/s (reduced for safety)
    require_confirmation: bool = True
    confirmation_duration: float = 1.0  # seconds
    auto_stop_timeout: float = 2.0     # seconds without signal

    # Signal processing
    sample_rate: float = 256.0         # Hz
    window_size: float = 1.0           # seconds
    confidence_threshold: float = 0.7

    # Hybrid mode
    autonomous_assist: bool = True
    obstacle_override: bool = True     # Safety override

    # Intent mapping velocities
    forward_speed: float = 0.3
    turn_speed: float = 0.5


@dataclass
class BCISignal:
    """Processed BCI signal"""
    intent: BCIIntent
    confidence: float
    timestamp: float
    raw_features: Optional[List[float]] = None


@dataclass
class IntentAccumulator:
    """Accumulates intent detections for confirmation"""
    current_intent: BCIIntent = BCIIntent.NONE
    start_time: float = 0.0
    accumulated_confidence: float = 0.0
    detection_count: int = 0


class BCIIntentClassifier:
    """
    Classifies BCI signals into wheelchair control intents.

    This is a simplified implementation. In production, this would
    integrate with actual BCI hardware (OpenBCI, Emotiv, etc.) and
    use trained ML models for classification.
    """

    def __init__(self, config: BCIConfig):
        self.config = config
        self._patterns: Dict[BCIIntent, BCIPattern] = {}
        self._setup_default_patterns()

        # Signal buffer
        self._buffer_size = int(config.window_size * config.sample_rate)
        self._signal_buffer: deque = deque(maxlen=self._buffer_size)

    def _setup_default_patterns(self):
        """Setup default motor imagery patterns"""
        # These are simplified patterns for demonstration
        # Real implementation would use trained classifiers

        self._patterns[BCIIntent.FORWARD] = BCIPattern(
            name='forward',
            channel_weights=[0.5, 0.5, 0.0, 0.0],  # Frontal channels
            frequency_bands=[(8, 12), (13, 30)],   # Alpha + Beta
            threshold=0.7
        )

        self._patterns[BCIIntent.LEFT] = BCIPattern(
            name='left',
            channel_weights=[0.0, 0.8, 0.2, 0.0],  # Right motor cortex
            frequency_bands=[(8, 12)],              # Mu rhythm
            threshold=0.7
        )

        self._patterns[BCIIntent.RIGHT] = BCIPattern(
            name='right',
            channel_weights=[0.8, 0.0, 0.0, 0.2],  # Left motor cortex
            frequency_bands=[(8, 12)],
            threshold=0.7
        )

        self._patterns[BCIIntent.STOP] = BCIPattern(
            name='stop',
            channel_weights=[0.3, 0.3, 0.2, 0.2],  # Bilateral
            frequency_bands=[(4, 8)],               # Theta
            threshold=0.6
        )

        self._patterns[BCIIntent.SELECT] = BCIPattern(
            name='select',
            channel_weights=[0.0, 0.0, 0.5, 0.5],  # Parietal
            frequency_bands=[(0.1, 1)],             # P300-like
            threshold=0.8
        )

    def add_sample(self, channels: List[float]):
        """Add EEG sample to buffer"""
        self._signal_buffer.append(channels)

    def classify(self) -> BCISignal:
        """Classify current buffer into intent"""
        if len(self._signal_buffer) < self._buffer_size // 2:
            return BCISignal(
                intent=BCIIntent.NONE,
                confidence=0.0,
                timestamp=time.time()
            )

        # Compute features (simplified)
        features = self._extract_features()

        # Match against patterns
        best_intent = BCIIntent.NONE
        best_confidence = 0.0

        for intent, pattern in self._patterns.items():
            confidence = self._match_pattern(features, pattern)
            if confidence > pattern.threshold and confidence > best_confidence:
                best_intent = intent
                best_confidence = confidence

        return BCISignal(
            intent=best_intent,
            confidence=best_confidence,
            timestamp=time.time(),
            raw_features=features
        )

    def _extract_features(self) -> List[float]:
        """Extract features from signal buffer"""
        # Simplified feature extraction
        # Real implementation would use FFT, wavelet, CSP, etc.
        buffer_list = list(self._signal_buffer)
        if not buffer_list:
            return [0.0] * 8

        # Calculate simple statistics per channel
        features = []
        num_channels = len(buffer_list[0]) if buffer_list else 4

        for ch in range(num_channels):
            channel_data = [sample[ch] for sample in buffer_list if len(sample) > ch]
            if channel_data:
                mean = sum(channel_data) / len(channel_data)
                variance = sum((x - mean) ** 2 for x in channel_data) / len(channel_data)
                features.extend([mean, math.sqrt(variance)])
            else:
                features.extend([0.0, 0.0])

        return features

    def _match_pattern(self, features: List[float], pattern: BCIPattern) -> float:
        """Match features against pattern"""
        # Simplified pattern matching
        # Real implementation would use trained classifier

        if len(features) < len(pattern.channel_weights) * 2:
            return 0.0

        score = 0.0
        for i, weight in enumerate(pattern.channel_weights):
            if i * 2 + 1 < len(features):
                # Use variance as activity indicator
                activity = features[i * 2 + 1]
                score += weight * min(1.0, activity / 10.0)

        return min(1.0, score)


class BCIControllerNode(Node):
    """
    ROS2 node for BCI-based wheelchair control.

    Subscribes:
        /wia_wheelchair/bci_signal (std_msgs/Float32MultiArray)
        /wia_wheelchair/bci_intent (std_msgs/String)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/goal_pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/bci_status (std_msgs/String)

    Services:
        /wia_wheelchair/bci/enable
        /wia_wheelchair/bci/set_mode
        /wia_wheelchair/bci/calibrate
    """

    def __init__(self):
        super().__init__('bci_controller')

        # Declare parameters
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('require_confirmation', True)
        self.declare_parameter('confirmation_duration', 1.0)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('autonomous_assist', True)
        self.declare_parameter('control_rate', 10.0)

        # Load config
        self.config = BCIConfig(
            max_speed=self.get_parameter('max_speed').value,
            require_confirmation=self.get_parameter('require_confirmation').value,
            confirmation_duration=self.get_parameter('confirmation_duration').value,
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            autonomous_assist=self.get_parameter('autonomous_assist').value,
        )

        # State
        self.enabled = False
        self.mode = BCIControlMode.DISABLED
        self.last_signal_time = 0.0
        self.accumulator = IntentAccumulator()
        self.is_moving = False

        # Components
        self.classifier = BCIIntentClassifier(self.config)

        # Saved locations for hybrid mode
        self.locations: Dict[str, PoseStamped] = {}

        # Subscribers
        self.signal_sub = self.create_subscription(
            Float32MultiArray,
            '/wia_wheelchair/bci_signal',
            self.signal_callback,
            10
        )
        self.intent_sub = self.create_subscription(
            String,
            '/wia_wheelchair/bci_intent',
            self.intent_callback,
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
            '/wia_wheelchair/bci_status',
            10
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/bci/enable',
            self.enable_callback
        )
        self.calibrate_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/bci/calibrate',
            self.calibrate_callback
        )

        # Control timer
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info('BCI controller initialized')

    def signal_callback(self, msg: Float32MultiArray):
        """Receive raw BCI signal"""
        self.classifier.add_sample(list(msg.data))
        self.last_signal_time = self.get_clock().now().nanoseconds / 1e9

    def intent_callback(self, msg: String):
        """Receive pre-classified intent from external system"""
        intent_map = {
            'forward': BCIIntent.FORWARD,
            'stop': BCIIntent.STOP,
            'left': BCIIntent.LEFT,
            'right': BCIIntent.RIGHT,
            'select': BCIIntent.SELECT,
            'back': BCIIntent.BACK,
        }

        intent = intent_map.get(msg.data.lower(), BCIIntent.NONE)
        if intent != BCIIntent.NONE:
            signal = BCISignal(
                intent=intent,
                confidence=0.9,  # External classification assumed high confidence
                timestamp=self.get_clock().now().nanoseconds / 1e9
            )
            self._process_signal(signal)

    def enable_callback(self, request, response):
        """Enable/disable BCI control"""
        self.enabled = request.data
        if self.enabled:
            self.mode = BCIControlMode.DISCRETE
            response.message = 'BCI control enabled (discrete mode)'
        else:
            self.mode = BCIControlMode.DISABLED
            self._stop_wheelchair()
            response.message = 'BCI control disabled'

        response.success = True
        return response

    def calibrate_callback(self, request, response):
        """Start calibration routine"""
        self.get_logger().info('Starting BCI calibration...')
        # In real implementation, this would guide through calibration
        response.success = True
        response.message = 'Calibration complete'
        return response

    def control_loop(self):
        """Main control loop"""
        if not self.enabled or self.mode == BCIControlMode.DISABLED:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Check for signal timeout
        signal_age = current_time - self.last_signal_time
        if signal_age > self.config.auto_stop_timeout:
            if self.is_moving:
                self._stop_wheelchair()
                self._publish_status('timeout_stop')
            return

        # Classify current signal
        signal = self.classifier.classify()

        if signal.confidence >= self.config.confidence_threshold:
            self._process_signal(signal)

    def _process_signal(self, signal: BCISignal):
        """Process classified BCI signal"""
        current_time = signal.timestamp

        # Accumulate for confirmation
        if self.config.require_confirmation:
            if signal.intent == self.accumulator.current_intent:
                # Same intent - accumulate
                self.accumulator.accumulated_confidence += signal.confidence
                self.accumulator.detection_count += 1

                duration = current_time - self.accumulator.start_time
                if duration >= self.config.confirmation_duration:
                    # Confirmed - execute
                    self._execute_intent(signal.intent)
                    self.accumulator = IntentAccumulator()
                else:
                    progress = duration / self.config.confirmation_duration
                    self._publish_status(f'confirming:{signal.intent.name}:{progress:.0%}')
            else:
                # New intent - reset accumulator
                self.accumulator = IntentAccumulator(
                    current_intent=signal.intent,
                    start_time=current_time,
                    accumulated_confidence=signal.confidence,
                    detection_count=1
                )
        else:
            # Direct execution
            self._execute_intent(signal.intent)

    def _execute_intent(self, intent: BCIIntent):
        """Execute detected intent"""
        self.get_logger().info(f'Executing BCI intent: {intent.name}')

        if self.mode == BCIControlMode.DIRECT or self.mode == BCIControlMode.DISCRETE:
            self._execute_velocity_intent(intent)
        elif self.mode == BCIControlMode.HYBRID:
            self._execute_hybrid_intent(intent)
        elif self.mode == BCIControlMode.SELECTION:
            self._execute_selection_intent(intent)

        self._publish_status(f'executed:{intent.name}')

    def _execute_velocity_intent(self, intent: BCIIntent):
        """Execute intent as velocity command"""
        twist = Twist()

        if intent == BCIIntent.FORWARD:
            twist.linear.x = self.config.forward_speed
            self.is_moving = True
        elif intent == BCIIntent.BACK:
            twist.linear.x = -self.config.forward_speed * 0.5
            self.is_moving = True
        elif intent == BCIIntent.LEFT:
            twist.angular.z = self.config.turn_speed
            self.is_moving = True
        elif intent == BCIIntent.RIGHT:
            twist.angular.z = -self.config.turn_speed
            self.is_moving = True
        elif intent == BCIIntent.STOP:
            self.is_moving = False

        self.cmd_vel_pub.publish(twist)

    def _execute_hybrid_intent(self, intent: BCIIntent):
        """Execute intent with autonomous assistance"""
        if intent == BCIIntent.FORWARD:
            # Request autonomous navigation to continue forward
            self._publish_status('hybrid:forward_assist')
            # In real implementation, would integrate with path planner

        elif intent == BCIIntent.STOP:
            self._stop_wheelchair()

        elif intent in (BCIIntent.LEFT, BCIIntent.RIGHT):
            # Request turn assistance
            self._execute_velocity_intent(intent)

        elif intent == BCIIntent.SELECT:
            # Select highlighted destination
            pass

    def _execute_selection_intent(self, intent: BCIIntent):
        """Execute intent for menu selection"""
        # Used for selecting from options
        if intent == BCIIntent.SELECT:
            self._publish_status('selection:confirmed')
        elif intent == BCIIntent.LEFT:
            self._publish_status('selection:previous')
        elif intent == BCIIntent.RIGHT:
            self._publish_status('selection:next')

    def _stop_wheelchair(self):
        """Send stop command"""
        self.cmd_vel_pub.publish(Twist())
        self.is_moving = False

    def _publish_status(self, status: str):
        """Publish BCI control status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def set_mode(self, mode: BCIControlMode):
        """Set control mode"""
        self.mode = mode
        self.get_logger().info(f'BCI mode set to: {mode.name}')


def main(args=None):
    rclpy.init(args=args)

    node = BCIControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
