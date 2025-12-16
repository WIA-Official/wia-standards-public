#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Exoskeleton Transition Manager

Manages safe transitions between wheelchair and exoskeleton systems.
Coordinates positioning, safety checks, and seamless mobility handoff.
"""

import asyncio
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import Trigger, SetBool


class TransitionState(IntEnum):
    """Transition state machine states"""
    IDLE = 0
    ALIGNING = 1
    PREPARING = 2
    SAFETY_CHECK = 3
    TRANSFERRING = 4
    COMPLETING = 5
    COORDINATED_MOVE = 6
    ERROR = 7


class TransitionDirection(IntEnum):
    """Direction of transition"""
    NONE = 0
    TO_EXOSKELETON = 1    # From wheelchair to exoskeleton
    FROM_EXOSKELETON = 2  # From exoskeleton to wheelchair


class DeviceType(IntEnum):
    """Device types"""
    WHEELCHAIR = 1
    EXOSKELETON = 2


@dataclass
class SafetyStatus:
    """Safety check status"""
    wheelchair_stable: bool = False
    exo_ready: bool = False
    path_clear: bool = False
    user_confirmed: bool = False
    armrests_open: bool = False
    position_aligned: bool = False

    def all_clear(self) -> bool:
        """Check if all safety requirements met"""
        return all([
            self.wheelchair_stable,
            self.exo_ready,
            self.path_clear,
            self.position_aligned
        ])


@dataclass
class TransitionConfig:
    """Transition manager configuration"""
    # Positioning
    alignment_tolerance: float = 0.05      # meters
    angle_tolerance: float = 0.1           # radians

    # Timeouts
    alignment_timeout: float = 30.0        # seconds
    safety_check_timeout: float = 10.0
    transfer_timeout: float = 60.0

    # Safety
    require_user_confirmation: bool = True
    auto_brake_on_transfer: bool = True
    monitor_during_transfer: bool = True

    # Coordinated mobility
    follow_distance: float = 1.5           # meters
    follow_timeout: float = 5.0            # seconds to wait if lost


@dataclass
class ExoskeletonState:
    """Exoskeleton device state"""
    connected: bool = False
    ready: bool = False
    walking: bool = False
    battery: float = 0.0
    pose: Optional[Pose] = None
    mode: str = 'idle'


@dataclass
class TransitionStep:
    """A step in transition sequence"""
    name: str
    action: Callable
    timeout: float
    required: bool = True
    description: str = ''


class TransitionSequence:
    """Sequence of transition steps"""

    def __init__(self, name: str, steps: List[TransitionStep]):
        self.name = name
        self.steps = steps
        self.current_index = 0
        self.completed = False
        self.error: Optional[str] = None

    def current_step(self) -> Optional[TransitionStep]:
        """Get current step"""
        if self.current_index < len(self.steps):
            return self.steps[self.current_index]
        return None

    def advance(self):
        """Move to next step"""
        self.current_index += 1
        if self.current_index >= len(self.steps):
            self.completed = True

    def reset(self):
        """Reset sequence"""
        self.current_index = 0
        self.completed = False
        self.error = None

    def progress(self) -> float:
        """Get progress (0-1)"""
        return self.current_index / len(self.steps) if self.steps else 0.0


class TransitionManagerNode(Node):
    """
    ROS2 node for wheelchair-exoskeleton transitions.

    Subscribes:
        /wia_wheelchair/pose (geometry_msgs/PoseStamped)
        /wia_exoskeleton/state (std_msgs/String)
        /wia_exoskeleton/pose (geometry_msgs/PoseStamped)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/transition_status (std_msgs/String)
        /wia_exoskeleton/command (std_msgs/String)

    Services:
        /wia_wheelchair/transition/to_exo
        /wia_wheelchair/transition/from_exo
        /wia_wheelchair/transition/cancel
        /wia_wheelchair/transition/follow_mode
    """

    def __init__(self):
        super().__init__('transition_manager')

        # Declare parameters
        self.declare_parameter('alignment_tolerance', 0.05)
        self.declare_parameter('require_confirmation', True)
        self.declare_parameter('follow_distance', 1.5)

        # Load config
        self.config = TransitionConfig(
            alignment_tolerance=self.get_parameter('alignment_tolerance').value,
            require_user_confirmation=self.get_parameter('require_confirmation').value,
            follow_distance=self.get_parameter('follow_distance').value,
        )

        # State
        self.state = TransitionState.IDLE
        self.direction = TransitionDirection.NONE
        self.safety = SafetyStatus()
        self.exo_state = ExoskeletonState()
        self.wheelchair_pose: Optional[PoseStamped] = None
        self.exo_pose: Optional[PoseStamped] = None
        self.follow_mode = False

        # Transition sequences
        self.current_sequence: Optional[TransitionSequence] = None
        self._setup_sequences()

        # Callback group for async operations
        self.cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.wheelchair_pose_sub = self.create_subscription(
            PoseStamped,
            '/wia_wheelchair/pose',
            self.wheelchair_pose_callback,
            10
        )
        self.exo_state_sub = self.create_subscription(
            String,
            '/wia_exoskeleton/state',
            self.exo_state_callback,
            10
        )
        self.exo_pose_sub = self.create_subscription(
            PoseStamped,
            '/wia_exoskeleton/pose',
            self.exo_pose_callback,
            10
        )
        self.user_confirm_sub = self.create_subscription(
            Bool,
            '/wia_wheelchair/user_confirmation',
            self.user_confirm_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/wia_wheelchair/cmd_vel',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/transition_status',
            10
        )
        self.exo_command_pub = self.create_publisher(
            String,
            '/wia_exoskeleton/command',
            10
        )
        self.armrest_pub = self.create_publisher(
            Bool,
            '/wia_wheelchair/armrest_command',
            10
        )

        # Services
        self.to_exo_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/transition/to_exo',
            self.to_exo_callback,
            callback_group=self.cb_group
        )
        self.from_exo_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/transition/from_exo',
            self.from_exo_callback,
            callback_group=self.cb_group
        )
        self.cancel_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/transition/cancel',
            self.cancel_callback
        )
        self.follow_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/transition/follow_mode',
            self.follow_mode_callback
        )

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Transition manager initialized')

    def _setup_sequences(self):
        """Setup transition step sequences"""
        # Transition to exoskeleton
        self.to_exo_sequence = TransitionSequence(
            'to_exoskeleton',
            [
                TransitionStep(
                    'align', self._align_with_exo, 30.0,
                    description='Aligning wheelchair with exoskeleton'
                ),
                TransitionStep(
                    'open_armrests', self._open_armrests, 5.0,
                    description='Opening armrests'
                ),
                TransitionStep(
                    'safety_check', self._safety_check, 10.0,
                    description='Performing safety check'
                ),
                TransitionStep(
                    'wait_confirmation', self._wait_user_confirmation, 60.0,
                    description='Waiting for user confirmation'
                ),
                TransitionStep(
                    'signal_exo', self._signal_exo_ready, 5.0,
                    description='Signaling exoskeleton'
                ),
                TransitionStep(
                    'monitor_transfer', self._monitor_transfer, 120.0,
                    description='Monitoring transfer'
                ),
            ]
        )

        # Transition from exoskeleton
        self.from_exo_sequence = TransitionSequence(
            'from_exoskeleton',
            [
                TransitionStep(
                    'position_check', self._confirm_wheelchair_position, 10.0,
                    description='Confirming wheelchair position'
                ),
                TransitionStep(
                    'prepare_seat', self._prepare_for_seating, 5.0,
                    description='Preparing seat'
                ),
                TransitionStep(
                    'safety_check', self._safety_check, 10.0,
                    description='Performing safety check'
                ),
                TransitionStep(
                    'wait_confirmation', self._wait_user_confirmation, 60.0,
                    description='Waiting for user confirmation'
                ),
                TransitionStep(
                    'assist_seating', self._assist_seating, 60.0,
                    description='Assisting seating'
                ),
                TransitionStep(
                    'close_armrests', self._close_armrests, 5.0,
                    description='Closing armrests'
                ),
                TransitionStep(
                    'activate', self._activate_wheelchair, 5.0,
                    description='Activating wheelchair'
                ),
            ]
        )

    # Callbacks
    def wheelchair_pose_callback(self, msg: PoseStamped):
        """Update wheelchair pose"""
        self.wheelchair_pose = msg

    def exo_state_callback(self, msg: String):
        """Update exoskeleton state"""
        # Parse state string (simplified)
        state_data = msg.data
        if 'connected' in state_data:
            self.exo_state.connected = True
        if 'ready' in state_data:
            self.exo_state.ready = True
        if 'walking' in state_data:
            self.exo_state.walking = True
            self.exo_state.mode = 'walking'

    def exo_pose_callback(self, msg: PoseStamped):
        """Update exoskeleton pose"""
        self.exo_pose = msg
        self.exo_state.pose = msg.pose

    def user_confirm_callback(self, msg: Bool):
        """Receive user confirmation"""
        if msg.data:
            self.safety.user_confirmed = True

    def to_exo_callback(self, request, response):
        """Start transition to exoskeleton"""
        if self.state != TransitionState.IDLE:
            response.success = False
            response.message = 'Transition already in progress'
            return response

        if not self.exo_state.connected:
            response.success = False
            response.message = 'Exoskeleton not connected'
            return response

        self.direction = TransitionDirection.TO_EXOSKELETON
        self.current_sequence = self.to_exo_sequence
        self.current_sequence.reset()
        self.state = TransitionState.ALIGNING

        response.success = True
        response.message = 'Starting transition to exoskeleton'
        self._publish_status('starting:to_exo')

        return response

    def from_exo_callback(self, request, response):
        """Start transition from exoskeleton"""
        if self.state != TransitionState.IDLE:
            response.success = False
            response.message = 'Transition already in progress'
            return response

        self.direction = TransitionDirection.FROM_EXOSKELETON
        self.current_sequence = self.from_exo_sequence
        self.current_sequence.reset()
        self.state = TransitionState.PREPARING

        response.success = True
        response.message = 'Starting transition from exoskeleton'
        self._publish_status('starting:from_exo')

        return response

    def cancel_callback(self, request, response):
        """Cancel transition"""
        self._cancel_transition()
        response.success = True
        response.message = 'Transition cancelled'
        return response

    def follow_mode_callback(self, request, response):
        """Enable/disable follow mode"""
        self.follow_mode = request.data

        if self.follow_mode:
            self.state = TransitionState.COORDINATED_MOVE
            response.message = 'Follow mode enabled'
        else:
            self.state = TransitionState.IDLE
            self._stop_wheelchair()
            response.message = 'Follow mode disabled'

        response.success = True
        return response

    def control_loop(self):
        """Main control loop"""
        if self.state == TransitionState.IDLE:
            return

        if self.state == TransitionState.COORDINATED_MOVE:
            self._follow_exoskeleton()
            return

        if self.current_sequence is None:
            return

        # Execute current step
        step = self.current_sequence.current_step()
        if step is None:
            # Sequence complete
            self._complete_transition()
            return

        self._publish_status(f'step:{step.name}:{self.current_sequence.progress():.0%}')

        # Step execution happens in service callbacks
        # This loop monitors progress

    # Transition steps
    async def _align_with_exo(self) -> bool:
        """Align wheelchair with exoskeleton position"""
        if self.exo_pose is None:
            return False

        target = self.exo_pose.pose.position
        if self.wheelchair_pose is None:
            return False

        current = self.wheelchair_pose.pose.position

        # Calculate distance
        dx = target.x - current.x - 0.5  # Offset for transfer position
        dy = target.y - current.y

        distance = (dx**2 + dy**2)**0.5

        if distance < self.config.alignment_tolerance:
            self.safety.position_aligned = True
            return True

        # Move towards target
        twist = Twist()
        twist.linear.x = min(0.2, distance) * (dx / distance if distance > 0 else 0)
        twist.angular.z = 0.0  # Simplified, would calculate proper angle

        self.cmd_vel_pub.publish(twist)
        return False

    async def _open_armrests(self) -> bool:
        """Open wheelchair armrests"""
        msg = Bool()
        msg.data = True  # Open command
        self.armrest_pub.publish(msg)
        self.safety.armrests_open = True
        return True

    async def _close_armrests(self) -> bool:
        """Close wheelchair armrests"""
        msg = Bool()
        msg.data = False  # Close command
        self.armrest_pub.publish(msg)
        self.safety.armrests_open = False
        return True

    async def _safety_check(self) -> bool:
        """Perform safety check"""
        # Check wheelchair stability
        self._stop_wheelchair()
        self.safety.wheelchair_stable = True

        # Check exoskeleton ready
        self.safety.exo_ready = self.exo_state.ready

        # Check path clear (would use obstacle detection)
        self.safety.path_clear = True

        return self.safety.all_clear()

    async def _wait_user_confirmation(self) -> bool:
        """Wait for user to confirm transfer"""
        if not self.config.require_user_confirmation:
            return True

        self._publish_status('waiting:user_confirmation')
        return self.safety.user_confirmed

    async def _signal_exo_ready(self) -> bool:
        """Signal exoskeleton that wheelchair is ready"""
        msg = String()
        msg.data = 'wheelchair_ready'
        self.exo_command_pub.publish(msg)
        return True

    async def _monitor_transfer(self) -> bool:
        """Monitor during transfer (user moving to exo)"""
        # Would monitor sensors during transfer
        # Simplified - just wait
        return True

    async def _confirm_wheelchair_position(self) -> bool:
        """Confirm wheelchair is in correct position"""
        self.safety.position_aligned = True  # Simplified
        return True

    async def _prepare_for_seating(self) -> bool:
        """Prepare wheelchair for user to sit"""
        await self._open_armrests()
        return True

    async def _assist_seating(self) -> bool:
        """Assist user seating (monitoring phase)"""
        # Would monitor sensors during seating
        return True

    async def _activate_wheelchair(self) -> bool:
        """Activate wheelchair systems after transfer"""
        self._publish_status('activated')
        return True

    def _follow_exoskeleton(self):
        """Follow exoskeleton in coordinated mobility mode"""
        if self.exo_pose is None or self.wheelchair_pose is None:
            return

        exo_pos = self.exo_pose.pose.position
        wc_pos = self.wheelchair_pose.pose.position

        # Calculate distance to exo
        dx = exo_pos.x - wc_pos.x
        dy = exo_pos.y - wc_pos.y
        distance = (dx**2 + dy**2)**0.5

        # Maintain follow distance
        if distance > self.config.follow_distance + 0.5:
            # Too far, move closer
            speed = min(0.3, (distance - self.config.follow_distance) * 0.5)
            twist = Twist()
            twist.linear.x = speed * (dx / distance) if distance > 0 else 0
            self.cmd_vel_pub.publish(twist)

        elif distance < self.config.follow_distance - 0.5:
            # Too close, slow down or stop
            self._stop_wheelchair()

        self._publish_status(f'following:distance={distance:.2f}')

    def _complete_transition(self):
        """Complete transition sequence"""
        self.state = TransitionState.IDLE
        self.direction = TransitionDirection.NONE
        self.current_sequence = None
        self._reset_safety()

        self._publish_status('completed')
        self.get_logger().info('Transition completed successfully')

    def _cancel_transition(self):
        """Cancel current transition"""
        self._stop_wheelchair()
        self.state = TransitionState.IDLE
        self.direction = TransitionDirection.NONE
        self.current_sequence = None
        self._reset_safety()

        self._publish_status('cancelled')
        self.get_logger().info('Transition cancelled')

    def _stop_wheelchair(self):
        """Stop wheelchair movement"""
        self.cmd_vel_pub.publish(Twist())

    def _reset_safety(self):
        """Reset safety status"""
        self.safety = SafetyStatus()

    def _publish_status(self, status: str):
        """Publish transition status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TransitionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
