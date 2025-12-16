#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Interface Manager Node

Central coordinator for all assistive device interfaces.
Manages input priority, mode switching, and unified control.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import SetBool, Trigger


class InputMode(IntEnum):
    """Active input mode"""
    MANUAL = 0          # Joystick control
    GAZE = 1            # Eye gaze control
    BCI = 2             # Brain-computer interface
    VOICE = 3           # Voice commands
    AUTONOMOUS = 4      # Autonomous navigation


class InputPriority(IntEnum):
    """Input priority levels"""
    EMERGENCY = 0       # Always override (e.g., emergency stop)
    SAFETY = 1          # Safety systems
    USER_DIRECT = 2     # Direct user input (joystick, gaze, BCI)
    VOICE = 3           # Voice commands
    AUTONOMOUS = 4      # Autonomous navigation
    SMARTHOME = 5       # Smart home triggered


@dataclass
class InputState:
    """State of an input source"""
    enabled: bool = False
    active: bool = False
    last_input_time: float = 0.0
    priority: InputPriority = InputPriority.USER_DIRECT


@dataclass
class InterfaceConfig:
    """Interface manager configuration"""
    # Input timeout (switch to lower priority after no input)
    input_timeout: float = 2.0

    # Default mode
    default_mode: InputMode = InputMode.MANUAL

    # Safety
    emergency_stop_all: bool = True

    # Mode switching
    auto_switch_on_input: bool = True


class InterfaceManagerNode(Node):
    """
    Central interface manager for WIA Smart Wheelchair.

    Coordinates all input interfaces and ensures safe, prioritized control.

    Subscribes:
        /wia_wheelchair/joystick_cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/gaze_cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/bci_cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/voice_cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/nav_cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/emergency_stop (std_msgs/Bool)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/active_mode (std_msgs/Int32)
        /wia_wheelchair/interface_status (std_msgs/String)

    Services:
        /wia_wheelchair/interface/set_mode
        /wia_wheelchair/interface/enable_gaze
        /wia_wheelchair/interface/enable_bci
        /wia_wheelchair/interface/enable_voice
    """

    def __init__(self):
        super().__init__('interface_manager')

        # Declare parameters
        self.declare_parameter('input_timeout', 2.0)
        self.declare_parameter('default_mode', 0)
        self.declare_parameter('auto_switch', True)
        self.declare_parameter('control_rate', 20.0)

        # Load config
        self.config = InterfaceConfig(
            input_timeout=self.get_parameter('input_timeout').value,
            default_mode=InputMode(self.get_parameter('default_mode').value),
            auto_switch_on_input=self.get_parameter('auto_switch').value,
        )

        # State
        self.current_mode = self.config.default_mode
        self.emergency_stop = False
        self.inputs: Dict[InputMode, InputState] = {
            InputMode.MANUAL: InputState(enabled=True, priority=InputPriority.USER_DIRECT),
            InputMode.GAZE: InputState(enabled=False, priority=InputPriority.USER_DIRECT),
            InputMode.BCI: InputState(enabled=False, priority=InputPriority.USER_DIRECT),
            InputMode.VOICE: InputState(enabled=False, priority=InputPriority.VOICE),
            InputMode.AUTONOMOUS: InputState(enabled=False, priority=InputPriority.AUTONOMOUS),
        }

        # Velocity commands from each source
        self.velocities: Dict[InputMode, Optional[Twist]] = {
            mode: None for mode in InputMode
        }

        # Subscribers for each input source
        self.joystick_sub = self.create_subscription(
            Twist,
            '/wia_wheelchair/joystick_cmd_vel',
            lambda msg: self._velocity_callback(InputMode.MANUAL, msg),
            10
        )
        self.gaze_sub = self.create_subscription(
            Twist,
            '/wia_wheelchair/gaze_cmd_vel',
            lambda msg: self._velocity_callback(InputMode.GAZE, msg),
            10
        )
        self.bci_sub = self.create_subscription(
            Twist,
            '/wia_wheelchair/bci_cmd_vel',
            lambda msg: self._velocity_callback(InputMode.BCI, msg),
            10
        )
        self.voice_sub = self.create_subscription(
            Twist,
            '/wia_wheelchair/voice_cmd_vel',
            lambda msg: self._velocity_callback(InputMode.VOICE, msg),
            10
        )
        self.nav_sub = self.create_subscription(
            Twist,
            '/wia_wheelchair/nav_cmd_vel',
            lambda msg: self._velocity_callback(InputMode.AUTONOMOUS, msg),
            10
        )
        self.estop_sub = self.create_subscription(
            Bool,
            '/wia_wheelchair/emergency_stop',
            self.estop_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/wia_wheelchair/cmd_vel',
            10
        )
        self.mode_pub = self.create_publisher(
            Int32,
            '/wia_wheelchair/active_mode',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/interface_status',
            10
        )

        # Services
        self.set_mode_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/interface/set_mode',
            self.set_mode_callback
        )
        self.enable_gaze_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/interface/enable_gaze',
            lambda req, resp: self._enable_mode_callback(InputMode.GAZE, req, resp)
        )
        self.enable_bci_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/interface/enable_bci',
            lambda req, resp: self._enable_mode_callback(InputMode.BCI, req, resp)
        )
        self.enable_voice_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/interface/enable_voice',
            lambda req, resp: self._enable_mode_callback(InputMode.VOICE, req, resp)
        )

        # Control timer
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info('Interface manager started')

    def _velocity_callback(self, mode: InputMode, msg: Twist):
        """Receive velocity command from input source"""
        if not self.inputs[mode].enabled:
            return

        current_time = time.time()
        self.inputs[mode].last_input_time = current_time
        self.inputs[mode].active = True
        self.velocities[mode] = msg

        # Auto-switch mode if enabled
        if self.config.auto_switch_on_input:
            if self._has_significant_velocity(msg):
                self._switch_to_mode(mode)

    def _has_significant_velocity(self, twist: Twist) -> bool:
        """Check if velocity command is significant"""
        return (abs(twist.linear.x) > 0.05 or
                abs(twist.angular.z) > 0.05)

    def estop_callback(self, msg: Bool):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self._send_stop_command()
            self._publish_status('emergency_stop')

    def set_mode_callback(self, request, response):
        """Set active input mode"""
        # Would parse mode from request
        response.success = True
        response.message = f'Mode set to {self.current_mode.name}'
        return response

    def _enable_mode_callback(self, mode: InputMode, request, response):
        """Enable/disable input mode"""
        self.inputs[mode].enabled = request.data
        response.success = True
        response.message = f'{mode.name} {"enabled" if request.data else "disabled"}'
        return response

    def control_loop(self):
        """Main control loop - select and publish appropriate velocity"""
        current_time = time.time()

        # Emergency stop overrides everything
        if self.emergency_stop:
            self._send_stop_command()
            return

        # Check for input timeouts
        for mode, state in self.inputs.items():
            if state.active:
                if current_time - state.last_input_time > self.config.input_timeout:
                    state.active = False
                    self.velocities[mode] = None

        # Select best velocity command based on priority
        selected_velocity = None
        selected_mode = None

        # Sort modes by priority (lower is higher priority)
        active_modes = [
            (mode, state)
            for mode, state in self.inputs.items()
            if state.enabled and state.active and self.velocities[mode] is not None
        ]

        if active_modes:
            # Get highest priority active input
            best = min(active_modes, key=lambda x: x[1].priority)
            selected_mode = best[0]
            selected_velocity = self.velocities[selected_mode]

        # Publish velocity
        if selected_velocity is not None:
            self.cmd_vel_pub.publish(selected_velocity)
            if selected_mode != self.current_mode:
                self.current_mode = selected_mode
                self._publish_mode()
        else:
            # No active input - send zero velocity
            self._send_stop_command()

        # Publish status
        self._publish_status(f'mode:{self.current_mode.name}')

    def _switch_to_mode(self, mode: InputMode):
        """Switch to a specific input mode"""
        if mode != self.current_mode:
            self.get_logger().info(f'Switching mode: {self.current_mode.name} -> {mode.name}')
            self.current_mode = mode
            self._publish_mode()

    def _send_stop_command(self):
        """Send zero velocity"""
        self.cmd_vel_pub.publish(Twist())

    def _publish_mode(self):
        """Publish current mode"""
        msg = Int32()
        msg.data = int(self.current_mode)
        self.mode_pub.publish(msg)

    def _publish_status(self, status: str):
        """Publish interface status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = InterfaceManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
