#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Voice Command Controller

Enables voice-based wheelchair control with AAC (Augmentative and
Alternative Communication) integration and natural language processing.
"""

import re
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class VoiceCommandType(IntEnum):
    """Types of voice commands"""
    MOTION = 1          # Direct motion commands
    NAVIGATION = 2      # Navigate to location
    CONTROL = 3         # System control
    QUERY = 4           # Information request
    CUSTOM = 5          # User-defined


@dataclass
class VoiceCommand:
    """Voice command definition"""
    patterns: List[str]           # Regex patterns to match
    command_type: VoiceCommandType
    action: str                   # Action identifier
    parameters: Dict[str, Any] = field(default_factory=dict)
    confirmation_required: bool = False
    description: str = ''


@dataclass
class VoiceConfig:
    """Voice controller configuration"""
    # Language settings
    language: str = 'ko-KR'       # Korean
    fallback_language: str = 'en-US'

    # Recognition settings
    confidence_threshold: float = 0.6
    timeout: float = 5.0           # seconds

    # Feedback settings
    voice_feedback: bool = True
    beep_on_recognition: bool = True

    # Safety
    require_wake_word: bool = False
    wake_words: List[str] = field(default_factory=lambda: ['휠체어', 'wheelchair'])

    # Speed settings
    default_linear_speed: float = 0.5
    default_angular_speed: float = 0.8


@dataclass
class RecognitionResult:
    """Speech recognition result"""
    text: str
    confidence: float
    language: str
    timestamp: float
    alternatives: List[str] = field(default_factory=list)


class VoiceCommandMatcher:
    """Matches spoken text to commands"""

    def __init__(self, config: VoiceConfig):
        self.config = config
        self._commands: List[VoiceCommand] = []
        self._aliases: Dict[str, str] = {}
        self._setup_default_commands()

    def _setup_default_commands(self):
        """Setup default voice commands"""
        # Motion commands (Korean)
        motion_commands = [
            VoiceCommand(
                patterns=[r'앞으로', r'전진', r'가자', r'go\s*forward', r'forward'],
                command_type=VoiceCommandType.MOTION,
                action='forward',
                parameters={'linear': 0.5, 'angular': 0.0},
                description='Move forward'
            ),
            VoiceCommand(
                patterns=[r'뒤로', r'후진', r'go\s*back', r'backward'],
                command_type=VoiceCommandType.MOTION,
                action='backward',
                parameters={'linear': -0.3, 'angular': 0.0},
                description='Move backward'
            ),
            VoiceCommand(
                patterns=[r'왼쪽', r'좌회전', r'turn\s*left', r'left'],
                command_type=VoiceCommandType.MOTION,
                action='left',
                parameters={'linear': 0.0, 'angular': 0.5},
                description='Turn left'
            ),
            VoiceCommand(
                patterns=[r'오른쪽', r'우회전', r'turn\s*right', r'right'],
                command_type=VoiceCommandType.MOTION,
                action='right',
                parameters={'linear': 0.0, 'angular': -0.5},
                description='Turn right'
            ),
            VoiceCommand(
                patterns=[r'멈춰', r'정지', r'스탑', r'stop', r'halt'],
                command_type=VoiceCommandType.MOTION,
                action='stop',
                parameters={'linear': 0.0, 'angular': 0.0},
                description='Stop moving'
            ),
            VoiceCommand(
                patterns=[r'천천히', r'느리게', r'slow\s*down', r'slower'],
                command_type=VoiceCommandType.CONTROL,
                action='slow',
                parameters={'speed_factor': 0.5},
                description='Reduce speed'
            ),
            VoiceCommand(
                patterns=[r'빨리', r'빠르게', r'speed\s*up', r'faster'],
                command_type=VoiceCommandType.CONTROL,
                action='fast',
                parameters={'speed_factor': 1.5},
                description='Increase speed'
            ),
        ]

        # Navigation commands
        navigation_commands = [
            VoiceCommand(
                patterns=[r'(.+)(으로|로)\s*가', r'(.+)\s*가자', r'go\s*to\s*(.+)'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                description='Navigate to location'
            ),
            VoiceCommand(
                patterns=[r'거실', r'리빙룸', r'living\s*room'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '거실'},
                description='Go to living room'
            ),
            VoiceCommand(
                patterns=[r'주방', r'부엌', r'kitchen'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '주방'},
                description='Go to kitchen'
            ),
            VoiceCommand(
                patterns=[r'침실', r'방', r'bedroom'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '침실'},
                description='Go to bedroom'
            ),
            VoiceCommand(
                patterns=[r'화장실', r'욕실', r'bathroom', r'restroom'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '화장실'},
                description='Go to bathroom'
            ),
            VoiceCommand(
                patterns=[r'충전', r'충전소', r'충전\s*하러', r'charge', r'charging'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '충전소'},
                description='Go to charging station'
            ),
            VoiceCommand(
                patterns=[r'집으로', r'홈', r'go\s*home', r'home'],
                command_type=VoiceCommandType.NAVIGATION,
                action='navigate',
                parameters={'location': '거실'},
                description='Go home'
            ),
        ]

        # Control commands
        control_commands = [
            VoiceCommand(
                patterns=[r'비상\s*정지', r'긴급\s*정지', r'emergency', r'e-?stop'],
                command_type=VoiceCommandType.CONTROL,
                action='emergency_stop',
                description='Emergency stop'
            ),
            VoiceCommand(
                patterns=[r'취소', r'cancel'],
                command_type=VoiceCommandType.CONTROL,
                action='cancel',
                description='Cancel current action'
            ),
            VoiceCommand(
                patterns=[r'도움', r'도와줘', r'help'],
                command_type=VoiceCommandType.QUERY,
                action='help',
                description='List available commands'
            ),
            VoiceCommand(
                patterns=[r'배터리', r'battery'],
                command_type=VoiceCommandType.QUERY,
                action='battery_status',
                description='Check battery level'
            ),
            VoiceCommand(
                patterns=[r'어디야', r'위치', r'where\s*am\s*i', r'location'],
                command_type=VoiceCommandType.QUERY,
                action='current_location',
                description='Report current location'
            ),
        ]

        self._commands.extend(motion_commands)
        self._commands.extend(navigation_commands)
        self._commands.extend(control_commands)

        # Setup location aliases
        self._aliases = {
            '리빙룸': '거실',
            '거실 중앙': '거실',
            '소파 앞': '소파',
            '텔레비전': 'TV 앞',
            '부엌': '주방',
            '밥 먹는 곳': '식탁',
            '잠자는 곳': '침실',
            '화장실 가기': '화장실',
            '바깥': '현관',
            '충전': '충전소',
        }

    def match(self, text: str) -> Optional[Tuple[VoiceCommand, Dict[str, Any]]]:
        """Match text to command"""
        text = text.strip().lower()

        for command in self._commands:
            for pattern in command.patterns:
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    # Extract parameters from match groups
                    params = dict(command.parameters)

                    if match.groups():
                        # Dynamic location extraction
                        if command.action == 'navigate' and 'location' not in params:
                            location = match.group(1).strip()
                            # Resolve alias
                            location = self._aliases.get(location, location)
                            params['location'] = location

                    return command, params

        return None

    def add_command(self, command: VoiceCommand):
        """Add custom command"""
        self._commands.append(command)

    def add_alias(self, alias: str, target: str):
        """Add location alias"""
        self._aliases[alias] = target

    def get_help_text(self) -> str:
        """Get help text listing all commands"""
        lines = ['사용 가능한 명령어:']

        for cmd in self._commands:
            example = cmd.patterns[0].replace(r'\s*', ' ').replace(r'(.+)', '...')
            lines.append(f'  - {example}: {cmd.description}')

        return '\n'.join(lines)


class AACIntegration:
    """Integration with AAC (Augmentative and Alternative Communication)"""

    def __init__(self):
        self._symbol_commands: Dict[str, str] = {}
        self._setup_default_symbols()

    def _setup_default_symbols(self):
        """Setup default AAC symbol mappings"""
        # Common AAC symbols to wheelchair commands
        self._symbol_commands = {
            'symbol_forward': '앞으로',
            'symbol_stop': '멈춰',
            'symbol_left': '왼쪽',
            'symbol_right': '오른쪽',
            'symbol_home': '집으로',
            'symbol_bathroom': '화장실',
            'symbol_kitchen': '주방',
            'symbol_help': '도움',
        }

    def symbol_to_text(self, symbol_id: str) -> Optional[str]:
        """Convert AAC symbol to command text"""
        return self._symbol_commands.get(symbol_id)

    def add_symbol_mapping(self, symbol_id: str, command_text: str):
        """Add custom symbol mapping"""
        self._symbol_commands[symbol_id] = command_text


class VoiceControllerNode(Node):
    """
    ROS2 node for voice-based wheelchair control.

    Subscribes:
        /wia_wheelchair/speech_text (std_msgs/String)
        /wia_wheelchair/aac_symbol (std_msgs/String)

    Publishes:
        /wia_wheelchair/cmd_vel (geometry_msgs/Twist)
        /wia_wheelchair/goal_pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/voice_feedback (std_msgs/String)
        /wia_wheelchair/voice_status (std_msgs/String)

    Services:
        /wia_wheelchair/voice/enable
        /wia_wheelchair/voice/add_command
    """

    def __init__(self):
        super().__init__('voice_controller')

        # Declare parameters
        self.declare_parameter('language', 'ko-KR')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('voice_feedback', True)
        self.declare_parameter('require_wake_word', False)
        self.declare_parameter('default_speed', 0.5)

        # Load config
        self.config = VoiceConfig(
            language=self.get_parameter('language').value,
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            voice_feedback=self.get_parameter('voice_feedback').value,
            require_wake_word=self.get_parameter('require_wake_word').value,
            default_linear_speed=self.get_parameter('default_speed').value,
        )

        # State
        self.enabled = False
        self.speed_factor = 1.0
        self.wake_word_detected = not self.config.require_wake_word

        # Components
        self.matcher = VoiceCommandMatcher(self.config)
        self.aac = AACIntegration()

        # Saved locations
        self.locations: Dict[str, PoseStamped] = {}

        # Subscribers
        self.speech_sub = self.create_subscription(
            String,
            '/wia_wheelchair/speech_text',
            self.speech_callback,
            10
        )
        self.aac_sub = self.create_subscription(
            String,
            '/wia_wheelchair/aac_symbol',
            self.aac_callback,
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
        self.feedback_pub = self.create_publisher(
            String,
            '/wia_wheelchair/voice_feedback',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/voice_status',
            10
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/voice/enable',
            self.enable_callback
        )

        self.get_logger().info('Voice controller initialized')

    def speech_callback(self, msg: String):
        """Receive recognized speech text"""
        if not self.enabled:
            return

        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Received speech: {text}')

        # Check wake word if required
        if self.config.require_wake_word and not self.wake_word_detected:
            for wake_word in self.config.wake_words:
                if wake_word in text.lower():
                    self.wake_word_detected = True
                    self._speak_feedback('네, 듣고 있어요.')
                    return
            return

        # Match and execute command
        result = self.matcher.match(text)

        if result:
            command, params = result
            self._execute_command(command, params)
        else:
            self._speak_feedback('명령을 이해하지 못했어요.')
            self._publish_status(f'unrecognized:{text}')

    def aac_callback(self, msg: String):
        """Receive AAC symbol selection"""
        if not self.enabled:
            return

        symbol_id = msg.data

        # Convert symbol to text
        text = self.aac.symbol_to_text(symbol_id)
        if text:
            # Process as voice command
            result = self.matcher.match(text)
            if result:
                command, params = result
                self._execute_command(command, params)

    def enable_callback(self, request, response):
        """Enable/disable voice control"""
        self.enabled = request.data
        if self.enabled:
            response.message = '음성 제어가 활성화되었습니다.'
            self._speak_feedback('음성 제어 시작')
        else:
            response.message = '음성 제어가 비활성화되었습니다.'
            self._stop_wheelchair()

        response.success = True
        return response

    def _execute_command(self, command: VoiceCommand, params: Dict[str, Any]):
        """Execute matched command"""
        self.get_logger().info(f'Executing: {command.action} with {params}')

        if command.command_type == VoiceCommandType.MOTION:
            self._execute_motion(command.action, params)

        elif command.command_type == VoiceCommandType.NAVIGATION:
            self._execute_navigation(params)

        elif command.command_type == VoiceCommandType.CONTROL:
            self._execute_control(command.action, params)

        elif command.command_type == VoiceCommandType.QUERY:
            self._execute_query(command.action)

        self._publish_status(f'executed:{command.action}')

    def _execute_motion(self, action: str, params: Dict[str, Any]):
        """Execute motion command"""
        twist = Twist()

        linear = params.get('linear', 0.0) * self.speed_factor
        angular = params.get('angular', 0.0) * self.speed_factor

        twist.linear.x = linear
        twist.angular.z = angular

        self.cmd_vel_pub.publish(twist)

        # Voice feedback
        feedback_map = {
            'forward': '앞으로 이동합니다.',
            'backward': '뒤로 이동합니다.',
            'left': '왼쪽으로 회전합니다.',
            'right': '오른쪽으로 회전합니다.',
            'stop': '정지합니다.',
        }
        self._speak_feedback(feedback_map.get(action, ''))

    def _execute_navigation(self, params: Dict[str, Any]):
        """Execute navigation command"""
        location = params.get('location')
        if not location:
            self._speak_feedback('목적지를 지정해주세요.')
            return

        if location in self.locations:
            goal = self.locations[location]
            self.goal_pub.publish(goal)
            self._speak_feedback(f'{location}(으)로 이동합니다.')
        else:
            # Publish location name for goal manager to resolve
            status_msg = String()
            status_msg.data = f'navigate_to:{location}'
            self.status_pub.publish(status_msg)
            self._speak_feedback(f'{location}(으)로 이동합니다.')

    def _execute_control(self, action: str, params: Dict[str, Any]):
        """Execute control command"""
        if action == 'emergency_stop':
            self._stop_wheelchair()
            self._speak_feedback('비상 정지!')

        elif action == 'cancel':
            self._stop_wheelchair()
            self._speak_feedback('취소되었습니다.')

        elif action == 'slow':
            self.speed_factor = params.get('speed_factor', 0.5)
            self._speak_feedback('속도를 줄입니다.')

        elif action == 'fast':
            self.speed_factor = min(2.0, params.get('speed_factor', 1.5))
            self._speak_feedback('속도를 높입니다.')

    def _execute_query(self, action: str):
        """Execute query command"""
        if action == 'help':
            help_text = self.matcher.get_help_text()
            self._speak_feedback('사용 가능한 명령어를 표시합니다.')
            self.get_logger().info(help_text)

        elif action == 'battery_status':
            # Would query actual battery status
            self._speak_feedback('배터리가 80퍼센트 남았습니다.')

        elif action == 'current_location':
            # Would query actual location
            self._speak_feedback('현재 거실에 있습니다.')

    def _stop_wheelchair(self):
        """Send stop command"""
        self.cmd_vel_pub.publish(Twist())

    def _speak_feedback(self, text: str):
        """Publish voice feedback"""
        if self.config.voice_feedback and text:
            msg = String()
            msg.data = text
            self.feedback_pub.publish(msg)

    def _publish_status(self, status: str):
        """Publish voice control status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def set_locations(self, locations: Dict[str, PoseStamped]):
        """Set available navigation locations"""
        self.locations = locations

    def add_custom_command(
        self,
        patterns: List[str],
        action: str,
        params: Dict[str, Any],
        description: str = ''
    ):
        """Add custom voice command"""
        command = VoiceCommand(
            patterns=patterns,
            command_type=VoiceCommandType.CUSTOM,
            action=action,
            parameters=params,
            description=description
        )
        self.matcher.add_command(command)


def main(args=None):
    rclpy.init(args=args)

    node = VoiceControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
