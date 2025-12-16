#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Smart Home Matter Bridge

Provides integration with Matter/Thread smart home devices.
Enables automatic door opening, elevator calls, lighting control,
and other accessibility automations.
"""

import asyncio
import json
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Callable, Dict, List, Optional, Set
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


class DeviceType(IntEnum):
    """Smart home device types"""
    UNKNOWN = 0
    LIGHT = 1
    DOOR = 2
    LOCK = 3
    ELEVATOR = 4
    SWITCH = 5
    THERMOSTAT = 6
    SENSOR = 7
    SPEAKER = 8
    BLIND = 9


class DeviceState(IntEnum):
    """Device state"""
    UNKNOWN = 0
    OFF = 1
    ON = 2
    OPENING = 3
    CLOSING = 4
    LOCKED = 5
    UNLOCKED = 6


@dataclass
class SmartDevice:
    """Smart home device"""
    device_id: str
    name: str
    device_type: DeviceType
    room: str = ''
    state: DeviceState = DeviceState.UNKNOWN
    reachable: bool = True
    attributes: Dict[str, Any] = field(default_factory=dict)

    # Matter-specific
    matter_node_id: Optional[int] = None
    matter_endpoint: int = 1


@dataclass
class AccessibilityTrigger:
    """Location-based accessibility trigger"""
    name: str
    location: str
    radius: float  # meters
    actions: List[Dict[str, Any]]
    enabled: bool = True
    cooldown: float = 30.0  # seconds between triggers
    last_triggered: float = 0.0


@dataclass
class SmartHomeConfig:
    """Smart home bridge configuration"""
    # Matter settings
    matter_enabled: bool = True
    discovery_timeout: float = 30.0

    # Automation settings
    auto_open_doors: bool = True
    auto_call_elevator: bool = True
    auto_turn_on_lights: bool = True
    path_lighting: bool = True

    # Timing
    door_open_lead_time: float = 3.0    # seconds before arrival
    light_on_lead_time: float = 2.0
    elevator_call_distance: float = 5.0  # meters

    # Voice assistant
    alexa_enabled: bool = False
    google_home_enabled: bool = False
    siri_enabled: bool = False


class MatterDeviceManager:
    """
    Manages Matter/Thread smart home devices.

    Note: This is a simulation layer. In production, this would
    integrate with actual Matter SDK (chip-tool, python-matter-server).
    """

    def __init__(self, config: SmartHomeConfig):
        self.config = config
        self._devices: Dict[str, SmartDevice] = {}
        self._callbacks: Dict[str, List[Callable]] = {}

    def discover_devices(self) -> List[SmartDevice]:
        """Discover Matter devices on network"""
        # Simulated device discovery
        # Real implementation would use Matter commissioning
        simulated_devices = [
            SmartDevice(
                device_id='door_front',
                name='현관문',
                device_type=DeviceType.DOOR,
                room='현관',
                matter_node_id=1,
            ),
            SmartDevice(
                device_id='door_bedroom',
                name='침실문',
                device_type=DeviceType.DOOR,
                room='침실',
                matter_node_id=2,
            ),
            SmartDevice(
                device_id='door_bathroom',
                name='화장실문',
                device_type=DeviceType.DOOR,
                room='화장실',
                matter_node_id=3,
            ),
            SmartDevice(
                device_id='light_living',
                name='거실 조명',
                device_type=DeviceType.LIGHT,
                room='거실',
                matter_node_id=4,
                attributes={'brightness': 100, 'color_temp': 4000}
            ),
            SmartDevice(
                device_id='light_hallway',
                name='복도 조명',
                device_type=DeviceType.LIGHT,
                room='복도',
                matter_node_id=5,
            ),
            SmartDevice(
                device_id='light_bedroom',
                name='침실 조명',
                device_type=DeviceType.LIGHT,
                room='침실',
                matter_node_id=6,
            ),
            SmartDevice(
                device_id='elevator_main',
                name='엘리베이터',
                device_type=DeviceType.ELEVATOR,
                room='복도',
                matter_node_id=7,
                attributes={'current_floor': 1, 'target_floor': None}
            ),
            SmartDevice(
                device_id='lock_front',
                name='현관 도어락',
                device_type=DeviceType.LOCK,
                room='현관',
                matter_node_id=8,
                state=DeviceState.LOCKED
            ),
        ]

        for device in simulated_devices:
            self._devices[device.device_id] = device

        return simulated_devices

    def get_device(self, device_id: str) -> Optional[SmartDevice]:
        """Get device by ID"""
        return self._devices.get(device_id)

    def get_devices_by_type(self, device_type: DeviceType) -> List[SmartDevice]:
        """Get all devices of a type"""
        return [d for d in self._devices.values() if d.device_type == device_type]

    def get_devices_in_room(self, room: str) -> List[SmartDevice]:
        """Get all devices in a room"""
        return [d for d in self._devices.values() if d.room == room]

    async def control_device(
        self,
        device_id: str,
        action: str,
        params: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Control a device"""
        device = self._devices.get(device_id)
        if device is None:
            return False

        params = params or {}

        # Simulate device control
        if device.device_type == DeviceType.LIGHT:
            return await self._control_light(device, action, params)
        elif device.device_type == DeviceType.DOOR:
            return await self._control_door(device, action)
        elif device.device_type == DeviceType.LOCK:
            return await self._control_lock(device, action)
        elif device.device_type == DeviceType.ELEVATOR:
            return await self._control_elevator(device, action, params)

        return False

    async def _control_light(
        self,
        device: SmartDevice,
        action: str,
        params: Dict[str, Any]
    ) -> bool:
        """Control light device"""
        if action == 'on':
            device.state = DeviceState.ON
            if 'brightness' in params:
                device.attributes['brightness'] = params['brightness']
        elif action == 'off':
            device.state = DeviceState.OFF
        elif action == 'set_brightness':
            device.attributes['brightness'] = params.get('brightness', 100)
        elif action == 'set_color_temp':
            device.attributes['color_temp'] = params.get('color_temp', 4000)

        self._notify_change(device)
        return True

    async def _control_door(self, device: SmartDevice, action: str) -> bool:
        """Control door device"""
        if action == 'open':
            device.state = DeviceState.OPENING
            # Simulate door opening time
            await asyncio.sleep(0.5)
            device.state = DeviceState.ON  # Open
        elif action == 'close':
            device.state = DeviceState.CLOSING
            await asyncio.sleep(0.5)
            device.state = DeviceState.OFF  # Closed

        self._notify_change(device)
        return True

    async def _control_lock(self, device: SmartDevice, action: str) -> bool:
        """Control lock device"""
        if action == 'unlock':
            device.state = DeviceState.UNLOCKED
        elif action == 'lock':
            device.state = DeviceState.LOCKED

        self._notify_change(device)
        return True

    async def _control_elevator(
        self,
        device: SmartDevice,
        action: str,
        params: Dict[str, Any]
    ) -> bool:
        """Control elevator"""
        if action == 'call':
            target_floor = params.get('floor', 1)
            device.attributes['target_floor'] = target_floor
            # Elevator would arrive after some time
        elif action == 'send':
            target_floor = params.get('floor', 1)
            device.attributes['target_floor'] = target_floor

        self._notify_change(device)
        return True

    def on_device_change(self, device_id: str, callback: Callable):
        """Register callback for device changes"""
        if device_id not in self._callbacks:
            self._callbacks[device_id] = []
        self._callbacks[device_id].append(callback)

    def _notify_change(self, device: SmartDevice):
        """Notify listeners of device change"""
        callbacks = self._callbacks.get(device.device_id, [])
        for callback in callbacks:
            callback(device)


class AccessibilityAutomation:
    """Manages accessibility-focused automations"""

    def __init__(
        self,
        config: SmartHomeConfig,
        device_manager: MatterDeviceManager
    ):
        self.config = config
        self.device_manager = device_manager
        self._triggers: Dict[str, AccessibilityTrigger] = {}
        self._path_rooms: List[str] = []

    def add_trigger(self, trigger: AccessibilityTrigger):
        """Add location-based trigger"""
        self._triggers[trigger.name] = trigger

    def setup_default_triggers(self, locations: Dict[str, Dict]):
        """Setup default accessibility triggers based on locations"""
        for name, loc in locations.items():
            # Auto-open doors
            if self.config.auto_open_doors:
                door_devices = self._find_door_for_location(name)
                if door_devices:
                    self.add_trigger(AccessibilityTrigger(
                        name=f'door_{name}',
                        location=name,
                        radius=2.0,
                        actions=[
                            {'device': d.device_id, 'action': 'open'}
                            for d in door_devices
                        ]
                    ))

            # Auto-lights
            if self.config.auto_turn_on_lights:
                light_devices = self.device_manager.get_devices_in_room(name)
                light_devices = [d for d in light_devices if d.device_type == DeviceType.LIGHT]
                if light_devices:
                    self.add_trigger(AccessibilityTrigger(
                        name=f'light_{name}',
                        location=name,
                        radius=1.5,
                        actions=[
                            {'device': d.device_id, 'action': 'on'}
                            for d in light_devices
                        ]
                    ))

    def _find_door_for_location(self, location: str) -> List[SmartDevice]:
        """Find door devices for a location"""
        doors = self.device_manager.get_devices_by_type(DeviceType.DOOR)
        return [d for d in doors if d.room == location]

    async def check_triggers(
        self,
        current_pos: tuple,
        locations: Dict[str, Dict],
        current_time: float
    ) -> List[str]:
        """Check and execute triggered automations"""
        triggered = []

        for trigger in self._triggers.values():
            if not trigger.enabled:
                continue

            # Check cooldown
            if current_time - trigger.last_triggered < trigger.cooldown:
                continue

            # Get location position
            loc_data = locations.get(trigger.location)
            if loc_data is None:
                continue

            loc_pos = (loc_data.get('x', 0), loc_data.get('y', 0))

            # Check distance
            dist = ((current_pos[0] - loc_pos[0])**2 +
                    (current_pos[1] - loc_pos[1])**2)**0.5

            if dist <= trigger.radius:
                # Execute actions
                for action in trigger.actions:
                    await self.device_manager.control_device(
                        action['device'],
                        action['action'],
                        action.get('params')
                    )

                trigger.last_triggered = current_time
                triggered.append(trigger.name)

        return triggered

    def set_path_lighting(self, path_rooms: List[str]):
        """Enable lights along navigation path"""
        self._path_rooms = path_rooms

    async def update_path_lighting(self, current_room: str):
        """Update path lighting based on current position"""
        if not self.config.path_lighting:
            return

        for room in self._path_rooms:
            lights = self.device_manager.get_devices_in_room(room)
            lights = [d for d in lights if d.device_type == DeviceType.LIGHT]

            for light in lights:
                # Turn on lights in upcoming rooms, dim passed rooms
                if room == current_room:
                    await self.device_manager.control_device(
                        light.device_id, 'on', {'brightness': 100}
                    )
                elif room in self._path_rooms[:self._path_rooms.index(current_room)]:
                    await self.device_manager.control_device(
                        light.device_id, 'on', {'brightness': 30}
                    )


class MatterBridgeNode(Node):
    """
    ROS2 node for smart home integration.

    Subscribes:
        /wia_wheelchair/pose (geometry_msgs/PoseStamped)
        /wia_wheelchair/navigation_goal (std_msgs/String)

    Publishes:
        /wia_wheelchair/smarthome_status (std_msgs/String)
        /wia_wheelchair/door_status (std_msgs/String)

    Services:
        /wia_wheelchair/smarthome/discover
        /wia_wheelchair/smarthome/control_device
        /wia_wheelchair/smarthome/auto_door
        /wia_wheelchair/smarthome/call_elevator
    """

    def __init__(self):
        super().__init__('matter_bridge')

        # Declare parameters
        self.declare_parameter('auto_open_doors', True)
        self.declare_parameter('auto_call_elevator', True)
        self.declare_parameter('auto_lights', True)
        self.declare_parameter('path_lighting', True)

        # Load config
        self.config = SmartHomeConfig(
            auto_open_doors=self.get_parameter('auto_open_doors').value,
            auto_call_elevator=self.get_parameter('auto_call_elevator').value,
            auto_turn_on_lights=self.get_parameter('auto_lights').value,
            path_lighting=self.get_parameter('path_lighting').value,
        )

        # Components
        self.device_manager = MatterDeviceManager(self.config)
        self.automation = AccessibilityAutomation(self.config, self.device_manager)

        # State
        self.current_pose: Optional[PoseStamped] = None
        self.navigation_goal: Optional[str] = None
        self.discovered = False

        # Saved locations (would be loaded from navigation package)
        self.locations: Dict[str, Dict] = {}

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/wia_wheelchair/pose',
            self.pose_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            String,
            '/wia_wheelchair/navigation_goal',
            self.goal_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/smarthome_status',
            10
        )
        self.door_status_pub = self.create_publisher(
            String,
            '/wia_wheelchair/door_status',
            10
        )

        # Services
        self.discover_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/smarthome/discover',
            self.discover_callback
        )
        self.auto_door_srv = self.create_service(
            SetBool,
            '/wia_wheelchair/smarthome/auto_door',
            self.auto_door_callback
        )
        self.elevator_srv = self.create_service(
            Trigger,
            '/wia_wheelchair/smarthome/call_elevator',
            self.call_elevator_callback
        )

        # Automation timer
        self.automation_timer = self.create_timer(0.5, self.automation_loop)

        self.get_logger().info('Matter bridge initialized')

    def pose_callback(self, msg: PoseStamped):
        """Update current pose"""
        self.current_pose = msg

    def goal_callback(self, msg: String):
        """Receive navigation goal"""
        self.navigation_goal = msg.data

        # Pre-open doors along path
        if self.config.auto_open_doors:
            self._prepare_path_to_goal(msg.data)

    def discover_callback(self, request, response):
        """Discover smart home devices"""
        devices = self.device_manager.discover_devices()
        self.discovered = True

        # Setup automations
        self.automation.setup_default_triggers(self.locations)

        response.success = True
        response.message = f'Discovered {len(devices)} devices'
        self._publish_status(f'discovered:{len(devices)}')

        return response

    def auto_door_callback(self, request, response):
        """Enable/disable automatic door opening"""
        self.config.auto_open_doors = request.data
        response.success = True
        response.message = f'Auto door: {"enabled" if request.data else "disabled"}'
        return response

    def call_elevator_callback(self, request, response):
        """Call elevator to current floor"""
        elevators = self.device_manager.get_devices_by_type(DeviceType.ELEVATOR)

        if not elevators:
            response.success = False
            response.message = 'No elevator found'
            return response

        # Call first elevator
        elevator = elevators[0]
        asyncio.create_task(
            self.device_manager.control_device(
                elevator.device_id,
                'call',
                {'floor': 1}  # Would determine current floor
            )
        )

        response.success = True
        response.message = 'Elevator called'
        self._publish_status('elevator:called')

        return response

    def automation_loop(self):
        """Check automation triggers"""
        if not self.discovered or self.current_pose is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        current_pos = (
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        )

        # Check triggers
        asyncio.create_task(
            self._check_automations(current_pos, current_time)
        )

    async def _check_automations(self, current_pos: tuple, current_time: float):
        """Check and execute automations"""
        triggered = await self.automation.check_triggers(
            current_pos,
            self.locations,
            current_time
        )

        for trigger_name in triggered:
            self._publish_status(f'triggered:{trigger_name}')

    def _prepare_path_to_goal(self, goal: str):
        """Pre-open doors along path to goal"""
        # Simplified - would get actual path from navigation
        goal_data = self.locations.get(goal)
        if goal_data is None:
            return

        # Find doors between current position and goal
        doors = self.device_manager.get_devices_by_type(DeviceType.DOOR)

        for door in doors:
            if door.room == goal:
                asyncio.create_task(
                    self.device_manager.control_device(door.device_id, 'open')
                )
                self._publish_door_status(door.device_id, 'opening')

    def _publish_status(self, status: str):
        """Publish smart home status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_door_status(self, door_id: str, status: str):
        """Publish door status"""
        msg = String()
        msg.data = f'{door_id}:{status}'
        self.door_status_pub.publish(msg)

    def set_locations(self, locations: Dict[str, Dict]):
        """Set navigation locations for automation"""
        self.locations = locations
        if self.discovered:
            self.automation.setup_default_triggers(locations)

    def control_device_sync(self, device_id: str, action: str, params: Optional[Dict] = None):
        """Synchronous device control wrapper"""
        asyncio.create_task(
            self.device_manager.control_device(device_id, action, params)
        )


def main(args=None):
    rclpy.init(args=args)

    node = MatterBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
