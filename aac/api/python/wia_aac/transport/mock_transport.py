"""
WIA AAC Mock Transport
Mock implementation for testing purposes
"""

import asyncio
import json
import time
import random
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Callable

from .base_transport import ITransport, TransportState, MessageHandler, OpenHandler, CloseHandler, ErrorHandler
from ..protocol.message import WiaAacMessage
from ..protocol.message_builder import MessageBuilder


@dataclass
class MockTransportOptions:
    """Mock transport options"""
    response_delay: float = 0.01  # seconds
    simulate_connection_failure: bool = False
    available_sensors: Optional[List[Dict[str, Any]]] = None
    auto_generate_signals: bool = False
    signal_interval: float = 0.1  # seconds

    def __post_init__(self):
        if self.available_sensors is None:
            self.available_sensors = [
                {
                    "type": "eye_tracker",
                    "manufacturer": "MockSensor",
                    "model": "Test Eye Tracker",
                    "deviceId": "mock-et-001",
                }
            ]


class MockTransport(ITransport):
    """Mock transport for testing"""

    def __init__(self, options: Optional[MockTransportOptions] = None):
        self.options = options or MockTransportOptions()
        self.builder = MessageBuilder()

        self._state: TransportState = "disconnected"
        self._session_id: Optional[str] = None
        self._subscription_id: Optional[str] = None
        self._subscribed_streams: List[str] = []
        self._signal_task: Optional[asyncio.Task] = None

        self._message_handler: MessageHandler = lambda x: None
        self._open_handler: OpenHandler = lambda: None
        self._close_handler: CloseHandler = lambda x: None
        self._error_handler: ErrorHandler = lambda x: None

    async def connect(self, url: str) -> None:
        """Connect (simulated)"""
        if self.options.simulate_connection_failure:
            self._state = "error"
            raise ConnectionError("Simulated connection failure")

        self._state = "connecting"
        await asyncio.sleep(self.options.response_delay)
        self._state = "connected"
        self._open_handler()

    async def disconnect(self) -> None:
        """Disconnect (simulated)"""
        self._stop_signal_generation()
        self._state = "disconnected"
        self._session_id = None
        self._subscription_id = None
        self._subscribed_streams = []
        self._close_handler("Client disconnect")

    async def send(self, message: WiaAacMessage) -> None:
        """Send a message and simulate response"""
        if self._state != "connected":
            raise RuntimeError("Not connected")

        await asyncio.sleep(self.options.response_delay)
        self._simulate_response(message)

    def is_connected(self) -> bool:
        """Check if connected"""
        return self._state == "connected"

    def get_state(self) -> TransportState:
        """Get transport state"""
        return self._state

    def on_message(self, handler: MessageHandler) -> None:
        """Set message handler"""
        self._message_handler = handler

    def on_open(self, handler: OpenHandler) -> None:
        """Set open handler"""
        self._open_handler = handler

    def on_close(self, handler: CloseHandler) -> None:
        """Set close handler"""
        self._close_handler = handler

    def on_error(self, handler: ErrorHandler) -> None:
        """Set error handler"""
        self._error_handler = handler

    # Mock-specific methods

    def inject_message(self, message: WiaAacMessage) -> None:
        """Manually inject a message (for testing)"""
        self._message_handler(json.dumps(message.to_dict()))

    def emit_signal(self, sensor_type: str = "eye_tracker") -> None:
        """Simulate sending a signal from the 'server'"""
        signal = self._create_mock_signal(sensor_type)
        message = self.builder.signal(signal)
        self._message_handler(json.dumps(message.to_dict()))

    # Private methods

    def _simulate_response(self, message: WiaAacMessage) -> None:
        """Simulate server response"""
        if message.type == "connect":
            self._handle_connect(message)
        elif message.type == "disconnect":
            self._handle_disconnect(message)
        elif message.type == "subscribe":
            self._handle_subscribe(message)
        elif message.type == "unsubscribe":
            self._handle_unsubscribe(message)
        elif message.type == "command":
            self._handle_command(message)
        elif message.type == "ping":
            self._handle_ping(message)

    def _handle_connect(self, message: WiaAacMessage) -> None:
        """Handle connect message"""
        self._session_id = f"session-{int(time.time() * 1000)}"
        response = self.builder.connect_ack(
            success=True,
            session_id=self._session_id,
            server_name="WIA AAC Mock Server",
            server_version="1.0.0",
            available_sensors=self.options.available_sensors,
            config={"maxSignalRate": 120, "heartbeatInterval": 30000},
        )
        self._message_handler(json.dumps(response.to_dict()))

    def _handle_disconnect(self, message: WiaAacMessage) -> None:
        """Handle disconnect message"""
        response = self.builder.disconnect_ack()
        self._message_handler(json.dumps(response.to_dict()))
        self._stop_signal_generation()

    def _handle_subscribe(self, message: WiaAacMessage) -> None:
        """Handle subscribe message"""
        streams = message.payload.get("streams", [])
        self._subscription_id = f"sub-{int(time.time() * 1000)}"
        self._subscribed_streams = streams

        response = self.builder.subscribe_ack(
            success=True,
            subscription_id=self._subscription_id,
            active_streams=streams,
            actual_signal_rate=60,
        )
        self._message_handler(json.dumps(response.to_dict()))

        if self.options.auto_generate_signals:
            self._start_signal_generation()

    def _handle_unsubscribe(self, message: WiaAacMessage) -> None:
        """Handle unsubscribe message"""
        self._stop_signal_generation()
        self._subscription_id = None
        self._subscribed_streams = []

        response = self.builder.unsubscribe_ack(success=True)
        self._message_handler(json.dumps(response.to_dict()))

    def _handle_command(self, message: WiaAacMessage) -> None:
        """Handle command message"""
        command = message.payload.get("command", "")
        response = self.builder.command_ack(
            success=True,
            command=command,
            result={"message": "Command executed successfully"},
        )
        self._message_handler(json.dumps(response.to_dict()))

    def _handle_ping(self, message: WiaAacMessage) -> None:
        """Handle ping message"""
        sequence = message.payload.get("sequence", 0)
        response = self.builder.pong(sequence, latency_ms=5)
        self._message_handler(json.dumps(response.to_dict()))

    def _start_signal_generation(self) -> None:
        """Start generating mock signals"""
        self._stop_signal_generation()

        async def signal_loop():
            while self._state == "connected" and self._subscribed_streams:
                for stream in self._subscribed_streams:
                    self.emit_signal(stream)
                await asyncio.sleep(self.options.signal_interval)

        self._signal_task = asyncio.create_task(signal_loop())

    def _stop_signal_generation(self) -> None:
        """Stop generating mock signals"""
        if self._signal_task:
            self._signal_task.cancel()
            self._signal_task = None

    def _create_mock_signal(self, sensor_type: str) -> Dict[str, Any]:
        """Create a mock signal"""
        base_signal = {
            "$schema": "https://wia.live/aac/signal/v1/schema.json",
            "version": "1.0.0",
            "type": sensor_type,
            "timestamp": {
                "unix_ms": int(time.time() * 1000),
                "iso": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()),
            },
            "sequence": random.randint(1, 10000),
            "device": {
                "manufacturer": "MockSensor",
                "model": f"Test {sensor_type}",
            },
            "meta": {
                "confidence": 0.95,
            },
        }

        # Add type-specific data
        if sensor_type == "eye_tracker":
            base_signal["data"] = {
                "gaze_point": {"x": random.random(), "y": random.random()},
                "fixation": {"active": False, "duration_ms": 0},
            }
        elif sensor_type == "switch":
            base_signal["data"] = {
                "switch_id": 1,
                "state": "pressed" if random.random() > 0.5 else "released",
                "duration_ms": random.randint(0, 500),
            }
        elif sensor_type == "muscle_sensor":
            base_signal["data"] = {
                "channel_id": 1,
                "activation_level": random.random(),
                "threshold_exceeded": random.random() > 0.7,
            }
        elif sensor_type == "brain_interface":
            base_signal["data"] = {
                "bands": {
                    "alpha": random.random(),
                    "beta": random.random(),
                    "theta": random.random(),
                    "delta": random.random(),
                },
                "mental_command": "neutral",
            }
        elif sensor_type == "breath":
            base_signal["data"] = {
                "action": "sip" if random.random() > 0.5 else "puff",
                "pressure_kpa": random.random() * 5,
                "intensity": "soft",
            }
        elif sensor_type == "head_movement":
            base_signal["data"] = {
                "position": {"x": random.random(), "y": random.random()},
                "rotation": {
                    "pitch": (random.random() - 0.5) * 20,
                    "yaw": (random.random() - 0.5) * 20,
                    "roll": (random.random() - 0.5) * 10,
                },
            }

        return base_signal
