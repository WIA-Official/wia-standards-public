"""
WIA BCI Mock Transport

Mock transport for development and testing.
"""

import asyncio
import math
import random
import time
from dataclasses import dataclass, field
from typing import Optional, List

from .base_transport import BaseTransport, TransportState
from ..protocol.types import BciMessage, TransportOptions, SignalPayload
from ..protocol.message_builder import MessageBuilder


@dataclass
class MockTransportOptions(TransportOptions):
    """Mock transport options."""

    latency: int = 10  # Simulated latency in ms
    drop_rate: float = 0  # Message drop rate 0-1
    disconnect_chance: float = 0  # Random disconnect chance per message
    auto_stream: bool = True  # Auto-generate signals when streaming
    signal_interval: int = 4  # Signal interval in ms (4 = 250Hz)


class MockTransport(BaseTransport):
    """Mock transport implementation for testing."""

    def __init__(self, options: Optional[MockTransportOptions] = None):
        super().__init__()
        opts = options or MockTransportOptions()
        self._latency = opts.latency
        self._drop_rate = opts.drop_rate
        self._disconnect_chance = opts.disconnect_chance
        self._auto_stream = opts.auto_stream
        self._signal_interval = opts.signal_interval
        self._builder = MessageBuilder()
        self._stream_task: Optional[asyncio.Task] = None
        self._sample_index: int = 0
        self._channels: List[str] = ["Fp1", "Fp2", "C3", "C4", "O1", "O2", "P3", "P4"]
        self._session_id: Optional[str] = None
        self._is_streaming: bool = False

    async def connect(
        self, url: str, options: Optional[MockTransportOptions] = None
    ) -> None:
        """Connect to mock server."""
        if self._state != TransportState.DISCONNECTED:
            raise ConnectionError("Already connected")

        if options:
            if options.latency is not None:
                self._latency = options.latency
            if options.drop_rate is not None:
                self._drop_rate = options.drop_rate
            if options.disconnect_chance is not None:
                self._disconnect_chance = options.disconnect_chance

        self._state = TransportState.CONNECTING

        await asyncio.sleep(self._latency / 1000)

        self._state = TransportState.CONNECTED
        self._session_id = f"mock-session-{int(time.time() * 1000)}"
        self._builder.set_session_id(self._session_id)
        self._emit_open()

    async def disconnect(self) -> None:
        """Disconnect."""
        self._stop_streaming()
        self._state = TransportState.DISCONNECTED
        self._emit_close(1000, "Client disconnect")

    async def send(self, message: BciMessage) -> None:
        """Send message."""
        if self._state != TransportState.CONNECTED:
            raise ConnectionError("Not connected")

        # Simulate random disconnect
        if random.random() < self._disconnect_chance:
            await self._handle_random_disconnect()
            return

        # Simulate message drop
        if random.random() < self._drop_rate:
            return

        await asyncio.sleep(self._latency / 1000)

        # Handle message types
        await self._handle_message(message)

    async def _handle_message(self, message: BciMessage) -> None:
        """Handle incoming message (from client)."""
        msg_type = message.type.value if hasattr(message.type, "value") else message.type

        if msg_type == "connect":
            await self._handle_connect(message)
        elif msg_type == "start_stream":
            await self._handle_start_stream()
        elif msg_type == "stop_stream":
            await self._handle_stop_stream()
        elif msg_type == "ping":
            await self._handle_ping(message)
        elif msg_type == "marker":
            self._emit_message(message)

    async def _handle_connect(self, message: BciMessage) -> None:
        """Handle connect message."""
        from ..protocol.types import ConnectAckPayload

        response = self._builder.connect_ack(
            ConnectAckPayload(
                session_id=self._session_id,
                status="connected",
                server_info={"name": "MockServer", "version": "1.0.0"},
                device_info={
                    "type": "simulator",
                    "manufacturer": "WIA",
                    "model": "Mock Device",
                    "channels": len(self._channels),
                    "samplingRate": 250,
                },
                negotiated={
                    "samplingRate": 250,
                    "channels": self._channels,
                    "compression": False,
                },
            )
        )
        self._emit_message(response)

    async def _handle_start_stream(self) -> None:
        """Handle start stream."""
        from ..protocol.types import StreamAckPayload

        ack = self._builder.stream_ack(StreamAckPayload(status="started"))
        self._emit_message(ack)

        if self._auto_stream:
            self._start_streaming()

    async def _handle_stop_stream(self) -> None:
        """Handle stop stream."""
        from ..protocol.types import StreamAckPayload

        self._stop_streaming()

        ack = self._builder.stream_ack(StreamAckPayload(status="stopped"))
        self._emit_message(ack)

    async def _handle_ping(self, message: BciMessage) -> None:
        """Handle ping."""
        client_time = message.payload.get("clientTime")
        pong = self._builder.pong(client_time)
        self._emit_message(pong)

    def _start_streaming(self) -> None:
        """Start streaming signals."""
        if self._is_streaming:
            return

        self._is_streaming = True
        self._sample_index = 0
        self._stream_task = asyncio.create_task(self._stream_loop())

    def _stop_streaming(self) -> None:
        """Stop streaming."""
        if self._stream_task:
            self._stream_task.cancel()
            self._stream_task = None
        self._is_streaming = False

    async def _stream_loop(self) -> None:
        """Stream loop."""
        try:
            while self._is_streaming:
                await asyncio.sleep(self._signal_interval / 1000)
                self._generate_signal()
        except asyncio.CancelledError:
            pass

    def _generate_signal(self) -> None:
        """Generate mock signal."""
        timestamp = int(time.time() * 1000)
        data: List[float] = []

        # Generate simulated EEG data for each channel
        for i in range(len(self._channels)):
            # Simulate EEG signal: alpha (10 Hz) + noise
            alpha = 10 * math.sin(2 * math.pi * 10 * (self._sample_index / 250))
            beta = 5 * math.sin(2 * math.pi * 20 * (self._sample_index / 250))
            noise = (random.random() - 0.5) * 5
            data.append(alpha + beta + noise)

        from ..protocol.types import SignalPayload as SP

        payload = SP(
            sample_index=self._sample_index,
            timestamp=timestamp,
            channels=list(range(len(self._channels))),
            data=data,
        )

        signal = self._builder.signal(payload.to_dict())
        self._emit_message(signal)

        self._sample_index += 1

    async def _handle_random_disconnect(self) -> None:
        """Handle random disconnect."""
        self._stop_streaming()
        self._state = TransportState.DISCONNECTED
        self._emit_close(4001, "Random disconnect (simulated)")

    def set_channels(self, channels: List[str]) -> None:
        """Set channels for simulation."""
        self._channels = channels

    def inject_marker(
        self, code: int, label: str, value: Optional[str] = None
    ) -> None:
        """Inject a marker event."""
        from ..protocol.types import MarkerPayload

        marker = self._builder.marker(
            MarkerPayload(
                sample_index=self._sample_index,
                code=code,
                label=label,
                value=value,
            )
        )
        self._emit_message(marker)

    def dispose(self) -> None:
        """Dispose."""
        self._stop_streaming()
        super().dispose()
