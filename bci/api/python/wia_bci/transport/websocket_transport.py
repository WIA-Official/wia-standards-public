"""
WIA BCI WebSocket Transport

WebSocket-based transport implementation.
"""

import asyncio
from dataclasses import dataclass, field
from typing import Optional

from .base_transport import BaseTransport, TransportState
from ..protocol.types import (
    BciMessage,
    TransportOptions,
    HeartbeatConfig,
    ReconnectConfig,
)
from ..protocol.message_parser import MessageParser, serialize, serialize_binary
from ..protocol.message_builder import MessageBuilder

# Optional websockets import
try:
    import websockets
    from websockets.client import WebSocketClientProtocol

    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    WebSocketClientProtocol = None


@dataclass
class WebSocketTransportOptions(TransportOptions):
    """WebSocket transport options."""

    protocols: list = field(default_factory=lambda: ["wia-bci-v1"])
    binary_mode: bool = False
    heartbeat: Optional[HeartbeatConfig] = None
    reconnect: Optional[ReconnectConfig] = None


# Default configurations
DEFAULT_HEARTBEAT = HeartbeatConfig(
    ping_interval=30000,
    pong_timeout=10000,
    max_missed_pongs=3,
)

DEFAULT_RECONNECT = ReconnectConfig(
    enabled=True,
    max_attempts=5,
    initial_delay=1000,
    max_delay=30000,
    backoff_multiplier=2.0,
)


class WebSocketTransport(BaseTransport):
    """WebSocket transport implementation."""

    def __init__(self, options: Optional[WebSocketTransportOptions] = None):
        super().__init__()
        if not WEBSOCKETS_AVAILABLE:
            raise ImportError(
                "websockets package is required for WebSocketTransport. "
                "Install with: pip install websockets"
            )

        self._ws: Optional[WebSocketClientProtocol] = None
        self._parser = MessageParser()
        self._builder = MessageBuilder()
        self._url: str = ""
        self._binary_mode: bool = options.binary_mode if options else False
        self._heartbeat_config = options.heartbeat if options and options.heartbeat else DEFAULT_HEARTBEAT
        self._reconnect_config = options.reconnect if options and options.reconnect else DEFAULT_RECONNECT
        self._ping_task: Optional[asyncio.Task] = None
        self._receive_task: Optional[asyncio.Task] = None
        self._reconnect_attempts: int = 0
        self._missed_pongs: int = 0
        self._intentional_close: bool = False
        self._pong_received: asyncio.Event = asyncio.Event()

    async def connect(
        self, url: str, options: Optional[WebSocketTransportOptions] = None
    ) -> None:
        """Connect to WebSocket server."""
        if self._state != TransportState.DISCONNECTED:
            raise ConnectionError("Transport already connected or connecting")

        self._url = url
        self._intentional_close = False

        if options:
            if options.binary_mode is not None:
                self._binary_mode = options.binary_mode
            if options.heartbeat:
                self._heartbeat_config = options.heartbeat
            if options.reconnect:
                self._reconnect_config = options.reconnect

        await self._do_connect()

    async def _do_connect(self) -> None:
        """Internal connect."""
        self._state = TransportState.CONNECTING

        try:
            self._ws = await websockets.connect(
                self._url,
                subprotocols=["wia-bci-v1"],
            )

            self._state = TransportState.CONNECTED
            self._reconnect_attempts = 0
            self._start_heartbeat()
            self._start_receive()
            self._emit_open()

        except Exception as e:
            self._state = TransportState.DISCONNECTED
            raise e

    async def disconnect(self) -> None:
        """Disconnect from server."""
        self._intentional_close = True
        self._stop_heartbeat()
        self._stop_receive()

        if self._ws and self._state == TransportState.CONNECTED:
            self._state = TransportState.CLOSING
            await self._ws.close(1000, "Client disconnect")

        self._state = TransportState.DISCONNECTED
        self._ws = None

    async def send(self, message: BciMessage) -> None:
        """Send message."""
        if not self._ws or self._state != TransportState.CONNECTED:
            raise ConnectionError("Not connected")

        if self._binary_mode:
            data = serialize_binary(message)
            await self._ws.send(data)
        else:
            data = serialize(message)
            await self._ws.send(data)

    def _start_receive(self) -> None:
        """Start receive task."""
        self._receive_task = asyncio.create_task(self._receive_loop())

    def _stop_receive(self) -> None:
        """Stop receive task."""
        if self._receive_task:
            self._receive_task.cancel()
            self._receive_task = None

    async def _receive_loop(self) -> None:
        """Receive loop."""
        try:
            async for data in self._ws:
                await self._handle_message(data)
        except websockets.ConnectionClosed as e:
            await self._handle_close(e.code, e.reason or "")
        except Exception as e:
            self._emit_error(e)

    async def _handle_message(self, data) -> None:
        """Handle incoming message."""
        result = self._parser.parse(data)

        if result.success and result.message:
            # Handle pong internally
            if result.message.type.value == "pong":
                self._handle_pong()
                return

            self._emit_message(result.message)
        elif result.error:
            self._emit_error(Exception(result.error.get("message", "Parse error")))

    async def _handle_close(self, code: int, reason: str) -> None:
        """Handle connection close."""
        self._stop_heartbeat()

        if self._intentional_close:
            self._state = TransportState.DISCONNECTED
            self._emit_close(code, reason)
            return

        # Attempt reconnection
        if (
            self._reconnect_config.enabled
            and self._reconnect_attempts < self._reconnect_config.max_attempts
        ):
            await self._attempt_reconnect()
        else:
            self._state = TransportState.DISCONNECTED
            self._emit_close(code, reason)

    async def _attempt_reconnect(self) -> None:
        """Attempt reconnection."""
        self._reconnect_attempts += 1

        delay = min(
            self._reconnect_config.initial_delay
            * (self._reconnect_config.backoff_multiplier ** (self._reconnect_attempts - 1)),
            self._reconnect_config.max_delay,
        )

        await asyncio.sleep(delay / 1000)

        try:
            await self._do_connect()
        except Exception:
            # Will be handled by _handle_close
            pass

    def _start_heartbeat(self) -> None:
        """Start heartbeat."""
        self._missed_pongs = 0
        self._ping_task = asyncio.create_task(self._heartbeat_loop())

    def _stop_heartbeat(self) -> None:
        """Stop heartbeat."""
        if self._ping_task:
            self._ping_task.cancel()
            self._ping_task = None

    async def _heartbeat_loop(self) -> None:
        """Heartbeat loop."""
        try:
            while self._state == TransportState.CONNECTED:
                await asyncio.sleep(self._heartbeat_config.ping_interval / 1000)
                await self._send_ping()
        except asyncio.CancelledError:
            pass

    async def _send_ping(self) -> None:
        """Send ping."""
        if not self._ws or self._state != TransportState.CONNECTED:
            return

        ping = self._builder.ping()

        try:
            await self.send(ping)
            self._pong_received.clear()

            # Wait for pong
            try:
                await asyncio.wait_for(
                    self._pong_received.wait(),
                    timeout=self._heartbeat_config.pong_timeout / 1000,
                )
            except asyncio.TimeoutError:
                self._handle_pong_timeout()
        except Exception:
            pass

    def _handle_pong(self) -> None:
        """Handle pong response."""
        self._missed_pongs = 0
        self._pong_received.set()

    def _handle_pong_timeout(self) -> None:
        """Handle pong timeout."""
        self._missed_pongs += 1

        if self._missed_pongs >= self._heartbeat_config.max_missed_pongs:
            # Connection lost
            if self._ws:
                asyncio.create_task(self._ws.close(4000, "Heartbeat timeout"))

    def dispose(self) -> None:
        """Dispose transport."""
        self._stop_heartbeat()
        self._stop_receive()
        super().dispose()
