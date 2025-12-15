"""
WIA Security WebSocket Protocol
Real-time event streaming
"""

import asyncio
import json
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, List, Any
from enum import Enum

try:
    import websockets
    from websockets.client import WebSocketClientProtocol
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

from ..types import WiaSecurityEvent, EventType
from ..validator import validate_event


# ============================================================================
# Types
# ============================================================================

@dataclass
class WebSocketConfig:
    """WebSocket connection configuration."""
    url: str
    reconnect: bool = True
    reconnect_interval: float = 5.0
    max_reconnect_attempts: int = 10
    heartbeat_interval: float = 30.0
    auth_token: Optional[str] = None


class MessageType(str, Enum):
    """WebSocket message types."""
    EVENT = "event"
    SUBSCRIBE = "subscribe"
    UNSUBSCRIBE = "unsubscribe"
    ACK = "ack"
    ERROR = "error"
    HEARTBEAT = "heartbeat"


@dataclass
class WebSocketMessage:
    """WebSocket message structure."""
    type: MessageType
    payload: Optional[Any] = None
    id: Optional[str] = None
    timestamp: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"type": self.type.value}
        if self.payload is not None:
            result["payload"] = self.payload
        if self.id is not None:
            result["id"] = self.id
        if self.timestamp is not None:
            result["timestamp"] = self.timestamp
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "WebSocketMessage":
        return cls(
            type=MessageType(data["type"]),
            payload=data.get("payload"),
            id=data.get("id"),
            timestamp=data.get("timestamp"),
        )


@dataclass
class SubscriptionFilter:
    """Filter for event subscriptions."""
    event_types: Optional[List[EventType]] = None
    severity_min: Optional[float] = None
    severity_max: Optional[float] = None
    tags: Optional[List[str]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.event_types:
            result["eventTypes"] = [t.value for t in self.event_types]
        if self.severity_min is not None:
            result["severityMin"] = self.severity_min
        if self.severity_max is not None:
            result["severityMax"] = self.severity_max
        if self.tags:
            result["tags"] = self.tags
        return result


EventHandler = Callable[[WiaSecurityEvent], None]
ErrorHandler = Callable[[Exception], None]
ConnectionHandler = Callable[[], None]


# ============================================================================
# WebSocket Client
# ============================================================================

class WiaWebSocketClient:
    """WebSocket client for real-time WIA Security events."""

    def __init__(self, config: WebSocketConfig):
        if not HAS_WEBSOCKETS:
            raise ImportError("websockets package is required. Install with: pip install websockets")

        self.config = config
        self._ws: Optional["WebSocketClientProtocol"] = None
        self._reconnect_attempts = 0
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._receive_task: Optional[asyncio.Task] = None
        self._running = False

        self._event_handlers: Dict[str, List[EventHandler]] = {}
        self._on_error: Optional[ErrorHandler] = None
        self._on_connect: Optional[ConnectionHandler] = None
        self._on_disconnect: Optional[ConnectionHandler] = None
        self._subscriptions: Dict[str, SubscriptionFilter] = {}

    async def connect(self) -> None:
        """Connect to WebSocket server."""
        url = self.config.url
        if self.config.auth_token:
            separator = "&" if "?" in url else "?"
            url = f"{url}{separator}token={self.config.auth_token}"

        try:
            self._ws = await websockets.connect(url)
            self._running = True
            self._reconnect_attempts = 0

            # Start heartbeat
            self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

            # Start receive loop
            self._receive_task = asyncio.create_task(self._receive_loop())

            # Resubscribe to channels
            await self._resubscribe()

            if self._on_connect:
                self._on_connect()

        except Exception as e:
            if self._on_error:
                self._on_error(e)
            raise

    async def disconnect(self) -> None:
        """Disconnect from WebSocket server."""
        self._running = False
        self.config.reconnect = False

        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            self._heartbeat_task = None

        if self._receive_task:
            self._receive_task.cancel()
            self._receive_task = None

        if self._ws:
            await self._ws.close()
            self._ws = None

    async def subscribe(self, channel: str, filter: Optional[SubscriptionFilter] = None) -> None:
        """Subscribe to events on a channel."""
        self._subscriptions[channel] = filter or SubscriptionFilter()

        if self.is_connected():
            await self._send(WebSocketMessage(
                type=MessageType.SUBSCRIBE,
                payload={"channel": channel, "filter": filter.to_dict() if filter else {}}
            ))

    async def unsubscribe(self, channel: str) -> None:
        """Unsubscribe from a channel."""
        self._subscriptions.pop(channel, None)

        if self.is_connected():
            await self._send(WebSocketMessage(
                type=MessageType.UNSUBSCRIBE,
                payload={"channel": channel}
            ))

    async def send_event(self, event: WiaSecurityEvent) -> None:
        """Send a security event."""
        result = validate_event(event)
        if not result.valid:
            raise ValueError(f"Invalid event: {', '.join(result.errors)}")

        await self._send(WebSocketMessage(
            type=MessageType.EVENT,
            payload=event.to_dict(),
            id=event.id,
            timestamp=event.timestamp
        ))

    def on(self, event_type: str, handler: EventHandler) -> None:
        """Register an event handler."""
        if event_type not in self._event_handlers:
            self._event_handlers[event_type] = []
        self._event_handlers[event_type].append(handler)

    def off(self, event_type: str, handler: EventHandler) -> None:
        """Remove an event handler."""
        if event_type in self._event_handlers:
            handlers = self._event_handlers[event_type]
            if handler in handlers:
                handlers.remove(handler)

    def on_error_handler(self, handler: ErrorHandler) -> None:
        """Set error handler."""
        self._on_error = handler

    def on_connect_handler(self, handler: ConnectionHandler) -> None:
        """Set connection handler."""
        self._on_connect = handler

    def on_disconnect_handler(self, handler: ConnectionHandler) -> None:
        """Set disconnection handler."""
        self._on_disconnect = handler

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._ws is not None and self._ws.open

    # -------------------------------------------------------------------------
    # Private Methods
    # -------------------------------------------------------------------------

    async def _send(self, message: WebSocketMessage) -> None:
        """Send a message."""
        if self.is_connected():
            await self._ws.send(json.dumps(message.to_dict()))

    async def _receive_loop(self) -> None:
        """Receive messages from WebSocket."""
        while self._running and self._ws:
            try:
                data = await self._ws.recv()
                await self._handle_message(data)
            except websockets.ConnectionClosed:
                if self._on_disconnect:
                    self._on_disconnect()
                await self._attempt_reconnect()
                break
            except Exception as e:
                if self._on_error:
                    self._on_error(e)

    async def _handle_message(self, data: str) -> None:
        """Handle incoming message."""
        try:
            message = WebSocketMessage.from_dict(json.loads(data))

            if message.type == MessageType.EVENT and message.payload:
                self._handle_event(message.payload)
            elif message.type == MessageType.ERROR:
                if self._on_error:
                    self._on_error(Exception(str(message.payload)))
            elif message.type == MessageType.HEARTBEAT:
                await self._send(WebSocketMessage(type=MessageType.HEARTBEAT))

        except Exception as e:
            if self._on_error:
                self._on_error(e)

    def _handle_event(self, payload: Dict[str, Any]) -> None:
        """Handle incoming event."""
        event = WiaSecurityEvent.from_dict(payload)

        # Notify handlers for this event type
        handlers = self._event_handlers.get(event.type.value, [])
        for handler in handlers:
            handler(event)

        # Notify wildcard handlers
        wildcard_handlers = self._event_handlers.get("*", [])
        for handler in wildcard_handlers:
            handler(event)

    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeats."""
        while self._running:
            await asyncio.sleep(self.config.heartbeat_interval)
            if self.is_connected():
                await self._send(WebSocketMessage(type=MessageType.HEARTBEAT))

    async def _attempt_reconnect(self) -> None:
        """Attempt to reconnect."""
        if not self.config.reconnect:
            return

        if self._reconnect_attempts >= self.config.max_reconnect_attempts:
            if self._on_error:
                self._on_error(Exception("Max reconnection attempts reached"))
            return

        self._reconnect_attempts += 1
        await asyncio.sleep(self.config.reconnect_interval)

        try:
            await self.connect()
        except Exception:
            pass  # Will try again

    async def _resubscribe(self) -> None:
        """Resubscribe to all channels."""
        for channel, filter in self._subscriptions.items():
            await self._send(WebSocketMessage(
                type=MessageType.SUBSCRIBE,
                payload={"channel": channel, "filter": filter.to_dict()}
            ))


# ============================================================================
# Factory
# ============================================================================

def create_websocket_client(config: WebSocketConfig) -> WiaWebSocketClient:
    """Create a WebSocket client instance."""
    return WiaWebSocketClient(config)
