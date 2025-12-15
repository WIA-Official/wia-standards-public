"""
WIA AAC Protocol Handler
Handles protocol-level operations (connection, subscription, commands)
"""

import asyncio
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List, Literal
from enum import Enum

from .message import (
    WiaAacMessage,
    ConnectPayload,
    SubscribePayload,
    CommandPayload,
    ProtocolError,
    ProtocolErrorCode,
)
from .message_builder import MessageBuilder
from .message_parser import MessageParser


ProtocolState = Literal[
    "disconnected",
    "connecting",
    "connected",
    "reconnecting",
    "error",
]


@dataclass
class ProtocolOptions:
    """Protocol configuration options"""
    auto_reconnect: bool = True
    reconnect_interval: int = 1000  # ms
    max_reconnect_attempts: int = 5
    heartbeat_interval: int = 30000  # ms
    heartbeat_timeout: int = 10000  # ms


class ProtocolHandler:
    """Protocol handler for WIA AAC"""

    def __init__(self, options: Optional[ProtocolOptions] = None):
        self.options = options or ProtocolOptions()
        self.builder = MessageBuilder()
        self.parser = MessageParser()

        self._transport = None
        self._state: ProtocolState = "disconnected"
        self._session_id: Optional[str] = None
        self._subscription_id: Optional[str] = None
        self._reconnect_attempts = 0
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._heartbeat_sequence = 0

        # Event handlers
        self._handlers: Dict[str, List[Callable]] = {
            "stateChange": [],
            "signal": [],
            "error": [],
            "message": [],
        }

        # Pending responses
        self._pending: Dict[str, asyncio.Future] = {}

    def set_transport(self, transport) -> None:
        """Set the transport layer"""
        self._transport = transport
        self._transport.on_message(self._handle_message)
        self._transport.on_close(self._handle_close)
        self._transport.on_error(self._handle_error)

    async def connect(
        self,
        url: str,
        client_id: str,
        client_name: Optional[str] = None,
        capabilities: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Connect to a server"""
        if not self._transport:
            raise RuntimeError("Transport not set. Call set_transport() first.")

        self._set_state("connecting")

        try:
            # Connect transport
            await self._transport.connect(url)

            # Send connect message
            payload = ConnectPayload(
                client_id=client_id,
                client_name=client_name,
                capabilities=capabilities,
            )
            connect_msg = self.builder.connect(payload)
            response = await self._send_and_wait(connect_msg, "connect_ack")

            if response.payload.get("success"):
                self._session_id = response.payload.get("sessionId")
                self._set_state("connected")
                self._start_heartbeat()
                self._reconnect_attempts = 0
            else:
                self._set_state("error")
                error = response.payload.get("error", {})
                raise Exception(error.get("message", "Connection failed"))

            return response.payload

        except Exception as e:
            self._set_state("error")
            raise

    async def disconnect(self, reason: str = "user_request") -> None:
        """Disconnect from the server"""
        if not self._transport or self._state == "disconnected":
            return

        self._stop_heartbeat()

        try:
            disconnect_msg = self.builder.disconnect(reason)
            await self._transport.send(disconnect_msg)
        except Exception:
            pass

        await self._transport.disconnect()
        self._set_state("disconnected")
        self._session_id = None
        self._subscription_id = None

    async def subscribe(
        self,
        streams: List[str],
        signal_rate: Optional[int] = None,
    ) -> Optional[str]:
        """Subscribe to sensor streams"""
        if self._state != "connected":
            raise RuntimeError("Not connected")

        payload = SubscribePayload(
            streams=streams,
            options={"signalRate": signal_rate} if signal_rate else None,
        )
        subscribe_msg = self.builder.subscribe(payload)
        response = await self._send_and_wait(subscribe_msg, "subscribe_ack")

        if response.payload.get("success"):
            self._subscription_id = response.payload.get("subscriptionId")
            return self._subscription_id
        else:
            error = response.payload.get("error", {})
            raise Exception(error.get("message", "Subscription failed"))

    async def unsubscribe(self) -> None:
        """Unsubscribe from sensor streams"""
        if self._state != "connected" or not self._subscription_id:
            return

        unsubscribe_msg = self.builder.unsubscribe(subscription_id=self._subscription_id)
        await self._send_and_wait(unsubscribe_msg, "unsubscribe_ack")
        self._subscription_id = None

    async def send_command(
        self,
        command: str,
        target: str,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Any:
        """Send a command to the sensor"""
        if self._state != "connected":
            raise RuntimeError("Not connected")

        payload = CommandPayload(command=command, target=target, parameters=parameters)
        command_msg = self.builder.command(payload)
        response = await self._send_and_wait(command_msg, "command_ack")

        if response.payload.get("success"):
            return response.payload.get("result")
        else:
            error = response.payload.get("error", {})
            raise Exception(error.get("message", "Command failed"))

    def get_state(self) -> ProtocolState:
        """Get current protocol state"""
        return self._state

    def get_session_id(self) -> Optional[str]:
        """Get session ID"""
        return self._session_id

    def on(self, event: str, handler: Callable) -> None:
        """Subscribe to protocol events"""
        if event not in self._handlers:
            self._handlers[event] = []
        self._handlers[event].append(handler)

    def off(self, event: str, handler: Callable) -> None:
        """Unsubscribe from protocol events"""
        if event in self._handlers:
            self._handlers[event] = [h for h in self._handlers[event] if h != handler]

    # Private methods

    def _set_state(self, new_state: ProtocolState) -> None:
        """Set protocol state and emit event"""
        if self._state != new_state:
            old_state = self._state
            self._state = new_state
            self._emit("stateChange", {"oldState": old_state, "newState": new_state})

    def _emit(self, event: str, data: Any) -> None:
        """Emit an event"""
        for handler in self._handlers.get(event, []):
            try:
                handler(data)
            except Exception as e:
                print(f"Error in event handler for '{event}': {e}")

    async def _send_and_wait(
        self,
        message: WiaAacMessage,
        expected_type: str,
        timeout: float = 10.0,
    ) -> WiaAacMessage:
        """Send a message and wait for response"""
        future = asyncio.get_event_loop().create_future()
        self._pending[message.message_id] = future

        try:
            await self._transport.send(message)
            response = await asyncio.wait_for(future, timeout=timeout)
            return response
        except asyncio.TimeoutError:
            raise Exception(f"Timeout waiting for {expected_type}")
        finally:
            self._pending.pop(message.message_id, None)

    def _handle_message(self, data: str) -> None:
        """Handle incoming message"""
        result = self.parser.parse(data)

        if not result.success or not result.message:
            self._emit("error", result.error)
            return

        message = result.message

        # Check for pending response
        if message.type.endswith("_ack"):
            for msg_id, future in list(self._pending.items()):
                if not future.done():
                    future.set_result(message)
                    return

        # Handle specific message types
        if message.type == "signal":
            self._emit("signal", message.payload)
        elif message.type == "error":
            self._emit("error", message.payload)
        elif message.type == "ping":
            self._send_pong(message.payload.get("sequence", 0))
        elif message.type == "disconnect":
            self._stop_heartbeat()
            self._set_state("disconnected")
            self._session_id = None
        else:
            self._emit("message", message)

    def _handle_close(self, reason: str) -> None:
        """Handle transport close"""
        self._stop_heartbeat()
        self._set_state("disconnected")

    def _handle_error(self, error: Exception) -> None:
        """Handle transport error"""
        self._emit("error", {
            "code": ProtocolErrorCode.CONNECTION_LOST,
            "name": "TRANSPORT_ERROR",
            "message": str(error),
            "recoverable": True,
        })

    def _start_heartbeat(self) -> None:
        """Start heartbeat task"""
        if self._heartbeat_task:
            self._heartbeat_task.cancel()

        async def heartbeat_loop():
            while self._state == "connected":
                await asyncio.sleep(self.options.heartbeat_interval / 1000)
                await self._send_ping()

        self._heartbeat_task = asyncio.create_task(heartbeat_loop())

    def _stop_heartbeat(self) -> None:
        """Stop heartbeat task"""
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            self._heartbeat_task = None

    async def _send_ping(self) -> None:
        """Send a ping message"""
        if not self._transport or self._state != "connected":
            return

        self._heartbeat_sequence += 1
        ping_msg = self.builder.ping(self._heartbeat_sequence)
        try:
            await self._transport.send(ping_msg)
        except Exception:
            pass

    def _send_pong(self, sequence: int) -> None:
        """Send a pong message (sync, fire-and-forget)"""
        if not self._transport or self._state != "connected":
            return

        pong_msg = self.builder.pong(sequence)
        asyncio.create_task(self._transport.send(pong_msg))
