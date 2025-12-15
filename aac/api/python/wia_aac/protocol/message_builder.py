"""
WIA AAC Protocol Message Builder
"""

import uuid
import time
from typing import Optional, List, Dict, Any

from .message import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    WiaAacMessage,
    ConnectPayload,
    DisconnectPayload,
    SubscribePayload,
    CommandPayload,
    ProtocolError,
)


class MessageBuilder:
    """Message builder for WIA AAC protocol"""

    def __init__(self, version: str = PROTOCOL_VERSION):
        self.version = version

    def _create_message(self, msg_type: str, payload: Dict[str, Any]) -> WiaAacMessage:
        """Create a base message"""
        return WiaAacMessage(
            protocol=PROTOCOL_NAME,
            version=self.version,
            message_id=str(uuid.uuid4()),
            timestamp=int(time.time() * 1000),
            type=msg_type,
            payload=payload,
        )

    def connect(self, payload: ConnectPayload) -> WiaAacMessage:
        """Create a connect message"""
        return self._create_message("connect", payload.to_dict())

    def connect_ack(
        self,
        success: bool,
        session_id: Optional[str] = None,
        server_name: Optional[str] = None,
        server_version: Optional[str] = None,
        available_sensors: Optional[List[Dict[str, Any]]] = None,
        config: Optional[Dict[str, Any]] = None,
        error: Optional[ProtocolError] = None,
    ) -> WiaAacMessage:
        """Create a connect acknowledgment message"""
        payload = {"success": success}
        if session_id:
            payload["sessionId"] = session_id
        if server_name:
            payload["serverName"] = server_name
        if server_version:
            payload["serverVersion"] = server_version
        if available_sensors:
            payload["availableSensors"] = available_sensors
        if config:
            payload["config"] = config
        if error:
            payload["error"] = error.to_dict()
        return self._create_message("connect_ack", payload)

    def disconnect(
        self,
        reason: str = "user_request",
        message: Optional[str] = None,
    ) -> WiaAacMessage:
        """Create a disconnect message"""
        payload = {"reason": reason}
        if message:
            payload["message"] = message
        return self._create_message("disconnect", payload)

    def disconnect_ack(self) -> WiaAacMessage:
        """Create a disconnect acknowledgment message"""
        return self._create_message("disconnect_ack", {})

    def signal(self, signal_data: Dict[str, Any]) -> WiaAacMessage:
        """Create a signal message (wraps Phase 1 Signal)"""
        return self._create_message("signal", signal_data)

    def subscribe(self, payload: SubscribePayload) -> WiaAacMessage:
        """Create a subscribe message"""
        return self._create_message("subscribe", payload.to_dict())

    def subscribe_ack(
        self,
        success: bool,
        subscription_id: Optional[str] = None,
        active_streams: Optional[List[str]] = None,
        actual_signal_rate: Optional[int] = None,
        error: Optional[ProtocolError] = None,
    ) -> WiaAacMessage:
        """Create a subscribe acknowledgment message"""
        payload = {"success": success}
        if subscription_id:
            payload["subscriptionId"] = subscription_id
        if active_streams:
            payload["activeStreams"] = active_streams
        if actual_signal_rate:
            payload["actualSignalRate"] = actual_signal_rate
        if error:
            payload["error"] = error.to_dict()
        return self._create_message("subscribe_ack", payload)

    def unsubscribe(
        self,
        subscription_id: Optional[str] = None,
        streams: Optional[List[str]] = None,
    ) -> WiaAacMessage:
        """Create an unsubscribe message"""
        payload = {}
        if subscription_id:
            payload["subscriptionId"] = subscription_id
        if streams:
            payload["streams"] = streams
        return self._create_message("unsubscribe", payload)

    def unsubscribe_ack(self, success: bool) -> WiaAacMessage:
        """Create an unsubscribe acknowledgment message"""
        return self._create_message("unsubscribe_ack", {"success": success})

    def command(self, payload: CommandPayload) -> WiaAacMessage:
        """Create a command message"""
        return self._create_message("command", payload.to_dict())

    def command_ack(
        self,
        success: bool,
        command: str,
        result: Optional[Any] = None,
        error: Optional[ProtocolError] = None,
    ) -> WiaAacMessage:
        """Create a command acknowledgment message"""
        payload = {"success": success, "command": command}
        if result is not None:
            payload["result"] = result
        if error:
            payload["error"] = error.to_dict()
        return self._create_message("command_ack", payload)

    def error(self, error: ProtocolError) -> WiaAacMessage:
        """Create an error message"""
        return self._create_message("error", error.to_dict())

    def ping(self, sequence: int) -> WiaAacMessage:
        """Create a ping message"""
        return self._create_message("ping", {"sequence": sequence})

    def pong(self, sequence: int, latency_ms: Optional[int] = None) -> WiaAacMessage:
        """Create a pong message"""
        payload = {"sequence": sequence}
        if latency_ms is not None:
            payload["latency_ms"] = latency_ms
        return self._create_message("pong", payload)

    @staticmethod
    def create_error(
        code: int,
        name: str,
        message: str,
        recoverable: bool = True,
        details: Optional[Dict[str, Any]] = None,
        related_message_id: Optional[str] = None,
    ) -> ProtocolError:
        """Create a protocol error"""
        return ProtocolError(
            code=code,
            name=name,
            message=message,
            recoverable=recoverable,
            details=details,
            related_message_id=related_message_id,
        )


# Singleton instance
message_builder = MessageBuilder()
