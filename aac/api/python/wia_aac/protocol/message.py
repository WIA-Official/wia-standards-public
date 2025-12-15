"""
WIA AAC Protocol Message Types
"""

from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Literal
from enum import IntEnum

# Protocol constants
PROTOCOL_NAME = "wia-aac"
PROTOCOL_VERSION = "1.0.0"
DEFAULT_PORT = 8765
SUB_PROTOCOL = "wia-aac-v1"

# Message types
MessageType = Literal[
    "connect",
    "connect_ack",
    "disconnect",
    "disconnect_ack",
    "signal",
    "command",
    "command_ack",
    "subscribe",
    "subscribe_ack",
    "unsubscribe",
    "unsubscribe_ack",
    "error",
    "ping",
    "pong",
]


@dataclass
class WiaAacMessage:
    """Base message structure"""
    protocol: str
    version: str
    message_id: str
    timestamp: int
    type: MessageType
    payload: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "protocol": self.protocol,
            "version": self.version,
            "messageId": self.message_id,
            "timestamp": self.timestamp,
            "type": self.type,
            "payload": self.payload,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "WiaAacMessage":
        return cls(
            protocol=data.get("protocol", ""),
            version=data.get("version", ""),
            message_id=data.get("messageId", ""),
            timestamp=data.get("timestamp", 0),
            type=data.get("type", ""),
            payload=data.get("payload", {}),
        )


@dataclass
class ConnectPayload:
    """Connect message payload"""
    client_id: str
    client_name: Optional[str] = None
    client_version: Optional[str] = None
    capabilities: Optional[List[str]] = None
    auth_token: Optional[str] = None
    options: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {"clientId": self.client_id}
        if self.client_name:
            result["clientName"] = self.client_name
        if self.client_version:
            result["clientVersion"] = self.client_version
        if self.capabilities:
            result["capabilities"] = self.capabilities
        if self.auth_token:
            result["authToken"] = self.auth_token
        if self.options:
            result["options"] = self.options
        return result


@dataclass
class ConnectAckPayload:
    """Connect acknowledgment payload"""
    success: bool
    session_id: Optional[str] = None
    server_name: Optional[str] = None
    server_version: Optional[str] = None
    available_sensors: Optional[List[Dict[str, Any]]] = None
    config: Optional[Dict[str, Any]] = None
    error: Optional["ProtocolError"] = None


@dataclass
class DisconnectPayload:
    """Disconnect message payload"""
    reason: Literal["user_request", "timeout", "error", "server_shutdown"]
    message: Optional[str] = None


@dataclass
class SubscribePayload:
    """Subscribe message payload"""
    streams: List[str]
    device_filter: Optional[Dict[str, Any]] = None
    options: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {"streams": self.streams}
        if self.device_filter:
            result["deviceFilter"] = self.device_filter
        if self.options:
            result["options"] = self.options
        return result


@dataclass
class SubscribeAckPayload:
    """Subscribe acknowledgment payload"""
    success: bool
    subscription_id: Optional[str] = None
    active_streams: Optional[List[str]] = None
    actual_signal_rate: Optional[int] = None
    error: Optional["ProtocolError"] = None


@dataclass
class CommandPayload:
    """Command message payload"""
    command: Literal["calibrate", "configure", "reset", "get_status", "get_config"]
    target: str
    parameters: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {"command": self.command, "target": self.target}
        if self.parameters:
            result["parameters"] = self.parameters
        return result


@dataclass
class CommandAckPayload:
    """Command acknowledgment payload"""
    success: bool
    command: str
    result: Optional[Any] = None
    error: Optional["ProtocolError"] = None


@dataclass
class ProtocolError:
    """Protocol error"""
    code: int
    name: str
    message: str
    recoverable: bool = True
    details: Optional[Dict[str, Any]] = None
    related_message_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "code": self.code,
            "name": self.name,
            "message": self.message,
            "recoverable": self.recoverable,
        }
        if self.details:
            result["details"] = self.details
        if self.related_message_id:
            result["relatedMessageId"] = self.related_message_id
        return result


@dataclass
class PingPayload:
    """Ping message payload"""
    sequence: int


@dataclass
class PongPayload:
    """Pong message payload"""
    sequence: int
    latency_ms: Optional[int] = None


class ProtocolErrorCode(IntEnum):
    """Protocol error codes"""
    # Connection errors (1xxx)
    CONNECTION_CLOSED = 1000
    CONNECTION_LOST = 1001
    CONNECTION_TIMEOUT = 1002
    PROTOCOL_ERROR = 1003
    VERSION_MISMATCH = 1004
    INVALID_MESSAGE = 1005

    # Sensor errors (2xxx)
    SENSOR_NOT_FOUND = 2001
    SENSOR_BUSY = 2002
    SENSOR_ERROR = 2003
    SENSOR_DISCONNECTED = 2004
    CALIBRATION_REQUIRED = 2005
    CALIBRATION_FAILED = 2006

    # Auth errors (3xxx)
    AUTH_REQUIRED = 3001
    AUTH_FAILED = 3002
    PERMISSION_DENIED = 3003
    SESSION_EXPIRED = 3004

    # Subscription errors (4xxx)
    SUBSCRIPTION_FAILED = 4001
    STREAM_NOT_AVAILABLE = 4002
    RATE_LIMIT_EXCEEDED = 4003
