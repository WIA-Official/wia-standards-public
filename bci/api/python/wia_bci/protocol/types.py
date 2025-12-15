"""
WIA BCI Protocol Types

Phase 3: Communication Protocol
"""

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Any, Union, Literal

# Protocol constants
PROTOCOL_NAME = "wia-bci"
PROTOCOL_VERSION = "1.0.0"
DEFAULT_PORT = 9876
BINARY_MAGIC = 0x57494142  # "WIAB"


class MessageType(str, Enum):
    """Message types."""

    CONNECT = "connect"
    CONNECT_ACK = "connect_ack"
    DISCONNECT = "disconnect"
    START_STREAM = "start_stream"
    STOP_STREAM = "stop_stream"
    STREAM_ACK = "stream_ack"
    SIGNAL = "signal"
    SIGNAL_BATCH = "signal_batch"
    MARKER = "marker"
    COMMAND = "command"
    COMMAND_ACK = "command_ack"
    CONFIG = "config"
    STATUS = "status"
    ERROR = "error"
    PING = "ping"
    PONG = "pong"


class ConnectionState(str, Enum):
    """Connection states."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    ERROR = "error"


class ErrorCodes(IntEnum):
    """Error codes."""

    # Connection errors (1xxx)
    CONNECTION_CLOSED = 1000
    CONNECTION_LOST = 1001
    CONNECTION_TIMEOUT = 1002
    PROTOCOL_ERROR = 1003
    VERSION_MISMATCH = 1004

    # Device errors (2xxx)
    DEVICE_NOT_FOUND = 2001
    DEVICE_BUSY = 2002
    DEVICE_ERROR = 2003
    STREAM_ERROR = 2004

    # Message errors (3xxx)
    INVALID_MESSAGE = 3001
    INVALID_PAYLOAD = 3002
    UNSUPPORTED_TYPE = 3003

    # Auth errors (4xxx)
    AUTH_FAILED = 4001
    PERMISSION_DENIED = 4002


@dataclass
class BciMessage:
    """Base message structure."""

    protocol: str
    version: str
    message_id: str
    timestamp: int
    type: MessageType
    payload: Dict[str, Any]
    sequence: Optional[int] = None
    session_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {
            "protocol": self.protocol,
            "version": self.version,
            "messageId": self.message_id,
            "timestamp": self.timestamp,
            "type": self.type.value if isinstance(self.type, MessageType) else self.type,
            "payload": self.payload,
        }
        if self.sequence is not None:
            result["sequence"] = self.sequence
        if self.session_id is not None:
            result["sessionId"] = self.session_id
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BciMessage":
        """Create from dictionary."""
        return cls(
            protocol=data["protocol"],
            version=data["version"],
            message_id=data["messageId"],
            timestamp=data["timestamp"],
            type=MessageType(data["type"]),
            payload=data["payload"],
            sequence=data.get("sequence"),
            session_id=data.get("sessionId"),
        )


@dataclass
class ConnectPayload:
    """Connect message payload."""

    client_id: str
    client_name: Optional[str] = None
    client_version: Optional[str] = None
    capabilities: Optional[List[str]] = None
    options: Optional[Dict[str, Any]] = None
    auth: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"clientId": self.client_id}
        if self.client_name:
            result["clientName"] = self.client_name
        if self.client_version:
            result["clientVersion"] = self.client_version
        if self.capabilities:
            result["capabilities"] = self.capabilities
        if self.options:
            result["options"] = self.options
        if self.auth:
            result["auth"] = self.auth
        return result


@dataclass
class ConnectAckPayload:
    """Connect ack message payload."""

    session_id: str
    status: Literal["connected", "rejected"]
    server_info: Optional[Dict[str, Any]] = None
    device_info: Optional[Dict[str, Any]] = None
    negotiated: Optional[Dict[str, Any]] = None
    error: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "sessionId": self.session_id,
            "status": self.status,
        }
        if self.server_info:
            result["serverInfo"] = self.server_info
        if self.device_info:
            result["deviceInfo"] = self.device_info
        if self.negotiated:
            result["negotiated"] = self.negotiated
        if self.error:
            result["error"] = self.error
        return result


@dataclass
class DisconnectPayload:
    """Disconnect message payload."""

    reason: Optional[str] = None
    code: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.reason:
            result["reason"] = self.reason
        if self.code:
            result["code"] = self.code
        return result


@dataclass
class StartStreamPayload:
    """Start stream message payload."""

    channels: Optional[List[str]] = None
    sampling_rate: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.channels:
            result["channels"] = self.channels
        if self.sampling_rate:
            result["samplingRate"] = self.sampling_rate
        return result


@dataclass
class StopStreamPayload:
    """Stop stream message payload."""

    reason: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.reason:
            result["reason"] = self.reason
        return result


@dataclass
class StreamAckPayload:
    """Stream ack message payload."""

    status: Literal["started", "stopped", "error"]
    error: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"status": self.status}
        if self.error:
            result["error"] = self.error
        return result


@dataclass
class SignalPayload:
    """Signal message payload."""

    sample_index: int
    timestamp: int
    channels: List[int]
    data: List[float]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sampleIndex": self.sample_index,
            "timestamp": self.timestamp,
            "channels": self.channels,
            "data": self.data,
        }


@dataclass
class SignalBatchPayload:
    """Signal batch message payload."""

    start_sample_index: int
    sample_count: int
    channels: List[int]
    data: List[List[float]]
    timestamps: List[int]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "startSampleIndex": self.start_sample_index,
            "sampleCount": self.sample_count,
            "channels": self.channels,
            "data": self.data,
            "timestamps": self.timestamps,
        }


@dataclass
class MarkerPayload:
    """Marker message payload."""

    sample_index: int
    code: int
    label: str
    value: Optional[str] = None
    duration: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "sampleIndex": self.sample_index,
            "code": self.code,
            "label": self.label,
        }
        if self.value:
            result["value"] = self.value
        if self.duration:
            result["duration"] = self.duration
        return result


@dataclass
class CommandPayload:
    """Command message payload."""

    command: str
    params: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"command": self.command}
        if self.params:
            result["params"] = self.params
        return result


@dataclass
class CommandAckPayload:
    """Command ack message payload."""

    command: str
    status: Literal["success", "error"]
    result: Optional[Any] = None
    error: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result_dict: Dict[str, Any] = {
            "command": self.command,
            "status": self.status,
        }
        if self.result:
            result_dict["result"] = self.result
        if self.error:
            result_dict["error"] = self.error
        return result_dict


@dataclass
class ConfigPayload:
    """Config message payload."""

    key: str
    value: Any

    def to_dict(self) -> Dict[str, Any]:
        return {"key": self.key, "value": self.value}


@dataclass
class StatusPayload:
    """Status message payload."""

    state: ConnectionState
    streaming: bool
    device_status: Optional[Dict[str, Any]] = None
    stats: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "state": self.state.value,
            "streaming": self.streaming,
        }
        if self.device_status:
            result["deviceStatus"] = self.device_status
        if self.stats:
            result["stats"] = self.stats
        return result


@dataclass
class ErrorPayload:
    """Error message payload."""

    code: int
    name: str
    message: str
    recoverable: bool
    details: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "code": self.code,
            "name": self.name,
            "message": self.message,
            "recoverable": self.recoverable,
        }
        if self.details:
            result["details"] = self.details
        return result


@dataclass
class PingPayload:
    """Ping message payload."""

    client_time: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.client_time:
            result["clientTime"] = self.client_time
        return result


@dataclass
class PongPayload:
    """Pong message payload."""

    server_time: int
    client_time: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"serverTime": self.server_time}
        if self.client_time:
            result["clientTime"] = self.client_time
        return result


@dataclass
class HeartbeatConfig:
    """Heartbeat configuration."""

    ping_interval: int = 30000  # ms
    pong_timeout: int = 10000  # ms
    max_missed_pongs: int = 3


@dataclass
class ReconnectConfig:
    """Reconnection configuration."""

    enabled: bool = True
    max_attempts: int = 5
    initial_delay: int = 1000  # ms
    max_delay: int = 30000  # ms
    backoff_multiplier: float = 2.0


@dataclass
class TransportOptions:
    """Transport options."""

    url: Optional[str] = None
    binary_mode: bool = False
    heartbeat: Optional[HeartbeatConfig] = None
    reconnect: Optional[ReconnectConfig] = None
