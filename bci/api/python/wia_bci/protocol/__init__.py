"""
WIA BCI Protocol Module

Phase 3: Communication Protocol
"""

from .types import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    DEFAULT_PORT,
    BINARY_MAGIC,
    MessageType,
    ConnectionState,
    ErrorCodes,
    BciMessage,
    ConnectPayload,
    ConnectAckPayload,
    DisconnectPayload,
    StartStreamPayload,
    StopStreamPayload,
    StreamAckPayload,
    SignalPayload,
    SignalBatchPayload,
    MarkerPayload,
    CommandPayload,
    CommandAckPayload,
    ConfigPayload,
    StatusPayload,
    ErrorPayload,
    PingPayload,
    PongPayload,
    HeartbeatConfig,
    ReconnectConfig,
    TransportOptions,
)

from .message_builder import MessageBuilder, message_builder
from .message_parser import MessageParser, message_parser, serialize, serialize_binary

__all__ = [
    # Constants
    "PROTOCOL_NAME",
    "PROTOCOL_VERSION",
    "DEFAULT_PORT",
    "BINARY_MAGIC",
    # Enums
    "MessageType",
    "ConnectionState",
    "ErrorCodes",
    # Message types
    "BciMessage",
    "ConnectPayload",
    "ConnectAckPayload",
    "DisconnectPayload",
    "StartStreamPayload",
    "StopStreamPayload",
    "StreamAckPayload",
    "SignalPayload",
    "SignalBatchPayload",
    "MarkerPayload",
    "CommandPayload",
    "CommandAckPayload",
    "ConfigPayload",
    "StatusPayload",
    "ErrorPayload",
    "PingPayload",
    "PongPayload",
    # Config types
    "HeartbeatConfig",
    "ReconnectConfig",
    "TransportOptions",
    # Builder/Parser
    "MessageBuilder",
    "message_builder",
    "MessageParser",
    "message_parser",
    "serialize",
    "serialize_binary",
]
