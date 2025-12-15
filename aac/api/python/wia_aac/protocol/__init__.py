"""
WIA AAC Protocol Module
"""

from .message import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    DEFAULT_PORT,
    SUB_PROTOCOL,
    MessageType,
    WiaAacMessage,
    ConnectPayload,
    ConnectAckPayload,
    DisconnectPayload,
    SubscribePayload,
    SubscribeAckPayload,
    CommandPayload,
    CommandAckPayload,
    ProtocolError,
    PingPayload,
    PongPayload,
    ProtocolErrorCode,
)

from .message_builder import MessageBuilder, message_builder
from .message_parser import MessageParser, ParseResult, message_parser
from .protocol_handler import ProtocolHandler, ProtocolState, ProtocolOptions

__all__ = [
    # Constants
    "PROTOCOL_NAME",
    "PROTOCOL_VERSION",
    "DEFAULT_PORT",
    "SUB_PROTOCOL",
    # Types
    "MessageType",
    "WiaAacMessage",
    "ConnectPayload",
    "ConnectAckPayload",
    "DisconnectPayload",
    "SubscribePayload",
    "SubscribeAckPayload",
    "CommandPayload",
    "CommandAckPayload",
    "ProtocolError",
    "PingPayload",
    "PongPayload",
    "ProtocolErrorCode",
    # Classes
    "MessageBuilder",
    "message_builder",
    "MessageParser",
    "ParseResult",
    "message_parser",
    "ProtocolHandler",
    "ProtocolState",
    "ProtocolOptions",
]
