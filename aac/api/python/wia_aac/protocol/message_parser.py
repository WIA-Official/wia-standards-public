"""
WIA AAC Protocol Message Parser
"""

import json
from dataclasses import dataclass
from typing import Optional, List, Union, Dict, Any

from .message import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    WiaAacMessage,
    ProtocolError,
    ProtocolErrorCode,
)
from .message_builder import MessageBuilder

VALID_MESSAGE_TYPES = [
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
class ParseResult:
    """Result of parsing a message"""
    success: bool
    message: Optional[WiaAacMessage] = None
    error: Optional[ProtocolError] = None


class MessageParser:
    """Message parser for WIA AAC protocol"""

    def __init__(self, supported_versions: Optional[List[str]] = None):
        self.supported_versions = supported_versions or [PROTOCOL_VERSION]

    def parse(self, data: Union[str, Dict[str, Any]]) -> ParseResult:
        """Parse a JSON string or dict into a WiaAacMessage"""
        try:
            # Parse JSON if string
            if isinstance(data, str):
                obj = json.loads(data)
            else:
                obj = data

            # Validate
            validation_error = self._validate(obj)
            if validation_error:
                return ParseResult(success=False, error=validation_error)

            # Create message
            message = WiaAacMessage.from_dict(obj)
            return ParseResult(success=True, message=message)

        except json.JSONDecodeError as e:
            return ParseResult(
                success=False,
                error=MessageBuilder.create_error(
                    ProtocolErrorCode.INVALID_MESSAGE,
                    "INVALID_MESSAGE",
                    f"Failed to parse JSON: {e}",
                    False,
                ),
            )
        except Exception as e:
            return ParseResult(
                success=False,
                error=MessageBuilder.create_error(
                    ProtocolErrorCode.INVALID_MESSAGE,
                    "INVALID_MESSAGE",
                    f"Failed to parse message: {e}",
                    False,
                ),
            )

    def _validate(self, obj: Any) -> Optional[ProtocolError]:
        """Validate a message object"""
        if not obj or not isinstance(obj, dict):
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                "Message must be an object",
                False,
            )

        # Check protocol
        if obj.get("protocol") != PROTOCOL_NAME:
            return MessageBuilder.create_error(
                ProtocolErrorCode.PROTOCOL_ERROR,
                "PROTOCOL_ERROR",
                f"Invalid protocol: expected '{PROTOCOL_NAME}', got '{obj.get('protocol')}'",
                False,
            )

        # Check version
        version = obj.get("version")
        if not isinstance(version, str):
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                "Missing or invalid version field",
                False,
            )

        if not self._is_version_supported(version):
            return MessageBuilder.create_error(
                ProtocolErrorCode.VERSION_MISMATCH,
                "VERSION_MISMATCH",
                f"Unsupported protocol version: {version}",
                True,
                {"supportedVersions": self.supported_versions},
            )

        # Check messageId
        if not obj.get("messageId") or not isinstance(obj.get("messageId"), str):
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                "Missing or invalid messageId field",
                False,
            )

        # Check timestamp
        timestamp = obj.get("timestamp")
        if not isinstance(timestamp, (int, float)) or timestamp < 0:
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                "Missing or invalid timestamp field",
                False,
            )

        # Check type
        msg_type = obj.get("type")
        if msg_type not in VALID_MESSAGE_TYPES:
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                f"Invalid message type: {msg_type}",
                False,
            )

        # Check payload
        if not isinstance(obj.get("payload"), dict):
            return MessageBuilder.create_error(
                ProtocolErrorCode.INVALID_MESSAGE,
                "INVALID_MESSAGE",
                "Missing or invalid payload field",
                False,
            )

        return None

    def _is_version_supported(self, version: str) -> bool:
        """Check if version is supported"""
        # Check exact match
        if version in self.supported_versions:
            return True

        # Check major version compatibility
        major = version.split(".")[0]
        return any(v.startswith(f"{major}.") for v in self.supported_versions)

    def serialize(self, message: WiaAacMessage) -> str:
        """Serialize a message to JSON string"""
        return json.dumps(message.to_dict())


# Singleton instance
message_parser = MessageParser()
