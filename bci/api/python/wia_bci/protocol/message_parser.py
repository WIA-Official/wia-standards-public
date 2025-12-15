"""
WIA BCI Message Parser

Parse and validate protocol messages.
"""

import json
import struct
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union, List

from .types import (
    BciMessage,
    MessageType,
    ErrorCodes,
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    BINARY_MAGIC,
)


@dataclass
class ParseResult:
    """Parse result."""

    success: bool
    message: Optional[BciMessage] = None
    error: Optional[Dict[str, Any]] = None


VALID_MESSAGE_TYPES = [t.value for t in MessageType]


class MessageParser:
    """Message parser for WIA BCI protocol."""

    def __init__(self, strict_mode: bool = True):
        self.strict_mode = strict_mode

    def parse(self, data: Union[str, bytes]) -> ParseResult:
        """Parse string or bytes to message."""
        try:
            if isinstance(data, bytes):
                return self._parse_binary(data)

            obj = json.loads(data)
            return self.validate(obj)
        except json.JSONDecodeError as e:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_MESSAGE,
                    "message": f"JSON parse error: {e}",
                },
            )
        except Exception as e:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_MESSAGE,
                    "message": f"Parse error: {e}",
                },
            )

    def validate(self, obj: Any) -> ParseResult:
        """Validate message structure."""
        if not obj or not isinstance(obj, dict):
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_MESSAGE,
                    "message": "Message must be an object",
                },
            )

        # Required fields
        required_fields = [
            "protocol",
            "version",
            "messageId",
            "timestamp",
            "type",
            "payload",
        ]
        for field in required_fields:
            if field not in obj:
                return ParseResult(
                    success=False,
                    error={
                        "code": ErrorCodes.INVALID_MESSAGE,
                        "message": f"Missing required field: {field}",
                    },
                )

        # Validate protocol
        if obj["protocol"] != PROTOCOL_NAME:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.PROTOCOL_ERROR,
                    "message": f"Invalid protocol: {obj['protocol']}",
                },
            )

        # Validate version (check major version only)
        if self.strict_mode:
            version = obj["version"]
            major_version = version.split(".")[0]
            expected_major = PROTOCOL_VERSION.split(".")[0]

            if major_version != expected_major:
                return ParseResult(
                    success=False,
                    error={
                        "code": ErrorCodes.VERSION_MISMATCH,
                        "message": f"Version mismatch: {version} (expected {PROTOCOL_VERSION})",
                    },
                )

        # Validate message type
        if obj["type"] not in VALID_MESSAGE_TYPES:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.UNSUPPORTED_TYPE,
                    "message": f"Invalid message type: {obj['type']}",
                },
            )

        # Validate timestamp
        if not isinstance(obj["timestamp"], (int, float)) or obj["timestamp"] < 0:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_PAYLOAD,
                    "message": "Invalid timestamp",
                },
            )

        # Validate payload is object
        if not isinstance(obj["payload"], dict):
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_PAYLOAD,
                    "message": "Payload must be an object",
                },
            )

        return ParseResult(success=True, message=BciMessage.from_dict(obj))

    def _parse_binary(self, data: bytes) -> ParseResult:
        """Parse binary message."""
        if len(data) < 16:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_MESSAGE,
                    "message": "Binary message too short",
                },
            )

        # Read header
        magic, version, msg_type, sequence, payload_length = struct.unpack(
            ">IHHII", data[:16]
        )

        if magic != BINARY_MAGIC:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.PROTOCOL_ERROR,
                    "message": "Invalid binary magic number",
                },
            )

        if len(data) < 16 + payload_length:
            return ParseResult(
                success=False,
                error={
                    "code": ErrorCodes.INVALID_MESSAGE,
                    "message": "Binary payload incomplete",
                },
            )

        # Parse payload
        payload_data = data[16 : 16 + payload_length]
        payload = self._parse_binary_payload(msg_type, payload_data)

        # Build message
        message = BciMessage(
            protocol=PROTOCOL_NAME,
            version=f"{(version >> 8) & 0xFF}.{version & 0xFF}.0",
            message_id=f"binary-{sequence}",
            timestamp=int(__import__("time").time() * 1000),
            type=self._message_type_from_code(msg_type),
            payload=payload,
            sequence=sequence,
        )

        return ParseResult(success=True, message=message)

    def _parse_binary_payload(self, msg_type: int, data: bytes) -> Dict[str, Any]:
        """Parse binary payload."""
        # Signal type (1)
        if msg_type == 1 and len(data) >= 16:
            # Read signal header
            timestamp_low, timestamp_high, sample_index, channel_count, _ = struct.unpack(
                ">IIIIH", data[:18]
            )
            timestamp = timestamp_high * 0x100000000 + timestamp_low

            # Read channel data
            channels = list(range(channel_count))
            signal_data = []
            offset = 16
            for i in range(channel_count):
                if offset + 4 <= len(data):
                    value = struct.unpack(">f", data[offset : offset + 4])[0]
                    signal_data.append(value)
                    offset += 4

            return {
                "sampleIndex": sample_index,
                "timestamp": timestamp // 1000,
                "channels": channels,
                "data": signal_data,
            }

        # Default: decode as JSON
        try:
            return json.loads(data.decode("utf-8"))
        except:
            return {}

    def _message_type_from_code(self, code: int) -> MessageType:
        """Convert message type code to enum."""
        types = {
            0: MessageType.CONNECT,
            1: MessageType.SIGNAL,
            2: MessageType.SIGNAL_BATCH,
            3: MessageType.MARKER,
            4: MessageType.ERROR,
            5: MessageType.PING,
            6: MessageType.PONG,
            10: MessageType.CONNECT_ACK,
            11: MessageType.DISCONNECT,
            12: MessageType.START_STREAM,
            13: MessageType.STOP_STREAM,
            14: MessageType.STREAM_ACK,
            20: MessageType.COMMAND,
            21: MessageType.COMMAND_ACK,
            22: MessageType.CONFIG,
            23: MessageType.STATUS,
        }
        return types.get(code, MessageType.ERROR)


def serialize(message: BciMessage) -> str:
    """Serialize message to JSON string."""
    return json.dumps(message.to_dict())


def serialize_binary(message: BciMessage) -> bytes:
    """Serialize message to binary."""
    type_code = _message_type_to_code(message.type)

    # For signal messages, use binary encoding
    if message.type == MessageType.SIGNAL:
        return _serialize_signal_binary(message)

    # For other messages, encode payload as JSON
    payload_bytes = json.dumps(message.payload).encode("utf-8")

    # Create buffer (16 byte header + payload)
    header = struct.pack(
        ">IHHII",
        BINARY_MAGIC,
        0x0100,  # Version 1.0
        type_code,
        message.sequence or 0,
        len(payload_bytes),
    )

    return header + payload_bytes


def _serialize_signal_binary(message: BciMessage) -> bytes:
    """Serialize signal message to binary."""
    payload = message.payload
    channel_count = len(payload.get("data", []))

    # Calculate payload size
    payload_size = 16 + channel_count * 4

    # Protocol header
    header = struct.pack(
        ">IHHII",
        BINARY_MAGIC,
        0x0100,
        1,  # Signal type
        message.sequence or 0,
        payload_size,
    )

    # Signal header
    timestamp_micro = int(payload.get("timestamp", 0) * 1000)
    signal_header = struct.pack(
        ">IIIIH",
        timestamp_micro & 0xFFFFFFFF,
        timestamp_micro >> 32,
        payload.get("sampleIndex", 0),
        channel_count,
        0,  # Reserved
    )

    # Channel data
    channel_data = b""
    for value in payload.get("data", []):
        channel_data += struct.pack(">f", value)

    return header + signal_header + channel_data


def _message_type_to_code(msg_type: MessageType) -> int:
    """Convert message type to code."""
    codes = {
        MessageType.CONNECT: 0,
        MessageType.SIGNAL: 1,
        MessageType.SIGNAL_BATCH: 2,
        MessageType.MARKER: 3,
        MessageType.ERROR: 4,
        MessageType.PING: 5,
        MessageType.PONG: 6,
        MessageType.CONNECT_ACK: 10,
        MessageType.DISCONNECT: 11,
        MessageType.START_STREAM: 12,
        MessageType.STOP_STREAM: 13,
        MessageType.STREAM_ACK: 14,
        MessageType.COMMAND: 20,
        MessageType.COMMAND_ACK: 21,
        MessageType.CONFIG: 22,
        MessageType.STATUS: 23,
    }
    return codes.get(msg_type, 4)


# Default parser instance
message_parser = MessageParser()
