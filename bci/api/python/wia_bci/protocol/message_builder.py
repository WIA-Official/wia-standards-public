"""
WIA BCI Message Builder

Factory for creating protocol messages.
"""

import uuid
import time
from typing import Optional, Dict, Any, List

from .types import (
    BciMessage,
    MessageType,
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
    ConnectionState,
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
)


def generate_uuid() -> str:
    """Generate UUID v4."""
    return str(uuid.uuid4())


def current_timestamp() -> int:
    """Get current timestamp in milliseconds."""
    return int(time.time() * 1000)


class MessageBuilder:
    """Message builder for WIA BCI protocol."""

    def __init__(self):
        self._sequence = 0
        self._session_id: Optional[str] = None

    def set_session_id(self, session_id: str) -> None:
        """Set session ID for messages."""
        self._session_id = session_id

    def reset_sequence(self) -> None:
        """Reset sequence counter."""
        self._sequence = 0

    def _create_base(self, msg_type: MessageType, payload: Dict[str, Any]) -> BciMessage:
        """Create base message."""
        return BciMessage(
            protocol=PROTOCOL_NAME,
            version=PROTOCOL_VERSION,
            message_id=generate_uuid(),
            timestamp=current_timestamp(),
            type=msg_type,
            payload=payload,
            session_id=self._session_id,
        )

    def _create_with_sequence(
        self, msg_type: MessageType, payload: Dict[str, Any]
    ) -> BciMessage:
        """Create message with sequence number."""
        message = self._create_base(msg_type, payload)
        message.sequence = self._sequence
        self._sequence += 1
        return message

    # Connection messages

    def connect(self, payload: ConnectPayload) -> BciMessage:
        """Create connect message."""
        return self._create_base(MessageType.CONNECT, payload.to_dict())

    def connect_ack(self, payload: ConnectAckPayload) -> BciMessage:
        """Create connect ack message."""
        if payload.session_id:
            self._session_id = payload.session_id
        return self._create_base(MessageType.CONNECT_ACK, payload.to_dict())

    def disconnect(
        self, payload: Optional[DisconnectPayload] = None
    ) -> BciMessage:
        """Create disconnect message."""
        p = payload or DisconnectPayload()
        return self._create_base(MessageType.DISCONNECT, p.to_dict())

    # Stream control messages

    def start_stream(
        self, payload: Optional[StartStreamPayload] = None
    ) -> BciMessage:
        """Create start stream message."""
        p = payload or StartStreamPayload()
        return self._create_base(MessageType.START_STREAM, p.to_dict())

    def stop_stream(
        self, payload: Optional[StopStreamPayload] = None
    ) -> BciMessage:
        """Create stop stream message."""
        p = payload or StopStreamPayload()
        return self._create_base(MessageType.STOP_STREAM, p.to_dict())

    def stream_ack(self, payload: StreamAckPayload) -> BciMessage:
        """Create stream ack message."""
        return self._create_base(MessageType.STREAM_ACK, payload.to_dict())

    # Signal messages

    def signal(self, payload: SignalPayload) -> BciMessage:
        """Create signal message."""
        return self._create_with_sequence(MessageType.SIGNAL, payload.to_dict())

    def signal_batch(self, payload: SignalBatchPayload) -> BciMessage:
        """Create signal batch message."""
        return self._create_with_sequence(MessageType.SIGNAL_BATCH, payload.to_dict())

    # Marker message

    def marker(self, payload: MarkerPayload) -> BciMessage:
        """Create marker message."""
        return self._create_with_sequence(MessageType.MARKER, payload.to_dict())

    # Command messages

    def command(self, payload: CommandPayload) -> BciMessage:
        """Create command message."""
        return self._create_base(MessageType.COMMAND, payload.to_dict())

    def command_ack(self, payload: CommandAckPayload) -> BciMessage:
        """Create command ack message."""
        return self._create_base(MessageType.COMMAND_ACK, payload.to_dict())

    # Config message

    def config(self, payload: ConfigPayload) -> BciMessage:
        """Create config message."""
        return self._create_base(MessageType.CONFIG, payload.to_dict())

    # Status message

    def status(self, payload: StatusPayload) -> BciMessage:
        """Create status message."""
        return self._create_base(MessageType.STATUS, payload.to_dict())

    # Error message

    def error(self, payload: ErrorPayload) -> BciMessage:
        """Create error message."""
        return self._create_base(MessageType.ERROR, payload.to_dict())

    # Heartbeat messages

    def ping(self, payload: Optional[PingPayload] = None) -> BciMessage:
        """Create ping message."""
        p = payload or PingPayload(client_time=current_timestamp())
        return self._create_base(MessageType.PING, p.to_dict())

    def pong(self, client_time: Optional[int] = None) -> BciMessage:
        """Create pong message."""
        payload = PongPayload(
            server_time=current_timestamp(), client_time=client_time
        )
        return self._create_base(MessageType.PONG, payload.to_dict())


# Default message builder instance
message_builder = MessageBuilder()
