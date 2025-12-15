"""
WIA AAC Event Types
"""

from dataclasses import dataclass
from typing import Optional, Dict, Any, Literal, Callable
from enum import IntEnum

from .signal import SensorType, WiaAacSignal, DeviceInfo


class EventType:
    """Event type constants"""
    SIGNAL = "signal"
    SELECTION = "selection"
    GESTURE = "gesture"
    ERROR = "error"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


class ErrorCode(IntEnum):
    """Error codes"""
    # Connection errors (1xx)
    CONNECTION_FAILED = 100
    CONNECTION_LOST = 101
    CONNECTION_TIMEOUT = 102
    DEVICE_NOT_FOUND = 103
    DEVICE_BUSY = 104
    PERMISSION_DENIED = 105

    # Configuration errors (2xx)
    INVALID_CONFIG = 200
    UNSUPPORTED_OPTION = 201
    INVALID_SENSOR_TYPE = 202

    # Runtime errors (3xx)
    SIGNAL_VALIDATION_FAILED = 300
    ADAPTER_ERROR = 301
    INTERNAL_ERROR = 302

    # Protocol errors (4xx)
    PROTOCOL_ERROR = 400
    MESSAGE_PARSE_ERROR = 401
    UNSUPPORTED_VERSION = 402


@dataclass
class SelectionEvent:
    """Selection event data"""
    timestamp: int
    target_id: str
    target_type: Literal["key", "button", "area", "custom"]
    selection_method: Literal["dwell", "click", "gesture", "switch"]
    confidence: float
    position: Optional[Dict[str, float]] = None


@dataclass
class GestureEvent:
    """Gesture event data"""
    timestamp: int
    gesture: str
    sensor_type: SensorType
    confidence: float
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class WiaAacError(Exception):
    """WIA AAC Error"""
    code: ErrorCode
    message: str
    timestamp: int
    recoverable: bool
    details: Optional[Dict[str, Any]] = None

    def __str__(self) -> str:
        return f"WiaAacError[{self.code}]: {self.message}"


@dataclass
class DisconnectReason:
    """Disconnect reason data"""
    reason: Literal["user", "error", "timeout", "device_removed"]
    message: Optional[str] = None


# Event handler type
EventHandler = Callable[[Any], None]
