"""
WIA AAC Standard API
Python Implementation

License: MIT
Author: WIA / SmileStory Inc.
"""

from .core import WiaAac, EventEmitter
from .types import *
from .adapters import BaseSensorAdapter, MockAdapter, SignalHandler

# Protocol (Phase 3)
from .protocol import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    MessageBuilder,
    MessageParser,
    ProtocolHandler,
    ProtocolOptions,
    WiaAacMessage,
    ProtocolError,
    ProtocolErrorCode,
)

# Transport (Phase 3)
from .transport import (
    ITransport,
    TransportState,
    MockTransport,
)

# Output (Phase 4)
from .output import (
    OutputType,
    OutputState,
    OutputOptions,
    OutputError,
    OutputErrorCode,
    IOutputAdapter,
    MockTTSAdapter,
    MockSignLanguageAdapter,
    MockBrailleAdapter,
    OutputManager,
)

__version__ = "1.0.0"
__author__ = "WIA / SmileStory Inc."
__license__ = "MIT"

__all__ = [
    # Core
    "WiaAac",
    "EventEmitter",
    # Adapters
    "BaseSensorAdapter",
    "MockAdapter",
    "SignalHandler",
    # Types (re-exported from types module)
    "SensorType",
    "EventType",
    "ErrorCode",
    "ConnectionState",
    "WiaAacSignal",
    "DeviceInfo",
    "SensorConfig",
    "WiaAacOptions",
    "WiaAacError",
    # Protocol (Phase 3)
    "PROTOCOL_NAME",
    "PROTOCOL_VERSION",
    "MessageBuilder",
    "MessageParser",
    "ProtocolHandler",
    "ProtocolOptions",
    "WiaAacMessage",
    "ProtocolError",
    "ProtocolErrorCode",
    # Transport (Phase 3)
    "ITransport",
    "TransportState",
    "MockTransport",
    # Output (Phase 4)
    "OutputType",
    "OutputState",
    "OutputOptions",
    "OutputError",
    "OutputErrorCode",
    "IOutputAdapter",
    "MockTTSAdapter",
    "MockSignLanguageAdapter",
    "MockBrailleAdapter",
    "OutputManager",
    # Version
    "__version__"
]
