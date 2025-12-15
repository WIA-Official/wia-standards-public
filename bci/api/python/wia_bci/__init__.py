"""
WIA BCI - Brain-Computer Interface Standard API

A Python SDK for WIA Brain-Computer Interface Standard.

Example:
    >>> from wia_bci import WiaBci
    >>> bci = WiaBci()
    >>> await bci.connect(type='eeg_headset')
    >>> bci.on('signal', lambda e: print(e.data))
    >>> await bci.start_stream()
"""

from wia_bci.core.wia_bci import WiaBci
from wia_bci.core.event_emitter import BciEventEmitter
from wia_bci.core.bci_error import BciError, ErrorCodes
from wia_bci.core.signal_processor import SignalProcessor

from wia_bci.adapters.base_adapter import BaseBciAdapter
from wia_bci.adapters.simulator_adapter import SimulatorAdapter

from wia_bci.types.config import (
    DeviceType,
    ConnectionProtocol,
    LogLevel,
    FilterType,
    FilterConfig,
    WiaBciOptions,
    DeviceConfig,
)

from wia_bci.types.device import (
    DeviceStatus,
    DeviceCapabilities,
    DeviceInfo,
    ChannelInfo,
    BciState,
)

from wia_bci.types.events import (
    EventType,
    SignalEvent,
    MarkerEvent,
    ClassificationEvent,
    QualityEvent,
    ErrorEvent,
)

from wia_bci.types.signal import (
    FrequencyBand,
    BandPowers,
    FREQUENCY_BANDS,
)

# Protocol (Phase 3)
from wia_bci.protocol import (
    PROTOCOL_NAME,
    PROTOCOL_VERSION,
    MessageType,
    ConnectionState,
    BciMessage,
    MessageBuilder,
    MessageParser,
    message_builder,
    message_parser,
    serialize,
    serialize_binary,
)

# Transport (Phase 3)
from wia_bci.transport import (
    ITransport,
    BaseTransport,
    TransportState,
    WebSocketTransport,
    MockTransport,
)

# Output (Phase 4)
from wia_bci.output import (
    OutputType,
    OutputContent,
    OutputManager,
    IOutputAdapter,
    BaseOutputAdapter,
    TTSAdapter,
    MockSignLanguageAdapter,
    MockBrailleAdapter,
    NeurofeedbackAdapter,
    OutputError,
    OutputErrorCode,
)

__version__ = "1.0.0"

__all__ = [
    # Core
    "WiaBci",
    "BciEventEmitter",
    "BciError",
    "ErrorCodes",
    "SignalProcessor",
    # Adapters
    "BaseBciAdapter",
    "SimulatorAdapter",
    # Types - Config
    "DeviceType",
    "ConnectionProtocol",
    "LogLevel",
    "FilterType",
    "FilterConfig",
    "WiaBciOptions",
    "DeviceConfig",
    # Types - Device
    "DeviceStatus",
    "DeviceCapabilities",
    "DeviceInfo",
    "ChannelInfo",
    "BciState",
    # Types - Events
    "EventType",
    "SignalEvent",
    "MarkerEvent",
    "ClassificationEvent",
    "QualityEvent",
    "ErrorEvent",
    # Types - Signal
    "FrequencyBand",
    "BandPowers",
    "FREQUENCY_BANDS",
    # Protocol (Phase 3)
    "PROTOCOL_NAME",
    "PROTOCOL_VERSION",
    "MessageType",
    "ConnectionState",
    "BciMessage",
    "MessageBuilder",
    "MessageParser",
    "message_builder",
    "message_parser",
    "serialize",
    "serialize_binary",
    # Transport (Phase 3)
    "ITransport",
    "BaseTransport",
    "TransportState",
    "WebSocketTransport",
    "MockTransport",
    # Output (Phase 4)
    "OutputType",
    "OutputContent",
    "OutputManager",
    "IOutputAdapter",
    "BaseOutputAdapter",
    "TTSAdapter",
    "MockSignLanguageAdapter",
    "MockBrailleAdapter",
    "NeurofeedbackAdapter",
    "OutputError",
    "OutputErrorCode",
    # Version
    "__version__",
]
