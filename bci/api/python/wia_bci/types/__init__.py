"""WIA BCI Type Definitions."""

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

__all__ = [
    # Config
    "DeviceType",
    "ConnectionProtocol",
    "LogLevel",
    "FilterType",
    "FilterConfig",
    "WiaBciOptions",
    "DeviceConfig",
    # Device
    "DeviceStatus",
    "DeviceCapabilities",
    "DeviceInfo",
    "ChannelInfo",
    "BciState",
    # Events
    "EventType",
    "SignalEvent",
    "MarkerEvent",
    "ClassificationEvent",
    "QualityEvent",
    "ErrorEvent",
    # Signal
    "FrequencyBand",
    "BandPowers",
    "FREQUENCY_BANDS",
]
