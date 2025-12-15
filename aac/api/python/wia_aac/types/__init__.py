"""
WIA AAC Types
"""

from .signal import (
    SensorType,
    Timestamp,
    DeviceInfo,
    SignalMeta,
    GazePoint,
    Fixation,
    PupilData,
    BlinkData,
    EyeValidity,
    EyeTrackerData,
    SwitchData,
    MuscleSensorData,
    EEGChannel,
    FrequencyBands,
    BCIClassification,
    Artifacts,
    BrainInterfaceData,
    BreathData,
    Position2D,
    Rotation3D,
    Velocity2D,
    HeadMovementData,
    CustomData,
    SensorData,
    WiaAacSignal
)

from .events import (
    EventType,
    ErrorCode,
    SelectionEvent,
    GestureEvent,
    WiaAacError,
    DisconnectReason,
    EventHandler
)

from .config import (
    ConnectionState,
    WiaAacOptions,
    ConnectionConfig,
    DeviceFilter,
    BaseSensorOptions,
    EyeTrackerOptions,
    SwitchOptions,
    MuscleSensorOptions,
    BrainInterfaceOptions,
    BreathOptions,
    HeadMovementOptions,
    SensorOptions,
    SensorConfig,
    SensorOptionDescriptor
)

__all__ = [
    # Signal types
    "SensorType",
    "Timestamp",
    "DeviceInfo",
    "SignalMeta",
    "GazePoint",
    "Fixation",
    "PupilData",
    "BlinkData",
    "EyeValidity",
    "EyeTrackerData",
    "SwitchData",
    "MuscleSensorData",
    "EEGChannel",
    "FrequencyBands",
    "BCIClassification",
    "Artifacts",
    "BrainInterfaceData",
    "BreathData",
    "Position2D",
    "Rotation3D",
    "Velocity2D",
    "HeadMovementData",
    "CustomData",
    "SensorData",
    "WiaAacSignal",
    # Event types
    "EventType",
    "ErrorCode",
    "SelectionEvent",
    "GestureEvent",
    "WiaAacError",
    "DisconnectReason",
    "EventHandler",
    # Config types
    "ConnectionState",
    "WiaAacOptions",
    "ConnectionConfig",
    "DeviceFilter",
    "BaseSensorOptions",
    "EyeTrackerOptions",
    "SwitchOptions",
    "MuscleSensorOptions",
    "BrainInterfaceOptions",
    "BreathOptions",
    "HeadMovementOptions",
    "SensorOptions",
    "SensorConfig",
    "SensorOptionDescriptor"
]
