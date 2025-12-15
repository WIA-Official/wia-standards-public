"""
WIA AAC Configuration Types
"""

from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Literal
from enum import Enum

from .signal import SensorType


class ConnectionState(str, Enum):
    """Connection state"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    ERROR = "error"


@dataclass
class WiaAacOptions:
    """WiaAac initialization options"""
    auto_reconnect: bool = True
    reconnect_interval: int = 3000  # ms
    max_reconnect_attempts: int = 5
    signal_buffer_size: int = 100
    validate_signals: bool = True
    log_level: Literal["debug", "info", "warn", "error", "none"] = "info"


@dataclass
class ConnectionConfig:
    """Connection configuration"""
    protocol: Optional[Literal["usb", "bluetooth", "wifi", "serial"]] = None
    port: Optional[str] = None
    baud_rate: Optional[int] = None


@dataclass
class DeviceFilter:
    """Device filter for connection"""
    manufacturer: Optional[str] = None
    model: Optional[str] = None
    serial: Optional[str] = None


@dataclass
class BaseSensorOptions:
    """Base sensor options"""
    sample_rate: Optional[int] = None
    sensitivity: Optional[float] = None
    dwell_time: Optional[int] = None
    smoothing: Optional[bool] = None
    smoothing_factor: Optional[float] = None


@dataclass
class EyeTrackerOptions(BaseSensorOptions):
    """Eye tracker options"""
    track_both_eyes: Optional[bool] = None
    track_pupil: Optional[bool] = None
    track_blink: Optional[bool] = None
    fixation_threshold: Optional[int] = None
    gaze_filter: Optional[Literal["none", "average", "kalman"]] = None


@dataclass
class SwitchOptions(BaseSensorOptions):
    """Switch options"""
    debounce_time: Optional[int] = None
    hold_threshold: Optional[int] = None
    multi_press_window: Optional[int] = None


@dataclass
class MuscleSensorOptions(BaseSensorOptions):
    """Muscle sensor options"""
    activation_threshold: Optional[float] = None
    gesture_recognition: Optional[bool] = None
    channels: Optional[List[int]] = None


@dataclass
class BrainInterfaceOptions(BaseSensorOptions):
    """Brain interface options"""
    channels: Optional[List[str]] = None
    band_pass_filter: Optional[Dict[str, float]] = None
    artifact_rejection: Optional[bool] = None
    classification_model: Optional[str] = None


@dataclass
class BreathOptions(BaseSensorOptions):
    """Breath sensor options"""
    sip_threshold: Optional[float] = None
    puff_threshold: Optional[float] = None
    hard_threshold: Optional[float] = None


@dataclass
class HeadMovementOptions(BaseSensorOptions):
    """Head movement options"""
    track_rotation: Optional[bool] = None
    gesture_recognition: Optional[bool] = None
    dwell_radius: Optional[float] = None


# Union type for all sensor options
SensorOptions = (
    BaseSensorOptions |
    EyeTrackerOptions |
    SwitchOptions |
    MuscleSensorOptions |
    BrainInterfaceOptions |
    BreathOptions |
    HeadMovementOptions
)


@dataclass
class SensorConfig:
    """Sensor configuration"""
    type: SensorType
    device: Optional[DeviceFilter] = None
    connection: Optional[ConnectionConfig] = None
    options: Optional[SensorOptions] = None


@dataclass
class SensorOptionDescriptor:
    """Describes a sensor option"""
    name: str
    type: Literal["number", "boolean", "string", "array"]
    description: str
    default: Optional[Any] = None
    min: Optional[float] = None
    max: Optional[float] = None
    options: Optional[List[str]] = None
