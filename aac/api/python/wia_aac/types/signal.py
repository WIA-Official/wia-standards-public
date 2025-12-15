"""
WIA AAC Signal Types
Based on Phase 1 Signal Format Standard
"""

from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Literal, Union
from enum import Enum


class SensorType(str, Enum):
    """Sensor type identifiers"""
    EYE_TRACKER = "eye_tracker"
    SWITCH = "switch"
    MUSCLE_SENSOR = "muscle_sensor"
    BRAIN_INTERFACE = "brain_interface"
    BREATH = "breath"
    HEAD_MOVEMENT = "head_movement"
    CUSTOM = "custom"


@dataclass
class Timestamp:
    """Timestamp data"""
    unix_ms: int
    iso8601: Optional[str] = None


@dataclass
class DeviceInfo:
    """Device information"""
    manufacturer: str
    model: str
    firmware: Optional[str] = None
    serial: Optional[str] = None


@dataclass
class SignalMeta:
    """Signal metadata"""
    confidence: Optional[float] = None
    validity: Optional[bool] = None
    raw: Optional[Dict[str, Any]] = None


# Eye Tracker Data
@dataclass
class GazePoint:
    """Gaze point coordinates"""
    x: float
    y: float
    z: Optional[float] = None


@dataclass
class Fixation:
    """Fixation data"""
    active: bool
    duration_ms: Optional[int] = None
    target_id: Optional[str] = None


@dataclass
class PupilData:
    """Pupil diameter data"""
    left_diameter_mm: Optional[float] = None
    right_diameter_mm: Optional[float] = None


@dataclass
class BlinkData:
    """Blink detection data"""
    detected: bool
    duration_ms: Optional[int] = None


@dataclass
class EyeValidity:
    """Eye data validity"""
    left: bool
    right: bool


@dataclass
class EyeTrackerData:
    """Eye tracker sensor data"""
    gaze: GazePoint
    fixation: Optional[Fixation] = None
    pupil: Optional[PupilData] = None
    blink: Optional[BlinkData] = None
    eye_validity: Optional[EyeValidity] = None


# Switch Data
@dataclass
class SwitchData:
    """Switch sensor data"""
    switch_id: int
    state: Literal["pressed", "released", "held"]
    channel: Optional[str] = None
    duration_ms: Optional[int] = None
    pressure: Optional[float] = None
    repeat_count: Optional[int] = None


# Muscle Sensor Data
@dataclass
class MuscleSensorData:
    """Muscle sensor (EMG) data"""
    channel_id: int
    activation: float
    muscle_group: Optional[str] = None
    raw_uv: Optional[float] = None
    envelope_uv: Optional[float] = None
    threshold_exceeded: Optional[bool] = None
    gesture: Optional[str] = None


# Brain Interface Data
@dataclass
class EEGChannel:
    """Single EEG channel data"""
    id: str
    value_uv: float


@dataclass
class FrequencyBands:
    """EEG frequency band powers"""
    delta: Optional[float] = None
    theta: Optional[float] = None
    alpha: Optional[float] = None
    beta: Optional[float] = None
    gamma: Optional[float] = None


@dataclass
class BCIClassification:
    """BCI classification result"""
    intent: str
    confidence: float


@dataclass
class Artifacts:
    """Artifact detection flags"""
    eye_blink: Optional[bool] = None
    muscle: Optional[bool] = None
    movement: Optional[bool] = None


@dataclass
class BrainInterfaceData:
    """Brain interface (EEG/BCI) data"""
    channel_count: int
    channels: List[EEGChannel]
    sample_rate_hz: Optional[float] = None
    bands: Optional[FrequencyBands] = None
    classification: Optional[BCIClassification] = None
    artifacts: Optional[Artifacts] = None


# Breath Data
@dataclass
class BreathData:
    """Breath (sip-and-puff) sensor data"""
    action: Literal["sip", "hard_sip", "puff", "hard_puff", "neutral"]
    pressure_kpa: Optional[float] = None
    pressure_normalized: Optional[float] = None
    duration_ms: Optional[int] = None
    intensity: Optional[Literal["soft", "medium", "hard"]] = None
    baseline_kpa: Optional[float] = None


# Head Movement Data
@dataclass
class Position2D:
    """2D position"""
    x: float
    y: float


@dataclass
class Rotation3D:
    """3D rotation angles"""
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    roll: Optional[float] = None


@dataclass
class Velocity2D:
    """2D velocity"""
    x: Optional[float] = None
    y: Optional[float] = None


@dataclass
class HeadMovementData:
    """Head movement sensor data"""
    position: Position2D
    rotation: Optional[Rotation3D] = None
    velocity: Optional[Velocity2D] = None
    gesture: Optional[Literal["dwell", "nod", "shake", "tilt_left", "tilt_right", "none"]] = None
    dwell_time_ms: Optional[int] = None
    face_detected: Optional[bool] = None


# Custom Data
@dataclass
class CustomData:
    """Custom sensor data"""
    custom_type: str
    custom_data: Dict[str, Any] = field(default_factory=dict)


# Union type for all sensor data
SensorData = Union[
    EyeTrackerData,
    SwitchData,
    MuscleSensorData,
    BrainInterfaceData,
    BreathData,
    HeadMovementData,
    CustomData
]


@dataclass
class WiaAacSignal:
    """WIA AAC Signal message"""
    version: str
    type: SensorType
    timestamp: Timestamp
    device: DeviceInfo
    data: SensorData
    schema: Optional[str] = None
    sequence: Optional[int] = None
    meta: Optional[SignalMeta] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert signal to dictionary"""
        result: Dict[str, Any] = {
            "version": self.version,
            "type": self.type.value if isinstance(self.type, SensorType) else self.type,
            "timestamp": {
                "unix_ms": self.timestamp.unix_ms
            },
            "device": {
                "manufacturer": self.device.manufacturer,
                "model": self.device.model
            },
            "data": {}  # Will be populated based on data type
        }

        if self.timestamp.iso8601:
            result["timestamp"]["iso8601"] = self.timestamp.iso8601

        if self.device.firmware:
            result["device"]["firmware"] = self.device.firmware
        if self.device.serial:
            result["device"]["serial"] = self.device.serial

        if self.schema:
            result["$schema"] = self.schema
        if self.sequence is not None:
            result["sequence"] = self.sequence
        if self.meta:
            result["meta"] = {}
            if self.meta.confidence is not None:
                result["meta"]["confidence"] = self.meta.confidence
            if self.meta.validity is not None:
                result["meta"]["validity"] = self.meta.validity

        return result
