"""
WIA Eye Gaze Standard - Type Definitions

弘益人間 - 널리 인간을 이롭게
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Callable
from enum import Enum


# ============================================
# Basic Types
# ============================================

@dataclass
class Vector2D:
    """2D coordinate"""
    x: float
    y: float


@dataclass
class Vector3D:
    """3D coordinate (mm)"""
    x: float
    y: float
    z: float


@dataclass
class BoundingBox:
    """Screen region (normalized coordinates)"""
    x: float
    y: float
    width: float
    height: float

    def contains(self, px: float, py: float) -> bool:
        """Check if point is inside bounding box"""
        return (self.x <= px <= self.x + self.width and
                self.y <= py <= self.y + self.height)


# ============================================
# Gaze Data Types
# ============================================

@dataclass
class EyeData:
    """Data for a single eye"""
    gaze: Vector2D
    valid: bool
    pupil_diameter: Optional[float] = None
    pupil_center: Optional[Vector2D] = None
    gaze_origin: Optional[Vector3D] = None
    gaze_direction: Optional[Vector3D] = None
    eye_openness: Optional[float] = None
    eye_openness_mm: Optional[float] = None


@dataclass
class GazePoint:
    """Single gaze data point"""
    timestamp: int
    x: float
    y: float
    confidence: float
    valid: bool
    left_eye: Optional[EyeData] = None
    right_eye: Optional[EyeData] = None
    fixation: Optional[bool] = None
    saccade: Optional[bool] = None
    fixation_id: Optional[str] = None
    device_timestamp: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class GazeEventType(Enum):
    """Gaze event types"""
    # Eye Movement Events
    FIXATION_START = "fixation_start"
    FIXATION_UPDATE = "fixation_update"
    FIXATION_END = "fixation_end"
    SACCADE_START = "saccade_start"
    SACCADE_END = "saccade_end"
    SMOOTH_PURSUIT_START = "smooth_pursuit_start"
    SMOOTH_PURSUIT_END = "smooth_pursuit_end"

    # Eye State Events
    BLINK_START = "blink_start"
    BLINK_END = "blink_end"
    BLINK = "blink"
    WINK_LEFT = "wink_left"
    WINK_RIGHT = "wink_right"
    DOUBLE_BLINK = "double_blink"

    # Interaction Events
    DWELL_START = "dwell_start"
    DWELL_PROGRESS = "dwell_progress"
    DWELL_COMPLETE = "dwell_complete"
    DWELL_CANCEL = "dwell_cancel"
    GAZE_ENTER = "gaze_enter"
    GAZE_LEAVE = "gaze_leave"

    # System Events
    CALIBRATION_START = "calibration_start"
    CALIBRATION_POINT = "calibration_point"
    CALIBRATION_END = "calibration_end"
    TRACKING_LOST = "tracking_lost"
    TRACKING_RECOVERED = "tracking_recovered"
    DEVICE_CONNECTED = "device_connected"
    DEVICE_DISCONNECTED = "device_disconnected"


class TargetSemanticType(Enum):
    """Semantic type for gaze targets"""
    BUTTON = "button"
    LINK = "link"
    TEXT = "text"
    INPUT = "input"
    IMAGE = "image"
    VIDEO = "video"
    MENU = "menu"
    MENUITEM = "menuitem"
    LISTITEM = "listitem"
    SCROLLBAR = "scrollbar"
    KEYBOARD_KEY = "keyboard_key"
    AAC_SYMBOL = "aac_symbol"
    CUSTOM = "custom"


@dataclass
class AccessibilityInfo:
    """Accessibility information for a target"""
    role: Optional[str] = None
    name: Optional[str] = None
    description: Optional[str] = None
    state: Optional[List[str]] = None


@dataclass
class GazeTarget:
    """UI element that can be targeted by gaze"""
    element_id: str
    bounding_box: BoundingBox
    semantic_type: TargetSemanticType
    label: Optional[str] = None
    accessibility: Optional[AccessibilityInfo] = None
    attributes: Optional[Dict[str, Any]] = None


@dataclass
class GazeEvent:
    """Gaze event"""
    type: GazeEventType
    timestamp: int
    event_id: str
    duration: Optional[int] = None
    position: Optional[Vector2D] = None
    target: Optional[GazeTarget] = None
    gaze_data: Optional[GazePoint] = None
    previous_event_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


# ============================================
# Device Capability Types
# ============================================

class DeviceType(Enum):
    """Eye tracker device type"""
    SCREEN_BASED = "screen_based"
    WEARABLE = "wearable"
    REMOTE = "remote"
    INTEGRATED = "integrated"
    WEBCAM_BASED = "webcam_based"
    UNKNOWN = "unknown"


@dataclass
class EyeTrackerInfo:
    """Device information"""
    device_id: str
    vendor: str
    model: str
    firmware_version: str
    protocol_version: str
    serial_number: Optional[str] = None
    device_type: Optional[DeviceType] = None
    vendor_url: Optional[str] = None
    product_url: Optional[str] = None


@dataclass
class SamplingRateSpec:
    """Sampling rate specification"""
    supported: List[int]
    default: int
    current: Optional[int] = None


@dataclass
class AccuracySpec:
    """Accuracy specification (degrees)"""
    typical: float
    best: Optional[float] = None


@dataclass
class PrecisionSpec:
    """Precision specification (degrees)"""
    typical: float
    rms: Optional[float] = None


@dataclass
class LatencySpec:
    """Latency specification (ms)"""
    average: float
    maximum: Optional[float] = None


@dataclass
class RangeSpec:
    """Range specification"""
    min: float
    max: float
    optimal: Optional[float] = None


@dataclass
class TrackingAreaSpec:
    """Tracking area (degrees)"""
    horizontal: float
    vertical: float


@dataclass
class TrackingCapabilities:
    """Tracking capabilities"""
    binocular: bool
    head_tracking: bool
    gaze_3d: bool
    sampling_rate: SamplingRateSpec
    accuracy: AccuracySpec
    precision: PrecisionSpec
    latency: LatencySpec
    operating_distance: RangeSpec
    tracking_area: TrackingAreaSpec


@dataclass
class DataCapabilities:
    """Data output capabilities"""
    gaze_point: bool
    eye_data: bool
    pupil_diameter: bool
    pupil_position: bool
    eye_openness: bool
    eye_images: bool
    gaze_origin_3d: bool
    gaze_direction_3d: bool
    built_in_fixation_detection: bool
    built_in_saccade_detection: bool
    built_in_blink_detection: bool
    device_timestamp: bool
    system_timestamp: bool
    external_sync: bool


class CalibrationType(Enum):
    """Calibration type"""
    STANDARD = "standard"
    QUICK = "quick"
    INFANT = "infant"
    GAMING = "gaming"
    ACCESSIBILITY = "accessibility"
    AUTOMATIC = "automatic"


@dataclass
class CalibrationCapabilities:
    """Calibration capabilities"""
    required: bool
    types: List[CalibrationType]
    point_options: List[int]
    default_points: int
    auto_calibration: bool
    profile_management: bool
    quality_assessment: bool
    adaptive_calibration: bool


@dataclass
class AccessibilityCapabilities:
    """Accessibility capabilities"""
    dwell_selection: bool = False
    blink_input: bool = False
    wink_input: bool = False
    switch_emulation: bool = False
    adaptive_dwell_time: bool = False
    adaptive_target_size: bool = False
    error_smoothing: bool = False
    tremor_compensation: bool = False
    aac_optimized_mode: bool = False
    long_session_optimization: bool = False
    fatigue_detection: bool = False
    break_reminder: bool = False


class ConnectionType(Enum):
    """Connection type"""
    USB = "usb"
    USB_C = "usb_c"
    BLUETOOTH = "bluetooth"
    WIFI = "wifi"
    ETHERNET = "ethernet"
    HDMI = "hdmi"


class ApiProtocol(Enum):
    """API protocol"""
    NATIVE_SDK = "native_sdk"
    WIA_STANDARD = "wia_standard"
    TCP_IP = "tcp_ip"
    WEBSOCKET = "websocket"
    REST_API = "rest_api"
    WEBRTC = "webrtc"


@dataclass
class ConnectivityCapabilities:
    """Connectivity capabilities"""
    connection_types: List[ConnectionType]
    api_protocols: List[ApiProtocol]
    multi_client: bool
    remote_connection: bool
    mobile_connection: bool


@dataclass
class EyeTrackerCapabilities:
    """Complete device capabilities"""
    device: EyeTrackerInfo
    tracking: TrackingCapabilities
    data: DataCapabilities
    calibration: CalibrationCapabilities
    connectivity: ConnectivityCapabilities
    supported_features: List[str]
    accessibility: Optional[AccessibilityCapabilities] = None


# ============================================
# Calibration Types
# ============================================

@dataclass
class CalibrationPoint:
    """Calibration point"""
    x: float
    y: float
    index: int


@dataclass
class PointCalibrationResult:
    """Result for a single calibration point"""
    point_index: int
    position: Vector2D
    accuracy: float
    precision: float
    valid: bool


@dataclass
class CalibrationResult:
    """Calibration result"""
    success: bool
    timestamp: int
    average_accuracy: Optional[float] = None
    average_precision: Optional[float] = None
    point_results: Optional[List[PointCalibrationResult]] = None


@dataclass
class CalibrationQuality:
    """Calibration quality assessment"""
    overall: str  # 'excellent', 'good', 'fair', 'poor'
    accuracy: float
    precision: float
    coverage: float


# ============================================
# Tracker Status Types
# ============================================

class TrackerState(Enum):
    """Tracker state"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    CALIBRATING = "calibrating"
    TRACKING = "tracking"
    ERROR = "error"


@dataclass
class TrackerStatus:
    """Tracker status"""
    state: TrackerState
    connected: bool
    tracking: bool
    calibrated: bool
    error: Optional[str] = None
    battery_level: Optional[int] = None
    user_present: Optional[bool] = None


# ============================================
# Subscription Types
# ============================================

@dataclass
class Subscription:
    """Data subscription"""
    id: str
    unsubscribe: Callable[[], None]


# Type aliases
GazeCallback = Callable[[GazePoint], None]
EventCallback = Callable[[GazeEvent], None]


# ============================================
# Dwell Types
# ============================================

class DwellFeedbackType(Enum):
    """Dwell feedback type"""
    CIRCULAR_FILL = "circular_fill"
    LINEAR_BAR = "linear_bar"
    SHRINKING_RING = "shrinking_ring"
    AUDIO_ONLY = "audio_only"
    CUSTOM = "custom"


@dataclass
class DwellConfig:
    """Dwell configuration"""
    threshold: int = 800
    progress_interval: int = 50
    visual_feedback: bool = True
    cooldown_period: int = 500


@dataclass
class DwellState:
    """Dwell state"""
    active: bool
    progress: float
    target: Optional[GazeTarget] = None
    start_time: Optional[int] = None
