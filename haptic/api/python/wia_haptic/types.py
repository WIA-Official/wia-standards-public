"""
WIA Haptic Standard - Type Definitions
"""

from enum import Enum
from typing import Literal


# ============================================================================
# Body Locations
# ============================================================================

class BodyRegion(str, Enum):
    """Anatomical regions of the body"""
    HEAD = "head"
    NECK = "neck"
    SHOULDER = "shoulder"
    ARM = "arm"
    WRIST = "wrist"
    HAND = "hand"
    FINGER = "finger"
    TORSO = "torso"
    WAIST = "waist"
    LEG = "leg"
    FOOT = "foot"


class Laterality(str, Enum):
    """Side of the body"""
    LEFT = "left"
    RIGHT = "right"
    CENTER = "center"
    BILATERAL = "bilateral"


# All body locations as a type
BodyLocation = Literal[
    # Head
    "forehead_left", "forehead_center", "forehead_right",
    "temple_left", "temple_right",
    # Neck
    "neck_left", "neck_center", "neck_right", "neck_back",
    # Shoulders
    "shoulder_left", "shoulder_right",
    # Arms
    "upper_arm_left", "upper_arm_right",
    "elbow_left", "elbow_right",
    "forearm_left", "forearm_right",
    # Wrists
    "wrist_left_dorsal", "wrist_left_volar",
    "wrist_right_dorsal", "wrist_right_volar",
    # Hands
    "palm_left", "palm_right",
    "back_hand_left", "back_hand_right",
    # Fingers
    "thumb_left", "index_left", "middle_left", "ring_left", "pinky_left",
    "thumb_right", "index_right", "middle_right", "ring_right", "pinky_right",
    # Torso Front
    "chest_left", "chest_center", "chest_right",
    "abdomen_left", "abdomen_center", "abdomen_right",
    # Torso Back
    "back_upper_left", "back_upper_center", "back_upper_right",
    "back_lower_left", "back_lower_center", "back_lower_right",
    # Waist
    "waist_left", "waist_front", "waist_right", "waist_back",
    # Legs
    "thigh_left", "thigh_right",
    "knee_left", "knee_right",
    "calf_left", "calf_right",
    "ankle_left", "ankle_right",
    # Feet
    "foot_left", "foot_right",
]


# ============================================================================
# Waveforms and Envelopes
# ============================================================================

class WaveformType(str, Enum):
    """Basic waveform types for haptic signal generation"""
    SINE = "sine"
    SQUARE = "square"
    TRIANGLE = "triangle"
    SAWTOOTH = "sawtooth"
    NOISE = "noise"


class NoiseType(str, Enum):
    """Types of noise waveform"""
    WHITE = "white"
    PINK = "pink"
    BROWN = "brown"


class EnvelopePreset(str, Enum):
    """Predefined envelope presets"""
    SHARP = "sharp"
    PUNCH = "punch"
    SMOOTH = "smooth"
    PULSE = "pulse"
    SWELL = "swell"
    FADE = "fade"


class FrequencyBand(str, Enum):
    """Frequency band classifications"""
    VERY_LOW = "very_low"   # 1-30 Hz
    LOW = "low"             # 30-80 Hz
    MID = "mid"             # 80-150 Hz
    HIGH = "high"           # 150-250 Hz
    VERY_HIGH = "very_high" # 250-300 Hz


class IntensityLevel(str, Enum):
    """Predefined intensity levels"""
    SUBTLE = "subtle"     # 0.2
    LIGHT = "light"       # 0.4
    MEDIUM = "medium"     # 0.6
    STRONG = "strong"     # 0.8
    MAXIMUM = "maximum"   # 1.0


# ============================================================================
# Haptic Categories
# ============================================================================

class HapticCategory(str, Enum):
    """Semantic categories for haptic patterns"""
    NAVIGATION = "navigation"
    NOTIFICATION = "notification"
    CONFIRMATION = "confirmation"
    SPATIAL = "spatial"
    TEMPORAL = "temporal"
    SOCIAL = "social"
    SYSTEM = "system"
    CONTENT = "content"


# ============================================================================
# Hardware Types
# ============================================================================

class ActuatorType(str, Enum):
    """Types of haptic actuators"""
    ERM = "erm"           # Eccentric Rotating Mass
    LRA = "lra"           # Linear Resonant Actuator
    PIEZO = "piezo"       # Piezoelectric
    VOICE_COIL = "voice_coil"


class DeviceType(str, Enum):
    """Types of haptic-enabled devices"""
    SMARTWATCH = "smartwatch"
    WRISTBAND = "wristband"
    BRACELET = "bracelet"
    HAPTIC_GLOVE = "haptic_glove"
    RING = "ring"
    HANDHELD = "handheld"
    ARMBAND = "armband"
    HEADBAND = "headband"
    SMART_GLASSES = "smart_glasses"
    HEADSET = "headset"
    VEST = "vest"
    BELT = "belt"
    SHIRT = "shirt"
    FULL_SUIT = "full_suit"
    INSOLE = "insole"
    ANKLET = "anklet"
    SMARTPHONE = "smartphone"
    TABLET = "tablet"
    CANE = "cane"
    GAMEPAD = "gamepad"


class ConnectionState(str, Enum):
    """Device connection states"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTING = "disconnecting"
    ERROR = "error"


class HapticErrorCode(str, Enum):
    """Error codes for haptic operations"""
    CONNECTION_FAILED = "CONNECTION_FAILED"
    CONNECTION_TIMEOUT = "CONNECTION_TIMEOUT"
    DEVICE_NOT_FOUND = "DEVICE_NOT_FOUND"
    NOT_CONNECTED = "NOT_CONNECTED"
    UNSUPPORTED_FEATURE = "UNSUPPORTED_FEATURE"
    INVALID_PATTERN = "INVALID_PATTERN"
    PLAYBACK_ERROR = "PLAYBACK_ERROR"
    PERMISSION_DENIED = "PERMISSION_DENIED"
    BLUETOOTH_DISABLED = "BLUETOOTH_DISABLED"
    UNKNOWN = "UNKNOWN"
