"""
WIA Haptic Standard - Pattern Definitions
"""

from dataclasses import dataclass, field
from typing import Optional, Union, List, Dict, Any

from .types import (
    WaveformType,
    NoiseType,
    EnvelopePreset,
    FrequencyBand,
    IntensityLevel,
    HapticCategory,
    BodyLocation,
)


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class Envelope:
    """ADSR envelope for shaping haptic signal amplitude"""
    attack: float = 10      # 0-500ms
    decay: float = 50       # 0-500ms
    sustain: float = 0.7    # 0.0-1.0
    release: float = 100    # 0-500ms


@dataclass
class WaveformConfig:
    """Waveform configuration"""
    type: WaveformType = WaveformType.SINE
    duty_cycle: float = 0.5      # for square waves
    smoothing: float = 0.0       # transition smoothing
    noise_type: Optional[NoiseType] = None  # for noise waveform


@dataclass
class IntensityModulation:
    """Intensity modulation configuration"""
    type: str = "none"  # none, pulse, wave, random
    rate: float = 0.0   # Hz
    depth: float = 0.0  # 0.0-1.0


@dataclass
class PatternMetadata:
    """Metadata for haptic patterns"""
    category: Optional[HapticCategory] = None
    description: Optional[str] = None
    learnability: Optional[str] = None  # easy, moderate, complex
    tags: List[str] = field(default_factory=list)


@dataclass
class HapticPrimitive:
    """A single haptic primitive - the basic building block"""
    id: str
    name: str
    waveform: Union[WaveformConfig, WaveformType]
    envelope: Union[Envelope, EnvelopePreset]
    frequency: float           # 1-300 Hz
    intensity: float           # 0.0-1.0
    duration: float            # milliseconds
    metadata: Optional[PatternMetadata] = None

    def get_envelope(self) -> Envelope:
        """Resolve envelope to Envelope object"""
        if isinstance(self.envelope, Envelope):
            return self.envelope
        return ENVELOPE_PRESETS[self.envelope]

    def get_waveform(self) -> WaveformConfig:
        """Resolve waveform to WaveformConfig object"""
        if isinstance(self.waveform, WaveformConfig):
            return self.waveform
        return WaveformConfig(type=self.waveform)


@dataclass
class SequenceStep:
    """A step in a haptic sequence"""
    primitive_id: Optional[str] = None
    primitive: Optional[HapticPrimitive] = None
    delay_before: float = 0      # ms
    repeat_count: int = 1
    repeat_delay: float = 0      # ms
    intensity_scale: float = 1.0


@dataclass
class HapticSequence:
    """A sequence of haptic primitives"""
    id: str
    name: str
    steps: List[SequenceStep]
    loop: bool = False
    loop_count: int = 1
    metadata: Optional[PatternMetadata] = None

    def calculate_duration(self) -> float:
        """Calculate total duration of sequence"""
        duration = 0.0
        for step in self.steps:
            duration += step.delay_before
            if step.primitive:
                duration += step.primitive.duration * step.repeat_count
                duration += step.repeat_delay * (step.repeat_count - 1)
        return duration * (self.loop_count if self.loop else 1)


@dataclass
class SpatialActuation:
    """An actuation at a specific body location"""
    location: Optional[BodyLocation] = None
    locations: Optional[List[BodyLocation]] = None
    pattern: Union[str, HapticPrimitive, HapticSequence, None] = None
    start_time: float = 0       # ms
    intensity_scale: float = 1.0


@dataclass
class SpatialPattern:
    """A pattern with spatial/location information"""
    id: str
    name: str
    actuations: List[SpatialActuation]
    metadata: Optional[PatternMetadata] = None


# Type alias for any pattern type
HapticPattern = Union[HapticPrimitive, HapticSequence, SpatialPattern]


# ============================================================================
# Presets
# ============================================================================

ENVELOPE_PRESETS: Dict[EnvelopePreset, Envelope] = {
    EnvelopePreset.SHARP:  Envelope(attack=5,   decay=20,  sustain=0.0, release=30),
    EnvelopePreset.PUNCH:  Envelope(attack=10,  decay=50,  sustain=0.3, release=50),
    EnvelopePreset.SMOOTH: Envelope(attack=100, decay=100, sustain=0.7, release=150),
    EnvelopePreset.PULSE:  Envelope(attack=5,   decay=0,   sustain=1.0, release=5),
    EnvelopePreset.SWELL:  Envelope(attack=200, decay=50,  sustain=0.8, release=200),
    EnvelopePreset.FADE:   Envelope(attack=50,  decay=200, sustain=0.5, release=300),
}

FREQUENCY_BANDS: Dict[FrequencyBand, tuple] = {
    FrequencyBand.VERY_LOW:  (1, 30),
    FrequencyBand.LOW:       (30, 80),
    FrequencyBand.MID:       (80, 150),
    FrequencyBand.HIGH:      (150, 250),
    FrequencyBand.VERY_HIGH: (250, 300),
}

INTENSITY_LEVELS: Dict[IntensityLevel, float] = {
    IntensityLevel.SUBTLE:  0.2,
    IntensityLevel.LIGHT:   0.4,
    IntensityLevel.MEDIUM:  0.6,
    IntensityLevel.STRONG:  0.8,
    IntensityLevel.MAXIMUM: 1.0,
}


# ============================================================================
# Standard Primitives Library
# ============================================================================

STANDARD_PRIMITIVES: Dict[str, HapticPrimitive] = {
    "tick": HapticPrimitive(
        id="tick",
        name="Tick",
        waveform=WaveformConfig(type=WaveformType.SQUARE, duty_cycle=0.5),
        envelope=EnvelopePreset.SHARP,
        frequency=150,
        intensity=0.6,
        duration=20,
        metadata=PatternMetadata(
            category=HapticCategory.CONFIRMATION,
            description="Quick acknowledgment tap",
            learnability="easy",
        ),
    ),
    "buzz": HapticPrimitive(
        id="buzz",
        name="Buzz",
        waveform=WaveformConfig(type=WaveformType.SQUARE, duty_cycle=0.3),
        envelope=EnvelopePreset.PUNCH,
        frequency=180,
        intensity=0.7,
        duration=100,
        metadata=PatternMetadata(
            category=HapticCategory.NOTIFICATION,
            description="Attention-getting buzz",
            learnability="easy",
        ),
    ),
    "warning_pulse": HapticPrimitive(
        id="warning_pulse",
        name="Warning Pulse",
        waveform=WaveformType.SINE,
        envelope=Envelope(attack=50, decay=100, sustain=0.5, release=100),
        frequency=200,
        intensity=0.8,
        duration=300,
        metadata=PatternMetadata(
            category=HapticCategory.NOTIFICATION,
            description="Alert warning pattern",
            learnability="easy",
        ),
    ),
    "soft_wave": HapticPrimitive(
        id="soft_wave",
        name="Soft Wave",
        waveform=WaveformType.SINE,
        envelope=EnvelopePreset.SMOOTH,
        frequency=60,
        intensity=0.4,
        duration=400,
        metadata=PatternMetadata(
            category=HapticCategory.NAVIGATION,
            description="Gentle directional cue",
            learnability="easy",
        ),
    ),
    "obstacle_texture": HapticPrimitive(
        id="obstacle_texture",
        name="Obstacle Texture",
        waveform=WaveformConfig(type=WaveformType.NOISE, noise_type=NoiseType.PINK),
        envelope=Envelope(attack=20, decay=50, sustain=0.7, release=50),
        frequency=100,
        intensity=0.7,
        duration=200,
        metadata=PatternMetadata(
            category=HapticCategory.NAVIGATION,
            description="Indicates nearby obstacle",
            learnability="moderate",
        ),
    ),
    "heartbeat": HapticPrimitive(
        id="heartbeat",
        name="Heartbeat",
        waveform=WaveformType.SINE,
        envelope=EnvelopePreset.PUNCH,
        frequency=80,
        intensity=0.6,
        duration=150,
        metadata=PatternMetadata(
            category=HapticCategory.SOCIAL,
            description="Presence indicator",
            learnability="easy",
        ),
    ),
    "click": HapticPrimitive(
        id="click",
        name="Click",
        waveform=WaveformType.SQUARE,
        envelope=Envelope(attack=2, decay=5, sustain=0, release=3),
        frequency=200,
        intensity=0.5,
        duration=10,
        metadata=PatternMetadata(
            category=HapticCategory.CONTENT,
            description="UI selection click",
            learnability="easy",
        ),
    ),
}


# ============================================================================
# Standard Sequences Library
# ============================================================================

STANDARD_SEQUENCES: Dict[str, HapticSequence] = {
    "success": HapticSequence(
        id="success",
        name="Success",
        steps=[
            SequenceStep(primitive=STANDARD_PRIMITIVES["tick"], delay_before=0),
            SequenceStep(primitive=STANDARD_PRIMITIVES["tick"], delay_before=100),
        ],
        metadata=PatternMetadata(
            category=HapticCategory.CONFIRMATION,
            description="Action succeeded",
        ),
    ),
    "failure": HapticSequence(
        id="failure",
        name="Failure",
        steps=[
            SequenceStep(primitive=STANDARD_PRIMITIVES["buzz"], delay_before=0),
            SequenceStep(primitive=STANDARD_PRIMITIVES["buzz"], delay_before=150, intensity_scale=0.7),
        ],
        metadata=PatternMetadata(
            category=HapticCategory.CONFIRMATION,
            description="Action failed",
        ),
    ),
    "urgent_alert": HapticSequence(
        id="urgent_alert",
        name="Urgent Alert",
        steps=[
            SequenceStep(
                primitive=STANDARD_PRIMITIVES["warning_pulse"],
                delay_before=0,
                repeat_count=3,
                repeat_delay=150,
            ),
        ],
        metadata=PatternMetadata(
            category=HapticCategory.NOTIFICATION,
            description="Urgent attention required",
        ),
    ),
}


# ============================================================================
# Helper Functions
# ============================================================================

def get_frequency_from_band(band: FrequencyBand) -> float:
    """Get center frequency from a frequency band"""
    min_freq, max_freq = FREQUENCY_BANDS[band]
    return (min_freq + max_freq) / 2


def get_intensity_from_level(level: IntensityLevel) -> float:
    """Get intensity value from preset level"""
    return INTENSITY_LEVELS[level]


def resolve_envelope(envelope: Union[Envelope, EnvelopePreset]) -> Envelope:
    """Resolve envelope preset to full Envelope object"""
    if isinstance(envelope, Envelope):
        return envelope
    return ENVELOPE_PRESETS[envelope]


def find_primitive(primitive_id: str) -> Optional[HapticPrimitive]:
    """Find a standard primitive by ID"""
    return STANDARD_PRIMITIVES.get(primitive_id)


def find_sequence(sequence_id: str) -> Optional[HapticSequence]:
    """Find a standard sequence by ID"""
    return STANDARD_SEQUENCES.get(sequence_id)


# ============================================================================
# Factory Functions
# ============================================================================

def success_sequence() -> HapticSequence:
    """Create a success confirmation sequence"""
    return STANDARD_SEQUENCES["success"]


def failure_sequence() -> HapticSequence:
    """Create a failure indication sequence"""
    return STANDARD_SEQUENCES["failure"]


def alert_sequence(urgency: str = "normal") -> HapticSequence:
    """Create an alert sequence with specified urgency"""
    if urgency == "urgent":
        return STANDARD_SEQUENCES["urgent_alert"]
    return HapticSequence(
        id="alert",
        name="Alert",
        steps=[
            SequenceStep(primitive=STANDARD_PRIMITIVES["buzz"], delay_before=0),
        ],
    )


def navigation_pulse(direction: str, intensity: float = 0.5) -> HapticPrimitive:
    """Create a navigation pulse for the specified direction"""
    return HapticPrimitive(
        id=f"nav_{direction}",
        name=f"Navigation {direction.title()}",
        waveform=WaveformType.SINE,
        envelope=EnvelopePreset.SMOOTH,
        frequency=80,
        intensity=intensity,
        duration=300,
        metadata=PatternMetadata(
            category=HapticCategory.NAVIGATION,
            description=f"Navigation cue for {direction}",
        ),
    )
