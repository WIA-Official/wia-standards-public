"""
WIA BCI Output Types

Phase 4: Ecosystem Integration
"""

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Any, Callable, Literal


# Output types
OutputType = Literal[
    "tts", "sign_language", "braille", "neurofeedback", "cursor", "custom"
]


@dataclass
class CommandContent:
    """Command content."""

    action: str
    params: Optional[Dict[str, Any]] = None


@dataclass
class ClassificationContent:
    """Classification content."""

    class_id: int
    class_name: str
    confidence: float


@dataclass
class ChannelData:
    """Channel data."""

    channel: str
    value: float
    quality: Optional[float] = None


@dataclass
class SignalContent:
    """Signal content."""

    band_powers: Dict[str, float]
    channels: List[ChannelData]


@dataclass
class OutputMetadata:
    """Output metadata."""

    timestamp: int
    source: Optional[str] = None
    priority: Literal["low", "normal", "high"] = "normal"


@dataclass
class OutputContent:
    """Output content."""

    type: Literal["text", "command", "classification", "signal"]
    text: Optional[str] = None
    command: Optional[CommandContent] = None
    classification: Optional[ClassificationContent] = None
    signal: Optional[SignalContent] = None
    metadata: Optional[OutputMetadata] = None


@dataclass
class OutputOptions:
    """Output options."""

    language: Optional[str] = None
    auto_start: bool = False


@dataclass
class Voice:
    """TTS Voice."""

    id: str
    name: str
    language: str
    gender: Optional[Literal["male", "female", "neutral"]] = None
    local_service: bool = False
    default: bool = False


@dataclass
class TTSOptions:
    """TTS options."""

    voice: Optional[str] = None
    rate: float = 1.0
    pitch: float = 1.0
    volume: float = 1.0
    language: Optional[str] = None


@dataclass
class ISPCode:
    """ISP (Sign Language) code."""

    code: str
    meaning: Optional[str] = None
    duration: Optional[int] = None
    metadata: Optional[Dict[str, str]] = None


@dataclass
class Avatar:
    """Sign language avatar."""

    id: str
    name: str
    style: Literal["realistic", "cartoon", "simple"]
    preview: Optional[str] = None


@dataclass
class BrailleOutput:
    """Braille output."""

    original: str
    ipa: str
    braille: str
    unicode: List[str]
    cells: int
    grade: Literal[1, 2]


@dataclass
class BrailleDisplay:
    """Braille display."""

    id: str
    name: str
    manufacturer: Optional[str] = None
    cells: int = 40
    rows: int = 1
    connected: bool = False
    battery: Optional[int] = None


# Visualization mode
VisualizationMode = Literal[
    "band_powers",
    "topography",
    "time_series",
    "spectrogram",
    "classification",
    "cursor",
    "combined",
]


@dataclass
class CursorPosition:
    """Cursor position."""

    x: float
    y: float
    click: bool = False


@dataclass
class NeurofeedbackTheme:
    """Neurofeedback theme."""

    background: str = "#1a1a2e"
    foreground: str = "#eee"
    positive: str = "#4ade80"
    negative: str = "#f87171"
    neutral: str = "#94a3b8"


# Events
OutputEvent = Literal["start", "end", "error", "ready", "busy"]


@dataclass
class OutputEventData:
    """Output event data."""

    type: OutputEvent
    adapter: OutputType
    timestamp: int
    content: Optional[OutputContent] = None
    error: Optional[Exception] = None


OutputEventHandler = Callable[[OutputEventData], None]


@dataclass
class OutputPreferences:
    """Output preferences."""

    primary_output: OutputType = "tts"
    enabled_outputs: List[OutputType] = field(
        default_factory=lambda: ["tts", "neurofeedback"]
    )
    language: str = "ko"
    tts: Optional[TTSOptions] = None


@dataclass
class ManagerOptions:
    """Manager options."""

    auto_initialize: bool = True
    default_outputs: Optional[List[OutputType]] = None
    preferences: Optional[OutputPreferences] = None


# Error codes
class OutputErrorCode(IntEnum):
    """Output error codes."""

    # Adapter errors (1xxx)
    ADAPTER_NOT_FOUND = 1001
    ADAPTER_NOT_READY = 1002
    ADAPTER_BUSY = 1003
    ADAPTER_INIT_FAILED = 1004

    # TTS errors (2xxx)
    TTS_NOT_SUPPORTED = 2001
    TTS_VOICE_NOT_FOUND = 2002
    TTS_SYNTHESIS_FAILED = 2003

    # Sign language errors (3xxx)
    SIGN_CONVERSION_FAILED = 3001
    SIGN_AVATAR_NOT_FOUND = 3002
    SIGN_ANIMATION_FAILED = 3003

    # Braille errors (4xxx)
    BRAILLE_DISPLAY_NOT_FOUND = 4001
    BRAILLE_CONVERSION_FAILED = 4002
    BRAILLE_SEND_FAILED = 4003

    # Neurofeedback errors (5xxx)
    NEURO_CANVAS_NOT_SET = 5001
    NEURO_RENDER_FAILED = 5002


class OutputError(Exception):
    """Output error."""

    def __init__(
        self,
        code: OutputErrorCode,
        message: str,
        adapter: Optional[OutputType] = None,
        cause: Optional[Exception] = None,
    ):
        super().__init__(message)
        self.code = code
        self.adapter = adapter
        self.cause = cause
