"""
WIA AAC Output Module Types
Phase 4: WIA Ecosystem Integration
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Callable


class OutputType(str, Enum):
    """Output adapter types"""
    TTS = "tts"
    SIGN_LANGUAGE = "sign_language"
    BRAILLE = "braille"
    CUSTOM = "custom"


class OutputState(str, Enum):
    """Output adapter state"""
    IDLE = "idle"
    OUTPUTTING = "outputting"
    PAUSED = "paused"
    ERROR = "error"


class OutputEventType(str, Enum):
    """Output event types"""
    START = "start"
    END = "end"
    PAUSE = "pause"
    RESUME = "resume"
    PROGRESS = "progress"
    ERROR = "error"


@dataclass
class OutputOptions:
    """Output options"""
    language: Optional[str] = None
    voice: Optional[str] = None
    speed: float = 1.0
    volume: float = 1.0
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class OutputEvent:
    """Output event"""
    type: OutputEventType
    adapter: OutputType
    timestamp: float
    data: Optional[Any] = None


# Type alias for event handler
OutputEventHandler = Callable[[OutputEvent], None]


@dataclass
class Voice:
    """Voice information (for TTS)"""
    id: str
    name: str
    language: str
    gender: Optional[str] = None  # 'male', 'female', 'neutral'
    local: bool = True


@dataclass
class ISPCode:
    """ISP code (for sign language)"""
    code: str  # e.g., "HS01-LC07-MV10-OR02-NM15"
    meaning: Optional[str] = None
    duration: Optional[int] = None  # milliseconds
    components: Optional[Dict[str, str]] = None


@dataclass
class BrailleOutput:
    """Braille output"""
    text: str
    ipa: str
    braille: str
    unicode: List[str]
    dots: List[int]


@dataclass
class BrailleDisplay:
    """Braille display information"""
    id: str
    name: str
    cells: int
    connected: bool


class OutputErrorCode(int, Enum):
    """Output error codes"""
    # Initialization errors (1xxx)
    INIT_FAILED = 1001
    NOT_AVAILABLE = 1002
    ALREADY_INITIALIZED = 1003

    # Output errors (2xxx)
    OUTPUT_FAILED = 2001
    OUTPUT_CANCELLED = 2002
    OUTPUT_TIMEOUT = 2003

    # Adapter errors (3xxx)
    ADAPTER_NOT_FOUND = 3001
    ADAPTER_NOT_READY = 3002
    ADAPTER_BUSY = 3003

    # Conversion errors (4xxx)
    CONVERSION_FAILED = 4001
    INVALID_INPUT = 4002
    MAPPING_NOT_FOUND = 4003

    # Device errors (5xxx)
    DEVICE_NOT_CONNECTED = 5001
    DEVICE_ERROR = 5002


class OutputError(Exception):
    """Output error"""
    def __init__(
        self,
        code: OutputErrorCode,
        message: str,
        recoverable: bool = True,
        details: Optional[Any] = None
    ):
        super().__init__(message)
        self.code = code
        self.message = message
        self.recoverable = recoverable
        self.details = details
