"""
WIA AAC Output Module
Phase 4: WIA Ecosystem Integration

Provides output adapters for TTS, Sign Language, and Braille
"""

from .types import (
    OutputType,
    OutputState,
    OutputEventType,
    OutputOptions,
    OutputEvent,
    Voice,
    ISPCode,
    BrailleOutput,
    BrailleDisplay,
    OutputErrorCode,
    OutputError
)

from .base_output import IOutputAdapter, BaseOutputAdapter

from .tts_adapter import (
    ITTSAdapter,
    MockTTSAdapter
)

from .sign_language_adapter import (
    ISignLanguageAdapter,
    MockSignLanguageAdapter
)

from .braille_adapter import (
    IBrailleAdapter,
    MockBrailleAdapter
)

from .output_manager import (
    IOutputManager,
    OutputManager
)

__all__ = [
    # Types
    "OutputType",
    "OutputState",
    "OutputEventType",
    "OutputOptions",
    "OutputEvent",
    "Voice",
    "ISPCode",
    "BrailleOutput",
    "BrailleDisplay",
    "OutputErrorCode",
    "OutputError",
    # Base
    "IOutputAdapter",
    "BaseOutputAdapter",
    # TTS
    "ITTSAdapter",
    "MockTTSAdapter",
    # Sign Language
    "ISignLanguageAdapter",
    "MockSignLanguageAdapter",
    # Braille
    "IBrailleAdapter",
    "MockBrailleAdapter",
    # Manager
    "IOutputManager",
    "OutputManager"
]
