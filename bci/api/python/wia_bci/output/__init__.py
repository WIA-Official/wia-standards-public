"""
WIA BCI Output Module

Phase 4: Ecosystem Integration
"""

from .types import (
    OutputType,
    OutputContent,
    CommandContent,
    ClassificationContent,
    SignalContent,
    ChannelData,
    OutputMetadata,
    OutputOptions,
    Voice,
    TTSOptions,
    ISPCode,
    Avatar,
    BrailleOutput,
    BrailleDisplay,
    VisualizationMode,
    CursorPosition,
    NeurofeedbackTheme,
    OutputEvent,
    OutputEventData,
    OutputPreferences,
    ManagerOptions,
    OutputErrorCode,
    OutputError,
)

from .base_output import IOutputAdapter, BaseOutputAdapter
from .tts_adapter import TTSAdapter
from .mock_output import (
    MockSignLanguageAdapter,
    MockBrailleAdapter,
    MockOutputAdapter,
)
from .neurofeedback_adapter import NeurofeedbackAdapter
from .output_manager import OutputManager

__all__ = [
    # Types
    "OutputType",
    "OutputContent",
    "CommandContent",
    "ClassificationContent",
    "SignalContent",
    "ChannelData",
    "OutputMetadata",
    "OutputOptions",
    "Voice",
    "TTSOptions",
    "ISPCode",
    "Avatar",
    "BrailleOutput",
    "BrailleDisplay",
    "VisualizationMode",
    "CursorPosition",
    "NeurofeedbackTheme",
    "OutputEvent",
    "OutputEventData",
    "OutputPreferences",
    "ManagerOptions",
    "OutputErrorCode",
    "OutputError",
    # Adapters
    "IOutputAdapter",
    "BaseOutputAdapter",
    "TTSAdapter",
    "MockSignLanguageAdapter",
    "MockBrailleAdapter",
    "MockOutputAdapter",
    "NeurofeedbackAdapter",
    # Manager
    "OutputManager",
]
