"""
WIA Screen Reader Standard - Python SDK
Universal Accessibility for 211 Languages

弘益人間 (홍익인간) - Benefit All Humanity

Copyright (c) 2025 WIA Standards - SmileStory Inc.
MIT License
"""

from .core import WIAScreenReader
from .wihp_engine import WIHPEngine
from .braille_engine import BrailleEngine
from .types import (
    ScreenReaderResult,
    Pronunciation,
    BrailleOutput,
    TTSConfig,
    Context,
)

__version__ = "1.0.0"
__author__ = "WIA Standards"
__email__ = "official@wia.codes"

__all__ = [
    "WIAScreenReader",
    "WIHPEngine",
    "BrailleEngine",
    "ScreenReaderResult",
    "Pronunciation",
    "BrailleOutput",
    "TTSConfig",
    "Context",
]
