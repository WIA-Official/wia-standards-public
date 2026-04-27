"""
WIA Screen Reader - Configuration for NVDA
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class WIAConfig:
    """WIA Screen Reader configuration."""
    default_language: str = "en"
    use_wihp: bool = True
    braille_grade: int = 1
    tts_rate: float = 1.0
    tts_pitch: float = 1.0
    tts_volume: float = 1.0

    def save(self) -> None:
        """Save configuration to NVDA config."""
        pass  # NVDA config integration

    def load(self) -> None:
        """Load configuration from NVDA config."""
        pass  # NVDA config integration
