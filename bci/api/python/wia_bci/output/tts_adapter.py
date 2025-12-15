"""
WIA BCI TTS Adapter

Text-to-Speech adapter using pyttsx3 or gtts.
"""

import asyncio
from typing import Optional, List

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputContent,
    OutputOptions,
    Voice,
    TTSOptions,
    OutputError,
    OutputErrorCode,
)

# Try to import TTS libraries
TTS_AVAILABLE = False
TTS_ENGINE = None

try:
    import pyttsx3

    TTS_AVAILABLE = True
    TTS_ENGINE = "pyttsx3"
except ImportError:
    pass


class TTSAdapter(BaseOutputAdapter):
    """TTS Adapter using pyttsx3."""

    def __init__(self):
        super().__init__()
        self._engine = None
        self._voices: List[Voice] = []
        self._current_voice: Optional[Voice] = None
        self._rate = 150
        self._volume = 1.0
        self._language = "ko"

    @property
    def type(self) -> OutputType:
        return "tts"

    @property
    def name(self) -> str:
        return f"TTS Adapter ({TTS_ENGINE or 'mock'})"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize TTS engine."""
        if not TTS_AVAILABLE:
            self._available = False
            # Use mock mode
            self._ready = True
            self._emit("ready")
            return

        try:
            self._engine = pyttsx3.init()
            await self._load_voices()

            if options and options.language:
                self._language = options.language

            self._select_default_voice()
            self._ready = True
            self._emit("ready")
        except Exception as e:
            raise OutputError(
                OutputErrorCode.TTS_NOT_SUPPORTED,
                f"Failed to initialize TTS: {e}",
                "tts",
            )

    async def _load_voices(self) -> None:
        """Load available voices."""
        if not self._engine:
            return

        voices = self._engine.getProperty("voices")
        self._voices = [
            Voice(
                id=v.id,
                name=v.name,
                language=getattr(v, "languages", ["en"])[0]
                if hasattr(v, "languages")
                else "en",
                gender=getattr(v, "gender", None),
            )
            for v in voices
        ]

    def _select_default_voice(self) -> None:
        """Select default voice for language."""
        if not self._voices:
            return

        # Try to find voice matching language
        for voice in self._voices:
            if self._language in voice.language.lower():
                self._current_voice = voice
                if self._engine:
                    self._engine.setProperty("voice", voice.id)
                return

        # Use first voice
        self._current_voice = self._voices[0]
        if self._engine:
            self._engine.setProperty("voice", self._current_voice.id)

    async def get_voices(self) -> List[Voice]:
        """Get available voices."""
        return self._voices

    def set_voice(self, voice_id: str) -> None:
        """Set current voice."""
        for voice in self._voices:
            if voice.id == voice_id:
                self._current_voice = voice
                if self._engine:
                    self._engine.setProperty("voice", voice_id)
                return

        raise OutputError(
            OutputErrorCode.TTS_VOICE_NOT_FOUND,
            f"Voice not found: {voice_id}",
            "tts",
        )

    def get_voice(self) -> Optional[Voice]:
        """Get current voice."""
        return self._current_voice

    async def speak(self, text: str, options: Optional[TTSOptions] = None) -> None:
        """Speak text."""
        self._emit("start")

        if not self._engine:
            # Mock mode - just delay
            await asyncio.sleep(len(text) * 0.05)
            print(f"[TTS Mock] {text}")
            self._emit("end")
            return

        try:
            # Apply options
            if options:
                if options.rate:
                    self._engine.setProperty("rate", int(options.rate * 150))
                if options.volume:
                    self._engine.setProperty("volume", options.volume)
                if options.voice:
                    self.set_voice(options.voice)

            # Run in executor to avoid blocking
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._engine.say, text)
            await loop.run_in_executor(None, self._engine.runAndWait)

            self._emit("end")
        except Exception as e:
            error = OutputError(
                OutputErrorCode.TTS_SYNTHESIS_FAILED,
                f"Speech synthesis failed: {e}",
                "tts",
            )
            self._emit("error", error=error)
            raise error

    def stop(self) -> None:
        """Stop speaking."""
        if self._engine:
            self._engine.stop()

    def set_rate(self, rate: float) -> None:
        """Set speech rate."""
        self._rate = int(max(50, min(300, rate * 150)))
        if self._engine:
            self._engine.setProperty("rate", self._rate)

    def set_volume(self, volume: float) -> None:
        """Set volume."""
        self._volume = max(0.0, min(1.0, volume))
        if self._engine:
            self._engine.setProperty("volume", self._volume)

    async def output(self, content: OutputContent) -> None:
        """Output content."""
        if content.type == "text" and content.text:
            await self.speak(content.text)
        elif content.type == "classification" and content.classification:
            text = (
                f"{content.classification.class_name}, "
                f"확신도 {int(content.classification.confidence * 100)}퍼센트"
            )
            await self.speak(text)

    async def dispose(self) -> None:
        """Dispose adapter."""
        self.stop()
        self._engine = None
        await super().dispose()
