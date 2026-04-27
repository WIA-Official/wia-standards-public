"""
WIA Screen Reader - Core SDK
"""
import time
from datetime import datetime
from typing import Optional, Dict, Any

from .types import (
    ScreenReaderResult,
    Pronunciation,
    BrailleOutput,
    TTSConfig,
    Context,
    Metadata,
    BrailleGrade,
)
from .wihp_engine import WIHPEngine
from .braille_engine import BrailleEngine


class WIAScreenReader:
    """
    WIA Screen Reader SDK

    Universal accessibility for 211 languages with WIHP pronunciation
    and WIA Braille support.

    Example:
        >>> reader = WIAScreenReader()
        >>> result = reader.process("Hello World")
        >>> print(result.wihp)  # "헬로우 월드"
        >>> print(result.braille.grade1)  # "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"

    弘益人間 (홍익인간) - Benefit All Humanity
    """

    VERSION = "1.0.0"

    def __init__(
        self,
        language: str = "en",
        braille_grade: BrailleGrade = BrailleGrade.GRADE_1,
        tts_config: Optional[TTSConfig] = None
    ):
        """
        Initialize WIA Screen Reader.

        Args:
            language: Default language code (ISO 639-1)
            braille_grade: Default braille grade
            tts_config: TTS configuration
        """
        self.language = language
        self.braille_grade = braille_grade
        self.tts_config = tts_config or TTSConfig()

        self._wihp_engine = WIHPEngine(language)
        self._braille_engine = BrailleEngine(braille_grade)
        self._tts_engine = None

        # Try to initialize TTS
        try:
            import pyttsx3
            self._tts_engine = pyttsx3.init()
        except Exception:
            pass

    def process(
        self,
        text: str,
        language: Optional[str] = None
    ) -> ScreenReaderResult:
        """
        Process text and return screen reader data.

        Args:
            text: Text to process
            language: Source language (overrides default)

        Returns:
            ScreenReaderResult with pronunciation, braille, and metadata
        """
        start_time = time.time()
        lang = language or self.language

        # Convert to WIHP
        wihp = self._wihp_engine.convert(text, lang)
        ipa = self._wihp_engine.get_ipa(text)

        # Convert to braille
        braille = self._braille_engine.convert(text)

        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000

        return ScreenReaderResult(
            text=text,
            language=lang,
            pronunciation=Pronunciation(
                ipa=ipa,
                wihp=wihp,
                romanized=text.lower(),
            ),
            braille=braille,
            tts=self.tts_config,
            metadata=Metadata(
                processed_at=datetime.now(),
                processing_time_ms=processing_time,
                engine_version=self.VERSION,
                confidence=0.95,
            ),
        )

    def speak(
        self,
        text: str,
        use_wihp: bool = False,
        interrupt: bool = True
    ) -> None:
        """
        Speak text using TTS.

        Args:
            text: Text to speak
            use_wihp: Use WIHP pronunciation
            interrupt: Interrupt current speech
        """
        if self._tts_engine is None:
            print(f"[TTS] {text}")
            return

        if interrupt:
            self._tts_engine.stop()

        text_to_speak = text
        if use_wihp:
            text_to_speak = self._wihp_engine.convert(text)

        self._tts_engine.setProperty('rate', int(150 * self.tts_config.rate))
        self._tts_engine.setProperty('volume', self.tts_config.volume)

        self._tts_engine.say(text_to_speak)
        self._tts_engine.runAndWait()

    def stop_speaking(self) -> None:
        """Stop current speech."""
        if self._tts_engine:
            self._tts_engine.stop()

    def set_tts(
        self,
        rate: Optional[float] = None,
        pitch: Optional[float] = None,
        volume: Optional[float] = None,
        voice: Optional[str] = None
    ) -> None:
        """
        Configure TTS settings.

        Args:
            rate: Speech rate (0.5-2.0, default 1.0)
            pitch: Voice pitch (0.5-2.0, default 1.0)
            volume: Volume (0.0-1.0, default 1.0)
            voice: Voice identifier
        """
        if rate is not None:
            self.tts_config.rate = max(0.5, min(2.0, rate))
        if pitch is not None:
            self.tts_config.pitch = max(0.5, min(2.0, pitch))
        if volume is not None:
            self.tts_config.volume = max(0.0, min(1.0, volume))
        if voice is not None:
            self.tts_config.voice = voice

    def set_language(self, language: str) -> None:
        """
        Set the default language.

        Args:
            language: ISO 639-1 language code
        """
        self.language = language
        self._wihp_engine.set_language(language)

    def set_braille_grade(self, grade: BrailleGrade) -> None:
        """
        Set the default braille grade.

        Args:
            grade: Braille grade (1 or 2)
        """
        self.braille_grade = grade
        self._braille_engine.set_grade(grade)

    def get_wihp(self, text: str, language: Optional[str] = None) -> str:
        """
        Get WIHP pronunciation only.

        Args:
            text: Text to convert
            language: Source language

        Returns:
            WIHP string in Korean Hangul
        """
        return self._wihp_engine.convert(text, language or self.language)

    def get_braille(
        self,
        text: str,
        grade: Optional[BrailleGrade] = None
    ) -> BrailleOutput:
        """
        Get braille output only.

        Args:
            text: Text to convert
            grade: Braille grade

        Returns:
            BrailleOutput with all formats
        """
        return self._braille_engine.convert(text, grade)

    def to_json(self, result: ScreenReaderResult) -> Dict[str, Any]:
        """
        Convert result to JSON format.

        Args:
            result: ScreenReaderResult to convert

        Returns:
            Dictionary matching WIA Screen Reader JSON schema
        """
        return result.to_dict()

    @property
    def version(self) -> str:
        """Get SDK version."""
        return self.VERSION
