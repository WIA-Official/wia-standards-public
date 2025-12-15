"""
WIA AAC TTS Adapter
Phase 4: WIA Ecosystem Integration

Text-to-Speech adapter
"""

import asyncio
from abc import abstractmethod
from typing import Optional, List

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputState,
    OutputOptions,
    OutputEventType,
    Voice,
    OutputError,
    OutputErrorCode
)


class ITTSAdapter(BaseOutputAdapter):
    """TTS adapter interface"""

    @abstractmethod
    async def get_voices(self) -> List[Voice]:
        """Get available voices"""
        pass

    @abstractmethod
    def set_voice(self, voice_id: str) -> None:
        """Set voice"""
        pass

    @abstractmethod
    def pause(self) -> None:
        """Pause output"""
        pass

    @abstractmethod
    def resume(self) -> None:
        """Resume output"""
        pass


class MockTTSAdapter(ITTSAdapter):
    """Mock TTS Adapter for testing"""

    def __init__(self):
        super().__init__()
        self._mock_voices = [
            Voice(id="mock-ko", name="Mock Korean", language="ko-KR", gender="neutral", local=True),
            Voice(id="mock-en", name="Mock English", language="en-US", gender="neutral", local=True),
            Voice(id="mock-ja", name="Mock Japanese", language="ja-JP", gender="neutral", local=True)
        ]
        self._selected_voice_id: Optional[str] = None
        self._default_options = OutputOptions()
        self._is_paused = False

    @property
    def type(self) -> OutputType:
        return OutputType.TTS

    @property
    def name(self) -> str:
        return "MockTTS"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize the adapter"""
        if options:
            self._default_options = options
        self._state = OutputState.IDLE
        self._emit(OutputEventType.START, {"message": "Mock TTS adapter initialized"})

    async def get_voices(self) -> List[Voice]:
        """Get available voices"""
        return self._mock_voices

    def set_voice(self, voice_id: str) -> None:
        """Set voice"""
        self._selected_voice_id = voice_id

    async def output(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """Output text as TTS"""
        if self._state == OutputState.OUTPUTTING:
            self.stop()

        merged_options = OutputOptions(
            language=options.language if options else self._default_options.language,
            voice=options.voice if options else self._default_options.voice,
            speed=options.speed if options else self._default_options.speed,
            volume=options.volume if options else self._default_options.volume
        )

        self._state = OutputState.OUTPUTTING
        self._is_paused = False
        self._emit(OutputEventType.START, {"text": text})

        # Simulate TTS duration based on text length
        duration = max(len(text) * 0.05, 0.5)
        speed = merged_options.speed or 1.0

        try:
            await asyncio.sleep(duration / speed)

            if self._state == OutputState.OUTPUTTING:
                self._state = OutputState.IDLE
                self._emit(OutputEventType.END, {"text": text})
        except asyncio.CancelledError:
            self._state = OutputState.IDLE
            raise

    def stop(self) -> None:
        """Stop output"""
        self._state = OutputState.IDLE
        self._is_paused = False

    def pause(self) -> None:
        """Pause output"""
        if self._state == OutputState.OUTPUTTING:
            self._state = OutputState.PAUSED
            self._is_paused = True
            self._emit(OutputEventType.PAUSE)

    def resume(self) -> None:
        """Resume output"""
        if self._state == OutputState.PAUSED:
            self._state = OutputState.OUTPUTTING
            self._is_paused = False
            self._emit(OutputEventType.RESUME)

    def is_available(self) -> bool:
        """Check if adapter is available"""
        return True

    async def dispose(self) -> None:
        """Dispose resources"""
        self.stop()
        self._handlers.clear()
