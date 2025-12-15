"""
WIA BCI Mock Output Adapters

Mock implementations for testing and development.
"""

import asyncio
from typing import Optional, List, Dict, Any

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputContent,
    OutputOptions,
    ISPCode,
    Avatar,
    BrailleOutput,
    BrailleDisplay,
)


class MockSignLanguageAdapter(BaseOutputAdapter):
    """Mock Sign Language Adapter."""

    def __init__(self):
        super().__init__()
        self._avatars = [
            Avatar(id="default", name="Default Avatar", style="simple"),
            Avatar(id="realistic", name="Realistic Avatar", style="realistic"),
            Avatar(id="cartoon", name="Cartoon Avatar", style="cartoon"),
        ]
        self._current_avatar = self._avatars[0]
        self._speed = 1.0

    @property
    def type(self) -> OutputType:
        return "sign_language"

    @property
    def name(self) -> str:
        return "Mock Sign Language"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize adapter."""
        self._ready = True
        self._emit("ready")

    async def output(self, content: OutputContent) -> None:
        """Output content."""
        if content.type == "text" and content.text:
            codes = await self.text_to_isp(content.text)
            await self.play_sequence(codes)

    async def text_to_isp(self, text: str) -> List[ISPCode]:
        """Convert text to ISP codes."""
        words = text.split()
        return [
            ISPCode(
                code=f"HS{i % 10:02d}-LC{(i + 1) % 10:02d}-MV{(i + 2) % 10:02d}",
                meaning=word,
                duration=max(500, len(word) * 100),
            )
            for i, word in enumerate(words)
        ]

    async def play_gesture(self, code: ISPCode) -> None:
        """Play single gesture."""
        self._emit("start")
        duration = (code.duration or 500) / 1000 / self._speed
        await asyncio.sleep(duration)
        self._emit("end")

    async def play_sequence(self, codes: List[ISPCode]) -> None:
        """Play gesture sequence."""
        for code in codes:
            await self.play_gesture(code)

    def get_avatars(self) -> List[Avatar]:
        """Get available avatars."""
        return self._avatars

    def set_avatar(self, avatar_id: str) -> None:
        """Set current avatar."""
        for avatar in self._avatars:
            if avatar.id == avatar_id:
                self._current_avatar = avatar
                return

    def get_avatar(self) -> Avatar:
        """Get current avatar."""
        return self._current_avatar

    def set_speed(self, speed: float) -> None:
        """Set playback speed."""
        self._speed = max(0.5, min(2.0, speed))


class MockBrailleAdapter(BaseOutputAdapter):
    """Mock Braille Adapter."""

    def __init__(self):
        super().__init__()
        self._displays = [
            BrailleDisplay(
                id="mock-40",
                name="Mock Braille 40",
                manufacturer="WIA",
                cells=40,
                rows=1,
                connected=True,
                battery=100,
            )
        ]
        self._current_display = self._displays[0]
        self._grade: int = 2

    @property
    def type(self) -> OutputType:
        return "braille"

    @property
    def name(self) -> str:
        return "Mock Braille"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize adapter."""
        self._ready = True
        self._emit("ready")

    async def output(self, content: OutputContent) -> None:
        """Output content."""
        if content.type == "text" and content.text:
            braille = await self.text_to_braille(content.text)
            await self.send_to_display(braille)

    async def text_to_ipa(
        self, text: str, language: Optional[str] = None
    ) -> str:
        """Convert text to IPA."""
        ipa_map = {
            "a": "ɑ",
            "e": "e",
            "i": "i",
            "o": "o",
            "u": "u",
            "안": "an",
            "녕": "njʌŋ",
            "하": "ha",
            "세": "se",
            "요": "jo",
        }
        ipa = ""
        for char in text:
            ipa += ipa_map.get(char.lower(), char)
        return f"/{ipa}/"

    async def text_to_braille(
        self, text: str, language: Optional[str] = None
    ) -> BrailleOutput:
        """Convert text to braille."""
        ipa = await self.text_to_ipa(text, language)

        braille_map = {
            "a": "⠁",
            "b": "⠃",
            "c": "⠉",
            "d": "⠙",
            "e": "⠑",
            "f": "⠋",
            "g": "⠛",
            "h": "⠓",
            "i": "⠊",
            "j": "⠚",
            "k": "⠅",
            "l": "⠇",
            "m": "⠍",
            "n": "⠝",
            "o": "⠕",
            "p": "⠏",
            "q": "⠟",
            "r": "⠗",
            "s": "⠎",
            "t": "⠞",
            "u": "⠥",
            "v": "⠧",
            "w": "⠺",
            "x": "⠭",
            "y": "⠽",
            "z": "⠵",
            " ": "⠀",
            "안": "⠁⠝",
            "녕": "⠝⠚",
            "하": "⠓⠁",
            "세": "⠎⠑",
            "요": "⠚⠕",
        }

        braille = ""
        unicode_list = []

        for char in text:
            bc = braille_map.get(char.lower(), "⠿")
            braille += bc
            for c in bc:
                unicode_list.append(f"U+{ord(c):04X}")

        return BrailleOutput(
            original=text,
            ipa=ipa,
            braille=braille,
            unicode=unicode_list,
            cells=len(braille),
            grade=self._grade,
        )

    async def send_to_display(self, braille: BrailleOutput) -> None:
        """Send to braille display."""
        self._emit("start")
        await asyncio.sleep(0.1)
        print(f"[Braille Display] {braille.braille}")
        self._emit("end")

    async def get_displays(self) -> List[BrailleDisplay]:
        """Get available displays."""
        return self._displays

    def set_display(self, display_id: str) -> None:
        """Set current display."""
        for display in self._displays:
            if display.id == display_id:
                self._current_display = display
                return

    def get_display(self) -> BrailleDisplay:
        """Get current display."""
        return self._current_display

    def set_grade(self, grade: int) -> None:
        """Set braille grade."""
        self._grade = grade


class MockOutputAdapter(BaseOutputAdapter):
    """Generic mock output adapter."""

    def __init__(
        self, adapter_type: OutputType = "custom", adapter_name: str = "Mock Output"
    ):
        super().__init__()
        self._type = adapter_type
        self._name = adapter_name
        self._output_log: List[OutputContent] = []

    @property
    def type(self) -> OutputType:
        return self._type

    @property
    def name(self) -> str:
        return self._name

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize adapter."""
        self._ready = True
        self._emit("ready")

    async def output(self, content: OutputContent) -> None:
        """Output content."""
        self._emit("start", content=content)
        self._output_log.append(content)
        await asyncio.sleep(0.05)
        self._emit("end", content=content)

    def get_output_log(self) -> List[OutputContent]:
        """Get output log."""
        return list(self._output_log)

    def clear_log(self) -> None:
        """Clear output log."""
        self._output_log.clear()
