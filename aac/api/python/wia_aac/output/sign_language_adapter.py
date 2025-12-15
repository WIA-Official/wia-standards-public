"""
WIA AAC Sign Language Adapter
Phase 4: WIA Ecosystem Integration

ISP/WIA Talk integration for sign language output
"""

import asyncio
import re
from abc import abstractmethod
from typing import Optional, List, Dict
from dataclasses import dataclass

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputState,
    OutputOptions,
    OutputEventType,
    ISPCode,
    OutputError,
    OutputErrorCode
)


@dataclass
class ISPDictionaryEntry:
    """ISP dictionary entry"""
    word: str
    code: str
    duration: int


class ISignLanguageAdapter(BaseOutputAdapter):
    """Sign language adapter interface"""

    @abstractmethod
    async def text_to_isp(self, text: str) -> List[ISPCode]:
        """Convert text to ISP codes"""
        pass

    @abstractmethod
    async def play_gesture(self, isp_code: ISPCode) -> None:
        """Play a single gesture"""
        pass

    @abstractmethod
    async def play_sequence(self, isp_codes: List[ISPCode]) -> None:
        """Play a sequence of gestures"""
        pass

    @abstractmethod
    def set_avatar(self, avatar_id: str) -> None:
        """Set avatar"""
        pass

    @abstractmethod
    def set_speed(self, speed: float) -> None:
        """Set playback speed"""
        pass


class MockSignLanguageAdapter(ISignLanguageAdapter):
    """Mock Sign Language Adapter using ISP code system"""

    def __init__(self):
        super().__init__()
        self._isp_dictionary: Dict[str, ISPDictionaryEntry] = {}
        self._speed = 1.0
        self._avatar_id = "default"
        self._is_playing = False

    @property
    def type(self) -> OutputType:
        return OutputType.SIGN_LANGUAGE

    @property
    def name(self) -> str:
        return "MockSignLanguage"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize the adapter"""
        self._load_default_dictionary()
        self._state = OutputState.IDLE
        self._emit(OutputEventType.START, {"message": "Sign language adapter initialized"})

    def _load_default_dictionary(self) -> None:
        """Load default ISP dictionary based on WIA Talk"""
        default_entries = [
            # Greetings
            ISPDictionaryEntry("안녕", "HS01-LC01-MV01-OR01-NM01", 1000),
            ISPDictionaryEntry("안녕하세요", "HS01-LC01-MV01-OR01-NM01", 1200),
            ISPDictionaryEntry("hello", "HS01-LC01-MV01-OR01-NM01", 1000),
            ISPDictionaryEntry("hi", "HS01-LC01-MV01-OR01-NM01", 800),

            # Thanks
            ISPDictionaryEntry("감사", "HS02-LC07-MV02-OR02-NM02", 1000),
            ISPDictionaryEntry("감사합니다", "HS02-LC07-MV02-OR02-NM02", 1200),
            ISPDictionaryEntry("thanks", "HS02-LC07-MV02-OR02-NM02", 1000),

            # Yes/No
            ISPDictionaryEntry("네", "HS03-LC01-MV03-OR01-NM03", 600),
            ISPDictionaryEntry("예", "HS03-LC01-MV03-OR01-NM03", 600),
            ISPDictionaryEntry("yes", "HS03-LC01-MV03-OR01-NM03", 600),
            ISPDictionaryEntry("아니오", "HS04-LC01-MV04-OR01-NM04", 800),
            ISPDictionaryEntry("no", "HS04-LC01-MV04-OR01-NM04", 600),

            # Love
            ISPDictionaryEntry("사랑", "HS09-LC07-MV10-OR02-NM01", 1200),
            ISPDictionaryEntry("사랑해", "HS09-LC07-MV10-OR02-NM01", 1200),
            ISPDictionaryEntry("love", "HS09-LC07-MV10-OR02-NM01", 1000),

            # Help
            ISPDictionaryEntry("도움", "HS05-LC08-MV07-OR03-NM05", 1000),
            ISPDictionaryEntry("도와주세요", "HS05-LC08-MV07-OR03-NM05", 1200),
            ISPDictionaryEntry("help", "HS05-LC08-MV07-OR03-NM05", 800),

            # Basic pronouns
            ISPDictionaryEntry("나", "HS06-LC07-MV10-OR01-NM01", 600),
            ISPDictionaryEntry("너", "HS07-LC02-MV10-OR02-NM01", 600),

            # Actions
            ISPDictionaryEntry("먹다", "HS08-LC03-MV05-OR01-NM01", 800),
            ISPDictionaryEntry("마시다", "HS08-LC03-MV06-OR01-NM01", 800),
            ISPDictionaryEntry("가다", "HS10-LC12-MV08-OR03-NM01", 800),
            ISPDictionaryEntry("오다", "HS10-LC12-MV09-OR04-NM01", 800),

            # Emotions
            ISPDictionaryEntry("좋아", "HS15-LC07-MV10-OR02-NM07", 800),
            ISPDictionaryEntry("싫어", "HS16-LC07-MV10-OR02-NM08", 800),

            # Common phrases
            ISPDictionaryEntry("미안", "HS17-LC07-MV13-OR01-NM11", 1000),
            ISPDictionaryEntry("미안합니다", "HS17-LC07-MV13-OR01-NM11", 1200),
            ISPDictionaryEntry("sorry", "HS17-LC07-MV13-OR01-NM11", 800),

            # Understanding
            ISPDictionaryEntry("알겠어", "HS18-LC07-MV14-OR01-NM12", 900),
            ISPDictionaryEntry("이해했어", "HS18-LC07-MV15-OR06-NM23", 1000),
        ]

        for entry in default_entries:
            self._isp_dictionary[entry.word.lower()] = entry

    def _parse_isp_code(self, code: str) -> Optional[Dict[str, str]]:
        """Parse ISP code into components"""
        pattern = r"^(HS\d+)-(LC\d+)-(MV\d+)-(OR\d+)-(NM\d+)$"
        match = re.match(pattern, code)
        if not match:
            return None

        return {
            "handshape": match.group(1),
            "location": match.group(2),
            "movement": match.group(3),
            "orientation": match.group(4),
            "nonManual": match.group(5)
        }

    async def text_to_isp(self, text: str) -> List[ISPCode]:
        """Convert text to ISP codes"""
        words = text.lower().split()
        codes = []

        for word in words:
            word = word.strip()
            if not word:
                continue

            entry = self._isp_dictionary.get(word)
            if entry:
                codes.append(ISPCode(
                    code=entry.code,
                    meaning=entry.word,
                    duration=entry.duration,
                    components=self._parse_isp_code(entry.code)
                ))
            else:
                # Unknown word - use default gesture
                codes.append(ISPCode(
                    code="HS00-LC00-MV00-OR00-NM00",
                    meaning=word,
                    duration=len(word) * 200 + 400,
                    components=self._parse_isp_code("HS00-LC00-MV00-OR00-NM00")
                ))

        return codes

    async def play_gesture(self, isp_code: ISPCode) -> None:
        """Play a single gesture"""
        if self._state == OutputState.OUTPUTTING:
            raise OutputError(
                OutputErrorCode.ADAPTER_BUSY,
                "Sign language adapter is busy",
                True
            )

        self._state = OutputState.OUTPUTTING
        self._is_playing = True

        duration = (isp_code.duration or 1000) / 1000.0 / self._speed

        self._emit(OutputEventType.START, {
            "code": isp_code.code,
            "meaning": isp_code.meaning,
            "avatarId": self._avatar_id
        })

        try:
            await asyncio.sleep(duration)

            if self._is_playing:
                self._state = OutputState.IDLE
                self._is_playing = False
                self._emit(OutputEventType.END, {"code": isp_code.code})
        except asyncio.CancelledError:
            self._state = OutputState.IDLE
            self._is_playing = False
            raise

    async def play_sequence(self, isp_codes: List[ISPCode]) -> None:
        """Play a sequence of gestures"""
        for i, code in enumerate(isp_codes):
            if not self._is_playing and self._state != OutputState.IDLE:
                break

            self._emit(OutputEventType.PROGRESS, {
                "current": i + 1,
                "total": len(isp_codes),
                "code": code.code
            })

            await self.play_gesture(code)

            # Small pause between gestures
            if i < len(isp_codes) - 1:
                await asyncio.sleep(0.1 / self._speed)

    async def output(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """Output text as sign language"""
        if options and options.speed:
            self._speed = options.speed

        codes = await self.text_to_isp(text)
        if codes:
            self._is_playing = True
            await self.play_sequence(codes)

    def set_avatar(self, avatar_id: str) -> None:
        """Set avatar"""
        self._avatar_id = avatar_id

    def set_speed(self, speed: float) -> None:
        """Set playback speed"""
        self._speed = max(0.5, min(2.0, speed))

    def stop(self) -> None:
        """Stop output"""
        self._is_playing = False
        self._state = OutputState.IDLE

    def is_available(self) -> bool:
        """Check if adapter is available"""
        return True

    async def dispose(self) -> None:
        """Dispose resources"""
        self.stop()
        self._isp_dictionary.clear()
        self._handlers.clear()

    def add_mapping(self, word: str, code: str, duration: int = 1000) -> None:
        """Add custom ISP mapping"""
        self._isp_dictionary[word.lower()] = ISPDictionaryEntry(word, code, duration)

    def get_mappings(self) -> Dict[str, ISPDictionaryEntry]:
        """Get all mappings"""
        return dict(self._isp_dictionary)
