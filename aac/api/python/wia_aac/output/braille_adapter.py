"""
WIA AAC Braille Adapter
Phase 4: WIA Ecosystem Integration

WIA Braille integration for braille output
"""

import asyncio
from abc import abstractmethod
from typing import Optional, List, Dict

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputState,
    OutputOptions,
    OutputEventType,
    BrailleOutput,
    BrailleDisplay,
    OutputError,
    OutputErrorCode
)


class IBrailleAdapter(BaseOutputAdapter):
    """Braille adapter interface"""

    @abstractmethod
    async def text_to_ipa(self, text: str) -> str:
        """Convert text to IPA"""
        pass

    @abstractmethod
    async def text_to_braille(self, text: str) -> BrailleOutput:
        """Convert text to braille"""
        pass

    @abstractmethod
    async def send_to_display(self, braille: BrailleOutput) -> None:
        """Send braille to display"""
        pass

    @abstractmethod
    async def get_connected_displays(self) -> List[BrailleDisplay]:
        """Get connected displays"""
        pass

    @abstractmethod
    def set_display(self, display_id: str) -> None:
        """Set active display"""
        pass


class MockBrailleAdapter(IBrailleAdapter):
    """Mock Braille Adapter using WIA Braille system (IPA-based)"""

    def __init__(self):
        super().__init__()
        self._ipa_map: Dict[str, str] = {}
        self._braille_map: Dict[str, str] = {}
        self._selected_display_id: Optional[str] = None
        self._mock_displays: List[BrailleDisplay] = []

    @property
    def type(self) -> OutputType:
        return OutputType.BRAILLE

    @property
    def name(self) -> str:
        return "MockBraille"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize the adapter"""
        self._load_ipa_map()
        self._load_braille_map()
        self._init_mock_displays()
        self._state = OutputState.IDLE
        self._emit(OutputEventType.START, {"message": "Braille adapter initialized"})

    def _init_mock_displays(self) -> None:
        """Initialize mock braille displays"""
        self._mock_displays = [
            BrailleDisplay(id="mock-40", name="Mock Braille Display 40", cells=40, connected=True),
            BrailleDisplay(id="mock-80", name="Mock Braille Display 80", cells=80, connected=False)
        ]
        self._selected_display_id = "mock-40"

    def _load_ipa_map(self) -> None:
        """Load IPA mapping for common words"""
        # Korean words
        self._ipa_map["안녕"] = "/annjʌŋ/"
        self._ipa_map["안녕하세요"] = "/annjʌŋhasejo/"
        self._ipa_map["감사"] = "/kamsa/"
        self._ipa_map["감사합니다"] = "/kamsahamnida/"
        self._ipa_map["사랑"] = "/saraŋ/"
        self._ipa_map["사랑해"] = "/saraŋhɛ/"
        self._ipa_map["네"] = "/ne/"
        self._ipa_map["아니오"] = "/anio/"
        self._ipa_map["도움"] = "/toum/"
        self._ipa_map["미안"] = "/mian/"
        self._ipa_map["좋아"] = "/tʃoa/"

        # English words
        self._ipa_map["hello"] = "/həˈloʊ/"
        self._ipa_map["thanks"] = "/θæŋks/"
        self._ipa_map["love"] = "/lʌv/"
        self._ipa_map["help"] = "/hɛlp/"
        self._ipa_map["yes"] = "/jɛs/"
        self._ipa_map["no"] = "/noʊ/"

    def _load_braille_map(self) -> None:
        """Load IPA to Braille mapping (WIA Braille system - 8-dot)"""
        # Vowels
        self._braille_map["a"] = "⠁"  # U+2801
        self._braille_map["e"] = "⠑"  # U+2811
        self._braille_map["i"] = "⠊"  # U+280A
        self._braille_map["o"] = "⠕"  # U+2815
        self._braille_map["u"] = "⠥"  # U+2825
        self._braille_map["ə"] = "⠢"  # U+2822 (schwa)
        self._braille_map["ʌ"] = "⠪"  # U+282A
        self._braille_map["ɛ"] = "⠫"  # U+282B
        self._braille_map["ɔ"] = "⠬"  # U+282C
        self._braille_map["æ"] = "⠭"  # U+282D
        self._braille_map["ʊ"] = "⠮"  # U+282E

        # Consonants
        self._braille_map["p"] = "⠏"  # U+280F
        self._braille_map["b"] = "⠃"  # U+2803
        self._braille_map["t"] = "⠞"  # U+281E
        self._braille_map["d"] = "⠙"  # U+2819
        self._braille_map["k"] = "⠅"  # U+2805
        self._braille_map["g"] = "⠛"  # U+281B
        self._braille_map["m"] = "⠍"  # U+280D
        self._braille_map["n"] = "⠝"  # U+281D
        self._braille_map["ŋ"] = "⠻"  # U+283B (eng)
        self._braille_map["f"] = "⠋"  # U+280B
        self._braille_map["v"] = "⠧"  # U+2827
        self._braille_map["s"] = "⠎"  # U+280E
        self._braille_map["z"] = "⠵"  # U+2835
        self._braille_map["h"] = "⠓"  # U+2813
        self._braille_map["l"] = "⠇"  # U+2807
        self._braille_map["r"] = "⠗"  # U+2817
        self._braille_map["w"] = "⠺"  # U+283A
        self._braille_map["j"] = "⠚"  # U+281A
        self._braille_map["ʃ"] = "⠱"  # U+2831 (sh)
        self._braille_map["ʒ"] = "⠴"  # U+2834 (zh)
        self._braille_map["tʃ"] = "⠹"  # U+2839 (ch)
        self._braille_map["θ"] = "⠹"  # U+2839 (th voiceless)
        self._braille_map["ð"] = "⠮"  # U+282E (th voiced)

        # Diacritics
        self._braille_map["ˈ"] = "⠄"  # U+2804 (primary stress)
        self._braille_map["ˌ"] = "⠠"  # U+2820 (secondary stress)
        self._braille_map["ː"] = "⠒"  # U+2812 (long)

        # Space
        self._braille_map[" "] = "⠀"  # U+2800 (blank)

    def _get_braille_char(self, ipa_char: str) -> str:
        """Get braille character for an IPA character"""
        return self._braille_map.get(ipa_char, "⠿")  # U+283F for unknown

    def _tokenize_ipa(self, ipa: str) -> List[str]:
        """Tokenize IPA string into phonemes"""
        tokens = []
        i = 0

        while i < len(ipa):
            # Check for two-character combinations
            if i < len(ipa) - 1:
                two_char = ipa[i:i+2]
                if two_char in self._braille_map:
                    tokens.append(two_char)
                    i += 2
                    continue

            # Single character
            tokens.append(ipa[i])
            i += 1

        return tokens

    async def text_to_ipa(self, text: str) -> str:
        """Convert text to IPA"""
        words = text.lower().split()
        ipa_words = []

        for word in words:
            ipa = self._ipa_map.get(word)
            if ipa:
                ipa_words.append(ipa)
            else:
                # Simple fallback
                ipa_words.append(f"/{word}/")

        return " ".join(ipa_words)

    async def text_to_braille(self, text: str) -> BrailleOutput:
        """Convert text to braille"""
        ipa = await self.text_to_ipa(text)

        # Remove IPA delimiters
        clean_ipa = ipa.replace("/", "").replace("[", "").replace("]", "")
        chars = self._tokenize_ipa(clean_ipa)

        braille = ""
        unicode_list = []
        dots = []

        for char in chars:
            braille_char = self._get_braille_char(char)
            braille += braille_char

            code_point = ord(braille_char)
            unicode_list.append(f"U+{code_point:04X}")
            dots.append(code_point - 0x2800)

        return BrailleOutput(
            text=text,
            ipa=ipa,
            braille=braille,
            unicode=unicode_list,
            dots=dots
        )

    async def send_to_display(self, braille: BrailleOutput) -> None:
        """Send braille to display"""
        if not self._selected_display_id:
            raise OutputError(
                OutputErrorCode.DEVICE_NOT_CONNECTED,
                "No braille display selected",
                True
            )

        display = next(
            (d for d in self._mock_displays if d.id == self._selected_display_id),
            None
        )

        if not display or not display.connected:
            raise OutputError(
                OutputErrorCode.DEVICE_NOT_CONNECTED,
                "Braille display not connected",
                True
            )

        self._state = OutputState.OUTPUTTING
        self._emit(OutputEventType.START, {
            "displayId": self._selected_display_id,
            "braille": braille.braille
        })

        # Simulate display reading time
        reading_time = max(len(braille.braille) * 0.1, 0.5)
        await asyncio.sleep(reading_time)

        self._state = OutputState.IDLE
        self._emit(OutputEventType.END, {
            "displayId": self._selected_display_id,
            "braille": braille.braille
        })

    async def output(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """Output text as braille"""
        braille_output = await self.text_to_braille(text)
        await self.send_to_display(braille_output)

    async def get_connected_displays(self) -> List[BrailleDisplay]:
        """Get connected displays"""
        return [d for d in self._mock_displays if d.connected]

    def set_display(self, display_id: str) -> None:
        """Set active display"""
        display = next(
            (d for d in self._mock_displays if d.id == display_id),
            None
        )
        if not display:
            raise OutputError(
                OutputErrorCode.DEVICE_NOT_CONNECTED,
                f"Display not found: {display_id}",
                True
            )
        self._selected_display_id = display_id

    def stop(self) -> None:
        """Stop output"""
        self._state = OutputState.IDLE

    def is_available(self) -> bool:
        """Check if adapter is available"""
        return any(d.connected for d in self._mock_displays)

    async def dispose(self) -> None:
        """Dispose resources"""
        self._ipa_map.clear()
        self._braille_map.clear()
        self._handlers.clear()

    def add_ipa_mapping(self, word: str, ipa: str) -> None:
        """Add custom IPA mapping"""
        self._ipa_map[word.lower()] = ipa

    def add_braille_mapping(self, ipa: str, braille: str) -> None:
        """Add custom braille mapping"""
        self._braille_map[ipa] = braille

    def connect_display(self, display_id: str) -> None:
        """Simulate connecting a display"""
        for d in self._mock_displays:
            if d.id == display_id:
                d.connected = True
                break

    def disconnect_display(self, display_id: str) -> None:
        """Simulate disconnecting a display"""
        for d in self._mock_displays:
            if d.id == display_id:
                d.connected = False
                break
