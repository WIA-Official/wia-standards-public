"""
WIA Screen Reader - Braille Engine for NVDA
"""
from typing import Dict, List
from dataclasses import dataclass


@dataclass
class BrailleOutput:
    """Braille conversion output."""
    grade1: str
    grade2: str
    wia: str
    cells: int


class WIABrailleEngine:
    """
    WIA Braille Engine for NVDA.
    Converts text to Grade 1, Grade 2, and WIA braille formats.
    """

    BRAILLE_MAP: Dict[str, str] = {
        'a': '⠁', 'b': '⠃', 'c': '⠉', 'd': '⠙', 'e': '⠑',
        'f': '⠋', 'g': '⠛', 'h': '⠓', 'i': '⠊', 'j': '⠚',
        'k': '⠅', 'l': '⠇', 'm': '⠍', 'n': '⠝', 'o': '⠕',
        'p': '⠏', 'q': '⠟', 'r': '⠗', 's': '⠎', 't': '⠞',
        'u': '⠥', 'v': '⠧', 'w': '⠺', 'x': '⠭', 'y': '⠽', 'z': '⠵',
        '0': '⠚', '1': '⠁', '2': '⠃', '3': '⠉', '4': '⠙',
        '5': '⠑', '6': '⠋', '7': '⠛', '8': '⠓', '9': '⠊',
        ' ': ' ', '.': '⠲', ',': '⠂', '!': '⠖', '?': '⠦',
    }

    WIA_MAP: Dict[str, str] = {
        'a': 'ㅏ', 'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'e': 'ㅔ',
        'f': 'ㅍ', 'g': 'ㄱ', 'h': 'ㅎ', 'i': 'ㅣ', 'j': 'ㅈ',
        'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ', 'n': 'ㄴ', 'o': 'ㅗ',
        'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ', 't': 'ㅌ',
        'u': 'ㅜ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
        ' ': ' ',
    }

    def convert(self, text: str) -> BrailleOutput:
        """Convert text to braille formats."""
        lower = text.lower()

        grade1 = ''.join(self.BRAILLE_MAP.get(c, c) for c in lower)
        grade2 = grade1  # Simplified - full Grade 2 would have contractions
        wia = ''.join(self.WIA_MAP.get(c, c) for c in lower)

        return BrailleOutput(
            grade1=grade1,
            grade2=grade2,
            wia=wia,
            cells=len([c for c in lower if c in self.BRAILLE_MAP and c != ' '])
        )
