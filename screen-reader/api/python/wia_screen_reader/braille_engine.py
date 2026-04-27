"""
WIA Screen Reader - Braille Engine
"""
from typing import Dict, List, Optional
from .types import BrailleOutput, BrailleCell, BrailleGrade


class BrailleEngine:
    """
    Braille Engine for converting text to various braille formats.
    Supports Grade 1, Grade 2, and WIA format.
    """

    # Unicode Braille mapping (Grade 1)
    BRAILLE_MAP: Dict[str, str] = {
        'a': '⠁', 'b': '⠃', 'c': '⠉', 'd': '⠙', 'e': '⠑',
        'f': '⠋', 'g': '⠛', 'h': '⠓', 'i': '⠊', 'j': '⠚',
        'k': '⠅', 'l': '⠇', 'm': '⠍', 'n': '⠝', 'o': '⠕',
        'p': '⠏', 'q': '⠟', 'r': '⠗', 's': '⠎', 't': '⠞',
        'u': '⠥', 'v': '⠧', 'w': '⠺', 'x': '⠭', 'y': '⠽', 'z': '⠵',
        # Numbers (with number indicator ⠼)
        '0': '⠚', '1': '⠁', '2': '⠃', '3': '⠉', '4': '⠙',
        '5': '⠑', '6': '⠋', '7': '⠛', '8': '⠓', '9': '⠊',
        # Punctuation
        ' ': ' ',
        '.': '⠲',
        ',': '⠂',
        '!': '⠖',
        '?': '⠦',
        "'": '⠄',
        '-': '⠤',
        ':': '⠒',
        ';': '⠆',
        '(': '⠶',
        ')': '⠶',
        '"': '⠦',
    }

    # WIA Braille (Korean Jamo based)
    WIA_MAP: Dict[str, str] = {
        'a': 'ㅏ', 'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'e': 'ㅔ',
        'f': 'ㅍ', 'g': 'ㄱ', 'h': 'ㅎ', 'i': 'ㅣ', 'j': 'ㅈ',
        'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ', 'n': 'ㄴ', 'o': 'ㅗ',
        'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ', 't': 'ㅌ',
        'u': 'ㅜ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
        ' ': ' ',
    }

    # Grade 2 contractions (common)
    CONTRACTIONS: Dict[str, str] = {
        'and': '⠯',
        'for': '⠿',
        'of': '⠷',
        'the': '⠮',
        'with': '⠾',
        'ch': '⠡',
        'gh': '⠣',
        'sh': '⠩',
        'th': '⠹',
        'wh': '⠱',
        'ed': '⠫',
        'er': '⠻',
        'ou': '⠳',
        'ow': '⠪',
        'st': '⠌',
        'ar': '⠜',
        'ing': '⠬',
    }

    def __init__(self, grade: BrailleGrade = BrailleGrade.GRADE_1):
        """Initialize Braille engine with default grade."""
        self.grade = grade

    def convert(
        self,
        text: str,
        grade: Optional[BrailleGrade] = None
    ) -> BrailleOutput:
        """
        Convert text to braille.

        Args:
            text: Text to convert
            grade: Braille grade (1 or 2)

        Returns:
            BrailleOutput with grade1, grade2, and wia formats
        """
        use_grade = grade or self.grade
        lower_text = text.lower()

        # Convert to Grade 1
        grade1 = self._convert_grade1(lower_text)

        # Convert to Grade 2 (with contractions)
        grade2 = self._convert_grade2(lower_text)

        # Convert to WIA format
        wia = self._convert_wia(lower_text)

        # Generate cell data
        cells = self._generate_cells(lower_text)

        return BrailleOutput(
            grade1=grade1,
            grade2=grade2,
            wia=wia,
            dots=cells,
            cells=len(cells)
        )

    def _convert_grade1(self, text: str) -> str:
        """Convert text to Grade 1 braille."""
        result = []
        for char in text:
            if char in self.BRAILLE_MAP:
                result.append(self.BRAILLE_MAP[char])
            else:
                result.append(char)
        return ''.join(result)

    def _convert_grade2(self, text: str) -> str:
        """Convert text to Grade 2 braille with contractions."""
        result = text

        # Apply contractions
        for word, contraction in self.CONTRACTIONS.items():
            result = result.replace(word, contraction)

        # Convert remaining characters
        final = []
        for char in result:
            if char in self.BRAILLE_MAP:
                final.append(self.BRAILLE_MAP[char])
            elif char in self.CONTRACTIONS.values():
                final.append(char)
            else:
                final.append(char)

        return ''.join(final)

    def _convert_wia(self, text: str) -> str:
        """Convert text to WIA braille format."""
        result = []
        for char in text:
            if char in self.WIA_MAP:
                result.append(self.WIA_MAP[char])
            else:
                result.append(char)
        return ''.join(result)

    def _generate_cells(self, text: str) -> List[BrailleCell]:
        """Generate braille cell data."""
        cells = []
        for char in text:
            if char in self.BRAILLE_MAP and char != ' ':
                braille_char = self.BRAILLE_MAP[char]
                dots = self._unicode_to_dots(braille_char)
                cells.append(BrailleCell(
                    char=char,
                    dots=dots,
                    unicode=braille_char
                ))
        return cells

    def _unicode_to_dots(self, char: str) -> List[int]:
        """Convert Unicode braille character to dot pattern."""
        code = ord(char) - 0x2800
        dots = []
        for i in range(8):
            if code & (1 << i):
                dots.append(i + 1)
        return dots

    def dots_to_unicode(self, dots: List[int]) -> str:
        """Convert dot pattern to Unicode braille character."""
        code = 0
        for dot in dots:
            if 1 <= dot <= 8:
                code |= (1 << (dot - 1))
        return chr(0x2800 + code)

    def set_grade(self, grade: BrailleGrade) -> None:
        """Set the default braille grade."""
        self.grade = grade
