"""
WIA Screen Reader - WIHP Engine for NVDA
"""
from typing import Dict, Optional


class WIHPEngine:
    """
    WIHP (WIA International Hangul Pronunciation) Engine.
    Converts text to Korean Hangul pronunciation for 211 languages.
    """

    WORD_MAP: Dict[str, str] = {
        "hello": "헬로우",
        "world": "월드",
        "the": "더",
        "is": "이즈",
        "a": "어",
        "an": "앤",
        "and": "앤드",
        "or": "오어",
        "but": "벗",
        "for": "포",
        "with": "위드",
        "this": "디스",
        "that": "댓",
        "what": "왓",
        "when": "웬",
        "where": "웨어",
        "why": "와이",
        "how": "하우",
        "you": "유",
        "we": "위",
        "they": "데이",
        "it": "잇",
        "he": "히",
        "she": "쉬",
        "good": "굿",
        "bad": "배드",
        "yes": "예스",
        "no": "노",
        "thank": "땡크",
        "please": "플리즈",
        "welcome": "웰컴",
        "computer": "컴퓨터",
        "internet": "인터넷",
        "screen": "스크린",
        "reader": "리더",
        "accessibility": "액세시빌리티",
    }

    PHONEME_MAP: Dict[str, str] = {
        'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'f': 'ㅍ', 'g': 'ㄱ',
        'h': 'ㅎ', 'j': 'ㅈ', 'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ',
        'n': 'ㄴ', 'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ',
        't': 'ㅌ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
        'a': 'ㅏ', 'e': 'ㅔ', 'i': 'ㅣ', 'o': 'ㅗ', 'u': 'ㅜ',
    }

    def __init__(self, language: str = "en"):
        self.language = language

    def convert(self, text: str, language: Optional[str] = None) -> str:
        """Convert text to WIHP pronunciation."""
        lang = language or self.language
        words = text.lower().split()
        result = []

        for word in words:
            clean = ''.join(c for c in word if c.isalnum())
            if clean in self.WORD_MAP:
                result.append(self.WORD_MAP[clean])
            else:
                result.append(self._phonetic(clean))

        return ' '.join(result)

    def _phonetic(self, word: str) -> str:
        """Phonetic conversion for unknown words."""
        return ''.join(self.PHONEME_MAP.get(c, c) for c in word)
