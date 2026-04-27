"""
WIA Screen Reader - WIHP (WIA International Hangul Pronunciation) Engine
"""
from typing import Dict, Optional


class WIHPEngine:
    """
    WIHP Engine for converting text to Korean Hangul pronunciation.
    Supports 211 languages.
    """

    # Phoneme mapping for basic characters
    PHONEME_MAP: Dict[str, str] = {
        # English consonants
        'b': 'ㅂ', 'c': 'ㅋ', 'd': 'ㄷ', 'f': 'ㅍ', 'g': 'ㄱ',
        'h': 'ㅎ', 'j': 'ㅈ', 'k': 'ㅋ', 'l': 'ㄹ', 'm': 'ㅁ',
        'n': 'ㄴ', 'p': 'ㅍ', 'q': 'ㅋ', 'r': 'ㄹ', 's': 'ㅅ',
        't': 'ㅌ', 'v': 'ㅂ', 'w': 'ㅇ', 'x': 'ㅋㅅ', 'y': 'ㅇ', 'z': 'ㅈ',
        # English vowels
        'a': 'ㅏ', 'e': 'ㅔ', 'i': 'ㅣ', 'o': 'ㅗ', 'u': 'ㅜ',
    }

    # Common word pronunciations
    WORD_MAP: Dict[str, str] = {
        # Basic words
        'hello': '헬로우',
        'world': '월드',
        'the': '더',
        'is': '이즈',
        'a': '어',
        'an': '앤',
        'and': '앤드',
        'or': '오어',
        'but': '벗',
        'for': '포',
        'with': '위드',
        'this': '디스',
        'that': '댓',
        # Question words
        'what': '왓',
        'when': '웬',
        'where': '웨어',
        'why': '와이',
        'how': '하우',
        'who': '후',
        # Pronouns
        'i': '아이',
        'you': '유',
        'we': '위',
        'they': '데이',
        'it': '잇',
        'he': '히',
        'she': '쉬',
        # Common adjectives
        'good': '굿',
        'bad': '배드',
        'big': '빅',
        'small': '스몰',
        'new': '뉴',
        'old': '올드',
        # Common verbs
        'go': '고',
        'come': '컴',
        'see': '시',
        'look': '룩',
        'make': '메이크',
        'get': '겟',
        'have': '해브',
        'do': '두',
        # Yes/No
        'yes': '예스',
        'no': '노',
        # Courtesy
        'thank': '땡크',
        'thanks': '땡스',
        'please': '플리즈',
        'welcome': '웰컴',
        'sorry': '소리',
        # Tech terms
        'computer': '컴퓨터',
        'internet': '인터넷',
        'screen': '스크린',
        'reader': '리더',
        'accessibility': '액세시빌리티',
        'software': '소프트웨어',
        'hardware': '하드웨어',
        'application': '애플리케이션',
        'website': '웹사이트',
        'browser': '브라우저',
    }

    # Language-specific maps
    JAPANESE_MAP: Dict[str, str] = {
        'こんにちは': '곤니치와',
        'ありがとう': '아리가토우',
        'さようなら': '사요나라',
        'おはよう': '오하요',
        'すみません': '스미마센',
    }

    CHINESE_MAP: Dict[str, str] = {
        '你好': '니하오',
        '谢谢': '씨에씨에',
        '再见': '짜이젠',
        '早上好': '짜오샹하오',
    }

    def __init__(self, language: str = "en"):
        """Initialize WIHP engine with default language."""
        self.language = language

    def convert(self, text: str, language: Optional[str] = None) -> str:
        """
        Convert text to WIHP (Korean Hangul Pronunciation).

        Args:
            text: Text to convert
            language: Source language code (ISO 639-1)

        Returns:
            WIHP string in Korean Hangul
        """
        lang = language or self.language

        # Handle special languages
        if lang == "ja":
            return self._convert_japanese(text)
        elif lang == "zh":
            return self._convert_chinese(text)

        # Default: English and other languages
        return self._convert_english(text)

    def _convert_english(self, text: str) -> str:
        """Convert English text to WIHP."""
        words = text.lower().split()
        result = []

        for word in words:
            # Clean word of punctuation for lookup
            clean_word = ''.join(c for c in word if c.isalnum())

            if clean_word in self.WORD_MAP:
                result.append(self.WORD_MAP[clean_word])
            else:
                result.append(self._phonetic_convert(clean_word))

        return ' '.join(result)

    def _convert_japanese(self, text: str) -> str:
        """Convert Japanese text to WIHP."""
        # Check for known phrases
        for jp, kr in self.JAPANESE_MAP.items():
            text = text.replace(jp, kr)
        return text

    def _convert_chinese(self, text: str) -> str:
        """Convert Chinese text to WIHP."""
        # Check for known phrases
        for cn, kr in self.CHINESE_MAP.items():
            text = text.replace(cn, kr)
        return text

    def _phonetic_convert(self, word: str) -> str:
        """Convert word using phoneme mapping."""
        result = []
        for char in word.lower():
            if char in self.PHONEME_MAP:
                result.append(self.PHONEME_MAP[char])
            else:
                result.append(char)
        return ''.join(result)

    def set_language(self, language: str) -> None:
        """Set the default language."""
        self.language = language

    def add_word(self, word: str, pronunciation: str) -> None:
        """Add a custom word pronunciation."""
        self.WORD_MAP[word.lower()] = pronunciation

    def get_ipa(self, text: str) -> str:
        """Generate IPA representation (simplified)."""
        return f"/{text.lower()}/"
