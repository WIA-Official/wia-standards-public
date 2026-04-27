"""
WIA Screen Reader - Type Definitions
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, List, Dict, Any, Literal
from enum import Enum


class BrailleGrade(Enum):
    """Braille grade types"""
    GRADE_1 = 1
    GRADE_2 = 2


class ElementType(Enum):
    """ARIA element types"""
    HEADING = "heading"
    PARAGRAPH = "paragraph"
    LIST = "list"
    LISTITEM = "listitem"
    LINK = "link"
    BUTTON = "button"
    TEXTBOX = "textbox"
    CHECKBOX = "checkbox"
    RADIO = "radio"
    SLIDER = "slider"
    TABLE = "table"
    ROW = "row"
    CELL = "cell"
    IMAGE = "image"
    FIGURE = "figure"
    FORM = "form"
    NAVIGATION = "navigation"
    MAIN = "main"
    ARTICLE = "article"
    SECTION = "section"


class LandmarkRole(Enum):
    """ARIA landmark roles"""
    BANNER = "banner"
    MAIN = "main"
    NAVIGATION = "navigation"
    CONTENTINFO = "contentinfo"
    SEARCH = "search"
    FORM = "form"
    COMPLEMENTARY = "complementary"
    REGION = "region"


@dataclass
class Pronunciation:
    """Pronunciation output"""
    ipa: str
    wihp: str
    romanized: Optional[str] = None
    syllables: Optional[List[str]] = None


@dataclass
class BrailleCell:
    """Braille cell representation"""
    char: str
    dots: List[int]
    unicode: str


@dataclass
class BrailleOutput:
    """Braille output"""
    grade1: str
    grade2: str
    wia: str
    dots: Optional[List[BrailleCell]] = None
    cells: int = 0


@dataclass
class TTSConfig:
    """TTS configuration"""
    rate: float = 1.0
    pitch: float = 1.0
    volume: float = 1.0
    voice: Optional[str] = None
    ssml: Optional[str] = None


@dataclass
class ElementState:
    """Element state"""
    expanded: Optional[bool] = None
    selected: Optional[bool] = None
    checked: Optional[bool] = None
    pressed: Optional[bool] = None
    disabled: Optional[bool] = None
    readonly: Optional[bool] = None
    required: Optional[bool] = None
    invalid: Optional[bool] = None


@dataclass
class Position:
    """Position in a set"""
    index: int
    total: int


@dataclass
class Context:
    """Semantic context"""
    element_type: Optional[ElementType] = None
    level: Optional[int] = None
    landmark: Optional[LandmarkRole] = None
    live: Optional[Literal["off", "polite", "assertive"]] = None
    position: Optional[Position] = None
    state: Optional[ElementState] = None


@dataclass
class Metadata:
    """Processing metadata"""
    processed_at: datetime = field(default_factory=datetime.now)
    processing_time_ms: float = 0.0
    engine_version: str = "1.0.0"
    confidence: Optional[float] = None


@dataclass
class ScreenReaderResult:
    """Screen reader result"""
    text: str
    language: str
    pronunciation: Pronunciation
    braille: BrailleOutput
    tts: Optional[TTSConfig] = None
    context: Optional[Context] = None
    metadata: Optional[Metadata] = None

    @property
    def wihp(self) -> str:
        """Get WIHP pronunciation"""
        return self.pronunciation.wihp

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            "wia_screen_reader": {
                "version": "1.0.0",
                "text": self.text,
                "language": self.language,
                "pronunciation": {
                    "ipa": self.pronunciation.ipa,
                    "wihp": self.pronunciation.wihp,
                    "romanized": self.pronunciation.romanized,
                },
                "braille": {
                    "grade1": self.braille.grade1,
                    "grade2": self.braille.grade2,
                    "wia": self.braille.wia,
                    "cells": self.braille.cells,
                },
            }
        }
