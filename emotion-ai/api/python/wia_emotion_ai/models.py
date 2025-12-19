"""
WIA Emotion AI Data Models

Pydantic models for emotion data structures.
"""

from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from datetime import datetime
from enum import Enum


class EmotionCategory(str, Enum):
    """Emotion categories based on Ekman's model + extensions."""
    HAPPINESS = "happiness"
    SADNESS = "sadness"
    ANGER = "anger"
    FEAR = "fear"
    DISGUST = "disgust"
    SURPRISE = "surprise"
    NEUTRAL = "neutral"
    CONTEMPT = "contempt"
    CONFUSION = "confusion"
    INTEREST = "interest"
    BOREDOM = "boredom"
    FRUSTRATION = "frustration"
    EXCITEMENT = "excitement"
    ANXIETY = "anxiety"


class ModalityType(str, Enum):
    """Input modality types."""
    FACIAL = "facial"
    VOICE = "voice"
    TEXT = "text"
    BIOSIGNAL = "biosignal"
    MULTIMODAL = "multimodal"


@dataclass
class Emotion:
    """Detected emotion with intensity and confidence."""
    category: EmotionCategory
    intensity: float  # 0.0 - 1.0
    confidence: float  # 0.0 - 1.0
    onset_time: Optional[float] = None
    apex_time: Optional[float] = None
    offset_time: Optional[float] = None
    source_modality: Optional[ModalityType] = None


@dataclass
class ActionUnit:
    """FACS Action Unit detection result."""
    au: str  # e.g., "AU6", "AU12L"
    intensity: float  # 0.0 - 1.0
    name: Optional[str] = None
    intensity_label: Optional[str] = None  # A, B, C, D, E
    confidence: Optional[float] = None
    symmetric: Optional[bool] = None


@dataclass
class DimensionalModel:
    """Valence-Arousal-Dominance dimensional model."""
    valence: float  # -1.0 to +1.0
    arousal: float  # 0.0 to 1.0
    dominance: Optional[float] = None  # 0.0 to 1.0


@dataclass
class Modality:
    """Per-modality analysis data."""
    type: ModalityType
    confidence: float
    weight: Optional[float] = None
    data: Optional[Dict[str, Any]] = None


@dataclass
class Metadata:
    """Event metadata."""
    provider: Optional[str] = None
    model_version: Optional[str] = None
    processing_time_ms: Optional[int] = None
    cultural_context: Optional[str] = None


@dataclass
class EmotionEvent:
    """
    Complete emotion event according to WIA standard.

    Example:
        event = EmotionEvent(
            event_id="550e8400-e29b-41d4-a716-446655440000",
            timestamp=datetime.now(),
            version="1.0.0",
            emotions=[
                Emotion(
                    category=EmotionCategory.HAPPINESS,
                    intensity=0.85,
                    confidence=0.92
                )
            ]
        )
    """
    event_id: str
    timestamp: datetime
    version: str
    emotions: List[Emotion]
    session_id: Optional[str] = None
    subject_id: Optional[str] = None
    action_units: Optional[List[ActionUnit]] = None
    dimensional: Optional[DimensionalModel] = None
    modalities: Optional[List[Modality]] = None
    metadata: Optional[Metadata] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            "event_id": self.event_id,
            "timestamp": self.timestamp.isoformat(),
            "version": self.version,
            "emotions": [
                {
                    "category": e.category.value,
                    "intensity": e.intensity,
                    "confidence": e.confidence
                }
                for e in self.emotions
            ]
        }
        if self.session_id:
            result["session_id"] = self.session_id
        if self.action_units:
            result["action_units"] = [
                {"au": au.au, "intensity": au.intensity}
                for au in self.action_units
            ]
        if self.dimensional:
            result["dimensional"] = {
                "valence": self.dimensional.valence,
                "arousal": self.dimensional.arousal
            }
        return result


@dataclass
class AnalysisResult:
    """Analysis result from API calls."""
    request_id: str
    timestamp: datetime
    processing_time_ms: int
    emotions: List[Emotion]
    action_units: Optional[List[ActionUnit]] = None
    dimensional: Optional[DimensionalModel] = None
    faces: Optional[List[Dict[str, Any]]] = None
    segments: Optional[List[Dict[str, Any]]] = None
    aggregate: Optional[Dict[str, Any]] = None
