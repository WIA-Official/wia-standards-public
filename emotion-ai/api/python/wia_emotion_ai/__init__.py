"""
WIA Emotion AI SDK for Python

Affective Computing / Emotion Recognition Standards

Usage:
    from wia_emotion_ai import EmotionClient

    client = EmotionClient(api_key="your-api-key")
    result = client.analyze_facial_image("photo.jpg")
    print(result.emotions[0].category)

弘益人間 (홍익인간) - Benefit All Humanity
"""

__version__ = "1.0.0"
__author__ = "WIA (World Interoperability Alliance)"
__license__ = "MIT"

from .client import EmotionClient
from .models import (
    EmotionEvent,
    Emotion,
    ActionUnit,
    DimensionalModel,
    Modality
)
from .stream import EmotionStream

__all__ = [
    "EmotionClient",
    "EmotionStream",
    "EmotionEvent",
    "Emotion",
    "ActionUnit",
    "DimensionalModel",
    "Modality"
]
