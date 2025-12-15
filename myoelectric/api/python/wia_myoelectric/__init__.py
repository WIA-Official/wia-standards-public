"""
WIA Myoelectric Gesture Recognition Library

This library provides gesture recognition algorithms for
EMG-based myoelectric prosthetic control systems.
"""

__version__ = "1.0.0"
__author__ = "WIA Standards Committee"

from .ml.base import GestureClassifier, Gesture, ClassificationResult
from .ml.svm import SVMClassifier
from .ml.cnn import CNNClassifier
from .ml.transformer import TransformerClassifier
from .calibration.session import CalibrationSession

__all__ = [
    "GestureClassifier",
    "Gesture",
    "ClassificationResult",
    "SVMClassifier",
    "CNNClassifier",
    "TransformerClassifier",
    "CalibrationSession",
]
