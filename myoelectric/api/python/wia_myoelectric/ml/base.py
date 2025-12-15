"""
Base classes and types for gesture classification.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Dict, Optional, Tuple
import numpy as np
from numpy.typing import NDArray
import time


class Gesture(Enum):
    """Supported hand gestures for myoelectric control."""

    REST = "rest"
    HAND_OPEN = "hand_open"
    HAND_CLOSE = "hand_close"
    WRIST_FLEXION = "wrist_flexion"
    WRIST_EXTENSION = "wrist_extension"
    WRIST_PRONATION = "wrist_pronation"
    WRIST_SUPINATION = "wrist_supination"
    PINCH_THUMB_INDEX = "pinch_thumb_index"
    PINCH_THUMB_MIDDLE = "pinch_thumb_middle"
    POINT_INDEX = "point_index"
    THUMBS_UP = "thumbs_up"


@dataclass
class FeatureVector:
    """Feature vector extracted from EMG signals."""

    timestamp: float
    channel_count: int
    feature_names: List[str]
    features: NDArray[np.float32]

    def to_numpy(self) -> NDArray[np.float32]:
        """Convert to numpy array."""
        return self.features


@dataclass
class ClassificationResult:
    """Result of gesture classification."""

    gesture: Gesture
    confidence: float
    alternatives: List[Tuple[Gesture, float]]
    latency_ms: float
    timestamp: float


@dataclass
class TrainingResult:
    """Result of model training."""

    success: bool
    accuracy: float
    gesture_metrics: Dict[Gesture, Dict[str, float]]
    confusion_matrix: NDArray[np.int32]
    training_duration_sec: float
    model_size_bytes: int


@dataclass
class LabeledSample:
    """Single labeled sample for training."""

    features: FeatureVector
    label: Gesture
    subject_id: Optional[str] = None
    session_id: Optional[str] = None


class GestureClassifier(ABC):
    """Abstract base class for gesture classifiers."""

    def __init__(
        self,
        confidence_threshold: float = 0.7,
        majority_voting_frames: int = 3,
    ):
        self.confidence_threshold = confidence_threshold
        self.majority_voting_frames = majority_voting_frames
        self._is_trained = False
        self._classes: List[Gesture] = []

    @abstractmethod
    def train(
        self,
        X: NDArray[np.float32],
        y: NDArray[np.int32],
        gestures: List[Gesture],
    ) -> TrainingResult:
        """
        Train the classifier.

        Args:
            X: Feature matrix of shape (n_samples, n_features)
            y: Label array of shape (n_samples,)
            gestures: List of gesture classes

        Returns:
            TrainingResult with metrics
        """
        pass

    @abstractmethod
    def predict(self, features: FeatureVector) -> ClassificationResult:
        """
        Classify a single feature vector.

        Args:
            features: Feature vector to classify

        Returns:
            ClassificationResult with gesture and confidence
        """
        pass

    @abstractmethod
    def predict_proba(
        self, features: FeatureVector
    ) -> Dict[Gesture, float]:
        """
        Get probability distribution over gestures.

        Args:
            features: Feature vector to classify

        Returns:
            Dictionary mapping gestures to probabilities
        """
        pass

    @abstractmethod
    def save(self, path: str) -> None:
        """Save model to file."""
        pass

    @abstractmethod
    def load(self, path: str) -> None:
        """Load model from file."""
        pass

    def predict_batch(
        self, features_list: List[FeatureVector]
    ) -> List[ClassificationResult]:
        """Classify multiple feature vectors."""
        return [self.predict(f) for f in features_list]

    def is_trained(self) -> bool:
        """Check if model is trained."""
        return self._is_trained

    def get_classes(self) -> List[Gesture]:
        """Get list of gesture classes."""
        return self._classes.copy()


def extract_features(
    signal: NDArray[np.float32],
    sample_rate: int = 1000,
    window_size_ms: int = 200,
    overlap_percent: float = 50.0,
) -> List[FeatureVector]:
    """
    Extract features from raw EMG signal.

    Args:
        signal: Raw EMG signal of shape (n_samples, n_channels)
        sample_rate: Sampling rate in Hz
        window_size_ms: Window size in milliseconds
        overlap_percent: Window overlap percentage

    Returns:
        List of feature vectors
    """
    window_samples = int(window_size_ms * sample_rate / 1000)
    hop_samples = int(window_samples * (1 - overlap_percent / 100))

    n_samples, n_channels = signal.shape
    features_list = []

    for start in range(0, n_samples - window_samples + 1, hop_samples):
        window = signal[start : start + window_samples]
        timestamp = (start + window_samples) / sample_rate * 1000

        features = []
        feature_names = []

        for ch in range(n_channels):
            channel_data = window[:, ch]

            # Mean Absolute Value (MAV)
            mav = np.mean(np.abs(channel_data))
            features.append(mav)
            feature_names.append(f"ch{ch}_mav")

            # Root Mean Square (RMS)
            rms = np.sqrt(np.mean(channel_data ** 2))
            features.append(rms)
            feature_names.append(f"ch{ch}_rms")

            # Waveform Length (WL)
            wl = np.sum(np.abs(np.diff(channel_data)))
            features.append(wl)
            feature_names.append(f"ch{ch}_wl")

            # Zero Crossings (ZC)
            threshold = 0.01
            zc = np.sum(
                (np.sign(channel_data[1:]) != np.sign(channel_data[:-1]))
                & (np.abs(np.diff(channel_data)) > threshold)
            )
            features.append(zc)
            feature_names.append(f"ch{ch}_zc")

            # Slope Sign Changes (SSC)
            diff1 = channel_data[1:-1] - channel_data[:-2]
            diff2 = channel_data[1:-1] - channel_data[2:]
            ssc = np.sum(diff1 * diff2 > 0.0001)
            features.append(ssc)
            feature_names.append(f"ch{ch}_ssc")

        features_list.append(
            FeatureVector(
                timestamp=timestamp,
                channel_count=n_channels,
                feature_names=feature_names,
                features=np.array(features, dtype=np.float32),
            )
        )

    return features_list
