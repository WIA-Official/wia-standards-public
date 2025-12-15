"""
Calibration session for collecting user-specific training data.
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Callable, Dict, List, Optional
import numpy as np

from ..ml.base import (
    GestureClassifier,
    Gesture,
    FeatureVector,
    LabeledSample,
)


class CalibrationQuality(Enum):
    """Quality levels for calibration data."""

    POOR = "poor"
    FAIR = "fair"
    GOOD = "good"
    EXCELLENT = "excellent"


@dataclass
class CalibrationProgress:
    """Progress information for calibration session."""

    current_gesture: Gesture
    samples_collected: int
    samples_required: int
    quality: CalibrationQuality
    overall_progress: float
    completed_gestures: List[Gesture]
    remaining_gestures: List[Gesture]


@dataclass
class UserModel:
    """User-specific trained model."""

    user_id: str
    created_at: datetime
    gestures: List[Gesture]
    accuracy: float
    calibration_quality: float
    model_path: Optional[str] = None


@dataclass
class CalibrationConfig:
    """Configuration for calibration session."""

    gestures: List[Gesture] = field(
        default_factory=lambda: [
            Gesture.REST,
            Gesture.HAND_OPEN,
            Gesture.HAND_CLOSE,
            Gesture.WRIST_FLEXION,
            Gesture.WRIST_EXTENSION,
        ]
    )
    samples_per_gesture: int = 10
    sample_duration_ms: int = 3000
    rest_period_ms: int = 2000
    min_quality_threshold: CalibrationQuality = CalibrationQuality.FAIR


class CalibrationSession:
    """
    Manages the calibration process for user-specific model training.

    Example:
        >>> classifier = SVMClassifier()
        >>> session = CalibrationSession("user123", classifier)
        >>> session.start()
        >>> for gesture in session.get_remaining_gestures():
        ...     print(f"Perform {gesture.value}")
        ...     for i in range(10):
        ...         features = capture_emg_features()
        ...         session.collect_sample(features)
        >>> model = session.complete()
    """

    def __init__(
        self,
        user_id: str,
        classifier: GestureClassifier,
        config: Optional[CalibrationConfig] = None,
        on_progress: Optional[Callable[[CalibrationProgress], None]] = None,
    ):
        """
        Initialize calibration session.

        Args:
            user_id: Unique user identifier
            classifier: Classifier to train
            config: Calibration configuration
            on_progress: Callback for progress updates
        """
        self.user_id = user_id
        self.classifier = classifier
        self.config = config or CalibrationConfig()
        self.on_progress = on_progress

        self._samples: Dict[Gesture, List[LabeledSample]] = {
            g: [] for g in self.config.gestures
        }
        self._current_gesture_index = 0
        self._is_active = False

    def start(self) -> None:
        """Start the calibration session."""
        self._is_active = True
        self._current_gesture_index = 0
        self._notify_progress()

    def stop(self) -> None:
        """Stop the calibration session."""
        self._is_active = False

    def get_current_gesture(self) -> Gesture:
        """Get the current gesture being calibrated."""
        return self.config.gestures[self._current_gesture_index]

    def get_remaining_gestures(self) -> List[Gesture]:
        """Get list of gestures that still need calibration."""
        return self.config.gestures[self._current_gesture_index:]

    def get_completed_gestures(self) -> List[Gesture]:
        """Get list of completed gestures."""
        return self.config.gestures[: self._current_gesture_index]

    def collect_sample(self, features: FeatureVector) -> bool:
        """
        Collect a sample for the current gesture.

        Args:
            features: Feature vector to add

        Returns:
            True if gesture is complete after this sample
        """
        if not self._is_active:
            raise RuntimeError("Calibration session is not active")

        gesture = self.get_current_gesture()
        sample = LabeledSample(
            features=features, label=gesture, subject_id=self.user_id
        )
        self._samples[gesture].append(sample)

        # Check if gesture is complete
        gesture_complete = (
            len(self._samples[gesture]) >= self.config.samples_per_gesture
        )

        if gesture_complete:
            self._current_gesture_index += 1

        self._notify_progress()

        return gesture_complete

    def get_progress(self) -> CalibrationProgress:
        """Get current calibration progress."""
        gesture = self.get_current_gesture()
        samples = self._samples.get(gesture, [])

        # Calculate overall progress
        total_samples = sum(len(s) for s in self._samples.values())
        total_required = len(self.config.gestures) * self.config.samples_per_gesture
        overall_progress = total_samples / total_required * 100

        return CalibrationProgress(
            current_gesture=gesture,
            samples_collected=len(samples),
            samples_required=self.config.samples_per_gesture,
            quality=self._assess_quality(samples),
            overall_progress=overall_progress,
            completed_gestures=self.get_completed_gestures(),
            remaining_gestures=self.get_remaining_gestures()[1:],
        )

    def is_complete(self) -> bool:
        """Check if all gestures have been calibrated."""
        return self._current_gesture_index >= len(self.config.gestures)

    def complete(self) -> UserModel:
        """
        Complete calibration and train the model.

        Returns:
            UserModel with trained classifier

        Raises:
            RuntimeError: If calibration is not complete
        """
        if not self.is_complete():
            raise RuntimeError(
                f"Calibration is not complete. "
                f"Remaining gestures: {self.get_remaining_gestures()}"
            )

        # Prepare training data
        all_samples = []
        for samples in self._samples.values():
            all_samples.extend(samples)

        X = np.array([s.features.features for s in all_samples])
        y = np.array(
            [self.config.gestures.index(s.label) for s in all_samples]
        )

        # Train classifier
        result = self.classifier.train(X, y, self.config.gestures)

        # Create user model
        user_model = UserModel(
            user_id=self.user_id,
            created_at=datetime.now(),
            gestures=self.config.gestures,
            accuracy=result.accuracy,
            calibration_quality=self._calculate_overall_quality(),
        )

        self._is_active = False

        return user_model

    def _assess_quality(self, samples: List[LabeledSample]) -> CalibrationQuality:
        """Assess quality of collected samples."""
        if len(samples) < 3:
            return CalibrationQuality.POOR

        # Calculate feature variance
        features = np.array([s.features.features for s in samples])
        variances = np.var(features, axis=0)
        avg_variance = np.mean(variances)

        # Lower variance = more consistent = better quality
        if avg_variance < 0.01:
            return CalibrationQuality.EXCELLENT
        elif avg_variance < 0.05:
            return CalibrationQuality.GOOD
        elif avg_variance < 0.1:
            return CalibrationQuality.FAIR
        else:
            return CalibrationQuality.POOR

    def _calculate_overall_quality(self) -> float:
        """Calculate overall calibration quality score."""
        quality_scores = {
            CalibrationQuality.EXCELLENT: 1.0,
            CalibrationQuality.GOOD: 0.75,
            CalibrationQuality.FAIR: 0.5,
            CalibrationQuality.POOR: 0.25,
        }

        scores = []
        for gesture in self.config.gestures:
            samples = self._samples[gesture]
            quality = self._assess_quality(samples)
            scores.append(quality_scores[quality])

        return sum(scores) / len(scores)

    def _notify_progress(self) -> None:
        """Notify progress callback if registered."""
        if self.on_progress and self._is_active:
            self.on_progress(self.get_progress())

    def reset(self) -> None:
        """Reset the calibration session."""
        self._samples = {g: [] for g in self.config.gestures}
        self._current_gesture_index = 0
        self._is_active = False
