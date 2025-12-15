"""
Support Vector Machine classifier for EMG gesture recognition.
"""

import pickle
import time
from typing import Dict, List, Optional
import numpy as np
from numpy.typing import NDArray

from .base import (
    GestureClassifier,
    Gesture,
    FeatureVector,
    ClassificationResult,
    TrainingResult,
)


class SVMClassifier(GestureClassifier):
    """
    Support Vector Machine classifier for gesture recognition.

    Uses sklearn's SVC with RBF kernel and probability estimates.
    """

    def __init__(
        self,
        C: float = 1.0,
        gamma: str = "scale",
        kernel: str = "rbf",
        confidence_threshold: float = 0.7,
        majority_voting_frames: int = 3,
    ):
        """
        Initialize SVM classifier.

        Args:
            C: Regularization parameter
            gamma: Kernel coefficient
            kernel: Kernel type ('rbf', 'linear', 'poly')
            confidence_threshold: Minimum confidence for valid classification
            majority_voting_frames: Number of frames for majority voting
        """
        super().__init__(confidence_threshold, majority_voting_frames)

        self.C = C
        self.gamma = gamma
        self.kernel = kernel
        self._model = None
        self._scaler = None

    def train(
        self,
        X: NDArray[np.float32],
        y: NDArray[np.int32],
        gestures: List[Gesture],
    ) -> TrainingResult:
        """Train the SVM classifier."""
        try:
            from sklearn.svm import SVC
            from sklearn.preprocessing import StandardScaler
            from sklearn.model_selection import cross_val_score
            from sklearn.metrics import confusion_matrix
        except ImportError:
            raise ImportError("scikit-learn is required for SVMClassifier")

        start_time = time.time()

        self._classes = gestures

        # Standardize features
        self._scaler = StandardScaler()
        X_scaled = self._scaler.fit_transform(X)

        # Train SVM with probability estimates
        self._model = SVC(
            C=self.C,
            gamma=self.gamma,
            kernel=self.kernel,
            probability=True,
            random_state=42,
        )
        self._model.fit(X_scaled, y)

        # Evaluate using cross-validation
        cv_scores = cross_val_score(self._model, X_scaled, y, cv=5)
        accuracy = float(np.mean(cv_scores))

        # Get predictions for confusion matrix
        y_pred = self._model.predict(X_scaled)
        conf_matrix = confusion_matrix(y, y_pred)

        # Calculate per-class metrics
        gesture_metrics: Dict[Gesture, Dict[str, float]] = {}
        for i, gesture in enumerate(self._classes):
            tp = conf_matrix[i, i]
            fp = np.sum(conf_matrix[:, i]) - tp
            fn = np.sum(conf_matrix[i, :]) - tp

            precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            f1 = (
                2 * precision * recall / (precision + recall)
                if (precision + recall) > 0
                else 0.0
            )

            gesture_metrics[gesture] = {
                "precision": precision,
                "recall": recall,
                "f1_score": f1,
            }

        self._is_trained = True
        training_duration = time.time() - start_time

        # Estimate model size
        model_size = len(pickle.dumps(self._model)) + len(
            pickle.dumps(self._scaler)
        )

        return TrainingResult(
            success=True,
            accuracy=accuracy,
            gesture_metrics=gesture_metrics,
            confusion_matrix=conf_matrix,
            training_duration_sec=training_duration,
            model_size_bytes=model_size,
        )

    def predict(self, features: FeatureVector) -> ClassificationResult:
        """Classify a single feature vector."""
        if not self._is_trained or self._model is None:
            return ClassificationResult(
                gesture=Gesture.REST,
                confidence=0.0,
                alternatives=[],
                latency_ms=0.0,
                timestamp=features.timestamp,
            )

        start_time = time.time()

        # Scale features
        X = features.features.reshape(1, -1)
        X_scaled = self._scaler.transform(X)

        # Get prediction and probabilities
        pred = self._model.predict(X_scaled)[0]
        proba = self._model.predict_proba(X_scaled)[0]

        # Build result
        gesture = self._classes[pred]
        confidence = float(proba[pred])

        # Build alternatives
        alternatives = []
        sorted_indices = np.argsort(proba)[::-1]
        for idx in sorted_indices[1:4]:  # Top 3 alternatives
            alternatives.append((self._classes[idx], float(proba[idx])))

        latency_ms = (time.time() - start_time) * 1000

        return ClassificationResult(
            gesture=gesture,
            confidence=confidence,
            alternatives=alternatives,
            latency_ms=latency_ms,
            timestamp=features.timestamp,
        )

    def predict_proba(self, features: FeatureVector) -> Dict[Gesture, float]:
        """Get probability distribution over gestures."""
        if not self._is_trained or self._model is None:
            return {g: 0.0 for g in Gesture}

        X = features.features.reshape(1, -1)
        X_scaled = self._scaler.transform(X)
        proba = self._model.predict_proba(X_scaled)[0]

        return {self._classes[i]: float(p) for i, p in enumerate(proba)}

    def save(self, path: str) -> None:
        """Save model to file."""
        if not self._is_trained:
            raise ValueError("Model is not trained")

        data = {
            "model": self._model,
            "scaler": self._scaler,
            "classes": self._classes,
            "params": {
                "C": self.C,
                "gamma": self.gamma,
                "kernel": self.kernel,
            },
        }
        with open(path, "wb") as f:
            pickle.dump(data, f)

    def load(self, path: str) -> None:
        """Load model from file."""
        with open(path, "rb") as f:
            data = pickle.load(f)

        self._model = data["model"]
        self._scaler = data["scaler"]
        self._classes = data["classes"]
        self.C = data["params"]["C"]
        self.gamma = data["params"]["gamma"]
        self.kernel = data["params"]["kernel"]
        self._is_trained = True
