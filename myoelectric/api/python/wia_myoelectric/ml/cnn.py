"""
Convolutional Neural Network classifier for EMG gesture recognition.
"""

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


class CNNClassifier(GestureClassifier):
    """
    Convolutional Neural Network classifier for gesture recognition.

    Uses a 1D CNN architecture optimized for EMG signals.
    Supports both PyTorch and ONNX runtime for inference.
    """

    def __init__(
        self,
        input_channels: int = 4,
        num_classes: int = 5,
        hidden_dim: int = 64,
        num_conv_layers: int = 3,
        dropout: float = 0.3,
        confidence_threshold: float = 0.7,
        majority_voting_frames: int = 3,
        use_onnx: bool = False,
    ):
        """
        Initialize CNN classifier.

        Args:
            input_channels: Number of EMG channels
            num_classes: Number of gesture classes
            hidden_dim: Hidden layer dimension
            num_conv_layers: Number of convolutional layers
            dropout: Dropout rate
            confidence_threshold: Minimum confidence for valid classification
            majority_voting_frames: Number of frames for majority voting
            use_onnx: Whether to use ONNX runtime for inference
        """
        super().__init__(confidence_threshold, majority_voting_frames)

        self.input_channels = input_channels
        self.num_classes = num_classes
        self.hidden_dim = hidden_dim
        self.num_conv_layers = num_conv_layers
        self.dropout = dropout
        self.use_onnx = use_onnx

        self._model = None
        self._onnx_session = None
        self._device = "cpu"

    def _build_model(self):
        """Build the CNN model using PyTorch."""
        try:
            import torch
            import torch.nn as nn
        except ImportError:
            raise ImportError("PyTorch is required for CNNClassifier")

        class EMG_CNN(nn.Module):
            def __init__(
                self,
                input_dim: int,
                num_classes: int,
                hidden_dim: int,
                dropout: float,
            ):
                super().__init__()

                self.conv_layers = nn.Sequential(
                    nn.Conv1d(1, hidden_dim, kernel_size=3, padding=1),
                    nn.BatchNorm1d(hidden_dim),
                    nn.ReLU(),
                    nn.MaxPool1d(2),
                    nn.Conv1d(hidden_dim, hidden_dim * 2, kernel_size=3, padding=1),
                    nn.BatchNorm1d(hidden_dim * 2),
                    nn.ReLU(),
                    nn.MaxPool1d(2),
                    nn.Conv1d(
                        hidden_dim * 2, hidden_dim * 4, kernel_size=3, padding=1
                    ),
                    nn.BatchNorm1d(hidden_dim * 4),
                    nn.ReLU(),
                    nn.AdaptiveAvgPool1d(1),
                )

                self.classifier = nn.Sequential(
                    nn.Flatten(),
                    nn.Linear(hidden_dim * 4, hidden_dim),
                    nn.ReLU(),
                    nn.Dropout(dropout),
                    nn.Linear(hidden_dim, num_classes),
                )

            def forward(self, x):
                x = x.unsqueeze(1)  # Add channel dimension
                x = self.conv_layers(x)
                x = self.classifier(x)
                return x

        return EMG_CNN(
            self.input_channels * 5,  # 5 features per channel
            self.num_classes,
            self.hidden_dim,
            self.dropout,
        )

    def train(
        self,
        X: NDArray[np.float32],
        y: NDArray[np.int32],
        gestures: List[Gesture],
        epochs: int = 100,
        batch_size: int = 32,
        learning_rate: float = 0.001,
        validation_split: float = 0.2,
    ) -> TrainingResult:
        """Train the CNN classifier."""
        try:
            import torch
            import torch.nn as nn
            import torch.optim as optim
            from torch.utils.data import DataLoader, TensorDataset
        except ImportError:
            raise ImportError("PyTorch is required for CNNClassifier")

        start_time = time.time()

        self._classes = gestures
        self.num_classes = len(gestures)

        # Build model
        self._model = self._build_model()
        self._model.to(self._device)

        # Prepare data
        X_tensor = torch.FloatTensor(X)
        y_tensor = torch.LongTensor(y)

        # Split into train/validation
        n_val = int(len(X) * validation_split)
        indices = np.random.permutation(len(X))
        train_indices = indices[n_val:]
        val_indices = indices[:n_val]

        train_dataset = TensorDataset(
            X_tensor[train_indices], y_tensor[train_indices]
        )
        val_dataset = TensorDataset(X_tensor[val_indices], y_tensor[val_indices])

        train_loader = DataLoader(
            train_dataset, batch_size=batch_size, shuffle=True
        )
        val_loader = DataLoader(val_dataset, batch_size=batch_size)

        # Training
        criterion = nn.CrossEntropyLoss()
        optimizer = optim.Adam(self._model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, mode="max", patience=10
        )

        best_accuracy = 0.0
        for epoch in range(epochs):
            # Training phase
            self._model.train()
            for batch_X, batch_y in train_loader:
                batch_X = batch_X.to(self._device)
                batch_y = batch_y.to(self._device)

                optimizer.zero_grad()
                outputs = self._model(batch_X)
                loss = criterion(outputs, batch_y)
                loss.backward()
                optimizer.step()

            # Validation phase
            self._model.eval()
            correct = 0
            total = 0
            with torch.no_grad():
                for batch_X, batch_y in val_loader:
                    batch_X = batch_X.to(self._device)
                    batch_y = batch_y.to(self._device)
                    outputs = self._model(batch_X)
                    _, predicted = torch.max(outputs, 1)
                    total += batch_y.size(0)
                    correct += (predicted == batch_y).sum().item()

            accuracy = correct / total
            scheduler.step(accuracy)
            best_accuracy = max(best_accuracy, accuracy)

        self._is_trained = True
        training_duration = time.time() - start_time

        # Calculate confusion matrix
        self._model.eval()
        all_preds = []
        all_labels = []
        with torch.no_grad():
            for batch_X, batch_y in val_loader:
                batch_X = batch_X.to(self._device)
                outputs = self._model(batch_X)
                _, predicted = torch.max(outputs, 1)
                all_preds.extend(predicted.cpu().numpy())
                all_labels.extend(batch_y.numpy())

        from sklearn.metrics import confusion_matrix

        conf_matrix = confusion_matrix(all_labels, all_preds)

        return TrainingResult(
            success=True,
            accuracy=best_accuracy,
            gesture_metrics={},
            confusion_matrix=conf_matrix,
            training_duration_sec=training_duration,
            model_size_bytes=sum(
                p.numel() * p.element_size() for p in self._model.parameters()
            ),
        )

    def predict(self, features: FeatureVector) -> ClassificationResult:
        """Classify a single feature vector."""
        if not self._is_trained:
            return ClassificationResult(
                gesture=Gesture.REST,
                confidence=0.0,
                alternatives=[],
                latency_ms=0.0,
                timestamp=features.timestamp,
            )

        start_time = time.time()

        if self.use_onnx and self._onnx_session is not None:
            return self._predict_onnx(features, start_time)

        return self._predict_torch(features, start_time)

    def _predict_torch(
        self, features: FeatureVector, start_time: float
    ) -> ClassificationResult:
        """Predict using PyTorch model."""
        import torch
        import torch.nn.functional as F

        self._model.eval()
        with torch.no_grad():
            X = torch.FloatTensor(features.features).unsqueeze(0).to(self._device)
            outputs = self._model(X)
            proba = F.softmax(outputs, dim=1)[0].cpu().numpy()

        pred_idx = int(np.argmax(proba))
        gesture = self._classes[pred_idx]
        confidence = float(proba[pred_idx])

        # Build alternatives
        sorted_indices = np.argsort(proba)[::-1]
        alternatives = [
            (self._classes[idx], float(proba[idx]))
            for idx in sorted_indices[1:4]
        ]

        latency_ms = (time.time() - start_time) * 1000

        return ClassificationResult(
            gesture=gesture,
            confidence=confidence,
            alternatives=alternatives,
            latency_ms=latency_ms,
            timestamp=features.timestamp,
        )

    def _predict_onnx(
        self, features: FeatureVector, start_time: float
    ) -> ClassificationResult:
        """Predict using ONNX runtime."""
        X = features.features.reshape(1, -1).astype(np.float32)
        outputs = self._onnx_session.run(None, {"input": X})
        proba = self._softmax(outputs[0][0])

        pred_idx = int(np.argmax(proba))
        gesture = self._classes[pred_idx]
        confidence = float(proba[pred_idx])

        sorted_indices = np.argsort(proba)[::-1]
        alternatives = [
            (self._classes[idx], float(proba[idx]))
            for idx in sorted_indices[1:4]
        ]

        latency_ms = (time.time() - start_time) * 1000

        return ClassificationResult(
            gesture=gesture,
            confidence=confidence,
            alternatives=alternatives,
            latency_ms=latency_ms,
            timestamp=features.timestamp,
        )

    def _softmax(self, x: NDArray) -> NDArray:
        """Compute softmax."""
        exp_x = np.exp(x - np.max(x))
        return exp_x / exp_x.sum()

    def predict_proba(self, features: FeatureVector) -> Dict[Gesture, float]:
        """Get probability distribution over gestures."""
        result = self.predict(features)
        proba = {result.gesture: result.confidence}
        for gesture, conf in result.alternatives:
            proba[gesture] = conf
        return proba

    def save(self, path: str) -> None:
        """Save model to file."""
        import torch

        if not self._is_trained:
            raise ValueError("Model is not trained")

        torch.save(
            {
                "model_state_dict": self._model.state_dict(),
                "classes": self._classes,
                "config": {
                    "input_channels": self.input_channels,
                    "num_classes": self.num_classes,
                    "hidden_dim": self.hidden_dim,
                    "dropout": self.dropout,
                },
            },
            path,
        )

    def load(self, path: str) -> None:
        """Load model from file."""
        import torch

        checkpoint = torch.load(path, map_location=self._device)

        config = checkpoint["config"]
        self.input_channels = config["input_channels"]
        self.num_classes = config["num_classes"]
        self.hidden_dim = config["hidden_dim"]
        self.dropout = config["dropout"]
        self._classes = checkpoint["classes"]

        self._model = self._build_model()
        self._model.load_state_dict(checkpoint["model_state_dict"])
        self._model.to(self._device)
        self._model.eval()
        self._is_trained = True

    def export_onnx(self, path: str, input_dim: int) -> None:
        """Export model to ONNX format."""
        import torch

        if not self._is_trained:
            raise ValueError("Model is not trained")

        self._model.eval()
        dummy_input = torch.randn(1, input_dim)

        torch.onnx.export(
            self._model,
            dummy_input,
            path,
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
            opset_version=11,
        )

    def load_onnx(self, path: str, classes: List[Gesture]) -> None:
        """Load ONNX model for inference."""
        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError("onnxruntime is required for ONNX inference")

        self._onnx_session = ort.InferenceSession(path)
        self._classes = classes
        self.use_onnx = True
        self._is_trained = True
