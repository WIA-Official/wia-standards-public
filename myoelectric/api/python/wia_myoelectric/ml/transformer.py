"""
Transformer-based classifier for EMG gesture recognition.
"""

import math
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


class TransformerClassifier(GestureClassifier):
    """
    Transformer-based classifier for gesture recognition.

    Uses a simplified transformer architecture suitable for
    real-time EMG signal classification.
    """

    def __init__(
        self,
        input_dim: int = 20,
        num_classes: int = 5,
        d_model: int = 64,
        nhead: int = 4,
        num_encoder_layers: int = 2,
        dim_feedforward: int = 128,
        dropout: float = 0.1,
        max_seq_length: int = 10,
        confidence_threshold: float = 0.7,
        majority_voting_frames: int = 3,
    ):
        """
        Initialize Transformer classifier.

        Args:
            input_dim: Input feature dimension
            num_classes: Number of gesture classes
            d_model: Transformer model dimension
            nhead: Number of attention heads
            num_encoder_layers: Number of encoder layers
            dim_feedforward: Feedforward network dimension
            dropout: Dropout rate
            max_seq_length: Maximum sequence length for temporal context
            confidence_threshold: Minimum confidence for valid classification
            majority_voting_frames: Number of frames for majority voting
        """
        super().__init__(confidence_threshold, majority_voting_frames)

        self.input_dim = input_dim
        self.num_classes = num_classes
        self.d_model = d_model
        self.nhead = nhead
        self.num_encoder_layers = num_encoder_layers
        self.dim_feedforward = dim_feedforward
        self.dropout = dropout
        self.max_seq_length = max_seq_length

        self._model = None
        self._device = "cpu"
        self._sequence_buffer: List[NDArray] = []

    def _build_model(self):
        """Build the Transformer model using PyTorch."""
        try:
            import torch
            import torch.nn as nn
        except ImportError:
            raise ImportError("PyTorch is required for TransformerClassifier")

        class PositionalEncoding(nn.Module):
            def __init__(self, d_model: int, max_len: int = 100, dropout: float = 0.1):
                super().__init__()
                self.dropout = nn.Dropout(p=dropout)

                pe = torch.zeros(max_len, d_model)
                position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
                div_term = torch.exp(
                    torch.arange(0, d_model, 2).float()
                    * (-math.log(10000.0) / d_model)
                )
                pe[:, 0::2] = torch.sin(position * div_term)
                pe[:, 1::2] = torch.cos(position * div_term)
                pe = pe.unsqueeze(0)
                self.register_buffer("pe", pe)

            def forward(self, x):
                x = x + self.pe[:, : x.size(1)]
                return self.dropout(x)

        class EMGTransformer(nn.Module):
            def __init__(
                self,
                input_dim: int,
                num_classes: int,
                d_model: int,
                nhead: int,
                num_encoder_layers: int,
                dim_feedforward: int,
                dropout: float,
                max_seq_length: int,
            ):
                super().__init__()

                self.input_projection = nn.Linear(input_dim, d_model)
                self.pos_encoder = PositionalEncoding(d_model, max_seq_length, dropout)

                encoder_layer = nn.TransformerEncoderLayer(
                    d_model=d_model,
                    nhead=nhead,
                    dim_feedforward=dim_feedforward,
                    dropout=dropout,
                    batch_first=True,
                )
                self.transformer_encoder = nn.TransformerEncoder(
                    encoder_layer, num_layers=num_encoder_layers
                )

                self.classifier = nn.Sequential(
                    nn.Linear(d_model, d_model // 2),
                    nn.ReLU(),
                    nn.Dropout(dropout),
                    nn.Linear(d_model // 2, num_classes),
                )

                self.d_model = d_model

            def forward(self, x):
                # x shape: (batch, seq_len, input_dim)
                x = self.input_projection(x) * math.sqrt(self.d_model)
                x = self.pos_encoder(x)
                x = self.transformer_encoder(x)

                # Use CLS token (first position) or mean pooling
                x = x.mean(dim=1)  # Mean pooling over sequence

                x = self.classifier(x)
                return x

        return EMGTransformer(
            self.input_dim,
            self.num_classes,
            self.d_model,
            self.nhead,
            self.num_encoder_layers,
            self.dim_feedforward,
            self.dropout,
            self.max_seq_length,
        )

    def train(
        self,
        X: NDArray[np.float32],
        y: NDArray[np.int32],
        gestures: List[Gesture],
        epochs: int = 100,
        batch_size: int = 32,
        learning_rate: float = 0.0001,
        validation_split: float = 0.2,
        sequence_length: int = 5,
    ) -> TrainingResult:
        """
        Train the Transformer classifier.

        Args:
            X: Feature matrix (n_samples, n_features)
            y: Labels (n_samples,)
            gestures: List of gesture classes
            epochs: Number of training epochs
            batch_size: Batch size
            learning_rate: Learning rate
            validation_split: Validation split ratio
            sequence_length: Number of frames per sequence
        """
        try:
            import torch
            import torch.nn as nn
            import torch.optim as optim
            from torch.utils.data import DataLoader, TensorDataset
        except ImportError:
            raise ImportError("PyTorch is required for TransformerClassifier")

        start_time = time.time()

        self._classes = gestures
        self.num_classes = len(gestures)
        self.input_dim = X.shape[1]

        # Create sequences from samples
        X_seq, y_seq = self._create_sequences(X, y, sequence_length)

        # Build model
        self._model = self._build_model()
        self._model.to(self._device)

        # Prepare data
        X_tensor = torch.FloatTensor(X_seq)
        y_tensor = torch.LongTensor(y_seq)

        # Split into train/validation
        n_val = int(len(X_seq) * validation_split)
        indices = np.random.permutation(len(X_seq))
        train_indices = indices[n_val:]
        val_indices = indices[:n_val]

        train_dataset = TensorDataset(
            X_tensor[train_indices], y_tensor[train_indices]
        )
        val_dataset = TensorDataset(X_tensor[val_indices], y_tensor[val_indices])

        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        val_loader = DataLoader(val_dataset, batch_size=batch_size)

        # Training
        criterion = nn.CrossEntropyLoss()
        optimizer = optim.AdamW(self._model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)

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
                torch.nn.utils.clip_grad_norm_(self._model.parameters(), 1.0)
                optimizer.step()

            scheduler.step()

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

            accuracy = correct / total if total > 0 else 0
            best_accuracy = max(best_accuracy, accuracy)

        self._is_trained = True
        training_duration = time.time() - start_time

        return TrainingResult(
            success=True,
            accuracy=best_accuracy,
            gesture_metrics={},
            confusion_matrix=np.array([[]]),
            training_duration_sec=training_duration,
            model_size_bytes=sum(
                p.numel() * p.element_size() for p in self._model.parameters()
            ),
        )

    def _create_sequences(
        self,
        X: NDArray[np.float32],
        y: NDArray[np.int32],
        seq_length: int,
    ) -> tuple:
        """Create sequences from individual samples."""
        X_seq = []
        y_seq = []

        for i in range(len(X) - seq_length + 1):
            X_seq.append(X[i : i + seq_length])
            y_seq.append(y[i + seq_length - 1])  # Label of last frame

        return np.array(X_seq), np.array(y_seq)

    def predict(self, features: FeatureVector) -> ClassificationResult:
        """Classify using temporal context."""
        if not self._is_trained:
            return ClassificationResult(
                gesture=Gesture.REST,
                confidence=0.0,
                alternatives=[],
                latency_ms=0.0,
                timestamp=features.timestamp,
            )

        start_time = time.time()

        # Add to sequence buffer
        self._sequence_buffer.append(features.features)
        if len(self._sequence_buffer) > self.max_seq_length:
            self._sequence_buffer.pop(0)

        # Need minimum sequence length
        if len(self._sequence_buffer) < 3:
            return ClassificationResult(
                gesture=Gesture.REST,
                confidence=0.0,
                alternatives=[],
                latency_ms=(time.time() - start_time) * 1000,
                timestamp=features.timestamp,
            )

        return self._predict_torch(features, start_time)

    def _predict_torch(
        self, features: FeatureVector, start_time: float
    ) -> ClassificationResult:
        """Predict using PyTorch model."""
        import torch
        import torch.nn.functional as F

        self._model.eval()
        with torch.no_grad():
            # Create sequence tensor
            seq = np.array(self._sequence_buffer)
            X = torch.FloatTensor(seq).unsqueeze(0).to(self._device)

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

    def predict_proba(self, features: FeatureVector) -> Dict[Gesture, float]:
        """Get probability distribution over gestures."""
        result = self.predict(features)
        proba = {result.gesture: result.confidence}
        for gesture, conf in result.alternatives:
            proba[gesture] = conf
        return proba

    def reset_sequence(self) -> None:
        """Reset the sequence buffer."""
        self._sequence_buffer = []

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
                    "input_dim": self.input_dim,
                    "num_classes": self.num_classes,
                    "d_model": self.d_model,
                    "nhead": self.nhead,
                    "num_encoder_layers": self.num_encoder_layers,
                    "dim_feedforward": self.dim_feedforward,
                    "dropout": self.dropout,
                    "max_seq_length": self.max_seq_length,
                },
            },
            path,
        )

    def load(self, path: str) -> None:
        """Load model from file."""
        import torch

        checkpoint = torch.load(path, map_location=self._device)

        config = checkpoint["config"]
        self.input_dim = config["input_dim"]
        self.num_classes = config["num_classes"]
        self.d_model = config["d_model"]
        self.nhead = config["nhead"]
        self.num_encoder_layers = config["num_encoder_layers"]
        self.dim_feedforward = config["dim_feedforward"]
        self.dropout = config["dropout"]
        self.max_seq_length = config["max_seq_length"]
        self._classes = checkpoint["classes"]

        self._model = self._build_model()
        self._model.load_state_dict(checkpoint["model_state_dict"])
        self._model.to(self._device)
        self._model.eval()
        self._is_trained = True

    def export_onnx(self, path: str) -> None:
        """Export model to ONNX format."""
        import torch

        if not self._is_trained:
            raise ValueError("Model is not trained")

        self._model.eval()
        dummy_input = torch.randn(1, self.max_seq_length, self.input_dim)

        torch.onnx.export(
            self._model,
            dummy_input,
            path,
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={
                "input": {0: "batch_size", 1: "seq_length"},
                "output": {0: "batch_size"},
            },
            opset_version=11,
        )
