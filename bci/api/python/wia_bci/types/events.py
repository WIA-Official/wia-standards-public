"""WIA BCI Event Types."""

from dataclasses import dataclass, field
from typing import Any, Literal, Optional

import numpy as np
from numpy.typing import NDArray

EventType = Literal[
    # Connection events
    "connected",
    "disconnected",
    "reconnecting",
    "connection_error",
    # Streaming events
    "stream_started",
    "stream_stopped",
    "data",
    "signal",
    # Marker events
    "marker",
    "trigger",
    # Classification events
    "classification",
    "prediction",
    # Device events
    "battery",
    "impedance",
    "quality",
    # Error events
    "error",
    "warning",
]


@dataclass
class SignalEvent:
    """Signal data event."""

    timestamp: float
    sample_index: int
    channels: list[int]
    data: NDArray[np.float32]


@dataclass
class MarkerEvent:
    """Marker/trigger event."""

    timestamp: float
    sample_index: int
    code: int
    label: Optional[str] = None
    value: Optional[Any] = None
    duration: float = 0.0


@dataclass
class ClassificationEvent:
    """Classification result event."""

    timestamp: float
    class_id: int
    class_name: str
    confidence: float
    probabilities: Optional[dict[str, float]] = None


@dataclass
class ChannelQuality:
    """Channel quality information."""

    channel: int
    label: str
    signal_quality: float
    artifacts: list[str] = field(default_factory=list)
    impedance: Optional[float] = None


@dataclass
class QualityEvent:
    """Signal quality event."""

    timestamp: float
    channel_qualities: list[ChannelQuality]
    overall_quality: float


@dataclass
class ErrorEvent:
    """Error event."""

    code: str
    message: str
    recoverable: bool
    details: Optional[Any] = None
