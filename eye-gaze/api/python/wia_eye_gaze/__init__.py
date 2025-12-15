"""
WIA Eye Gaze Interoperability Protocol SDK

Standard interface for eye tracking devices and gaze-aware applications.

Example:
    >>> from wia_eye_gaze import create_eye_tracker, MockAdapter
    >>> adapter = MockAdapter(sampling_rate=60)
    >>> tracker = create_eye_tracker(adapter)
    >>> await tracker.connect()
    >>> tracker.subscribe(lambda gaze: print(f"Gaze: ({gaze.x:.2f}, {gaze.y:.2f})"))
    >>> tracker.start_tracking()

弘益人間 (홍익인간) - 널리 인간을 이롭게

License: MIT
Author: SmileStory Inc. / WIA
"""

__version__ = "1.0.0a1"
__protocol_version__ = "1.0.0"

# Types
from .types import (
    Vector2D,
    Vector3D,
    BoundingBox,
    EyeData,
    GazePoint,
    GazeEvent,
    GazeEventType,
    GazeTarget,
    TargetSemanticType,
    EyeTrackerCapabilities,
    EyeTrackerInfo,
    TrackerStatus,
    TrackerState,
    CalibrationResult,
    CalibrationQuality,
    DwellConfig,
    DwellFeedbackType,
    DwellState,
)

# Tracker
from .tracker import (
    IEyeTracker,
    IEyeTrackerAdapter,
    WiaEyeTracker,
    create_eye_tracker,
)

# Adapters
from .adapters import MockAdapter

# Dwell
from .dwell import DwellController, create_dwell_controller

# App
from .app import GazeAwareApp, create_gaze_aware_app

__all__ = [
    # Version
    "__version__",
    "__protocol_version__",
    # Types
    "Vector2D",
    "Vector3D",
    "BoundingBox",
    "EyeData",
    "GazePoint",
    "GazeEvent",
    "GazeEventType",
    "GazeTarget",
    "TargetSemanticType",
    "EyeTrackerCapabilities",
    "EyeTrackerInfo",
    "TrackerStatus",
    "TrackerState",
    "CalibrationResult",
    "CalibrationQuality",
    "DwellConfig",
    "DwellFeedbackType",
    "DwellState",
    # Tracker
    "IEyeTracker",
    "IEyeTrackerAdapter",
    "WiaEyeTracker",
    "create_eye_tracker",
    # Adapters
    "MockAdapter",
    # Dwell
    "DwellController",
    "create_dwell_controller",
    # App
    "GazeAwareApp",
    "create_gaze_aware_app",
]
