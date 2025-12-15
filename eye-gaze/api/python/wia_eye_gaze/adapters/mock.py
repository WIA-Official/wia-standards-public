"""
WIA Eye Gaze Standard - Mock Adapter

Mock adapter for testing and development.

弘益人間 - 널리 인간을 이롭게
"""

import asyncio
import random
import time
from typing import Optional, List, Callable

from ..tracker import IEyeTrackerAdapter
from ..types import (
    GazePoint,
    GazeCallback,
    EyeTrackerCapabilities,
    EyeTrackerInfo,
    TrackingCapabilities,
    DataCapabilities,
    CalibrationCapabilities,
    ConnectivityCapabilities,
    SamplingRateSpec,
    AccuracySpec,
    PrecisionSpec,
    LatencySpec,
    RangeSpec,
    TrackingAreaSpec,
    CalibrationPoint,
    CalibrationResult,
    PointCalibrationResult,
    Vector2D,
    EyeData,
    DeviceType,
    CalibrationType,
    ConnectionType,
    ApiProtocol,
)


class MockAdapter(IEyeTrackerAdapter):
    """Mock eye tracker adapter for testing"""

    def __init__(
        self,
        sampling_rate: int = 60,
        add_noise: bool = True,
        noise_level: float = 0.01,
        simulate_blinks: bool = True,
        blink_interval: int = 4000,
    ):
        self._sampling_rate = sampling_rate
        self._add_noise = add_noise
        self._noise_level = noise_level
        self._simulate_blinks = simulate_blinks
        self._blink_interval = blink_interval

        self._connected = False
        self._streaming = False
        self._callback: Optional[GazeCallback] = None
        self._stream_task: Optional[asyncio.Task] = None

        self._current_x = 0.5
        self._current_y = 0.5
        self._blink_state = False
        self._last_blink_time = 0

    @property
    def name(self) -> str:
        return "mock"

    def can_handle(self, device_info: dict) -> bool:
        return True

    async def initialize(self, options: Optional[dict] = None) -> None:
        pass

    async def connect_native(self) -> None:
        await asyncio.sleep(0.1)
        self._connected = True

    async def disconnect_native(self) -> None:
        self.stop_native_stream()
        self._connected = False

    def start_native_stream(self, callback: GazeCallback) -> None:
        if self._streaming:
            return

        self._callback = callback
        self._streaming = True

        # Start streaming in background
        try:
            loop = asyncio.get_event_loop()
            self._stream_task = loop.create_task(self._stream_loop())
        except RuntimeError:
            # No event loop - use threading fallback
            import threading
            self._stream_thread = threading.Thread(target=self._stream_thread_loop)
            self._stream_thread.daemon = True
            self._stream_thread.start()

    def stop_native_stream(self) -> None:
        self._streaming = False
        if self._stream_task:
            self._stream_task.cancel()
            self._stream_task = None
        self._callback = None

    async def _stream_loop(self) -> None:
        interval = 1.0 / self._sampling_rate
        while self._streaming and self._callback:
            self._callback(self._generate_gaze_point())
            await asyncio.sleep(interval)

    def _stream_thread_loop(self) -> None:
        interval = 1.0 / self._sampling_rate
        while self._streaming and self._callback:
            self._callback(self._generate_gaze_point())
            time.sleep(interval)

    def get_capabilities(self) -> EyeTrackerCapabilities:
        return EyeTrackerCapabilities(
            device=EyeTrackerInfo(
                device_id="mock-tracker-001",
                vendor="WIA",
                model="Mock Eye Tracker",
                firmware_version="1.0.0",
                protocol_version="1.0.0",
                device_type=DeviceType.WEBCAM_BASED,
            ),
            tracking=TrackingCapabilities(
                binocular=True,
                head_tracking=False,
                gaze_3d=False,
                sampling_rate=SamplingRateSpec(
                    supported=[30, 60, 120],
                    default=60,
                    current=self._sampling_rate,
                ),
                accuracy=AccuracySpec(typical=1.0),
                precision=PrecisionSpec(typical=0.5),
                latency=LatencySpec(average=20),
                operating_distance=RangeSpec(min=40, max=90),
                tracking_area=TrackingAreaSpec(horizontal=40, vertical=30),
            ),
            data=DataCapabilities(
                gaze_point=True,
                eye_data=True,
                pupil_diameter=True,
                pupil_position=False,
                eye_openness=True,
                eye_images=False,
                gaze_origin_3d=False,
                gaze_direction_3d=False,
                built_in_fixation_detection=False,
                built_in_saccade_detection=False,
                built_in_blink_detection=False,
                device_timestamp=True,
                system_timestamp=True,
                external_sync=False,
            ),
            calibration=CalibrationCapabilities(
                required=False,
                types=[CalibrationType.QUICK],
                point_options=[1, 5],
                default_points=5,
                auto_calibration=True,
                profile_management=False,
                quality_assessment=True,
                adaptive_calibration=False,
            ),
            connectivity=ConnectivityCapabilities(
                connection_types=[ConnectionType.USB],
                api_protocols=[ApiProtocol.WIA_STANDARD],
                multi_client=True,
                remote_connection=False,
                mobile_connection=False,
            ),
            supported_features=[
                "GAZE_POINT",
                "BINOCULAR",
                "PUPIL_DIAMETER",
                "EYE_OPENNESS",
                "CALIBRATION",
                "DEVICE_TIMESTAMP",
            ],
        )

    async def calibrate(self, points: List[CalibrationPoint]) -> CalibrationResult:
        await asyncio.sleep(0.5)
        return CalibrationResult(
            success=True,
            timestamp=int(time.time() * 1000),
            average_accuracy=0.8 + random.random() * 0.4,
            average_precision=0.1 + random.random() * 0.1,
            point_results=[
                PointCalibrationResult(
                    point_index=i,
                    position=Vector2D(x=p.x, y=p.y),
                    accuracy=0.5 + random.random() * 0.5,
                    precision=0.05 + random.random() * 0.1,
                    valid=True,
                )
                for i, p in enumerate(points)
            ],
        )

    def convert_gaze_data(self, native_data: dict) -> GazePoint:
        return GazePoint(**native_data)

    def dispose(self) -> None:
        self.stop_native_stream()
        self._connected = False

    def _generate_gaze_point(self) -> GazePoint:
        now = int(time.time() * 1000)

        # Check for blink
        if self._simulate_blinks:
            if now - self._last_blink_time > self._blink_interval:
                self._blink_state = True
                self._last_blink_time = now

                def reset_blink():
                    time.sleep(0.15)
                    self._blink_state = False

                import threading
                threading.Thread(target=reset_blink, daemon=True).start()

        if self._blink_state:
            return self._create_blink_point(now)

        x = self._current_x
        y = self._current_y

        # Add noise
        if self._add_noise:
            x += (random.random() - 0.5) * self._noise_level * 2
            y += (random.random() - 0.5) * self._noise_level * 2

        # Simulate smooth movement
        self._current_x += (random.random() - 0.5) * 0.02
        self._current_y += (random.random() - 0.5) * 0.02
        self._current_x = max(0.1, min(0.9, self._current_x))
        self._current_y = max(0.1, min(0.9, self._current_y))

        pupil_diameter = 3.5 + random.random() * 1.0
        eye_openness = 0.85 + random.random() * 0.1

        return GazePoint(
            timestamp=now,
            x=max(0, min(1, x)),
            y=max(0, min(1, y)),
            confidence=0.9 + random.random() * 0.1,
            valid=True,
            left_eye=EyeData(
                gaze=Vector2D(x=x - 0.005, y=y),
                valid=True,
                pupil_diameter=pupil_diameter,
                eye_openness=eye_openness,
            ),
            right_eye=EyeData(
                gaze=Vector2D(x=x + 0.005, y=y),
                valid=True,
                pupil_diameter=pupil_diameter + 0.1,
                eye_openness=eye_openness,
            ),
            device_timestamp=now,
        )

    def _create_blink_point(self, timestamp: int) -> GazePoint:
        return GazePoint(
            timestamp=timestamp,
            x=self._current_x,
            y=self._current_y,
            confidence=0,
            valid=False,
            left_eye=EyeData(
                gaze=Vector2D(x=self._current_x, y=self._current_y),
                valid=False,
                eye_openness=0,
            ),
            right_eye=EyeData(
                gaze=Vector2D(x=self._current_x, y=self._current_y),
                valid=False,
                eye_openness=0,
            ),
            device_timestamp=timestamp,
            metadata={"invalid_reason": "blink"},
        )
