"""
WIA Eye Gaze Standard - Eye Tracker Implementation

弘益人間 - 널리 인간을 이롭게
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Set, List
import time
import uuid

from .types import (
    GazePoint,
    GazeEvent,
    GazeEventType,
    EyeTrackerCapabilities,
    TrackerStatus,
    TrackerState,
    CalibrationPoint,
    CalibrationResult,
    CalibrationQuality,
    Subscription,
    GazeCallback,
    EventCallback,
)


class IEyeTrackerAdapter(ABC):
    """Abstract base class for eye tracker adapters"""

    @property
    @abstractmethod
    def name(self) -> str:
        """Adapter name"""
        pass

    @abstractmethod
    def can_handle(self, device_info: dict) -> bool:
        """Check if adapter can handle device"""
        pass

    @abstractmethod
    async def initialize(self, options: Optional[dict] = None) -> None:
        """Initialize adapter"""
        pass

    @abstractmethod
    async def connect_native(self) -> None:
        """Connect to native device"""
        pass

    @abstractmethod
    async def disconnect_native(self) -> None:
        """Disconnect from native device"""
        pass

    @abstractmethod
    def start_native_stream(self, callback: GazeCallback) -> None:
        """Start native data streaming"""
        pass

    @abstractmethod
    def stop_native_stream(self) -> None:
        """Stop native data streaming"""
        pass

    @abstractmethod
    def get_capabilities(self) -> EyeTrackerCapabilities:
        """Get device capabilities"""
        pass

    @abstractmethod
    async def calibrate(self, points: List[CalibrationPoint]) -> CalibrationResult:
        """Perform calibration"""
        pass

    @abstractmethod
    def convert_gaze_data(self, native_data: dict) -> GazePoint:
        """Convert native data to GazePoint"""
        pass

    @abstractmethod
    def dispose(self) -> None:
        """Dispose resources"""
        pass


class IEyeTracker(ABC):
    """Abstract base class for eye trackers"""

    @abstractmethod
    async def connect(self) -> None:
        """Connect to device"""
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from device"""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check connection status"""
        pass

    @abstractmethod
    async def start_calibration(
        self, points: Optional[List[CalibrationPoint]] = None
    ) -> CalibrationResult:
        """Start calibration"""
        pass

    @abstractmethod
    def subscribe(self, callback: GazeCallback) -> Subscription:
        """Subscribe to gaze data"""
        pass

    @abstractmethod
    def unsubscribe(self, subscription: Subscription) -> None:
        """Unsubscribe from gaze data"""
        pass

    @abstractmethod
    def start_tracking(self) -> None:
        """Start tracking"""
        pass

    @abstractmethod
    def stop_tracking(self) -> None:
        """Stop tracking"""
        pass

    @abstractmethod
    def on(self, event: GazeEventType, handler: EventCallback) -> None:
        """Register event handler"""
        pass

    @abstractmethod
    def off(self, event: GazeEventType, handler: EventCallback) -> None:
        """Remove event handler"""
        pass

    @abstractmethod
    def get_capabilities(self) -> EyeTrackerCapabilities:
        """Get device capabilities"""
        pass

    @abstractmethod
    def get_status(self) -> TrackerStatus:
        """Get device status"""
        pass


class WiaEyeTracker(IEyeTracker):
    """WIA Eye Tracker implementation"""

    def __init__(
        self,
        adapter: IEyeTrackerAdapter,
        smoothing: bool = False,
        smoothing_factor: float = 0.3,
        auto_reconnect: bool = True,
    ):
        self._adapter = adapter
        self._smoothing = smoothing
        self._smoothing_factor = smoothing_factor
        self._auto_reconnect = auto_reconnect

        self._status = TrackerStatus(
            state=TrackerState.DISCONNECTED,
            connected=False,
            tracking=False,
            calibrated=False,
        )

        self._subscriptions: Dict[str, GazeCallback] = {}
        self._event_handlers: Dict[GazeEventType, Set[EventCallback]] = {
            event_type: set() for event_type in GazeEventType
        }
        self._latest_gaze: Optional[GazePoint] = None
        self._calibration_result: Optional[CalibrationResult] = None
        self._subscription_counter = 0

    async def connect(self) -> None:
        if self._status.connected:
            return

        self._update_status(state=TrackerState.CONNECTING)

        try:
            await self._adapter.connect_native()
            self._update_status(state=TrackerState.CONNECTED, connected=True)
            self._emit(GazeEventType.DEVICE_CONNECTED, self._create_system_event(GazeEventType.DEVICE_CONNECTED))
        except Exception as e:
            self._update_status(state=TrackerState.ERROR, error=str(e))
            raise

    async def disconnect(self) -> None:
        if not self._status.connected:
            return

        self.stop_tracking()
        await self._adapter.disconnect_native()

        self._update_status(
            state=TrackerState.DISCONNECTED,
            connected=False,
            tracking=False,
        )
        self._emit(GazeEventType.DEVICE_DISCONNECTED, self._create_system_event(GazeEventType.DEVICE_DISCONNECTED))

    def is_connected(self) -> bool:
        return self._status.connected

    async def start_calibration(
        self, points: Optional[List[CalibrationPoint]] = None
    ) -> CalibrationResult:
        if not self._status.connected:
            raise RuntimeError("Tracker not connected")

        self._update_status(state=TrackerState.CALIBRATING)
        self._emit(GazeEventType.CALIBRATION_START, self._create_system_event(GazeEventType.CALIBRATION_START))

        calibration_points = points or self._generate_default_calibration_points()

        try:
            self._calibration_result = await self._adapter.calibrate(calibration_points)
            self._update_status(
                state=TrackerState.CONNECTED,
                calibrated=self._calibration_result.success,
            )
            self._emit(GazeEventType.CALIBRATION_END, GazeEvent(
                type=GazeEventType.CALIBRATION_END,
                timestamp=int(time.time() * 1000),
                event_id=self._generate_event_id(),
                metadata={"result": self._calibration_result},
            ))
            return self._calibration_result
        except Exception:
            self._update_status(state=TrackerState.CONNECTED)
            raise

    def get_calibration_quality(self) -> Optional[CalibrationQuality]:
        if not self._calibration_result or not self._calibration_result.success:
            return None

        accuracy = self._calibration_result.average_accuracy or 1.0

        if accuracy < 0.5:
            overall = "excellent"
        elif accuracy < 1.0:
            overall = "good"
        elif accuracy < 2.0:
            overall = "fair"
        else:
            overall = "poor"

        coverage = 1.0
        if self._calibration_result.point_results:
            valid_count = sum(1 for p in self._calibration_result.point_results if p.valid)
            coverage = valid_count / len(self._calibration_result.point_results)

        return CalibrationQuality(
            overall=overall,
            accuracy=accuracy,
            precision=self._calibration_result.average_precision or 0.1,
            coverage=coverage,
        )

    def subscribe(self, callback: GazeCallback) -> Subscription:
        self._subscription_counter += 1
        sub_id = f"sub_{self._subscription_counter}"
        self._subscriptions[sub_id] = callback

        def unsubscribe():
            self._subscriptions.pop(sub_id, None)

        return Subscription(id=sub_id, unsubscribe=unsubscribe)

    def unsubscribe(self, subscription: Subscription) -> None:
        subscription.unsubscribe()

    def start_tracking(self) -> None:
        if not self._status.connected:
            raise RuntimeError("Tracker not connected")

        if self._status.tracking:
            return

        self._adapter.start_native_stream(self._handle_gaze_data)
        self._update_status(state=TrackerState.TRACKING, tracking=True)

    def stop_tracking(self) -> None:
        if not self._status.tracking:
            return

        self._adapter.stop_native_stream()
        self._update_status(state=TrackerState.CONNECTED, tracking=False)

    def is_tracking(self) -> bool:
        return self._status.tracking

    def on(self, event: GazeEventType, handler: EventCallback) -> None:
        self._event_handlers[event].add(handler)

    def off(self, event: GazeEventType, handler: EventCallback) -> None:
        self._event_handlers[event].discard(handler)

    def once(self, event: GazeEventType, handler: EventCallback) -> None:
        def once_handler(e: GazeEvent):
            self.off(event, once_handler)
            handler(e)
        self.on(event, once_handler)

    def get_capabilities(self) -> EyeTrackerCapabilities:
        return self._adapter.get_capabilities()

    def get_status(self) -> TrackerStatus:
        return TrackerStatus(
            state=self._status.state,
            connected=self._status.connected,
            tracking=self._status.tracking,
            calibrated=self._status.calibrated,
            error=self._status.error,
            battery_level=self._status.battery_level,
            user_present=self._status.user_present,
        )

    def get_latest_gaze(self) -> Optional[GazePoint]:
        return self._latest_gaze

    def dispose(self) -> None:
        self.stop_tracking()
        if self._status.connected:
            import asyncio
            try:
                loop = asyncio.get_event_loop()
                loop.run_until_complete(self._adapter.disconnect_native())
            except RuntimeError:
                pass
        self._adapter.dispose()
        self._subscriptions.clear()
        for handlers in self._event_handlers.values():
            handlers.clear()

    # Private methods

    def _handle_gaze_data(self, gaze_point: GazePoint) -> None:
        processed = self._apply_smoothing(gaze_point) if self._smoothing else gaze_point
        self._latest_gaze = processed

        for callback in self._subscriptions.values():
            try:
                callback(processed)
            except Exception as e:
                print(f"Error in gaze subscriber: {e}")

    def _apply_smoothing(self, current: GazePoint) -> GazePoint:
        if not self._latest_gaze or not current.valid:
            return current

        factor = self._smoothing_factor
        return GazePoint(
            timestamp=current.timestamp,
            x=self._latest_gaze.x * (1 - factor) + current.x * factor,
            y=self._latest_gaze.y * (1 - factor) + current.y * factor,
            confidence=current.confidence,
            valid=current.valid,
            left_eye=current.left_eye,
            right_eye=current.right_eye,
            fixation=current.fixation,
            saccade=current.saccade,
            fixation_id=current.fixation_id,
            device_timestamp=current.device_timestamp,
            metadata=current.metadata,
        )

    def _update_status(self, **kwargs) -> None:
        self._status = TrackerStatus(
            state=kwargs.get("state", self._status.state),
            connected=kwargs.get("connected", self._status.connected),
            tracking=kwargs.get("tracking", self._status.tracking),
            calibrated=kwargs.get("calibrated", self._status.calibrated),
            error=kwargs.get("error", self._status.error),
            battery_level=kwargs.get("battery_level", self._status.battery_level),
            user_present=kwargs.get("user_present", self._status.user_present),
        )

    def _emit(self, event_type: GazeEventType, event: GazeEvent) -> None:
        for handler in self._event_handlers[event_type]:
            try:
                handler(event)
            except Exception as e:
                print(f"Error in event handler for {event_type}: {e}")

    def _create_system_event(self, event_type: GazeEventType) -> GazeEvent:
        return GazeEvent(
            type=event_type,
            timestamp=int(time.time() * 1000),
            event_id=self._generate_event_id(),
        )

    def _generate_event_id(self) -> str:
        return f"evt_{int(time.time() * 1000)}_{uuid.uuid4().hex[:7]}"

    def _generate_default_calibration_points(self) -> List[CalibrationPoint]:
        return [
            CalibrationPoint(x=0.5, y=0.5, index=0),
            CalibrationPoint(x=0.1, y=0.1, index=1),
            CalibrationPoint(x=0.9, y=0.1, index=2),
            CalibrationPoint(x=0.1, y=0.9, index=3),
            CalibrationPoint(x=0.9, y=0.9, index=4),
        ]


def create_eye_tracker(
    adapter: IEyeTrackerAdapter,
    smoothing: bool = False,
    smoothing_factor: float = 0.3,
    auto_reconnect: bool = True,
) -> IEyeTracker:
    """Create an eye tracker with the given adapter"""
    return WiaEyeTracker(
        adapter=adapter,
        smoothing=smoothing,
        smoothing_factor=smoothing_factor,
        auto_reconnect=auto_reconnect,
    )
