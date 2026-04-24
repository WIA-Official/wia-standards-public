"""
WIA Haptic Standard - Device Abstraction Layer
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Callable, Any, Union
import asyncio

from .types import (
    BodyLocation,
    ActuatorType,
    DeviceType,
    ConnectionState,
    HapticErrorCode,
)
from .patterns import HapticPattern, HapticPrimitive, HapticSequence, SpatialPattern


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class HapticCapabilities:
    """Device capabilities"""
    actuator_type: ActuatorType
    frequency_range: tuple  # (min, max) in Hz
    locations: List[BodyLocation]
    max_intensity: float = 1.0
    latency: float = 20  # ms
    actuator_count: int = 1
    supports_custom_waveforms: bool = False
    supports_amplitude_modulation: bool = True
    supports_frequency_modulation: bool = False
    battery_level: Optional[int] = None  # 0-100 or None


@dataclass
class UserCalibration:
    """User-specific calibration settings"""
    perception_thresholds: Dict[BodyLocation, float] = field(default_factory=dict)
    comfort_maximum: Dict[BodyLocation, float] = field(default_factory=dict)
    unavailable_locations: List[BodyLocation] = field(default_factory=list)


@dataclass
class DeviceConfig:
    """Device configuration"""
    device_id: Optional[str] = None
    name: Optional[str] = None
    connection_timeout: float = 10000  # ms
    auto_reconnect: bool = True
    intensity_scale: float = 1.0
    calibration: Optional[UserCalibration] = None


@dataclass
class HapticDeviceEvent:
    """Event from haptic device"""
    type: str  # connected, disconnected, error, battery, pattern_complete
    timestamp: float
    data: Any = None


# Type alias for event listeners
HapticEventListener = Callable[[HapticDeviceEvent], None]


# ============================================================================
# Exceptions
# ============================================================================

class HapticError(Exception):
    """Haptic operation error"""

    def __init__(
        self,
        message: str,
        code: HapticErrorCode = HapticErrorCode.UNKNOWN,
        details: Any = None
    ):
        super().__init__(message)
        self.code = code
        self.details = details


# ============================================================================
# Abstract Device Interface
# ============================================================================

class IHapticDevice(ABC):
    """Interface for haptic devices"""

    @property
    @abstractmethod
    def state(self) -> ConnectionState:
        """Current connection state"""
        pass

    @property
    @abstractmethod
    def capabilities(self) -> Optional[HapticCapabilities]:
        """Device capabilities (available after connection)"""
        pass

    @property
    @abstractmethod
    def device_type(self) -> DeviceType:
        """Device type"""
        pass

    @property
    @abstractmethod
    def config(self) -> DeviceConfig:
        """Device configuration"""
        pass

    @abstractmethod
    async def connect(self) -> None:
        """Connect to the haptic device"""
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from the haptic device"""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if device is connected"""
        pass

    @abstractmethod
    def get_capabilities(self) -> HapticCapabilities:
        """Get device capabilities"""
        pass

    @abstractmethod
    def supports_location(self, location: BodyLocation) -> bool:
        """Check if device supports a specific location"""
        pass

    @abstractmethod
    def supports_frequency(self, frequency: float) -> bool:
        """Check if device supports a specific frequency"""
        pass

    @abstractmethod
    async def play(self, pattern: HapticPattern) -> None:
        """Play a haptic pattern"""
        pass

    @abstractmethod
    async def play_sequence(self, patterns: List[HapticPattern]) -> None:
        """Play multiple patterns in sequence"""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop all haptic playback"""
        pass

    @abstractmethod
    def pause(self) -> None:
        """Pause current playback"""
        pass

    @abstractmethod
    def resume(self) -> None:
        """Resume paused playback"""
        pass

    @abstractmethod
    def set_intensity(self, location: BodyLocation, intensity: float) -> None:
        """Set intensity for a specific location"""
        pass

    @abstractmethod
    def pulse(
        self,
        location: BodyLocation,
        duration: float,
        intensity: float = 0.5
    ) -> None:
        """Trigger a quick pulse at a location"""
        pass

    @abstractmethod
    def vibrate(
        self,
        location: BodyLocation,
        frequency: float,
        intensity: float
    ) -> None:
        """Set continuous vibration at a location"""
        pass

    @abstractmethod
    def add_event_listener(self, listener: HapticEventListener) -> None:
        """Add event listener"""
        pass

    @abstractmethod
    def remove_event_listener(self, listener: HapticEventListener) -> None:
        """Remove event listener"""
        pass


# ============================================================================
# Base Device Implementation
# ============================================================================

class BaseHapticDevice(IHapticDevice):
    """Base implementation of haptic device"""

    def __init__(
        self,
        device_type: DeviceType,
        config: Optional[DeviceConfig] = None
    ):
        self._device_type = device_type
        self._config = config or DeviceConfig()
        self._state = ConnectionState.DISCONNECTED
        self._capabilities: Optional[HapticCapabilities] = None
        self._listeners: List[HapticEventListener] = []
        self._is_playing = False
        self._is_paused = False

    @property
    def state(self) -> ConnectionState:
        return self._state

    @property
    def capabilities(self) -> Optional[HapticCapabilities]:
        return self._capabilities

    @property
    def device_type(self) -> DeviceType:
        return self._device_type

    @property
    def config(self) -> DeviceConfig:
        return self._config

    def is_connected(self) -> bool:
        return self._state == ConnectionState.CONNECTED

    def get_capabilities(self) -> HapticCapabilities:
        if not self._capabilities:
            raise HapticError("Device not connected", HapticErrorCode.NOT_CONNECTED)
        return self._capabilities

    def supports_location(self, location: BodyLocation) -> bool:
        if not self._capabilities:
            return False
        return location in self._capabilities.locations

    def supports_frequency(self, frequency: float) -> bool:
        if not self._capabilities:
            return False
        min_freq, max_freq = self._capabilities.frequency_range
        return min_freq <= frequency <= max_freq

    async def play_sequence(self, patterns: List[HapticPattern]) -> None:
        for pattern in patterns:
            await self.play(pattern)

    def pause(self) -> None:
        if self._is_playing:
            self._is_paused = True

    def resume(self) -> None:
        self._is_paused = False

    def add_event_listener(self, listener: HapticEventListener) -> None:
        if listener not in self._listeners:
            self._listeners.append(listener)

    def remove_event_listener(self, listener: HapticEventListener) -> None:
        if listener in self._listeners:
            self._listeners.remove(listener)

    def _emit(self, event: HapticDeviceEvent) -> None:
        """Emit event to all listeners"""
        for listener in self._listeners:
            try:
                listener(event)
            except Exception as e:
                print(f"Error in haptic event listener: {e}")

    def _assert_connected(self) -> None:
        """Assert device is connected"""
        if not self.is_connected():
            raise HapticError("Device not connected", HapticErrorCode.NOT_CONNECTED)

    def _scale_intensity(self, intensity: float) -> float:
        """Scale intensity by config scale factor"""
        scale = self._config.intensity_scale
        return max(0.0, min(1.0, intensity * scale))

    def _apply_calibration(
        self,
        location: BodyLocation,
        intensity: float
    ) -> float:
        """Apply user calibration to intensity"""
        calibration = self._config.calibration
        if not calibration:
            return intensity

        # Check if location is unavailable
        if location in calibration.unavailable_locations:
            return 0.0

        # Apply perception threshold
        threshold = calibration.perception_thresholds.get(location, 0.0)
        if intensity < threshold:
            return 0.0

        # Apply comfort maximum
        maximum = calibration.comfort_maximum.get(location, 1.0)
        return min(intensity, maximum)

    # Abstract methods to be implemented by subclasses
    @abstractmethod
    async def connect(self) -> None:
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        pass

    @abstractmethod
    async def play(self, pattern: HapticPattern) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def set_intensity(self, location: BodyLocation, intensity: float) -> None:
        pass

    @abstractmethod
    def pulse(
        self,
        location: BodyLocation,
        duration: float,
        intensity: float = 0.5
    ) -> None:
        pass

    @abstractmethod
    def vibrate(
        self,
        location: BodyLocation,
        frequency: float,
        intensity: float
    ) -> None:
        pass


# ============================================================================
# Device Manager
# ============================================================================

class HapticDeviceManager:
    """Manages multiple haptic devices"""

    def __init__(self):
        self._devices: Dict[str, IHapticDevice] = {}
        self._default_device: Optional[IHapticDevice] = None

    def register(self, device_id: str, device: IHapticDevice) -> None:
        """Register a device"""
        self._devices[device_id] = device
        if self._default_device is None:
            self._default_device = device

    def unregister(self, device_id: str) -> None:
        """Unregister a device"""
        device = self._devices.get(device_id)
        if device:
            if device.is_connected():
                asyncio.create_task(device.disconnect())
            del self._devices[device_id]
            if self._default_device == device:
                self._default_device = next(iter(self._devices.values()), None)

    def get(self, device_id: str) -> Optional[IHapticDevice]:
        """Get device by ID"""
        return self._devices.get(device_id)

    def get_default(self) -> Optional[IHapticDevice]:
        """Get default device"""
        return self._default_device

    def set_default(self, device_id: str) -> None:
        """Set default device"""
        device = self._devices.get(device_id)
        if device:
            self._default_device = device

    def get_all(self) -> List[IHapticDevice]:
        """Get all registered devices"""
        return list(self._devices.values())

    def get_connected(self) -> List[IHapticDevice]:
        """Get all connected devices"""
        return [d for d in self._devices.values() if d.is_connected()]

    async def connect_all(self) -> None:
        """Connect all devices"""
        tasks = [d.connect() for d in self._devices.values()]
        await asyncio.gather(*tasks, return_exceptions=True)

    async def disconnect_all(self) -> None:
        """Disconnect all devices"""
        tasks = [d.disconnect() for d in self.get_connected()]
        await asyncio.gather(*tasks, return_exceptions=True)

    async def play_on_all(self, pattern: HapticPattern) -> None:
        """Play pattern on all connected devices"""
        tasks = [d.play(pattern) for d in self.get_connected()]
        await asyncio.gather(*tasks, return_exceptions=True)

    def stop_all(self) -> None:
        """Stop all devices"""
        for device in self.get_connected():
            device.stop()


# Singleton instance
haptic_manager = HapticDeviceManager()
