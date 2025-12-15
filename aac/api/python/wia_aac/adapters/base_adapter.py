"""
WIA AAC Base Adapter
Abstract base class for all sensor adapters
"""

import time
from abc import ABC, abstractmethod
from typing import Optional, List, Callable, Any, Dict

from ..types import (
    WiaAacSignal,
    DeviceInfo,
    SensorType,
    SensorConfig,
    SensorOptions,
    SensorOptionDescriptor,
    Timestamp,
    SignalMeta
)

# Signal handler type
SignalHandler = Callable[[WiaAacSignal], None]


class BaseSensorAdapter(ABC):
    """Abstract base class for sensor adapters"""

    def __init__(self):
        self._device_info: Optional[DeviceInfo] = None
        self._is_connected = False
        self._last_signal: Optional[WiaAacSignal] = None
        self._signal_handlers: List[SignalHandler] = []
        self._config: Optional[SensorConfig] = None
        self._options: Dict[str, Any] = {}

    @property
    @abstractmethod
    def type(self) -> str:
        """Sensor type"""
        pass

    @property
    def device_info(self) -> Optional[DeviceInfo]:
        """Device information"""
        return self._device_info

    @property
    def is_connected(self) -> bool:
        """Connection status"""
        return self._is_connected

    async def connect(self, config: SensorConfig) -> None:
        """Connect to sensor"""
        self._config = config
        self._device_info = DeviceInfo(
            manufacturer=config.device.manufacturer if config.device else "Unknown",
            model=config.device.model if config.device else self.type,
            firmware="1.0.0",
            serial=config.device.serial if config.device else None
        )
        self._is_connected = True

    async def disconnect(self) -> None:
        """Disconnect from sensor"""
        self._is_connected = False
        self._device_info = None
        self._config = None
        self._last_signal = None

    def get_last_signal(self) -> Optional[WiaAacSignal]:
        """Get last signal"""
        return self._last_signal

    def on_signal(self, handler: SignalHandler) -> None:
        """Register signal handler"""
        self._signal_handlers.append(handler)

    def off_signal(self, handler: SignalHandler) -> None:
        """Unregister signal handler"""
        if handler in self._signal_handlers:
            self._signal_handlers.remove(handler)

    def configure(self, options: SensorOptions) -> None:
        """Configure sensor options"""
        if hasattr(options, '__dict__'):
            self._options.update({k: v for k, v in options.__dict__.items() if v is not None})
        elif isinstance(options, dict):
            self._options.update(options)

    @abstractmethod
    def get_supported_options(self) -> List[SensorOptionDescriptor]:
        """Get supported options"""
        pass

    def _emit_signal(self, signal: WiaAacSignal) -> None:
        """Emit signal to handlers"""
        self._last_signal = signal
        for handler in self._signal_handlers:
            try:
                handler(signal)
            except Exception as e:
                print(f"Error in signal handler: {e}")

    def _create_signal(
        self,
        sensor_type: str,
        data: Dict[str, Any],
        confidence: float = 0.9
    ) -> WiaAacSignal:
        """Create a signal object"""
        now_ms = int(time.time() * 1000)
        return WiaAacSignal(
            version="1.0.0",
            type=SensorType(sensor_type) if sensor_type in [e.value for e in SensorType] else SensorType.CUSTOM,
            timestamp=Timestamp(
                unix_ms=now_ms,
                iso8601=time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(now_ms / 1000))
            ),
            device=self._device_info or DeviceInfo(manufacturer="Unknown", model=sensor_type),
            data=data,
            sequence=now_ms,
            meta=SignalMeta(confidence=confidence, validity=True)
        )
