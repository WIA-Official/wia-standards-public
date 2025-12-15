"""
WIA AAC Main Class
Primary API entry point for AAC sensor interaction
"""

import time
import logging
from typing import Optional, List, Callable, Any, Dict, Type

from .event_emitter import EventEmitter
from ..types import (
    WiaAacSignal,
    DeviceInfo,
    SensorType,
    EventType,
    ErrorCode,
    WiaAacError,
    SelectionEvent,
    DisconnectReason,
    SensorConfig,
    WiaAacOptions,
    ConnectionState,
    SensorOptions
)


# Adapter registry
_adapter_registry: Dict[str, Type["BaseSensorAdapter"]] = {}


class WiaAac:
    """WIA AAC API main class"""

    def __init__(self, options: Optional[WiaAacOptions] = None):
        self._options = options or WiaAacOptions()
        self._emitter = EventEmitter()
        self._adapter: Optional["BaseSensorAdapter"] = None
        self._config: Optional[SensorConfig] = None
        self._state = ConnectionState.DISCONNECTED
        self._signal_buffer: List[WiaAacSignal] = []
        self._logger = self._create_logger()

    # ============================================
    # Static Methods
    # ============================================

    @staticmethod
    def register_adapter(sensor_type: str, adapter_class: Type["BaseSensorAdapter"]) -> None:
        """Register a custom adapter"""
        _adapter_registry[sensor_type] = adapter_class

    @staticmethod
    def get_adapter_class(sensor_type: str) -> Optional[Type["BaseSensorAdapter"]]:
        """Get registered adapter class"""
        return _adapter_registry.get(sensor_type)

    # ============================================
    # Connection Management
    # ============================================

    async def connect(self, config: SensorConfig) -> None:
        """Connect to a sensor"""
        if self._state == ConnectionState.CONNECTED:
            raise WiaAacError(
                code=ErrorCode.DEVICE_BUSY,
                message="Already connected to a device",
                timestamp=int(time.time() * 1000),
                recoverable=True
            )

        self._config = config
        self._state = ConnectionState.CONNECTING
        sensor_type = config.type.value if isinstance(config.type, SensorType) else config.type
        self._logger.info(f"Connecting to {sensor_type} sensor...")

        try:
            # Get adapter (use pre-set adapter if available)
            if not self._adapter:
                adapter_class = _adapter_registry.get(sensor_type)
                if not adapter_class:
                    self._logger.warning(f"No adapter registered for {sensor_type}, using mock")
                    from ..adapters.mock_adapter import MockAdapter
                    self._adapter = MockAdapter(sensor_type=sensor_type)
                else:
                    self._adapter = adapter_class()

            # Connect
            await self._adapter.connect(config)

            # Setup signal handler
            self._adapter.on_signal(self._handle_signal)

            # Apply options
            if config.options:
                self._adapter.configure(config.options)

            self._state = ConnectionState.CONNECTED
            manufacturer = config.device.manufacturer if config.device else sensor_type
            self._logger.info(f"Connected to {manufacturer}")

            # Emit connected event
            device_info = self._adapter.device_info or DeviceInfo(
                manufacturer=config.device.manufacturer if config.device else "Unknown",
                model=config.device.model if config.device else sensor_type
            )
            self._emitter.emit(EventType.CONNECTED, device_info)

        except Exception as e:
            self._state = ConnectionState.ERROR
            error = WiaAacError(
                code=ErrorCode.CONNECTION_FAILED,
                message=f"Failed to connect: {str(e)}",
                timestamp=int(time.time() * 1000),
                recoverable=True
            )
            self._emitter.emit(EventType.ERROR, error)
            raise error

    async def disconnect(self) -> None:
        """Disconnect from the sensor"""
        if self._adapter:
            try:
                await self._adapter.disconnect()
            except Exception as e:
                self._logger.error(f"Error during disconnect: {e}")
            self._adapter = None

        self._state = ConnectionState.DISCONNECTED
        self._config = None
        self._signal_buffer = []

        self._emitter.emit(EventType.DISCONNECTED, DisconnectReason(reason="user"))
        self._logger.info("Disconnected")

    def is_connected(self) -> bool:
        """Check if connected"""
        return self._state == ConnectionState.CONNECTED

    def get_connection_state(self) -> ConnectionState:
        """Get connection state"""
        return self._state

    # ============================================
    # Device Discovery
    # ============================================

    async def list_devices(self) -> List[DeviceInfo]:
        """List available devices"""
        self._logger.debug("Listing devices...")
        return []

    def get_device_info(self) -> Optional[DeviceInfo]:
        """Get connected device info"""
        return self._adapter.device_info if self._adapter else None

    # ============================================
    # Event Handling
    # ============================================

    def on(self, event: str, handler: Callable[[Any], None]) -> None:
        """Subscribe to an event"""
        self._emitter.on(event, handler)

    def off(self, event: str, handler: Callable[[Any], None]) -> None:
        """Unsubscribe from an event"""
        self._emitter.off(event, handler)

    def once(self, event: str, handler: Callable[[Any], None]) -> None:
        """Subscribe to an event once"""
        self._emitter.once(event, handler)

    # ============================================
    # Signal Management
    # ============================================

    def get_last_signal(self) -> Optional[WiaAacSignal]:
        """Get the last received signal"""
        return self._signal_buffer[-1] if self._signal_buffer else None

    def get_signal_buffer(self) -> List[WiaAacSignal]:
        """Get signal buffer"""
        return self._signal_buffer.copy()

    # ============================================
    # Configuration
    # ============================================

    def configure(self, options: SensorOptions) -> None:
        """Update sensor options"""
        if self._adapter:
            self._adapter.configure(options)

    def get_config(self) -> Optional[SensorConfig]:
        """Get current configuration"""
        return self._config

    # ============================================
    # Private Methods
    # ============================================

    def _handle_signal(self, signal: WiaAacSignal) -> None:
        """Handle incoming signal"""
        # Add to buffer
        self._signal_buffer.append(signal)
        if len(self._signal_buffer) > self._options.signal_buffer_size:
            self._signal_buffer.pop(0)

        # Emit signal event
        self._emitter.emit(EventType.SIGNAL, signal)

        # Check for selection events
        self._check_selection_event(signal)

    def _check_selection_event(self, signal: WiaAacSignal) -> None:
        """Check if signal triggers a selection event"""
        dwell_time = 1000
        if self._config and self._config.options:
            if hasattr(self._config.options, 'dwell_time') and self._config.options.dwell_time:
                dwell_time = self._config.options.dwell_time

        # Eye tracker fixation -> selection
        if isinstance(signal.data, dict):
            data = signal.data
        else:
            return

        signal_type = signal.type.value if isinstance(signal.type, SensorType) else signal.type

        if signal_type == "eye_tracker":
            fixation = data.get("fixation", {})
            if fixation.get("active") and fixation.get("duration_ms", 0) >= dwell_time:
                gaze = data.get("gaze", {})
                self._emitter.emit(EventType.SELECTION, SelectionEvent(
                    timestamp=signal.timestamp.unix_ms,
                    target_id=fixation.get("target_id", "unknown"),
                    target_type="area",
                    selection_method="dwell",
                    position={"x": gaze.get("x", 0), "y": gaze.get("y", 0)},
                    confidence=signal.meta.confidence if signal.meta else 0.8
                ))

        # Switch press -> selection
        elif signal_type == "switch" and data.get("state") == "pressed":
            self._emitter.emit(EventType.SELECTION, SelectionEvent(
                timestamp=signal.timestamp.unix_ms,
                target_id=f"switch_{data.get('switch_id', 1)}",
                target_type="button",
                selection_method="switch",
                confidence=1.0
            ))

    def _create_logger(self) -> logging.Logger:
        """Create logger"""
        logger = logging.getLogger("WiaAac")

        level_map = {
            "debug": logging.DEBUG,
            "info": logging.INFO,
            "warn": logging.WARNING,
            "error": logging.ERROR,
            "none": logging.CRITICAL + 1
        }

        logger.setLevel(level_map.get(self._options.log_level, logging.INFO))

        if not logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("[WiaAac:%(levelname)s] %(message)s"))
            logger.addHandler(handler)

        return logger

    def use_adapter(self, adapter: "BaseSensorAdapter") -> None:
        """Use a specific adapter instance (for testing)"""
        self._adapter = adapter


# Import at the end to avoid circular imports
from ..adapters.base_adapter import BaseSensorAdapter
