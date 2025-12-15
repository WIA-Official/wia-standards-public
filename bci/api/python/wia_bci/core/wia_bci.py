"""WIA BCI Main Class."""

import logging
from typing import Any, Callable, Optional

from wia_bci.core.event_emitter import BciEventEmitter
from wia_bci.core.bci_error import BciError
from wia_bci.types.config import DeviceConfig, LogLevel, WiaBciOptions
from wia_bci.types.device import BciState, ChannelInfo, DeviceInfo
from wia_bci.types.events import EventType, SignalEvent, ErrorEvent

from wia_bci.adapters.base_adapter import BaseBciAdapter
from wia_bci.adapters.simulator_adapter import SimulatorAdapter


def _create_logger(level: LogLevel) -> logging.Logger:
    """Create logger with specified level."""
    logger = logging.getLogger("wia-bci")

    level_map = {
        "debug": logging.DEBUG,
        "info": logging.INFO,
        "warn": logging.WARNING,
        "error": logging.ERROR,
    }

    logger.setLevel(level_map.get(level, logging.INFO))

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("[WIA-BCI] [%(levelname)s] %(message)s")
        )
        logger.addHandler(handler)

    return logger


class WiaBci:
    """
    WiaBci - Main BCI Interface Class.

    Example:
        >>> bci = WiaBci()
        >>> await bci.connect(DeviceConfig(type='eeg_headset'))
        >>> bci.on('signal', lambda e: print(e.data))
        >>> await bci.start_stream()
    """

    def __init__(self, options: Optional[WiaBciOptions] = None) -> None:
        self._options = options or WiaBciOptions()
        self._emitter = BciEventEmitter()
        self._logger = _create_logger(self._options.log_level)
        self._adapter: Optional[BaseBciAdapter] = None
        self._connected = False
        self._streaming = False
        self._samples_received = 0
        self._reconnect_attempts = 0

        self._logger.debug(f"WiaBci initialized with options: {self._options}")

    async def list_devices(self) -> list[DeviceInfo]:
        """List available BCI devices."""
        self._logger.debug("Listing devices...")

        from wia_bci.types.device import DeviceCapabilities

        # Return simulator as available device
        devices = [
            DeviceInfo(
                id="simulator-001",
                name="WIA BCI Simulator",
                type="simulator",
                manufacturer="WIA",
                model="Simulator",
                capabilities=DeviceCapabilities(
                    channels=8,
                    max_sampling_rate=250,
                    supported_sampling_rates=[125, 250],
                    resolution=24,
                ),
                status="available",
            )
        ]

        return devices

    async def connect(self, config: DeviceConfig) -> None:
        """Connect to a BCI device."""
        if self._connected:
            raise BciError.already_connected()

        self._logger.info(f"Connecting to device: {config}")

        try:
            self._adapter = self._create_adapter(config)
            await self._adapter.connect(config)

            # Set up handlers
            def on_data(event: SignalEvent) -> None:
                self._samples_received += 1
                self._emitter.emit("signal", event)
                self._emitter.emit("data", event)

            def on_error(error: ErrorEvent) -> None:
                self._logger.error(f"Adapter error: {error}")
                self._emitter.emit("error", error)

            self._adapter.on_data(on_data)
            self._adapter.on_error(on_error)

            self._connected = True
            self._reconnect_attempts = 0

            self._logger.info("Connected successfully")
            self._emitter.emit("connected")

        except BciError:
            raise
        except Exception as e:
            self._logger.error(f"Connection failed: {e}")
            raise BciError.connection_failed(str(e), e)

    async def disconnect(self) -> None:
        """Disconnect from the current device."""
        if not self._connected or not self._adapter:
            raise BciError.not_connected()

        self._logger.info("Disconnecting...")

        if self._streaming:
            await self.stop_stream()

        await self._adapter.disconnect()
        self._adapter = None
        self._connected = False

        self._logger.info("Disconnected")
        self._emitter.emit("disconnected")

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected

    async def start_stream(self) -> None:
        """Start data streaming."""
        if not self._connected or not self._adapter:
            raise BciError.not_connected()

        if self._streaming:
            self._logger.warning("Already streaming")
            return

        self._logger.info("Starting stream...")

        await self._adapter.start_stream()
        self._streaming = True

        self._logger.info("Stream started")
        self._emitter.emit("stream_started")

    async def stop_stream(self) -> None:
        """Stop data streaming."""
        if not self._connected or not self._adapter:
            raise BciError.not_connected()

        if not self._streaming:
            self._logger.warning("Not streaming")
            return

        self._logger.info("Stopping stream...")

        await self._adapter.stop_stream()
        self._streaming = False

        self._logger.info("Stream stopped")
        self._emitter.emit("stream_stopped")

    def is_streaming(self) -> bool:
        """Check if streaming."""
        return self._streaming

    def get_device_info(self) -> Optional[DeviceInfo]:
        """Get current device info."""
        if self._adapter:
            return self._adapter.get_device_info()
        return None

    def get_channels(self) -> list[ChannelInfo]:
        """Get channel information."""
        if self._adapter:
            return self._adapter.get_channels()
        return []

    def get_state(self) -> BciState:
        """Get current BCI state."""
        device_info = self.get_device_info()
        return BciState(
            connected=self._connected,
            streaming=self._streaming,
            device=device_info,
            channels=self.get_channels(),
            sampling_rate=(
                device_info.capabilities.max_sampling_rate if device_info else 0
            ),
            samples_received=self._samples_received,
            last_error=None,
        )

    def on(self, event: EventType, handler: Callable[..., None]) -> "WiaBci":
        """Subscribe to an event."""
        self._emitter.on(event, handler)
        return self

    def off(self, event: EventType, handler: Callable[..., None]) -> "WiaBci":
        """Unsubscribe from an event."""
        self._emitter.off(event, handler)
        return self

    def once(self, event: EventType, handler: Callable[..., None]) -> "WiaBci":
        """Subscribe to an event once."""
        self._emitter.once(event, handler)
        return self

    def emit(self, event: EventType, data: Optional[Any] = None) -> "WiaBci":
        """Emit an event (for external use, e.g., markers)."""
        self._emitter.emit(event, data)
        return self

    def dispose(self) -> None:
        """Clean up resources."""
        self._logger.debug("Disposing WiaBci...")

        if self._adapter:
            self._adapter.dispose()
            self._adapter = None

        self._emitter.remove_all_listeners()
        self._connected = False
        self._streaming = False

        self._logger.debug("Disposed")

    def _create_adapter(self, config: DeviceConfig) -> BaseBciAdapter:
        """Create appropriate adapter for device type."""
        if config.type == "simulator":
            return SimulatorAdapter()

        if config.type in ("eeg_headset", "eeg_cap"):
            self._logger.warning(
                f"Using simulator for device type: {config.type}"
            )
            return SimulatorAdapter()

        raise BciError.invalid_config(f"Unsupported device type: {config.type}")
