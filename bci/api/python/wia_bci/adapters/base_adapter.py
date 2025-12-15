"""WIA BCI Base Adapter."""

from abc import ABC, abstractmethod
from typing import Callable, Optional

from wia_bci.types.config import DeviceConfig, DeviceType
from wia_bci.types.device import ChannelInfo, DeviceInfo
from wia_bci.types.events import SignalEvent, ErrorEvent


class BaseBciAdapter(ABC):
    """
    Base BCI Adapter Interface.

    All device-specific adapters must inherit from this class.
    """

    def __init__(self) -> None:
        self._connected = False
        self._streaming = False
        self._device_info: Optional[DeviceInfo] = None
        self._channels: list[ChannelInfo] = []
        self._data_handler: Optional[Callable[[SignalEvent], None]] = None
        self._error_handler: Optional[Callable[[ErrorEvent], None]] = None

    @property
    @abstractmethod
    def type(self) -> DeviceType:
        """Device type this adapter handles."""
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        """Adapter name."""
        ...

    @abstractmethod
    async def connect(self, config: DeviceConfig) -> None:
        """Connect to the BCI device."""
        ...

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from the BCI device."""
        ...

    @abstractmethod
    async def start_stream(self) -> None:
        """Start data streaming."""
        ...

    @abstractmethod
    async def stop_stream(self) -> None:
        """Stop data streaming."""
        ...

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected

    def is_streaming(self) -> bool:
        """Check if streaming."""
        return self._streaming

    def get_device_info(self) -> Optional[DeviceInfo]:
        """Get device information."""
        return self._device_info

    def get_channels(self) -> list[ChannelInfo]:
        """Get channel information."""
        return self._channels

    def on_data(self, handler: Callable[[SignalEvent], None]) -> None:
        """Register data handler."""
        self._data_handler = handler

    def on_error(self, handler: Callable[[ErrorEvent], None]) -> None:
        """Register error handler."""
        self._error_handler = handler

    def dispose(self) -> None:
        """Clean up resources."""
        self._data_handler = None
        self._error_handler = None

    def _emit_data(self, data: SignalEvent) -> None:
        """Emit data event to handler."""
        if self._data_handler:
            self._data_handler(data)

    def _emit_error(self, error: ErrorEvent) -> None:
        """Emit error event to handler."""
        if self._error_handler:
            self._error_handler(error)
