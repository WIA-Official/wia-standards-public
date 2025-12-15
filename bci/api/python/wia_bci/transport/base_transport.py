"""
WIA BCI Transport Interface

Phase 3: Transport Layer Abstraction
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, Optional, Any
from dataclasses import dataclass, field

from ..protocol.types import BciMessage, TransportOptions


class TransportState(str, Enum):
    """Transport state."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    CLOSING = "closing"


@dataclass
class TransportEventHandlers:
    """Transport event handlers."""

    on_open: Optional[Callable[[], None]] = None
    on_close: Optional[Callable[[int, str], None]] = None
    on_error: Optional[Callable[[Exception], None]] = None
    on_message: Optional[Callable[[BciMessage], None]] = None


class ITransport(ABC):
    """Transport interface."""

    @abstractmethod
    def get_state(self) -> TransportState:
        """Get transport state."""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if connected."""
        pass

    @abstractmethod
    async def connect(self, url: str, options: Optional[TransportOptions] = None) -> None:
        """Connect to server."""
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from server."""
        pass

    @abstractmethod
    async def send(self, message: BciMessage) -> None:
        """Send message."""
        pass

    @abstractmethod
    def on_message(self, handler: Callable[[BciMessage], None]) -> None:
        """Set message handler."""
        pass

    @abstractmethod
    def on_open(self, handler: Callable[[], None]) -> None:
        """Set open handler."""
        pass

    @abstractmethod
    def on_close(self, handler: Callable[[int, str], None]) -> None:
        """Set close handler."""
        pass

    @abstractmethod
    def on_error(self, handler: Callable[[Exception], None]) -> None:
        """Set error handler."""
        pass

    @abstractmethod
    def dispose(self) -> None:
        """Dispose transport."""
        pass


class BaseTransport(ITransport):
    """Base transport class with common functionality."""

    def __init__(self):
        self._state: TransportState = TransportState.DISCONNECTED
        self._handlers: TransportEventHandlers = TransportEventHandlers()
        self._options: Optional[TransportOptions] = None

    def get_state(self) -> TransportState:
        """Get transport state."""
        return self._state

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._state == TransportState.CONNECTED

    def on_message(self, handler: Callable[[BciMessage], None]) -> None:
        """Set message handler."""
        self._handlers.on_message = handler

    def on_open(self, handler: Callable[[], None]) -> None:
        """Set open handler."""
        self._handlers.on_open = handler

    def on_close(self, handler: Callable[[int, str], None]) -> None:
        """Set close handler."""
        self._handlers.on_close = handler

    def on_error(self, handler: Callable[[Exception], None]) -> None:
        """Set error handler."""
        self._handlers.on_error = handler

    def _emit_open(self) -> None:
        """Emit open event."""
        if self._handlers.on_open:
            self._handlers.on_open()

    def _emit_close(self, code: int, reason: str) -> None:
        """Emit close event."""
        if self._handlers.on_close:
            self._handlers.on_close(code, reason)

    def _emit_error(self, error: Exception) -> None:
        """Emit error event."""
        if self._handlers.on_error:
            self._handlers.on_error(error)

    def _emit_message(self, message: BciMessage) -> None:
        """Emit message event."""
        if self._handlers.on_message:
            self._handlers.on_message(message)

    def dispose(self) -> None:
        """Dispose transport."""
        self._handlers = TransportEventHandlers()
