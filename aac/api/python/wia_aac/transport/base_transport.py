"""
WIA AAC Transport Interface
"""

from abc import ABC, abstractmethod
from typing import Callable, Literal, Any

from ..protocol.message import WiaAacMessage

TransportState = Literal["disconnected", "connecting", "connected", "error"]

MessageHandler = Callable[[str], None]
OpenHandler = Callable[[], None]
CloseHandler = Callable[[str], None]
ErrorHandler = Callable[[Exception], None]


class ITransport(ABC):
    """Transport interface for WIA AAC protocol"""

    @abstractmethod
    async def connect(self, url: str) -> None:
        """Connect to the server"""
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from the server"""
        pass

    @abstractmethod
    async def send(self, message: WiaAacMessage) -> None:
        """Send a message"""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if connected"""
        pass

    @abstractmethod
    def get_state(self) -> TransportState:
        """Get transport state"""
        pass

    @abstractmethod
    def on_message(self, handler: MessageHandler) -> None:
        """Set message handler"""
        pass

    @abstractmethod
    def on_open(self, handler: OpenHandler) -> None:
        """Set open handler"""
        pass

    @abstractmethod
    def on_close(self, handler: CloseHandler) -> None:
        """Set close handler"""
        pass

    @abstractmethod
    def on_error(self, handler: ErrorHandler) -> None:
        """Set error handler"""
        pass
