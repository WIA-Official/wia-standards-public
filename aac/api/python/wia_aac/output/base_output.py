"""
WIA AAC Output Adapter Base Class
Phase 4: WIA Ecosystem Integration
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Set
import time

from .types import (
    OutputType,
    OutputState,
    OutputOptions,
    OutputEvent,
    OutputEventType,
    OutputEventHandler
)


class IOutputAdapter(ABC):
    """Output adapter interface"""

    @property
    @abstractmethod
    def type(self) -> OutputType:
        """Output type"""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Adapter name"""
        pass

    @property
    @abstractmethod
    def state(self) -> OutputState:
        """Current state"""
        pass

    @abstractmethod
    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize the adapter"""
        pass

    @abstractmethod
    async def output(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """Output text"""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop output"""
        pass

    @abstractmethod
    def is_available(self) -> bool:
        """Check if adapter is available"""
        pass

    @abstractmethod
    async def dispose(self) -> None:
        """Dispose resources"""
        pass

    @abstractmethod
    def on(self, event: OutputEventType, handler: OutputEventHandler) -> None:
        """Register event handler"""
        pass

    @abstractmethod
    def off(self, event: OutputEventType, handler: OutputEventHandler) -> None:
        """Unregister event handler"""
        pass


class BaseOutputAdapter(IOutputAdapter):
    """Base output adapter class"""

    def __init__(self):
        self._state = OutputState.IDLE
        self._handlers: Dict[OutputEventType, Set[OutputEventHandler]] = {}

    @property
    def state(self) -> OutputState:
        return self._state

    def on(self, event: OutputEventType, handler: OutputEventHandler) -> None:
        """Register event handler"""
        if event not in self._handlers:
            self._handlers[event] = set()
        self._handlers[event].add(handler)

    def off(self, event: OutputEventType, handler: OutputEventHandler) -> None:
        """Unregister event handler"""
        if event in self._handlers:
            self._handlers[event].discard(handler)

    def _emit(self, event_type: OutputEventType, data: Optional[any] = None) -> None:
        """Emit event to all registered handlers"""
        if event_type in self._handlers:
            event = OutputEvent(
                type=event_type,
                adapter=self.type,
                timestamp=time.time(),
                data=data
            )
            for handler in self._handlers[event_type]:
                try:
                    handler(event)
                except Exception:
                    pass  # Don't let handler errors propagate
