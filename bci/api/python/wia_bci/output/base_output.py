"""
WIA BCI Output Adapter Interface

Phase 4: Ecosystem Integration
"""

from abc import ABC, abstractmethod
from typing import Callable, Dict, Set, Optional
import time

from .types import (
    OutputType,
    OutputContent,
    OutputOptions,
    OutputEvent,
    OutputEventData,
    OutputEventHandler,
)


class IOutputAdapter(ABC):
    """Output adapter interface."""

    @property
    @abstractmethod
    def type(self) -> OutputType:
        """Get adapter type."""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Get adapter name."""
        pass

    @property
    def version(self) -> str:
        """Get adapter version."""
        return "1.0.0"

    @abstractmethod
    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize adapter."""
        pass

    @abstractmethod
    async def output(self, content: OutputContent) -> None:
        """Output content."""
        pass

    @abstractmethod
    def is_available(self) -> bool:
        """Check if available."""
        pass

    @abstractmethod
    def is_ready(self) -> bool:
        """Check if ready."""
        pass

    @abstractmethod
    def on(self, event: OutputEvent, handler: OutputEventHandler) -> None:
        """Add event handler."""
        pass

    @abstractmethod
    def off(self, event: OutputEvent, handler: OutputEventHandler) -> None:
        """Remove event handler."""
        pass

    @abstractmethod
    async def dispose(self) -> None:
        """Dispose adapter."""
        pass


class BaseOutputAdapter(IOutputAdapter):
    """Base output adapter with common functionality."""

    def __init__(self):
        self._ready = False
        self._available = True
        self._handlers: Dict[OutputEvent, Set[OutputEventHandler]] = {}

    def is_available(self) -> bool:
        """Check if available."""
        return self._available

    def is_ready(self) -> bool:
        """Check if ready."""
        return self._ready

    def on(self, event: OutputEvent, handler: OutputEventHandler) -> None:
        """Add event handler."""
        if event not in self._handlers:
            self._handlers[event] = set()
        self._handlers[event].add(handler)

    def off(self, event: OutputEvent, handler: OutputEventHandler) -> None:
        """Remove event handler."""
        if event in self._handlers:
            self._handlers[event].discard(handler)

    def _emit(self, event: OutputEvent, **kwargs) -> None:
        """Emit event."""
        event_data = OutputEventData(
            type=event,
            adapter=self.type,
            timestamp=int(time.time() * 1000),
            **kwargs,
        )
        if event in self._handlers:
            for handler in self._handlers[event]:
                handler(event_data)

    async def dispose(self) -> None:
        """Dispose adapter."""
        self._ready = False
        self._handlers.clear()
