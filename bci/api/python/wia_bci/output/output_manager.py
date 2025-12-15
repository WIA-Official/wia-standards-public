"""
WIA BCI Output Manager

Central controller for all output adapters.
"""

import asyncio
from typing import Dict, List, Optional, Set, Callable

from .base_output import IOutputAdapter
from .types import (
    OutputType,
    OutputContent,
    OutputPreferences,
    ManagerOptions,
    OutputEvent,
    OutputEventData,
    OutputEventHandler,
    OutputError,
    OutputErrorCode,
)
from .tts_adapter import TTSAdapter
from .mock_output import MockSignLanguageAdapter, MockBrailleAdapter
from .neurofeedback_adapter import NeurofeedbackAdapter


DEFAULT_PREFERENCES = OutputPreferences(
    primary_output="tts",
    enabled_outputs=["tts", "neurofeedback"],
    language="ko",
)


class OutputManager:
    """Output Manager."""

    def __init__(self, options: Optional[ManagerOptions] = None):
        self._adapters: Dict[OutputType, IOutputAdapter] = {}
        self._preferences = (
            options.preferences if options and options.preferences else DEFAULT_PREFERENCES
        )
        self._handlers: Dict[str, Set[OutputEventHandler]] = {}
        self._initialized = False

    async def initialize(self, options: Optional[ManagerOptions] = None) -> None:
        """Initialize manager and all adapters."""
        if options and options.preferences:
            self._preferences = options.preferences

        # Register default adapters
        self.register(TTSAdapter())
        self.register(MockSignLanguageAdapter())
        self.register(MockBrailleAdapter())
        self.register(NeurofeedbackAdapter())

        # Initialize all adapters
        init_tasks = []
        for adapter in self._adapters.values():
            init_tasks.append(self._init_adapter(adapter))

        await asyncio.gather(*init_tasks, return_exceptions=True)
        self._initialized = True
        self._emit("ready")

    async def _init_adapter(self, adapter: IOutputAdapter) -> None:
        """Initialize single adapter."""
        try:
            from .types import OutputOptions

            await adapter.initialize(OutputOptions(language=self._preferences.language))
        except Exception as e:
            print(f"Warning: Failed to initialize {adapter.name}: {e}")

    def register(self, adapter: IOutputAdapter) -> None:
        """Register an adapter."""
        self._adapters[adapter.type] = adapter

        # Forward adapter events
        events: List[OutputEvent] = ["start", "end", "error", "ready", "busy"]
        for event in events:
            adapter.on(event, lambda data: self._emit(data.type, data))

    def unregister(self, adapter_type: OutputType) -> None:
        """Unregister an adapter."""
        adapter = self._adapters.get(adapter_type)
        if adapter:
            asyncio.create_task(adapter.dispose())
            del self._adapters[adapter_type]

    def get(self, adapter_type: OutputType) -> Optional[IOutputAdapter]:
        """Get an adapter by type."""
        return self._adapters.get(adapter_type)

    def get_all(self) -> List[IOutputAdapter]:
        """Get all adapters."""
        return list(self._adapters.values())

    def get_available(self) -> List[IOutputAdapter]:
        """Get available adapters."""
        return [a for a in self._adapters.values() if a.is_available() and a.is_ready()]

    async def output(
        self, content: OutputContent, targets: Optional[List[OutputType]] = None
    ) -> None:
        """Output content to specific targets."""
        if not self._initialized:
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_READY, "OutputManager not initialized"
            )

        output_types = targets if targets else self._preferences.enabled_outputs
        results = []

        async def output_to_adapter(adapter_type: OutputType):
            adapter = self._adapters.get(adapter_type)
            if adapter and adapter.is_available() and adapter.is_ready():
                try:
                    await adapter.output(content)
                    return {"type": adapter_type, "success": True}
                except Exception as e:
                    return {"type": adapter_type, "success": False, "error": e}
            return {"type": adapter_type, "success": False, "error": None}

        results = await asyncio.gather(
            *[output_to_adapter(t) for t in output_types], return_exceptions=True
        )

        # Check for all failures
        failures = [r for r in results if isinstance(r, dict) and not r.get("success")]
        if failures and len(failures) == len(results):
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_READY,
                "All output adapters failed",
            )

    async def broadcast(self, content: OutputContent) -> None:
        """Broadcast content to all enabled outputs."""
        await self.output(content, self._preferences.enabled_outputs)

    async def output_primary(self, content: OutputContent) -> None:
        """Output to primary adapter only."""
        await self.output(content, [self._preferences.primary_output])

    def set_preferences(self, **kwargs) -> None:
        """Set preferences."""
        for key, value in kwargs.items():
            if hasattr(self._preferences, key):
                setattr(self._preferences, key, value)

    def get_preferences(self) -> OutputPreferences:
        """Get preferences."""
        return self._preferences

    def enable_output(self, adapter_type: OutputType) -> None:
        """Enable an output type."""
        if adapter_type not in self._preferences.enabled_outputs:
            self._preferences.enabled_outputs.append(adapter_type)

    def disable_output(self, adapter_type: OutputType) -> None:
        """Disable an output type."""
        self._preferences.enabled_outputs = [
            t for t in self._preferences.enabled_outputs if t != adapter_type
        ]

    def is_output_enabled(self, adapter_type: OutputType) -> bool:
        """Check if output type is enabled."""
        return adapter_type in self._preferences.enabled_outputs

    def on(self, event: str, handler: OutputEventHandler) -> None:
        """Add event handler."""
        if event not in self._handlers:
            self._handlers[event] = set()
        self._handlers[event].add(handler)

    def off(self, event: str, handler: OutputEventHandler) -> None:
        """Remove event handler."""
        if event in self._handlers:
            self._handlers[event].discard(handler)

    def _emit(self, event: str, data: Optional[OutputEventData] = None) -> None:
        """Emit event."""
        if event in self._handlers:
            for handler in self._handlers[event]:
                if data:
                    handler(data)

    async def dispose(self) -> None:
        """Dispose manager and all adapters."""
        tasks = [adapter.dispose() for adapter in self._adapters.values()]
        await asyncio.gather(*tasks, return_exceptions=True)
        self._adapters.clear()
        self._handlers.clear()
        self._initialized = False
