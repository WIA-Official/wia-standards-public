"""
WIA AAC Output Manager
Phase 4: WIA Ecosystem Integration

Unified output management for all adapters
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Optional, Dict, List, Set, Any, TypeVar, Callable

from .base_output import IOutputAdapter
from .types import (
    OutputType,
    OutputOptions,
    OutputError,
    OutputErrorCode
)

T = TypeVar('T', bound=IOutputAdapter)


class IOutputManager(ABC):
    """Output manager interface"""

    @abstractmethod
    def register(self, adapter: IOutputAdapter) -> None:
        """Register an adapter"""
        pass

    @abstractmethod
    def unregister(self, output_type: OutputType) -> None:
        """Unregister an adapter"""
        pass

    @abstractmethod
    def get_adapter(self, output_type: OutputType) -> Optional[IOutputAdapter]:
        """Get adapter by type"""
        pass

    @abstractmethod
    def get_active_adapters(self) -> List[IOutputAdapter]:
        """Get all active adapters"""
        pass

    @abstractmethod
    async def broadcast(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """Broadcast to all active adapters"""
        pass

    @abstractmethod
    async def output_to(
        self,
        output_type: OutputType,
        text: str,
        options: Optional[OutputOptions] = None
    ) -> None:
        """Output to specific adapter"""
        pass

    @abstractmethod
    def stop_all(self) -> None:
        """Stop all output"""
        pass

    @abstractmethod
    async def dispose(self) -> None:
        """Dispose all resources"""
        pass


# Event handler type
OutputManagerEventHandler = Callable[[Any], None]


class OutputManager(IOutputManager):
    """
    Output Manager
    Manages multiple output adapters and provides unified interface
    """

    def __init__(self):
        self._adapters: Dict[OutputType, IOutputAdapter] = {}
        self._handlers: Dict[str, Set[OutputManagerEventHandler]] = {}

    def register(self, adapter: IOutputAdapter) -> None:
        """Register an output adapter"""
        if adapter.type in self._adapters:
            # Dispose existing adapter first
            existing = self._adapters[adapter.type]
            asyncio.create_task(existing.dispose())

        self._adapters[adapter.type] = adapter
        self._emit("adapter_registered", {
            "type": adapter.type.value,
            "name": adapter.name
        })

    def unregister(self, output_type: OutputType) -> None:
        """Unregister an output adapter"""
        adapter = self._adapters.pop(output_type, None)
        if adapter:
            asyncio.create_task(adapter.dispose())
            self._emit("adapter_unregistered", {"type": output_type.value})

    def get_adapter(self, output_type: OutputType) -> Optional[IOutputAdapter]:
        """Get adapter by type"""
        return self._adapters.get(output_type)

    def get_active_adapters(self) -> List[IOutputAdapter]:
        """Get all active adapters"""
        return [a for a in self._adapters.values() if a.is_available()]

    def get_all_adapters(self) -> List[IOutputAdapter]:
        """Get all registered adapters"""
        return list(self._adapters.values())

    def has_adapter(self, output_type: OutputType) -> bool:
        """Check if an adapter type is registered"""
        return output_type in self._adapters

    async def broadcast(self, text: str, options: Optional[OutputOptions] = None) -> None:
        """
        Broadcast to all active adapters
        Errors from individual adapters are caught and emitted as events
        """
        active_adapters = self.get_active_adapters()

        if not active_adapters:
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_FOUND,
                "No active adapters available",
                True
            )

        self._emit("output_start", {
            "text": text,
            "adapters": [a.type.value for a in active_adapters]
        })

        results = await asyncio.gather(
            *[adapter.output(text, options) for adapter in active_adapters],
            return_exceptions=True
        )

        # Collect errors
        errors = []
        for adapter, result in zip(active_adapters, results):
            if isinstance(result, Exception):
                errors.append({
                    "type": adapter.type.value,
                    "error": str(result)
                })
                self._emit("error", {
                    "adapter": adapter.type.value,
                    "error": str(result)
                })

        self._emit("output_end", {
            "text": text,
            "errors": errors if errors else None
        })

    async def output_to(
        self,
        output_type: OutputType,
        text: str,
        options: Optional[OutputOptions] = None
    ) -> None:
        """Output to specific adapter"""
        adapter = self._adapters.get(output_type)

        if not adapter:
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_FOUND,
                f"Adapter not found: {output_type.value}",
                True
            )

        if not adapter.is_available():
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_READY,
                f"Adapter not available: {output_type.value}",
                True
            )

        self._emit("output_start", {
            "text": text,
            "adapters": [output_type.value]
        })

        try:
            await adapter.output(text, options)
            self._emit("output_end", {"text": text})
        except Exception as e:
            self._emit("error", {
                "adapter": output_type.value,
                "error": str(e)
            })
            raise

    async def output_to_multiple(
        self,
        output_types: List[OutputType],
        text: str,
        options: Optional[OutputOptions] = None
    ) -> None:
        """Output to multiple specific adapters"""
        adapters = [
            self._adapters[t]
            for t in output_types
            if t in self._adapters and self._adapters[t].is_available()
        ]

        if not adapters:
            raise OutputError(
                OutputErrorCode.ADAPTER_NOT_FOUND,
                "No specified adapters available",
                True
            )

        self._emit("output_start", {
            "text": text,
            "adapters": [a.type.value for a in adapters]
        })

        results = await asyncio.gather(
            *[adapter.output(text, options) for adapter in adapters],
            return_exceptions=True
        )

        errors = []
        for adapter, result in zip(adapters, results):
            if isinstance(result, Exception):
                errors.append({
                    "type": adapter.type.value,
                    "error": str(result)
                })

        self._emit("output_end", {
            "text": text,
            "errors": errors if errors else None
        })

        if errors:
            raise OutputError(
                OutputErrorCode.OUTPUT_FAILED,
                f"Output failed for: {', '.join(e['type'] for e in errors)}",
                True,
                errors
            )

    def stop_all(self) -> None:
        """Stop all output"""
        for adapter in self._adapters.values():
            adapter.stop()

    async def dispose(self) -> None:
        """Dispose all resources"""
        for adapter in self._adapters.values():
            await adapter.dispose()
        self._adapters.clear()
        self._handlers.clear()

    def on(self, event: str, handler: OutputManagerEventHandler) -> None:
        """Register event handler"""
        if event not in self._handlers:
            self._handlers[event] = set()
        self._handlers[event].add(handler)

    def off(self, event: str, handler: OutputManagerEventHandler) -> None:
        """Unregister event handler"""
        if event in self._handlers:
            self._handlers[event].discard(handler)

    def _emit(self, event: str, data: Any = None) -> None:
        """Emit event"""
        if event in self._handlers:
            for handler in self._handlers[event]:
                try:
                    handler(data)
                except Exception:
                    pass
