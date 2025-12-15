"""
WIA AAC Event Emitter
"""

from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass


@dataclass
class HandlerEntry:
    """Event handler entry"""
    handler: Callable[[Any], None]
    once: bool = False
    filter_fn: Optional[Callable[[Any], bool]] = None


class EventEmitter:
    """Type-safe event emitter for AAC signals and events"""

    def __init__(self):
        self._handlers: Dict[str, List[HandlerEntry]] = {}

    def on(
        self,
        event: str,
        handler: Callable[[Any], None],
        *,
        once: bool = False,
        filter_fn: Optional[Callable[[Any], bool]] = None
    ) -> None:
        """Subscribe to an event"""
        if event not in self._handlers:
            self._handlers[event] = []

        entry = HandlerEntry(handler=handler, once=once, filter_fn=filter_fn)
        self._handlers[event].append(entry)

    def off(self, event: str, handler: Callable[[Any], None]) -> None:
        """Unsubscribe from an event"""
        if event not in self._handlers:
            return

        self._handlers[event] = [
            entry for entry in self._handlers[event]
            if entry.handler != handler
        ]

    def once(self, event: str, handler: Callable[[Any], None]) -> None:
        """Subscribe to an event once"""
        self.on(event, handler, once=True)

    def emit(self, event: str, data: Any) -> None:
        """Emit an event"""
        if event not in self._handlers:
            return

        to_remove: List[HandlerEntry] = []

        for entry in self._handlers[event]:
            # Apply filter if exists
            if entry.filter_fn and not entry.filter_fn(data):
                continue

            # Call handler
            try:
                entry.handler(data)
            except Exception as e:
                print(f"Error in event handler for '{event}': {e}")

            # Mark for removal if once
            if entry.once:
                to_remove.append(entry)

        # Remove once handlers
        for entry in to_remove:
            self._handlers[event].remove(entry)

    def remove_all_listeners(self, event: Optional[str] = None) -> None:
        """Remove all handlers for an event or all events"""
        if event:
            self._handlers.pop(event, None)
        else:
            self._handlers.clear()

    def listener_count(self, event: str) -> int:
        """Get listener count for an event"""
        return len(self._handlers.get(event, []))
