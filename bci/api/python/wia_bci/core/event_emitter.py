"""WIA BCI Event Emitter."""

from collections import defaultdict
from typing import Any, Callable, Optional

from wia_bci.types.events import EventType

EventHandler = Callable[..., None]


class BciEventEmitter:
    """Type-safe event emitter for BCI events."""

    def __init__(self) -> None:
        self._listeners: dict[EventType, list[EventHandler]] = defaultdict(list)
        self._once_handlers: set[EventHandler] = set()

    def on(self, event: EventType, handler: EventHandler) -> "BciEventEmitter":
        """Subscribe to an event."""
        self._listeners[event].append(handler)
        return self

    def off(self, event: EventType, handler: EventHandler) -> "BciEventEmitter":
        """Unsubscribe from an event."""
        if event in self._listeners:
            try:
                self._listeners[event].remove(handler)
            except ValueError:
                pass
        return self

    def once(self, event: EventType, handler: EventHandler) -> "BciEventEmitter":
        """Subscribe to an event once."""
        self._once_handlers.add(handler)
        self._listeners[event].append(handler)
        return self

    def emit(self, event: EventType, data: Optional[Any] = None) -> "BciEventEmitter":
        """Emit an event."""
        handlers = self._listeners.get(event, []).copy()

        for handler in handlers:
            try:
                if data is None:
                    handler()
                else:
                    handler(data)
            except Exception as e:
                print(f"Error in event handler for '{event}': {e}")

            # Remove once handlers
            if handler in self._once_handlers:
                self._once_handlers.discard(handler)
                try:
                    self._listeners[event].remove(handler)
                except ValueError:
                    pass

        return self

    def remove_all_listeners(self, event: Optional[EventType] = None) -> "BciEventEmitter":
        """Remove all listeners for an event (or all events)."""
        if event is not None:
            self._listeners.pop(event, None)
        else:
            self._listeners.clear()
            self._once_handlers.clear()
        return self

    def listener_count(self, event: EventType) -> int:
        """Get listener count for an event."""
        return len(self._listeners.get(event, []))

    def event_names(self) -> list[EventType]:
        """Get all events with listeners."""
        return list(self._listeners.keys())
