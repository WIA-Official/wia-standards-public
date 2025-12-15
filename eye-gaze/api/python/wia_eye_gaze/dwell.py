"""
WIA Eye Gaze Standard - Dwell Controller

弘益人間 - 널리 인간을 이롭게
"""

import time
from typing import Optional, Dict, Set, Callable
import threading

from .tracker import IEyeTracker
from .types import (
    GazePoint,
    GazeTarget,
    DwellConfig,
    DwellState,
    Subscription,
)


DwellEventHandler = Callable[[GazeTarget, Optional[float]], None]


class DwellController:
    """
    Dwell Controller for gaze-based selection.

    Example:
        >>> dwell = DwellController(tracker=tracker, threshold=800)
        >>> dwell.register_target(GazeTarget(...))
        >>> dwell.on_dwell_complete(lambda t, _: print(f"Selected: {t.label}"))
        >>> dwell.start()
    """

    def __init__(
        self,
        tracker: IEyeTracker,
        threshold: int = 800,
        progress_interval: int = 50,
        visual_feedback: bool = True,
        cooldown_period: int = 500,
    ):
        self._tracker = tracker
        self._config = DwellConfig(
            threshold=threshold,
            progress_interval=progress_interval,
            visual_feedback=visual_feedback,
            cooldown_period=cooldown_period,
        )

        self._targets: Dict[str, GazeTarget] = {}
        self._dwell_state = DwellState(active=False, progress=0)
        self._subscription: Optional[Subscription] = None
        self._progress_timer: Optional[threading.Timer] = None
        self._cooldown_until = 0
        self._enabled = False

        self._on_dwell_start_handlers: Set[DwellEventHandler] = set()
        self._on_dwell_progress_handlers: Set[DwellEventHandler] = set()
        self._on_dwell_complete_handlers: Set[DwellEventHandler] = set()
        self._on_dwell_cancel_handlers: Set[DwellEventHandler] = set()

    def set_dwell_time(self, ms: int) -> None:
        """Set dwell selection time in milliseconds"""
        self._config.threshold = ms

    def get_dwell_time(self) -> int:
        """Get current dwell time"""
        return self._config.threshold

    def register_target(self, target: GazeTarget) -> None:
        """Register a target for dwell selection"""
        self._targets[target.element_id] = target

    def unregister_target(self, target_id: str) -> None:
        """Unregister a target"""
        self._targets.pop(target_id, None)

    def clear_targets(self) -> None:
        """Clear all targets"""
        self._targets.clear()
        self._cancel_dwell()

    def get_targets(self) -> list:
        """Get all registered targets"""
        return list(self._targets.values())

    def on_dwell_start(self, handler: DwellEventHandler) -> None:
        """Register handler for dwell start"""
        self._on_dwell_start_handlers.add(handler)

    def on_dwell_progress(self, handler: DwellEventHandler) -> None:
        """Register handler for dwell progress"""
        self._on_dwell_progress_handlers.add(handler)

    def on_dwell_complete(self, handler: DwellEventHandler) -> None:
        """Register handler for dwell complete"""
        self._on_dwell_complete_handlers.add(handler)

    def on_dwell_cancel(self, handler: DwellEventHandler) -> None:
        """Register handler for dwell cancel"""
        self._on_dwell_cancel_handlers.add(handler)

    def start(self) -> None:
        """Start dwell detection"""
        if self._enabled:
            return

        self._enabled = True
        self._subscription = self._tracker.subscribe(self._handle_gaze_data)

    def stop(self) -> None:
        """Stop dwell detection"""
        if not self._enabled:
            return

        self._enabled = False
        self._cancel_dwell()

        if self._subscription:
            self._tracker.unsubscribe(self._subscription)
            self._subscription = None

    def pause(self) -> None:
        """Temporarily pause dwell detection"""
        self._enabled = False
        self._cancel_dwell()

    def resume(self) -> None:
        """Resume dwell detection"""
        self._enabled = True

    def get_state(self) -> DwellState:
        """Get current dwell state"""
        return DwellState(
            active=self._dwell_state.active,
            progress=self._dwell_state.progress,
            target=self._dwell_state.target,
            start_time=self._dwell_state.start_time,
        )

    def dispose(self) -> None:
        """Dispose resources"""
        self.stop()
        self._targets.clear()
        self._on_dwell_start_handlers.clear()
        self._on_dwell_progress_handlers.clear()
        self._on_dwell_complete_handlers.clear()
        self._on_dwell_cancel_handlers.clear()

    def _handle_gaze_data(self, gaze: GazePoint) -> None:
        if not self._enabled or not gaze.valid:
            if self._dwell_state.active:
                self._cancel_dwell()
            return

        # Check cooldown
        if time.time() * 1000 < self._cooldown_until:
            return

        # Find target under gaze
        hit_target = self._find_target_at_point(gaze.x, gaze.y)

        if not hit_target:
            if self._dwell_state.active:
                self._cancel_dwell()
            return

        if not self._dwell_state.active:
            self._start_dwell(hit_target)
        elif self._dwell_state.target and self._dwell_state.target.element_id != hit_target.element_id:
            self._cancel_dwell()
            self._start_dwell(hit_target)

    def _find_target_at_point(self, x: float, y: float) -> Optional[GazeTarget]:
        for target in self._targets.values():
            if target.bounding_box.contains(x, y):
                return target
        return None

    def _start_dwell(self, target: GazeTarget) -> None:
        self._dwell_state = DwellState(
            active=True,
            target=target,
            start_time=int(time.time() * 1000),
            progress=0,
        )

        self._emit("start", target)
        self._start_progress_timer()

    def _start_progress_timer(self) -> None:
        def update():
            self._update_progress()
            if self._dwell_state.active:
                self._progress_timer = threading.Timer(
                    self._config.progress_interval / 1000,
                    update
                )
                self._progress_timer.daemon = True
                self._progress_timer.start()

        self._progress_timer = threading.Timer(
            self._config.progress_interval / 1000,
            update
        )
        self._progress_timer.daemon = True
        self._progress_timer.start()

    def _update_progress(self) -> None:
        if not self._dwell_state.active or not self._dwell_state.start_time or not self._dwell_state.target:
            return

        elapsed = int(time.time() * 1000) - self._dwell_state.start_time
        progress = min(1.0, elapsed / self._config.threshold)

        self._dwell_state.progress = progress
        self._emit("progress", self._dwell_state.target, progress)

        if progress >= 1.0:
            self._complete_dwell()

    def _complete_dwell(self) -> None:
        if not self._dwell_state.target:
            return

        target = self._dwell_state.target
        self._clear_progress_timer()
        self._dwell_state = DwellState(active=False, progress=0)
        self._cooldown_until = time.time() * 1000 + self._config.cooldown_period

        self._emit("complete", target, 1.0)

    def _cancel_dwell(self) -> None:
        if not self._dwell_state.active:
            return

        target = self._dwell_state.target
        self._clear_progress_timer()
        self._dwell_state = DwellState(active=False, progress=0)

        if target:
            self._emit("cancel", target)

    def _clear_progress_timer(self) -> None:
        if self._progress_timer:
            self._progress_timer.cancel()
            self._progress_timer = None

    def _emit(self, event: str, target: GazeTarget, progress: Optional[float] = None) -> None:
        handlers = {
            "start": self._on_dwell_start_handlers,
            "progress": self._on_dwell_progress_handlers,
            "complete": self._on_dwell_complete_handlers,
            "cancel": self._on_dwell_cancel_handlers,
        }.get(event, set())

        for handler in handlers:
            try:
                handler(target, progress)
            except Exception as e:
                print(f"Error in dwell {event} handler: {e}")


def create_dwell_controller(
    tracker: IEyeTracker,
    threshold: int = 800,
    **kwargs
) -> DwellController:
    """Create a dwell controller"""
    return DwellController(tracker=tracker, threshold=threshold, **kwargs)
