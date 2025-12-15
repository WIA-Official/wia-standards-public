"""
WIA Eye Gaze Standard - Gaze Aware Application

弘益人間 - 널리 인간을 이롭게
"""

import time
import json
from typing import Optional, Dict, Set, Callable
from dataclasses import dataclass
from enum import Enum

from .types import GazeTarget


class GazeAppMessageType(Enum):
    """App message types"""
    APP_ANNOUNCE = "app_announce"
    APP_QUERY = "app_query"
    APP_RESPONSE = "app_response"
    CONTROL_REQUEST = "control_request"
    CONTROL_GRANT = "control_grant"
    CONTROL_DENY = "control_deny"
    CONTROL_RELEASE = "control_release"
    PAUSE_TRACKING = "pause_tracking"
    RESUME_TRACKING = "resume_tracking"
    TARGET_SYNC = "target_sync"


@dataclass
class AppCapabilities:
    """Application capabilities"""
    name: str
    version: str
    supports_control: bool
    supports_dwell: bool
    supports_blink: bool
    priority: int = 0


@dataclass
class GazeAppMessage:
    """Inter-app message"""
    type: GazeAppMessageType
    app_id: str
    timestamp: int
    payload: Optional[dict] = None


ControlRequestHandler = Callable[[str], bool]
MessageHandler = Callable[[GazeAppMessage], None]


class GazeAwareApp:
    """
    Gaze-aware application for ecosystem participation.

    Example:
        >>> app = GazeAwareApp(
        ...     app_id="my-aac-app",
        ...     capabilities=AppCapabilities(
        ...         name="My AAC App",
        ...         version="1.0.0",
        ...         supports_control=True,
        ...         supports_dwell=True,
        ...         supports_blink=False,
        ...     ),
        ... )
        >>> await app.register()
        >>> app.announce_gaze_control()
    """

    def __init__(
        self,
        app_id: str,
        capabilities: AppCapabilities,
        transport_type: str = "memory",
    ):
        self._app_id = app_id
        self._capabilities = capabilities
        self._transport_type = transport_type

        self._registered = False
        self._has_control = False
        self._targets: Dict[str, GazeTarget] = {}
        self._control_request_handler: Optional[ControlRequestHandler] = None
        self._message_handlers: Set[MessageHandler] = set()
        self._known_apps: Dict[str, AppCapabilities] = {}

    @property
    def app_id(self) -> str:
        return self._app_id

    async def register(self) -> None:
        """Register with the gaze-aware ecosystem"""
        if self._registered:
            return

        self._registered = True

        # Announce presence
        self._send_message(GazeAppMessage(
            type=GazeAppMessageType.APP_ANNOUNCE,
            app_id=self._app_id,
            timestamp=int(time.time() * 1000),
            payload={
                "name": self._capabilities.name,
                "version": self._capabilities.version,
                "supports_control": self._capabilities.supports_control,
                "supports_dwell": self._capabilities.supports_dwell,
                "supports_blink": self._capabilities.supports_blink,
                "priority": self._capabilities.priority,
            },
        ))

    async def unregister(self) -> None:
        """Unregister from ecosystem"""
        if not self._registered:
            return

        if self._has_control:
            self.release_gaze_control()

        self._registered = False

    def is_registered(self) -> bool:
        return self._registered

    def announce_gaze_control(self) -> None:
        """Announce taking gaze control"""
        self._send_message(GazeAppMessage(
            type=GazeAppMessageType.CONTROL_REQUEST,
            app_id=self._app_id,
            timestamp=int(time.time() * 1000),
            payload={"priority": self._capabilities.priority},
        ))

    def release_gaze_control(self) -> None:
        """Release gaze control"""
        if not self._has_control:
            return

        self._has_control = False
        self._send_message(GazeAppMessage(
            type=GazeAppMessageType.CONTROL_RELEASE,
            app_id=self._app_id,
            timestamp=int(time.time() * 1000),
        ))

    def has_gaze_control(self) -> bool:
        return self._has_control

    def on_gaze_control_request(self, handler: ControlRequestHandler) -> None:
        """Register handler for control requests"""
        self._control_request_handler = handler

    def register_target(self, target: GazeTarget) -> None:
        """Register a gaze target"""
        self._targets[target.element_id] = target

    def unregister_target(self, target_id: str) -> None:
        """Unregister a target"""
        self._targets.pop(target_id, None)

    def get_active_targets(self) -> list:
        """Get active targets"""
        return list(self._targets.values())

    def get_known_apps(self) -> Dict[str, AppCapabilities]:
        """Get known apps"""
        return dict(self._known_apps)

    def on_message(self, handler: MessageHandler) -> None:
        """Register message handler"""
        self._message_handlers.add(handler)

    def off_message(self, handler: MessageHandler) -> None:
        """Remove message handler"""
        self._message_handlers.discard(handler)

    def _send_message(self, message: GazeAppMessage) -> None:
        if not self._registered:
            return
        # In real implementation, send via transport
        pass

    def _handle_message(self, message: GazeAppMessage) -> None:
        if message.app_id == self._app_id:
            return

        if message.type == GazeAppMessageType.APP_ANNOUNCE:
            self._handle_app_announce(message)
        elif message.type == GazeAppMessageType.CONTROL_REQUEST:
            self._handle_control_request(message)
        elif message.type == GazeAppMessageType.CONTROL_GRANT:
            self._has_control = True
        elif message.type == GazeAppMessageType.CONTROL_DENY:
            pass

        for handler in self._message_handlers:
            try:
                handler(message)
            except Exception as e:
                print(f"Error in message handler: {e}")

    def _handle_app_announce(self, message: GazeAppMessage) -> None:
        if message.payload:
            self._known_apps[message.app_id] = AppCapabilities(
                name=message.payload.get("name", ""),
                version=message.payload.get("version", ""),
                supports_control=message.payload.get("supports_control", False),
                supports_dwell=message.payload.get("supports_dwell", False),
                supports_blink=message.payload.get("supports_blink", False),
                priority=message.payload.get("priority", 0),
            )

    def _handle_control_request(self, message: GazeAppMessage) -> None:
        if not self._has_control:
            return

        granted = False
        if self._control_request_handler:
            granted = self._control_request_handler(message.app_id)

        self._send_message(GazeAppMessage(
            type=GazeAppMessageType.CONTROL_GRANT if granted else GazeAppMessageType.CONTROL_DENY,
            app_id=self._app_id,
            timestamp=int(time.time() * 1000),
            payload={"requesting_app": message.app_id},
        ))

        if granted:
            self._has_control = False

    def dispose(self) -> None:
        """Dispose resources"""
        import asyncio
        try:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.unregister())
        except RuntimeError:
            self._registered = False
        self._targets.clear()
        self._known_apps.clear()
        self._message_handlers.clear()


def create_gaze_aware_app(
    app_id: str,
    capabilities: AppCapabilities,
    **kwargs
) -> GazeAwareApp:
    """Create a gaze-aware application"""
    return GazeAwareApp(app_id=app_id, capabilities=capabilities, **kwargs)
