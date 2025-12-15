"""WIA BCI Error Classes."""

from dataclasses import dataclass
from typing import Any, Optional


class ErrorCodes:
    """Error codes."""

    CONNECTION_FAILED = "E001"
    DEVICE_NOT_FOUND = "E002"
    PERMISSION_DENIED = "E003"
    STREAM_ERROR = "E004"
    INVALID_CONFIG = "E005"
    ADAPTER_ERROR = "E006"
    TIMEOUT = "E007"
    DISCONNECTED = "E008"
    NOT_CONNECTED = "E009"
    ALREADY_CONNECTED = "E010"
    INVALID_STATE = "E011"


class BciError(Exception):
    """BCI Error class."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        recoverable: bool = False,
        details: Optional[Any] = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.recoverable = recoverable
        self.details = details

    def to_event(self) -> dict[str, Any]:
        """Convert to ErrorEvent format."""
        return {
            "code": self.code,
            "message": self.message,
            "details": self.details,
            "recoverable": self.recoverable,
        }

    @classmethod
    def connection_failed(
        cls, message: str, details: Optional[Any] = None
    ) -> "BciError":
        """Create ConnectionFailed error."""
        return cls(
            ErrorCodes.CONNECTION_FAILED,
            message,
            recoverable=True,
            details=details,
        )

    @classmethod
    def device_not_found(cls, message: str = "Device not found") -> "BciError":
        """Create DeviceNotFound error."""
        return cls(ErrorCodes.DEVICE_NOT_FOUND, message, recoverable=False)

    @classmethod
    def not_connected(cls) -> "BciError":
        """Create NotConnected error."""
        return cls(
            ErrorCodes.NOT_CONNECTED,
            "Not connected to any device",
            recoverable=False,
        )

    @classmethod
    def already_connected(cls) -> "BciError":
        """Create AlreadyConnected error."""
        return cls(
            ErrorCodes.ALREADY_CONNECTED,
            "Already connected to a device",
            recoverable=False,
        )

    @classmethod
    def invalid_config(
        cls, message: str, details: Optional[Any] = None
    ) -> "BciError":
        """Create InvalidConfig error."""
        return cls(
            ErrorCodes.INVALID_CONFIG,
            message,
            recoverable=False,
            details=details,
        )

    @classmethod
    def timeout(cls, message: str = "Operation timed out") -> "BciError":
        """Create Timeout error."""
        return cls(ErrorCodes.TIMEOUT, message, recoverable=True)

    @classmethod
    def stream_error(
        cls, message: str, details: Optional[Any] = None
    ) -> "BciError":
        """Create StreamError."""
        return cls(
            ErrorCodes.STREAM_ERROR,
            message,
            recoverable=True,
            details=details,
        )
