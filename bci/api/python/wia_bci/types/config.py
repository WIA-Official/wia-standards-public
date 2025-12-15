"""WIA BCI Configuration Types."""

from dataclasses import dataclass, field
from typing import Literal, Optional, Union

# Type aliases
DeviceType = Literal[
    "eeg_headset",
    "eeg_cap",
    "implant_cortical",
    "implant_endovascular",
    "fnirs",
    "hybrid",
    "simulator",
]

ConnectionProtocol = Literal[
    "usb",
    "bluetooth",
    "bluetooth_le",
    "wifi",
    "serial",
    "lsl",
]

LogLevel = Literal["debug", "info", "warn", "error"]

FilterType = Literal["highpass", "lowpass", "bandpass", "notch"]


@dataclass
class FilterConfig:
    """Filter configuration."""

    type: FilterType
    frequency: Union[float, tuple[float, float]]
    order: int = 4


@dataclass
class WiaBciOptions:
    """WiaBci constructor options."""

    auto_reconnect: bool = True
    reconnect_interval: float = 3.0
    max_reconnect_attempts: int = 5
    buffer_size: int = 1000
    log_level: LogLevel = "info"


@dataclass
class DeviceIdentifier:
    """Device identifier."""

    manufacturer: Optional[str] = None
    model: Optional[str] = None
    serial_number: Optional[str] = None


@dataclass
class ConnectionConfig:
    """Connection configuration."""

    protocol: Optional[ConnectionProtocol] = None
    address: Optional[str] = None
    port: Optional[int] = None


@dataclass
class AcquisitionConfig:
    """Acquisition configuration."""

    sampling_rate: Optional[int] = None
    channels: Optional[list[str]] = None
    filters: Optional[list[FilterConfig]] = None


@dataclass
class DeviceConfig:
    """Complete device configuration."""

    type: DeviceType
    device: Optional[DeviceIdentifier] = None
    connection: Optional[ConnectionConfig] = None
    acquisition: Optional[AcquisitionConfig] = None
