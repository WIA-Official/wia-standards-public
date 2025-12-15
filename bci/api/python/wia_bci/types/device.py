"""WIA BCI Device Types."""

from dataclasses import dataclass, field
from typing import Literal, Optional

from wia_bci.types.config import ConnectionProtocol, DeviceType

DeviceStatus = Literal[
    "available",
    "connected",
    "streaming",
    "error",
    "disconnected",
]

ChannelType = Literal[
    "eeg",
    "emg",
    "eog",
    "ecg",
    "trigger",
    "accel",
    "aux",
]


@dataclass
class DeviceCapabilities:
    """Device capabilities."""

    channels: int
    max_sampling_rate: int
    supported_sampling_rates: list[int]
    resolution: int
    has_accelerometer: bool = False
    has_gyroscope: bool = False
    has_impedance_check: bool = False
    has_battery_indicator: bool = False
    supported_protocols: list[ConnectionProtocol] = field(default_factory=list)


@dataclass
class DeviceInfo:
    """Device information."""

    id: str
    name: str
    type: DeviceType
    manufacturer: str
    model: str
    capabilities: DeviceCapabilities
    status: DeviceStatus
    serial_number: Optional[str] = None
    firmware_version: Optional[str] = None


@dataclass
class ChannelInfo:
    """Channel information."""

    index: int
    label: str
    type: ChannelType
    unit: str
    enabled: bool = True
    sampling_rate: Optional[int] = None


@dataclass
class BciState:
    """Current BCI state."""

    connected: bool
    streaming: bool
    device: Optional[DeviceInfo]
    channels: list[ChannelInfo]
    sampling_rate: int
    samples_received: int
    last_error: Optional[str]


# Standard electrode positions
STANDARD_10_20_CHANNELS = [
    "Fp1", "Fp2",
    "F7", "F3", "Fz", "F4", "F8",
    "T3", "C3", "Cz", "C4", "T4",
    "T5", "P3", "Pz", "P4", "T6",
    "O1", "O2",
]

STANDARD_10_10_CHANNELS = [
    "Fp1", "Fpz", "Fp2",
    "AF7", "AF3", "AFz", "AF4", "AF8",
    "F7", "F5", "F3", "F1", "Fz", "F2", "F4", "F6", "F8",
    "FT7", "FC5", "FC3", "FC1", "FCz", "FC2", "FC4", "FC6", "FT8",
    "T7", "C5", "C3", "C1", "Cz", "C2", "C4", "C6", "T8",
    "TP7", "CP5", "CP3", "CP1", "CPz", "CP2", "CP4", "CP6", "TP8",
    "P7", "P5", "P3", "P1", "Pz", "P2", "P4", "P6", "P8",
    "PO7", "PO3", "POz", "PO4", "PO8",
    "O1", "Oz", "O2",
]
