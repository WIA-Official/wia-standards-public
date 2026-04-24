"""
WIA Haptic Standard - Python API
================================

Hardware-independent haptic feedback API for assistive technology.

Example usage:
    from wia_haptic import HapticDeviceManager, BluetoothAdapter
    from wia_haptic.patterns import STANDARD_PRIMITIVES, success_sequence

    # Create and connect device
    manager = HapticDeviceManager()
    device = BluetoothAdapter()
    manager.register("my_device", device)
    await device.connect()

    # Play pattern
    await device.play(STANDARD_PRIMITIVES["tick"])

    # Play sequence
    await device.play(success_sequence())
"""

__version__ = "1.0.0"
__author__ = "WIA Standards"

from .types import (
    BodyLocation,
    BodyRegion,
    Laterality,
    WaveformType,
    NoiseType,
    EnvelopePreset,
    FrequencyBand,
    IntensityLevel,
    HapticCategory,
    ActuatorType,
    DeviceType,
    ConnectionState,
)

from .patterns import (
    Envelope,
    WaveformConfig,
    HapticPrimitive,
    HapticSequence,
    SpatialPattern,
    ENVELOPE_PRESETS,
    FREQUENCY_BANDS,
    INTENSITY_LEVELS,
    STANDARD_PRIMITIVES,
    STANDARD_SEQUENCES,
)

from .device import (
    HapticCapabilities,
    DeviceConfig,
    UserCalibration,
    IHapticDevice,
    BaseHapticDevice,
    HapticDeviceManager,
    HapticError,
    HapticErrorCode,
)

from .adapters import (
    BluetoothAdapter,
    SerialAdapter,
)

__all__ = [
    # Types
    "BodyLocation",
    "BodyRegion",
    "Laterality",
    "WaveformType",
    "NoiseType",
    "EnvelopePreset",
    "FrequencyBand",
    "IntensityLevel",
    "HapticCategory",
    "ActuatorType",
    "DeviceType",
    "ConnectionState",
    # Patterns
    "Envelope",
    "WaveformConfig",
    "HapticPrimitive",
    "HapticSequence",
    "SpatialPattern",
    "ENVELOPE_PRESETS",
    "FREQUENCY_BANDS",
    "INTENSITY_LEVELS",
    "STANDARD_PRIMITIVES",
    "STANDARD_SEQUENCES",
    # Device
    "HapticCapabilities",
    "DeviceConfig",
    "UserCalibration",
    "IHapticDevice",
    "BaseHapticDevice",
    "HapticDeviceManager",
    "HapticError",
    "HapticErrorCode",
    # Adapters
    "BluetoothAdapter",
    "SerialAdapter",
]
