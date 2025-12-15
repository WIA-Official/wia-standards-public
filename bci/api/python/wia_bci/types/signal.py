"""WIA BCI Signal Types."""

from dataclasses import dataclass
from typing import Literal, Union

FrequencyBand = Union[
    Literal["delta", "theta", "alpha", "beta", "gamma"],
    tuple[float, float],
]


@dataclass
class BandPowers:
    """Frequency band powers."""

    delta: float  # 0.5-4 Hz
    theta: float  # 4-8 Hz
    alpha: float  # 8-13 Hz
    beta: float   # 13-30 Hz
    gamma: float  # 30-100 Hz


@dataclass
class PowerSpectrum:
    """Power spectral density result."""

    frequencies: list[float]
    powers: list[float]
    resolution: float


# Frequency band ranges (Hz)
FREQUENCY_BANDS: dict[str, tuple[float, float]] = {
    "delta": (0.5, 4.0),
    "theta": (4.0, 8.0),
    "alpha": (8.0, 13.0),
    "beta": (13.0, 30.0),
    "gamma": (30.0, 100.0),
}
