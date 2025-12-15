"""WIA BCI Signal Processor."""

import math
from typing import Union

import numpy as np
from numpy.typing import NDArray

from wia_bci.types.signal import BandPowers, FrequencyBand, PowerSpectrum, FREQUENCY_BANDS


class SignalProcessor:
    """Signal processing utilities."""

    @staticmethod
    def highpass(
        data: NDArray[np.float32], cutoff: float, fs: float
    ) -> NDArray[np.float32]:
        """Apply high-pass filter (first-order IIR)."""
        result = np.zeros_like(data)
        rc = 1.0 / (2.0 * math.pi * cutoff)
        dt = 1.0 / fs
        alpha = rc / (rc + dt)

        result[0] = data[0]
        for i in range(1, len(data)):
            result[i] = alpha * (result[i - 1] + data[i] - data[i - 1])

        return result

    @staticmethod
    def lowpass(
        data: NDArray[np.float32], cutoff: float, fs: float
    ) -> NDArray[np.float32]:
        """Apply low-pass filter (first-order IIR)."""
        result = np.zeros_like(data)
        rc = 1.0 / (2.0 * math.pi * cutoff)
        dt = 1.0 / fs
        alpha = dt / (rc + dt)

        result[0] = data[0]
        for i in range(1, len(data)):
            result[i] = result[i - 1] + alpha * (data[i] - result[i - 1])

        return result

    @staticmethod
    def bandpass(
        data: NDArray[np.float32], low: float, high: float, fs: float
    ) -> NDArray[np.float32]:
        """Apply band-pass filter."""
        hp = SignalProcessor.highpass(data, low, fs)
        return SignalProcessor.lowpass(hp, high, fs)

    @staticmethod
    def notch(
        data: NDArray[np.float32], freq: float, fs: float, q: float = 30.0
    ) -> NDArray[np.float32]:
        """Apply notch filter (e.g., for 50/60 Hz line noise)."""
        result = np.zeros_like(data)
        w0 = (2.0 * math.pi * freq) / fs
        alpha = math.sin(w0) / (2.0 * q)

        b0 = 1.0
        b1 = -2.0 * math.cos(w0)
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * math.cos(w0)
        a2 = 1.0 - alpha

        # Normalize
        nb0 = b0 / a0
        nb1 = b1 / a0
        nb2 = b2 / a0
        na1 = a1 / a0
        na2 = a2 / a0

        x1, x2, y1, y2 = 0.0, 0.0, 0.0, 0.0

        for i in range(len(data)):
            x = data[i]
            y = nb0 * x + nb1 * x1 + nb2 * x2 - na1 * y1 - na2 * y2

            x2, x1 = x1, x
            y2, y1 = y1, y

            result[i] = y

        return result

    @staticmethod
    def psd(data: NDArray[np.float32], fs: float) -> PowerSpectrum:
        """Compute Power Spectral Density using FFT."""
        n = len(data)
        nfft = int(2 ** math.ceil(math.log2(n)))

        # Apply Hanning window
        window = 0.5 * (1.0 - np.cos(2.0 * math.pi * np.arange(n) / (n - 1)))
        windowed = np.zeros(nfft, dtype=np.float32)
        windowed[:n] = data * window

        # FFT
        fft_result = np.fft.fft(windowed)
        powers = np.abs(fft_result[: nfft // 2]) ** 2 / nfft
        frequencies = np.fft.fftfreq(nfft, 1.0 / fs)[: nfft // 2]

        return PowerSpectrum(
            frequencies=frequencies.tolist(),
            powers=powers.tolist(),
            resolution=fs / nfft,
        )

    @staticmethod
    def bandpower(
        data: NDArray[np.float32], fs: float, band: FrequencyBand
    ) -> float:
        """Calculate power in a frequency band."""
        if isinstance(band, str):
            low, high = FREQUENCY_BANDS[band]
        else:
            low, high = band

        spectrum = SignalProcessor.psd(data, fs)
        power = 0.0
        count = 0

        for freq, p in zip(spectrum.frequencies, spectrum.powers):
            if low <= freq <= high:
                power += p
                count += 1

        return power / count if count > 0 else 0.0

    @staticmethod
    def all_band_powers(data: NDArray[np.float32], fs: float) -> BandPowers:
        """Calculate all standard frequency band powers."""
        return BandPowers(
            delta=SignalProcessor.bandpower(data, fs, "delta"),
            theta=SignalProcessor.bandpower(data, fs, "theta"),
            alpha=SignalProcessor.bandpower(data, fs, "alpha"),
            beta=SignalProcessor.bandpower(data, fs, "beta"),
            gamma=SignalProcessor.bandpower(data, fs, "gamma"),
        )

    @staticmethod
    def epoch(
        data: NDArray[np.float32], start: int, end: int
    ) -> NDArray[np.float32]:
        """Extract epoch from continuous data."""
        return data[start:end].copy()

    @staticmethod
    def sliding(
        data: NDArray[np.float32], window_size: int, step: int
    ) -> list[NDArray[np.float32]]:
        """Generate sliding windows."""
        windows = []
        for start in range(0, len(data) - window_size + 1, step):
            windows.append(data[start : start + window_size].copy())
        return windows

    @staticmethod
    def detect_blinks(
        eog: NDArray[np.float32], threshold: float = 100.0
    ) -> list[int]:
        """Detect eye blinks in EOG channel."""
        blinks = []
        in_blink = False

        for i, val in enumerate(eog):
            if abs(val) > threshold and not in_blink:
                blinks.append(i)
                in_blink = True
            elif abs(val) < threshold / 2:
                in_blink = False

        return blinks

    @staticmethod
    def rms(data: NDArray[np.float32]) -> float:
        """Calculate RMS (Root Mean Square)."""
        return float(np.sqrt(np.mean(data**2)))

    @staticmethod
    def mean(data: NDArray[np.float32]) -> float:
        """Calculate mean."""
        return float(np.mean(data))

    @staticmethod
    def variance(data: NDArray[np.float32]) -> float:
        """Calculate variance."""
        return float(np.var(data))

    @staticmethod
    def zero_crossings(data: NDArray[np.float32]) -> int:
        """Count zero crossings."""
        return int(np.sum(np.diff(np.sign(data)) != 0))
