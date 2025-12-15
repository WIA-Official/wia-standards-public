"""WIA BCI Simulator Adapter."""

import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from wia_bci.adapters.base_adapter import BaseBciAdapter
from wia_bci.types.config import DeviceConfig, DeviceType
from wia_bci.types.device import ChannelInfo, DeviceCapabilities, DeviceInfo
from wia_bci.types.events import SignalEvent


@dataclass
class SimulatorOptions:
    """Simulator configuration options."""

    channels: int = 8
    sampling_rate: int = 250
    alpha: bool = True
    noise: bool = True
    noise_amplitude: float = 10.0


class SimulatorAdapter(BaseBciAdapter):
    """
    Simulator Adapter.

    Generates realistic synthetic EEG signals for development and testing.
    """

    def __init__(self, options: Optional[SimulatorOptions] = None) -> None:
        super().__init__()
        self._options = options or SimulatorOptions()
        self._task: Optional[asyncio.Task] = None
        self._sample_index = 0
        self._start_time = 0.0

    @property
    def type(self) -> DeviceType:
        return "simulator"

    @property
    def name(self) -> str:
        return "WIA BCI Simulator"

    async def connect(self, config: DeviceConfig) -> None:
        """Connect to simulator."""
        # Apply config overrides
        if config.acquisition and config.acquisition.sampling_rate:
            self._options.sampling_rate = config.acquisition.sampling_rate
        if config.acquisition and config.acquisition.channels:
            self._options.channels = len(config.acquisition.channels)

        # Create device info
        self._device_info = DeviceInfo(
            id=f"simulator-{int(time.time() * 1000)}",
            name="WIA BCI Simulator",
            type="simulator",
            manufacturer="WIA",
            model="Simulator v1.0",
            firmware_version="1.0.0",
            capabilities=DeviceCapabilities(
                channels=self._options.channels,
                max_sampling_rate=250,
                supported_sampling_rates=[125, 250],
                resolution=24,
            ),
            status="connected",
        )

        # Create channel info
        channel_labels = ["Fp1", "Fp2", "F3", "F4", "C3", "C4", "O1", "O2"]
        self._channels = [
            ChannelInfo(
                index=i,
                label=channel_labels[i] if i < len(channel_labels) else f"CH{i + 1}",
                type="eeg",
                unit="µV",
                sampling_rate=self._options.sampling_rate,
                enabled=True,
            )
            for i in range(self._options.channels)
        ]

        self._connected = True

    async def disconnect(self) -> None:
        """Disconnect from simulator."""
        if self._streaming:
            await self.stop_stream()
        self._connected = False
        self._device_info = None
        self._channels = []

    async def start_stream(self) -> None:
        """Start data streaming."""
        if not self._connected:
            raise RuntimeError("Not connected")
        if self._streaming:
            return

        self._sample_index = 0
        self._start_time = time.time()
        self._streaming = True

        # Start async generation task
        self._task = asyncio.create_task(self._generate_samples())

    async def stop_stream(self) -> None:
        """Stop data streaming."""
        self._streaming = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _generate_samples(self) -> None:
        """Generate synthetic EEG samples."""
        interval = 1.0 / self._options.sampling_rate

        while self._streaming:
            timestamp = time.time()
            t = timestamp - self._start_time

            data = np.zeros(self._options.channels, dtype=np.float32)

            for ch in range(self._options.channels):
                value = 0.0

                # Add alpha rhythm (10 Hz sine wave, ~50 µV amplitude)
                if self._options.alpha:
                    alpha_amplitude = 50.0 if ch >= 6 else 25.0
                    value += alpha_amplitude * math.sin(
                        2 * math.pi * 10 * t + ch * 0.5
                    )

                # Add theta (6 Hz)
                value += 15.0 * math.sin(2 * math.pi * 6 * t + ch * 0.3)

                # Add beta (20 Hz, smaller amplitude)
                value += 8.0 * math.sin(2 * math.pi * 20 * t + ch * 0.7)

                # Add noise
                if self._options.noise:
                    value += (np.random.random() - 0.5) * 2 * self._options.noise_amplitude

                data[ch] = value

            event = SignalEvent(
                timestamp=timestamp * 1000,  # Convert to ms
                sample_index=self._sample_index,
                channels=list(range(self._options.channels)),
                data=data,
            )

            self._sample_index += 1
            self._emit_data(event)

            await asyncio.sleep(interval)
