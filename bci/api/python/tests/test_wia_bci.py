"""WiaBci Tests."""

import asyncio
import numpy as np
import pytest

from wia_bci import WiaBci, BciError, SignalProcessor, DeviceConfig


class TestWiaBci:
    """Tests for WiaBci class."""

    @pytest.fixture
    def bci(self):
        """Create WiaBci instance."""
        from wia_bci import WiaBciOptions
        return WiaBci(WiaBciOptions(log_level="error"))

    @pytest.mark.asyncio
    async def test_connect_to_simulator(self, bci):
        """Should connect to simulator."""
        await bci.connect(DeviceConfig(type="simulator"))
        assert bci.is_connected() is True
        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_throw_when_connecting_twice(self, bci):
        """Should throw when connecting twice."""
        await bci.connect(DeviceConfig(type="simulator"))

        with pytest.raises(BciError):
            await bci.connect(DeviceConfig(type="simulator"))

        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_disconnect(self, bci):
        """Should disconnect."""
        await bci.connect(DeviceConfig(type="simulator"))
        await bci.disconnect()
        assert bci.is_connected() is False

    @pytest.mark.asyncio
    async def test_throw_when_disconnecting_without_connection(self, bci):
        """Should throw when disconnecting without connection."""
        with pytest.raises(BciError):
            await bci.disconnect()

    @pytest.mark.asyncio
    async def test_start_and_stop_stream(self, bci):
        """Should start and stop stream."""
        await bci.connect(DeviceConfig(type="simulator"))

        await bci.start_stream()
        assert bci.is_streaming() is True

        await bci.stop_stream()
        assert bci.is_streaming() is False

        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_emit_signal_events(self, bci):
        """Should emit signal events."""
        await bci.connect(DeviceConfig(type="simulator"))

        signals = []

        @bci.on("signal")
        def on_signal(event):
            signals.append(event)

        await bci.start_stream()
        await asyncio.sleep(0.1)
        await bci.stop_stream()

        assert len(signals) > 0
        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_get_device_info(self, bci):
        """Should return device info when connected."""
        await bci.connect(DeviceConfig(type="simulator"))

        info = bci.get_device_info()
        assert info is not None
        assert info.type == "simulator"

        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_get_channels(self, bci):
        """Should return channels when connected."""
        await bci.connect(DeviceConfig(type="simulator"))

        channels = bci.get_channels()
        assert len(channels) > 0

        await bci.disconnect()

    @pytest.mark.asyncio
    async def test_list_devices(self, bci):
        """Should list available devices."""
        devices = await bci.list_devices()
        assert len(devices) > 0
        assert devices[0].type == "simulator"


class TestSignalProcessor:
    """Tests for SignalProcessor."""

    @staticmethod
    def create_test_signal(length: int, freq: float, fs: float) -> np.ndarray:
        """Create test sine wave signal."""
        t = np.arange(length) / fs
        return np.sin(2 * np.pi * freq * t).astype(np.float32)

    def test_highpass_filter(self):
        """Should apply highpass filter."""
        data = self.create_test_signal(250, 10, 250)
        filtered = SignalProcessor.highpass(data, 5, 250)
        assert len(filtered) == len(data)

    def test_lowpass_filter(self):
        """Should apply lowpass filter."""
        data = self.create_test_signal(250, 10, 250)
        filtered = SignalProcessor.lowpass(data, 15, 250)
        assert len(filtered) == len(data)

    def test_bandpass_filter(self):
        """Should apply bandpass filter."""
        data = self.create_test_signal(250, 10, 250)
        filtered = SignalProcessor.bandpass(data, 8, 13, 250)
        assert len(filtered) == len(data)

    def test_all_band_powers(self):
        """Should calculate all band powers."""
        data = self.create_test_signal(500, 10, 250)
        powers = SignalProcessor.all_band_powers(data, 250)

        assert powers.alpha > 0
        assert isinstance(powers.delta, float)
        assert isinstance(powers.theta, float)
        assert isinstance(powers.beta, float)
        assert isinstance(powers.gamma, float)

    def test_mean(self):
        """Should calculate mean."""
        data = np.array([1, 2, 3, 4, 5], dtype=np.float32)
        assert SignalProcessor.mean(data) == 3.0

    def test_variance(self):
        """Should calculate variance."""
        data = np.array([1, 2, 3, 4, 5], dtype=np.float32)
        assert SignalProcessor.variance(data) == 2.0

    def test_zero_crossings(self):
        """Should count zero crossings."""
        data = np.array([-1, 1, -1, 1, -1], dtype=np.float32)
        assert SignalProcessor.zero_crossings(data) == 4

    def test_epoch_extraction(self):
        """Should extract epoch."""
        data = np.arange(10, dtype=np.float32)
        epoch = SignalProcessor.epoch(data, 2, 5)

        assert len(epoch) == 3
        assert epoch[0] == 2
        assert epoch[2] == 4

    def test_sliding_windows(self):
        """Should generate sliding windows."""
        data = np.arange(10, dtype=np.float32)
        windows = SignalProcessor.sliding(data, 3, 2)

        assert len(windows) == 4
        assert windows[0][0] == 0
        assert windows[1][0] == 2
