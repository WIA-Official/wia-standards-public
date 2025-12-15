"""
WIA AAC API Tests
"""

import pytest
import asyncio
from wia_aac import WiaAac, MockAdapter, SensorType, ConnectionState, EventType, WiaAacOptions
from wia_aac.types import SensorConfig, DeviceFilter


class TestWiaAac:
    """Test WiaAac class"""

    @pytest.fixture
    def aac(self):
        """Create WiaAac instance"""
        return WiaAac(WiaAacOptions(log_level="none"))

    @pytest.mark.asyncio
    async def test_starts_disconnected(self, aac):
        """Should start disconnected"""
        assert not aac.is_connected()
        assert aac.get_connection_state() == ConnectionState.DISCONNECTED

    @pytest.mark.asyncio
    async def test_connect_with_mock_adapter(self, aac):
        """Should connect with mock adapter"""
        mock_adapter = MockAdapter(sensor_type="eye_tracker")
        aac.use_adapter(mock_adapter)

        await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))

        assert aac.is_connected()
        assert aac.get_connection_state() == ConnectionState.CONNECTED

        await aac.disconnect()

    @pytest.mark.asyncio
    async def test_disconnect_properly(self, aac):
        """Should disconnect properly"""
        mock_adapter = MockAdapter(sensor_type="eye_tracker")
        aac.use_adapter(mock_adapter)

        await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))
        await aac.disconnect()

        assert not aac.is_connected()
        assert aac.get_connection_state() == ConnectionState.DISCONNECTED

    @pytest.mark.asyncio
    async def test_emits_connected_event(self, aac):
        """Should emit connected event"""
        mock_adapter = MockAdapter(sensor_type="switch")
        aac.use_adapter(mock_adapter)

        connected_called = False

        def on_connected(info):
            nonlocal connected_called
            connected_called = True

        aac.once(EventType.CONNECTED, on_connected)
        await aac.connect(SensorConfig(type=SensorType.SWITCH))

        assert connected_called
        await aac.disconnect()

    @pytest.mark.asyncio
    async def test_emits_disconnected_event(self, aac):
        """Should emit disconnected event"""
        mock_adapter = MockAdapter(sensor_type="switch")
        aac.use_adapter(mock_adapter)

        await aac.connect(SensorConfig(type=SensorType.SWITCH))

        disconnected_called = False

        def on_disconnected(reason):
            nonlocal disconnected_called
            disconnected_called = True

        aac.once(EventType.DISCONNECTED, on_disconnected)
        await aac.disconnect()

        assert disconnected_called

    @pytest.mark.asyncio
    async def test_handle_signal_events(self, aac):
        """Should handle signal events"""
        mock_adapter = MockAdapter(sensor_type="eye_tracker")
        aac.use_adapter(mock_adapter)

        await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))

        received_signal = None

        def on_signal(signal):
            nonlocal received_signal
            received_signal = signal

        aac.on(EventType.SIGNAL, on_signal)
        mock_adapter.emit_mock_signal()

        assert received_signal is not None
        assert received_signal.type == SensorType.EYE_TRACKER

        await aac.disconnect()

    @pytest.mark.asyncio
    async def test_stores_signals_in_buffer(self, aac):
        """Should store signals in buffer"""
        mock_adapter = MockAdapter(sensor_type="eye_tracker")
        aac.use_adapter(mock_adapter)

        await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))

        assert aac.get_last_signal() is None

        mock_adapter.emit_mock_signal()

        assert aac.get_last_signal() is not None
        assert len(aac.get_signal_buffer()) == 1

        await aac.disconnect()

    @pytest.mark.asyncio
    async def test_limits_buffer_size(self, aac):
        """Should limit buffer size"""
        small_buffer_aac = WiaAac(WiaAacOptions(log_level="none", signal_buffer_size=5))
        mock_adapter = MockAdapter(sensor_type="switch")
        small_buffer_aac.use_adapter(mock_adapter)

        await small_buffer_aac.connect(SensorConfig(type=SensorType.SWITCH))

        # Emit 10 signals
        for _ in range(10):
            mock_adapter.emit_mock_signal()

        assert len(small_buffer_aac.get_signal_buffer()) == 5

        await small_buffer_aac.disconnect()

    @pytest.mark.asyncio
    async def test_get_config_after_connect(self, aac):
        """Should get config after connect"""
        mock_adapter = MockAdapter(sensor_type="breath")
        aac.use_adapter(mock_adapter)

        assert aac.get_config() is None

        config = SensorConfig(type=SensorType.BREATH)
        await aac.connect(config)

        assert aac.get_config() is not None
        assert aac.get_config().type == SensorType.BREATH

        await aac.disconnect()


class TestMockAdapter:
    """Test MockAdapter class"""

    def test_generates_correct_signal_types(self):
        """Should generate correct signal types"""
        sensor_types = [
            "eye_tracker",
            "switch",
            "muscle_sensor",
            "brain_interface",
            "breath",
            "head_movement"
        ]

        for sensor_type in sensor_types:
            adapter = MockAdapter(sensor_type=sensor_type)
            received_signal = None

            def on_signal(signal):
                nonlocal received_signal
                received_signal = signal

            adapter.on_signal(on_signal)
            adapter.emit_mock_signal()

            assert received_signal is not None
            assert received_signal.type.value == sensor_type
            assert received_signal.version == "1.0.0"
