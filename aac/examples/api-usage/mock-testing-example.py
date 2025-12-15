#!/usr/bin/env python3
"""
WIA AAC Standard - Mock Adapter Testing Example

This example demonstrates how to use the MockAdapter for testing
AAC applications without real hardware.
"""

import asyncio
from wia_aac import (
    WiaAac,
    WiaAacOptions,
    MockAdapter,
    SensorType,
    EventType,
    SensorConfig
)


async def test_eye_tracker():
    """Test eye tracker functionality with mock adapter"""
    print("=== Eye Tracker Mock Test ===")

    aac = WiaAac(WiaAacOptions(log_level="none"))
    mock = MockAdapter(sensor_type="eye_tracker")
    aac.use_adapter(mock)

    signals_received = []

    def on_signal(signal):
        signals_received.append(signal)
        gaze = signal.data.get("gaze_point", {})
        print(f"  Gaze at ({gaze.get('x', 0):.2f}, {gaze.get('y', 0):.2f})")

    aac.on(EventType.SIGNAL, on_signal)

    await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))
    print(f"Connected: {aac.is_connected()}")

    # Emit several mock signals
    for i in range(5):
        mock.emit_mock_signal()
        await asyncio.sleep(0.1)

    await aac.disconnect()
    print(f"Received {len(signals_received)} signals\n")


async def test_switch():
    """Test switch functionality with mock adapter"""
    print("=== Switch Mock Test ===")

    aac = WiaAac(WiaAacOptions(log_level="none"))
    mock = MockAdapter(sensor_type="switch")
    aac.use_adapter(mock)

    def on_signal(signal):
        print(f"  Switch {signal.data['switch_id']}: {signal.data['state']}")

    aac.on(EventType.SIGNAL, on_signal)

    await aac.connect(SensorConfig(type=SensorType.SWITCH))

    # Emit mock switch signals
    for i in range(3):
        mock.emit_mock_signal()
        await asyncio.sleep(0.1)

    await aac.disconnect()
    print()


async def test_all_sensors():
    """Test all sensor types"""
    sensor_types = [
        ("eye_tracker", SensorType.EYE_TRACKER),
        ("switch", SensorType.SWITCH),
        ("muscle_sensor", SensorType.MUSCLE_SENSOR),
        ("brain_interface", SensorType.BRAIN_INTERFACE),
        ("breath", SensorType.BREATH),
        ("head_movement", SensorType.HEAD_MOVEMENT),
    ]

    print("=== All Sensors Test ===")

    for sensor_name, sensor_type in sensor_types:
        aac = WiaAac(WiaAacOptions(log_level="none"))
        mock = MockAdapter(sensor_type=sensor_name)
        aac.use_adapter(mock)

        received = [None]  # Use list to allow mutation in closure

        def make_handler(container):
            def on_signal(signal):
                container[0] = signal
            return on_signal

        aac.on(EventType.SIGNAL, make_handler(received))

        await aac.connect(SensorConfig(type=sensor_type))
        mock.emit_mock_signal()
        await aac.disconnect()

        status = "OK" if received[0] and received[0].type == sensor_type else "FAIL"
        print(f"  {sensor_name}: {status}")

    print()


async def main():
    await test_eye_tracker()
    await test_switch()
    await test_all_sensors()
    print("All tests completed!")


if __name__ == "__main__":
    asyncio.run(main())
