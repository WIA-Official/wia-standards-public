#!/usr/bin/env python3
"""
WIA AAC Standard - Python API Example

This example demonstrates how to use the WIA AAC Python API
to connect to sensors, handle signals, and process selections.
"""

import asyncio
from wia_aac import (
    WiaAac,
    WiaAacOptions,
    SensorType,
    EventType,
    ConnectionState,
    SensorConfig,
    DeviceFilter,
    SensorOptions
)


async def main():
    # Create WiaAac instance with options
    aac = WiaAac(WiaAacOptions(
        auto_reconnect=True,
        reconnect_interval=3000,
        max_reconnect_attempts=5,
        signal_buffer_size=100,
        validate_signals=True,
        log_level="info"
    ))

    # Register event handlers
    def on_signal(signal):
        print(f"Signal received: {signal.type.value}")

        if signal.type == SensorType.EYE_TRACKER:
            gaze = signal.data.get("gaze_point")
            if gaze:
                print(f"  Gaze: ({gaze['x']}, {gaze['y']})")

        elif signal.type == SensorType.SWITCH:
            print(f"  Switch {signal.data['switch_id']}: {signal.data['state']}")

        elif signal.type == SensorType.MUSCLE_SENSOR:
            activation = signal.data.get("activation_level", 0)
            print(f"  EMG Activation: {activation * 100:.1f}%")

        elif signal.type == SensorType.BRAIN_INTERFACE:
            command = signal.data.get("mental_command", "neutral")
            print(f"  BCI Command: {command}")

        elif signal.type == SensorType.BREATH:
            print(f"  Breath: {signal.data['action']}")

        elif signal.type == SensorType.HEAD_MOVEMENT:
            pos = signal.data["position"]
            print(f"  Head position: ({pos['x']}, {pos['y']})")

    def on_selection(event):
        print(f"Selection: {event.target_id} via {event.method}")

    def on_error(error):
        print(f"Error: {error.message} ({error.code})")

    def on_connected(info):
        print(f"Connected to: {info.get('name', 'Unknown')}")

    def on_disconnected(reason):
        print(f"Disconnected: {reason}")

    aac.on(EventType.SIGNAL, on_signal)
    aac.on(EventType.SELECTION, on_selection)
    aac.on(EventType.ERROR, on_error)
    aac.on(EventType.CONNECTED, on_connected)
    aac.on(EventType.DISCONNECTED, on_disconnected)

    try:
        # Connect to an eye tracker
        await aac.connect(SensorConfig(
            type=SensorType.EYE_TRACKER,
            device=DeviceFilter(manufacturer="Tobii"),
            options=SensorOptions(
                dwell_time=1000,
                smoothing=0.3
            )
        ))

        print(f"Connection state: {aac.get_connection_state().value}")

        # Let it run for a while (in a real app, this would be event-driven)
        await asyncio.sleep(5)

        # Get last signal
        last_signal = aac.get_last_signal()
        if last_signal:
            print(f"Last signal timestamp: {last_signal.timestamp}")

        # Disconnect
        await aac.disconnect()

    except Exception as e:
        print(f"Failed to connect: {e}")


if __name__ == "__main__":
    asyncio.run(main())
