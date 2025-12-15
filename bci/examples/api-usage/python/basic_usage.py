"""
WIA BCI Basic Usage Example (Python)

Demonstrates basic connection, streaming, and data handling.
"""

import asyncio
from wia_bci import WiaBci, SignalProcessor, DeviceConfig, WiaBciOptions


async def main():
    # Create BCI instance with logging
    bci = WiaBci(WiaBciOptions(log_level="info"))

    print("=== WIA BCI Basic Usage Example ===\n")

    try:
        # List available devices
        print("Scanning for devices...")
        devices = await bci.list_devices()
        print(f"Found {len(devices)} device(s):")
        for d in devices:
            print(f"  - {d.name} ({d.type})")

        # Connect to simulator
        print("\nConnecting to simulator...")
        await bci.connect(
            DeviceConfig(
                type="simulator",
            )
        )

        print("Connected!")
        device_info = bci.get_device_info()
        if device_info:
            print(f"Device: {device_info.name}")

        channels = bci.get_channels()
        print(f"Channels: {', '.join(c.label for c in channels)}")

        # Set up event handlers
        sample_count = 0
        buffer = []

        @bci.on("signal")
        def on_signal(event):
            nonlocal sample_count
            sample_count += 1
            buffer.append(event.data)

            # Log every 250 samples (1 second)
            if sample_count % 250 == 0:
                print(f"\nReceived {sample_count} samples")

                # Calculate band powers for channel 0
                if len(buffer) >= 250:
                    import numpy as np

                    # Extract channel 0 from last 250 samples
                    epoch = np.array([b[0] for b in buffer[-250:]], dtype=np.float32)
                    powers = SignalProcessor.all_band_powers(epoch, 250)

                    print("Band Powers (Ch0):")
                    print(f"  Delta: {powers.delta:.2f}")
                    print(f"  Theta: {powers.theta:.2f}")
                    print(f"  Alpha: {powers.alpha:.2f}")
                    print(f"  Beta:  {powers.beta:.2f}")
                    print(f"  Gamma: {powers.gamma:.2f}")

        @bci.on("error")
        def on_error(error):
            print(f"Error: {error.message}")

        # Start streaming
        print("\nStarting stream...")
        await bci.start_stream()

        # Stream for 5 seconds
        print("Streaming for 5 seconds...")
        await asyncio.sleep(5)

        # Stop and disconnect
        print("\nStopping stream...")
        await bci.stop_stream()

        print("Disconnecting...")
        await bci.disconnect()

        print("\n=== Example Complete ===")
        print(f"Total samples received: {sample_count}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        bci.dispose()


if __name__ == "__main__":
    asyncio.run(main())
