"""
WIA BCI Motor Imagery Example (Python)

Demonstrates a simple motor imagery BCI with feature extraction.
"""

import asyncio
import numpy as np
from wia_bci import WiaBci, SignalProcessor, DeviceConfig, WiaBciOptions


class SimpleClassifier:
    """Simple rule-based classifier (placeholder for real ML model)."""

    CLASSES = ["rest", "left_hand", "right_hand", "feet"]

    def predict(self, features: dict) -> dict:
        """Predict class based on features."""
        # Very simple rule-based classification (for demo only)
        ratio = features["beta"] / (features["alpha"] + 0.001)

        if ratio < 0.3:
            class_id = 0  # rest
            confidence = 0.7
        elif ratio < 0.5:
            class_id = 1  # left_hand
            confidence = 0.6
        elif ratio < 0.7:
            class_id = 2  # right_hand
            confidence = 0.6
        else:
            class_id = 3  # feet
            confidence = 0.5

        return {
            "class_id": class_id,
            "class_name": self.CLASSES[class_id],
            "confidence": confidence,
        }


async def main():
    bci = WiaBci(WiaBciOptions(log_level="info"))
    classifier = SimpleClassifier()

    print("=== Motor Imagery BCI Example ===\n")

    # Data buffer for epoching
    buffer = []
    EPOCH_SIZE = 250  # 1 second @ 250 Hz
    EPOCH_OVERLAP = 125  # 50% overlap

    try:
        # Connect
        await bci.connect(DeviceConfig(type="simulator"))

        device_info = bci.get_device_info()
        if device_info:
            print(f"Connected to: {device_info.name}")

        # Process incoming signals
        @bci.on("signal")
        def on_signal(event):
            buffer.append(event.data)

            # Process when we have enough data
            if len(buffer) >= EPOCH_SIZE:
                # Extract epoch
                epoch_data = buffer[:EPOCH_SIZE]
                del buffer[:EPOCH_OVERLAP]  # Remove with overlap

                # Extract channels (assuming 8 channels from simulator)
                c3_data = np.array([s[0] for s in epoch_data], dtype=np.float32)
                c4_data = np.array([s[1] for s in epoch_data], dtype=np.float32)

                # Band-pass filter (8-30 Hz for mu/beta)
                c3_filtered = SignalProcessor.bandpass(c3_data, 8, 30, 250)
                c4_filtered = SignalProcessor.bandpass(c4_data, 8, 30, 250)

                # Calculate band powers
                c3_powers = SignalProcessor.all_band_powers(c3_filtered, 250)
                c4_powers = SignalProcessor.all_band_powers(c4_filtered, 250)

                # Average powers from both motor cortex channels
                avg_alpha = (c3_powers.alpha + c4_powers.alpha) / 2
                avg_beta = (c3_powers.beta + c4_powers.beta) / 2

                # Classify
                prediction = classifier.predict({
                    "alpha": avg_alpha,
                    "beta": avg_beta,
                })

                # Display result
                bar = "█" * int(prediction["confidence"] * 20)
                print(
                    f"[{prediction['class_name']:10}] {bar} "
                    f"({prediction['confidence'] * 100:.0f}%)"
                )

        # Start streaming
        await bci.start_stream()
        print("\nStreaming... (running for 30 seconds)\n")

        # Run for 30 seconds
        await asyncio.sleep(30)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        await bci.stop_stream()
        await bci.disconnect()
        bci.dispose()
        print("\nExample complete.")


if __name__ == "__main__":
    asyncio.run(main())
