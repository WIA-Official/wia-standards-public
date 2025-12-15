# WIA BCI Python API

[![PyPI version](https://badge.fury.io/py/wia-bci.svg)](https://pypi.org/project/wia-bci/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

Python SDK for WIA Brain-Computer Interface Standard.

## Installation

```bash
pip install wia-bci
```

## Quick Start

```python
import asyncio
from wia_bci import WiaBci, DeviceConfig

async def main():
    bci = WiaBci()

    # Connect to device
    await bci.connect(DeviceConfig(
        type='eeg_headset',
        acquisition={'sampling_rate': 250}
    ))

    # Listen for signal events
    @bci.on('signal')
    def on_signal(event):
        print(f"Sample {event.sample_index}: {event.data}")

    # Start streaming
    await bci.start_stream()

    # Wait for some data
    await asyncio.sleep(10)

    # Cleanup
    await bci.stop_stream()
    await bci.disconnect()
    bci.dispose()

asyncio.run(main())
```

## Features

- **Device Agnostic**: Works with any BCI device through adapters
- **Type-Safe**: Full type hints with dataclasses
- **Async/Await**: Modern async Python API
- **Signal Processing**: Built-in filtering and analysis with NumPy
- **Extensible**: Easy to add new device adapters

## API Reference

### WiaBci Class

Main entry point for BCI interactions.

```python
bci = WiaBci(options=WiaBciOptions())
```

#### Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `auto_reconnect` | bool | True | Auto-reconnect on disconnect |
| `reconnect_interval` | float | 3.0 | Reconnect interval (seconds) |
| `max_reconnect_attempts` | int | 5 | Max reconnect attempts |
| `buffer_size` | int | 1000 | Internal buffer size |
| `log_level` | str | 'info' | Log level |

#### Methods

| Method | Description |
|--------|-------------|
| `connect(config)` | Connect to BCI device |
| `disconnect()` | Disconnect from device |
| `start_stream()` | Start data streaming |
| `stop_stream()` | Stop data streaming |
| `list_devices()` | List available devices |
| `get_device_info()` | Get connected device info |
| `get_channels()` | Get channel information |
| `get_state()` | Get current BCI state |
| `dispose()` | Clean up resources |

#### Events

| Event | Data | Description |
|-------|------|-------------|
| `connected` | - | Connected to device |
| `disconnected` | - | Disconnected from device |
| `signal` | `SignalEvent` | Signal data received |
| `marker` | `MarkerEvent` | Event marker received |
| `error` | `ErrorEvent` | Error occurred |

### SignalProcessor

Signal processing utilities.

```python
from wia_bci import SignalProcessor
import numpy as np

# Create test signal
data = np.sin(2 * np.pi * 10 * np.arange(250) / 250).astype(np.float32)

# Filter data
filtered = SignalProcessor.bandpass(data, 8, 30, 250)

# Calculate band powers
powers = SignalProcessor.all_band_powers(data, 250)
print(f"Alpha power: {powers.alpha}")
```

## Device Adapters

### Available Adapters

| Adapter | Device | Status |
|---------|--------|--------|
| `SimulatorAdapter` | Synthetic data | ✅ Ready |
| `OpenBciAdapter` | OpenBCI Cyton/Ganglion | 🚧 Planned |
| `LslAdapter` | Lab Streaming Layer | 🚧 Planned |

### Using Simulator

```python
import asyncio
from wia_bci import WiaBci, DeviceConfig

async def main():
    bci = WiaBci()
    await bci.connect(DeviceConfig(type='simulator'))
    await bci.start_stream()

    bci.on('signal', lambda e: print(e.data))

    await asyncio.sleep(5)
    await bci.disconnect()

asyncio.run(main())
```

## Examples

### Motor Imagery Classification

```python
import asyncio
import numpy as np
from wia_bci import WiaBci, SignalProcessor, DeviceConfig

buffer = []
WINDOW_SIZE = 250  # 1 second

async def main():
    bci = WiaBci()
    await bci.connect(DeviceConfig(type='eeg_headset'))

    @bci.on('signal')
    def on_signal(event):
        buffer.append(event.data)

        if len(buffer) >= WINDOW_SIZE:
            epoch = np.vstack(buffer[:WINDOW_SIZE])
            buffer.clear()

            # Filter 8-30 Hz (mu/beta bands)
            for ch in range(epoch.shape[1]):
                epoch[:, ch] = SignalProcessor.bandpass(
                    epoch[:, ch], 8, 30, 250
                )

            # Extract features
            powers = SignalProcessor.all_band_powers(epoch[:, 0], 250)
            print(f"Alpha: {powers.alpha:.2f}, Beta: {powers.beta:.2f}")

    await bci.start_stream()
    await asyncio.sleep(60)
    await bci.disconnect()

asyncio.run(main())
```

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

## License

MIT License - Part of WIA Standards.

---

弘益人間 - *Benefit All Humanity*
