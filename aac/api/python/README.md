# WIA AAC Standard - Python API

Python implementation of the WIA AAC Standard API.

## Installation

```bash
pip install wia-aac
```

## Quick Start

```python
import asyncio
from wia_aac import WiaAac, SensorType, EventType, SensorConfig

async def main():
    aac = WiaAac()

    # Event handlers
    def on_signal(signal):
        print(f"Signal: {signal.type}")

    def on_selection(event):
        print(f"Selected: {event.target_id}")

    def on_error(error):
        print(f"Error: {error.message}")

    aac.on(EventType.SIGNAL, on_signal)
    aac.on(EventType.SELECTION, on_selection)
    aac.on(EventType.ERROR, on_error)

    # Connect to sensor
    await aac.connect(SensorConfig(type=SensorType.EYE_TRACKER))

    # ... do something ...

    # Disconnect
    await aac.disconnect()

asyncio.run(main())
```

## API Overview

### WiaAac Class

Main entry point for AAC sensor interaction.

#### Constructor

```python
from wia_aac import WiaAac, WiaAacOptions

aac = WiaAac(WiaAacOptions(
    auto_reconnect=True,       # Auto-reconnect on disconnect
    reconnect_interval=3000,   # Reconnect delay in ms
    max_reconnect_attempts=5,  # Max reconnect attempts
    signal_buffer_size=100,    # Signal buffer size
    validate_signals=True,     # Validate signals against schema
    log_level="info"           # Log level
))
```

#### Methods

| Method | Description |
|--------|-------------|
| `await connect(config)` | Connect to a sensor |
| `await disconnect()` | Disconnect from sensor |
| `is_connected()` | Check connection status |
| `on(event, handler)` | Subscribe to event |
| `off(event, handler)` | Unsubscribe from event |
| `once(event, handler)` | One-time event subscription |
| `get_last_signal()` | Get last received signal |
| `get_config()` | Get current configuration |
| `configure(options)` | Update sensor options |

### Events

| Event | Data Type | Description |
|-------|-----------|-------------|
| `signal` | `WiaAacSignal` | Sensor signal received |
| `selection` | `SelectionEvent` | Selection completed |
| `gesture` | `GestureEvent` | Gesture recognized |
| `error` | `WiaAacError` | Error occurred |
| `connected` | `DeviceInfo` | Connected to device |
| `disconnected` | `DisconnectReason` | Disconnected |

### Sensor Types

```python
from wia_aac import SensorType

SensorType.EYE_TRACKER      # "eye_tracker"
SensorType.SWITCH           # "switch"
SensorType.MUSCLE_SENSOR    # "muscle_sensor"
SensorType.BRAIN_INTERFACE  # "brain_interface"
SensorType.BREATH           # "breath"
SensorType.HEAD_MOVEMENT    # "head_movement"
```

## Testing with Mock Adapter

```python
from wia_aac import WiaAac, MockAdapter, SensorType

aac = WiaAac()
mock_adapter = MockAdapter(
    sensor_type="eye_tracker",
    simulate_signals=True,
    signal_interval=0.1
)

aac.use_adapter(mock_adapter)
await aac.connect({"type": SensorType.EYE_TRACKER})

# Mock adapter will emit simulated signals
```

## Development

```bash
# Install dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Run tests with coverage
pytest --cov=wia_aac
```

## License

MIT License - WIA / SmileStory Inc.
