# WIA AAC Standard - TypeScript API

TypeScript implementation of the WIA AAC Standard API.

## Installation

```bash
npm install wia-aac
```

## Quick Start

```typescript
import { WiaAac, SensorType } from 'wia-aac';

const aac = new WiaAac();

// Event handlers
aac.on('signal', (signal) => {
  console.log('Signal:', signal.type, signal.data);
});

aac.on('selection', (event) => {
  console.log('Selected:', event.targetId);
});

aac.on('error', (error) => {
  console.error('Error:', error.message);
});

// Connect to sensor
await aac.connect({
  type: SensorType.EYE_TRACKER,
  device: { manufacturer: 'Tobii' },
  options: { dwellTime: 1000 }
});

// Later: disconnect
await aac.disconnect();
```

## API Overview

### WiaAac Class

Main entry point for AAC sensor interaction.

#### Constructor

```typescript
const aac = new WiaAac({
  autoReconnect: true,      // Auto-reconnect on disconnect
  reconnectInterval: 3000,  // Reconnect delay in ms
  maxReconnectAttempts: 5,  // Max reconnect attempts
  signalBufferSize: 100,    // Signal buffer size
  validateSignals: true,    // Validate signals against schema
  logLevel: 'info'          // Log level
});
```

#### Methods

| Method | Description |
|--------|-------------|
| `connect(config)` | Connect to a sensor |
| `disconnect()` | Disconnect from sensor |
| `isConnected()` | Check connection status |
| `on(event, handler)` | Subscribe to event |
| `off(event, handler)` | Unsubscribe from event |
| `once(event, handler)` | One-time event subscription |
| `getLastSignal()` | Get last received signal |
| `getConfig()` | Get current configuration |
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

```typescript
import { SensorType } from 'wia-aac';

SensorType.EYE_TRACKER      // 'eye_tracker'
SensorType.SWITCH           // 'switch'
SensorType.MUSCLE_SENSOR    // 'muscle_sensor'
SensorType.BRAIN_INTERFACE  // 'brain_interface'
SensorType.BREATH           // 'breath'
SensorType.HEAD_MOVEMENT    // 'head_movement'
```

## Testing with Mock Adapter

```typescript
import { WiaAac, MockAdapter } from 'wia-aac';

const aac = new WiaAac();
const mockAdapter = new MockAdapter({
  type: 'eye_tracker',
  simulateSignals: true,
  signalInterval: 100
});

aac.useAdapter(mockAdapter);
await aac.connect({ type: 'eye_tracker' });

// Mock adapter will emit simulated signals
```

## Custom Adapters

```typescript
import { BaseAdapter, ISensorAdapter, WiaAac } from 'wia-aac';

class MyCustomAdapter extends BaseAdapter {
  readonly type = 'custom' as const;

  async connect(config) {
    await super.connect(config);
    // Custom connection logic
  }

  getSupportedOptions() {
    return [];
  }
}

// Register adapter
WiaAac.registerAdapter('custom', MyCustomAdapter);
```

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test
```

## License

MIT License - WIA / SmileStory Inc.
