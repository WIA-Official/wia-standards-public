# WIA BCI TypeScript API

[![npm version](https://badge.fury.io/js/wia-bci.svg)](https://www.npmjs.com/package/wia-bci)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

TypeScript/JavaScript SDK for WIA Brain-Computer Interface Standard.

## Installation

```bash
npm install wia-bci
# or
yarn add wia-bci
# or
pnpm add wia-bci
```

## Quick Start

```typescript
import { WiaBci } from 'wia-bci';

const bci = new WiaBci({ logLevel: 'info' });

// Connect to device
await bci.connect({
  type: 'eeg_headset',
  device: { manufacturer: 'OpenBCI' },
  acquisition: {
    samplingRate: 250,
    channels: ['Fp1', 'Fp2', 'C3', 'C4', 'O1', 'O2']
  }
});

// Listen for signal events
bci.on('signal', (event) => {
  console.log(`Sample ${event.sampleIndex}:`, event.data);
});

// Start streaming
await bci.startStream();

// ... do something

// Cleanup
await bci.stopStream();
await bci.disconnect();
bci.dispose();
```

## Features

- **Device Agnostic**: Works with any BCI device through adapters
- **Type-Safe**: Full TypeScript support with strict types
- **Event-Driven**: Async event-based architecture
- **Signal Processing**: Built-in filtering and analysis utilities
- **Extensible**: Easy to add new device adapters

## API Reference

### WiaBci Class

Main entry point for BCI interactions.

```typescript
const bci = new WiaBci(options?: WiaBciOptions);
```

#### Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `autoReconnect` | boolean | true | Auto-reconnect on disconnect |
| `reconnectInterval` | number | 3000 | Reconnect interval (ms) |
| `maxReconnectAttempts` | number | 5 | Max reconnect attempts |
| `bufferSize` | number | 1000 | Internal buffer size |
| `logLevel` | string | 'info' | Log level |

#### Methods

| Method | Description |
|--------|-------------|
| `connect(config)` | Connect to BCI device |
| `disconnect()` | Disconnect from device |
| `startStream()` | Start data streaming |
| `stopStream()` | Stop data streaming |
| `listDevices()` | List available devices |
| `getDeviceInfo()` | Get connected device info |
| `getChannels()` | Get channel information |
| `getState()` | Get current BCI state |
| `dispose()` | Clean up resources |

#### Events

| Event | Data | Description |
|-------|------|-------------|
| `connected` | - | Connected to device |
| `disconnected` | - | Disconnected from device |
| `signal` | `SignalEvent` | Signal data received |
| `marker` | `MarkerEvent` | Event marker received |
| `classification` | `ClassificationEvent` | Classification result |
| `error` | `ErrorEvent` | Error occurred |

### SignalProcessor

Signal processing utilities.

```typescript
import { SignalProcessor } from 'wia-bci';

// Filter data
const filtered = SignalProcessor.bandpass(data, 8, 30, 250);

// Calculate band powers
const powers = SignalProcessor.allBandPowers(data, 250);
console.log('Alpha power:', powers.alpha);
```

## Device Adapters

### Available Adapters

| Adapter | Device | Status |
|---------|--------|--------|
| `SimulatorAdapter` | Synthetic data | ✅ Ready |
| `OpenBciAdapter` | OpenBCI Cyton/Ganglion | 🚧 Planned |
| `EmotivAdapter` | Emotiv EPOC/Insight | 🚧 Planned |
| `MuseAdapter` | InteraXon Muse | 🚧 Planned |
| `LslAdapter` | Lab Streaming Layer | 🚧 Planned |

### Using Simulator

```typescript
import { WiaBci } from 'wia-bci';

const bci = new WiaBci();
await bci.connect({ type: 'simulator' });
await bci.startStream();

bci.on('signal', (event) => {
  // Synthetic EEG data with alpha rhythm
  console.log(event.data);
});
```

## Examples

### Motor Imagery Classification

```typescript
import { WiaBci, SignalProcessor } from 'wia-bci';

const bci = new WiaBci();
const buffer: Float32Array[] = [];
const WINDOW_SIZE = 250; // 1 second

await bci.connect({ type: 'eeg_headset' });

bci.on('signal', (event) => {
  buffer.push(event.data);

  if (buffer.length >= WINDOW_SIZE) {
    const epoch = concatenate(buffer.splice(0, WINDOW_SIZE));

    // Filter 8-30 Hz (mu/beta bands)
    const filtered = SignalProcessor.bandpass(epoch, 8, 30, 250);

    // Extract features
    const powers = SignalProcessor.allBandPowers(filtered, 250);

    // Classify (your model here)
    const prediction = classify(powers);
    console.log('Predicted:', prediction);
  }
});

await bci.startStream();
```

### Recording Data

```typescript
import { WiaBci } from 'wia-bci';
import * as fs from 'fs';

const bci = new WiaBci();
const samples: Float32Array[] = [];

await bci.connect({ type: 'eeg_headset' });

bci.on('signal', (event) => {
  samples.push(event.data);
});

await bci.startStream();

// Record for 60 seconds
await new Promise(r => setTimeout(r, 60000));

await bci.stopStream();
await bci.disconnect();

// Save data (Phase 1 format)
const recording = {
  wiaVersion: '1.0.0',
  recordingId: `rec-${Date.now()}`,
  recordingInfo: {
    startTime: new Date().toISOString(),
    durationSeconds: 60
  },
  // ... more metadata
};

fs.writeFileSync('recording.json', JSON.stringify(recording, null, 2));
```

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

## License

MIT License - Part of WIA Standards.

---

弘益人間 - *Benefit All Humanity*
