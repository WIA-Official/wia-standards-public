# WIA-MED-003 Phase 2: API Interface Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 2 defines the standard API for software applications to communicate with vital sign monitoring devices.

## VitalSignStream Class

The core API for all WIA-MED-003 implementations:

```typescript
class VitalSignStream {
  // Device Management
  async discoverDevices(filter?: DeviceFilter): Promise<Device[]>
  async connect(deviceId: string): Promise<Connection>
  async disconnect(deviceId: string): Promise<void>
  getDeviceStatus(deviceId: string): DeviceStatus

  // Data Streaming
  async startStream(deviceId: string, config?: StreamOptions): Promise<Stream>
  async stopStream(streamId: string): Promise<void>
  onData(callback: (data: VitalSignData) => void): void
  onError(callback: (error: Error) => void): void

  // Data Storage
  async saveData(data: VitalSignData, storage?: StorageConfig): Promise<string>
  async loadData(query: DataQuery): Promise<VitalSignData[]>
  async exportData(format: ExportFormat, query: DataQuery): Promise<Blob>

  // Alert System
  setAlert(config: AlertConfig): string
  removeAlert(alertId: string): void
  onAlert(callback: (alert: Alert) => void): void
}
```

## Device Discovery

### Discovery Protocol

Devices advertise using standard service UUIDs:
- **Bluetooth LE:** Service UUID `00001820-0000-1000-8000-00805f9b34fb`
- **WiFi/mDNS:** Service type `_wia-med-003._tcp`

### Example

```javascript
const devices = await stream.discoverDevices({
  signalType: 'ECG',
  manufacturer: 'AcmeMedical'
});
```

## Data Streaming

### Stream Configuration

```typescript
interface StreamOptions {
  signalType: SignalType;
  samplingRate?: number;
  duration?: number;        // 0 = infinite
  compression?: 'none' | 'gzip' | 'delta';
  realtime?: boolean;
  filters?: {
    lowpass?: number;
    highpass?: number;
    notch?: number;
  };
}
```

### Example

```javascript
await stream.startStream(deviceId, {
  signalType: 'ECG',
  samplingRate: 512,
  compression: 'delta',
  realtime: true
});

stream.onData((data) => {
  console.log('Heart Rate:', data.analysis?.heartRate);
});
```

## Alert System

### Alert Configuration

```typescript
interface AlertConfig {
  name: string;
  signalType: SignalType;
  condition: string | ((data: VitalSignData) => boolean);
  severity: 'low' | 'medium' | 'high' | 'critical';
  actions: AlertAction[];
  cooldown?: number;
}
```

### Example

```javascript
stream.setAlert({
  name: 'High Heart Rate',
  signalType: 'ECG',
  condition: 'heartRate > 120',
  severity: 'high',
  actions: [
    { type: 'notification', message: 'Heart rate too high!' },
    { type: 'sound', sound: 'alert.mp3' }
  ]
});
```

## Error Handling

### Error Codes

- **1xxx:** Connection errors (DEVICE_NOT_FOUND, CONNECTION_FAILED, etc.)
- **2xxx:** Streaming errors (STREAM_START_FAILED, BUFFER_OVERFLOW, etc.)
- **3xxx:** Data errors (INVALID_DATA_FORMAT, SCHEMA_VALIDATION_FAILED, etc.)
- **4xxx:** System errors (LOW_BATTERY, SENSOR_FAILURE, etc.)

### Example

```javascript
stream.onError((error) => {
  if (error.code === ErrorCode.CONNECTION_LOST && error.recoverable) {
    // Attempt reconnection
    await stream.connect(deviceId);
  }
});
```

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
