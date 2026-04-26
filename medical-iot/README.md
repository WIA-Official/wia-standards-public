# WIA Medical IoT Standard

> **Medical IoT Standard for Connected Healthcare Devices**

## Overview

The WIA Medical IoT Standard provides a unified framework for connecting medical devices, enabling seamless data exchange between healthcare IoT devices, electronic health records (EHR), hospital systems, and health monitoring platforms.

## Features

- 📊 **Standardized Data Format** - Unified schema for medical device readings
- 🔒 **Security First** - Built-in encryption and authentication
- 📡 **Protocol Support** - MQTT, CoAP, HTTP, WebSocket protocols
- ⚡ **Real-time Streaming** - Low-latency vital sign monitoring
- 🔗 **EHR Integration** - Direct integration with electronic health records
- ✅ **Validation** - Automated data validation and quality checks

## Supported Devices

- Pulse Oximeters (SpO2)
- Blood Pressure Monitors
- Glucose Meters
- Digital Thermometers
- ECG/EKG Monitors
- Smart Scales
- Respiratory Monitors
- Infusion Pumps
- Ventilators

## Quick Start

### Installation

```bash
npm install @wia/medical-iot
```

### Usage

```typescript
import { MedicalIoTSDK } from '@wia/medical-iot';

const sdk = new MedicalIoTSDK('https://api.wia.medical', 'YOUR_API_KEY');

// Create a device reading
const reading = sdk.createDeviceReading(
  'DEV-SPO2-001',           // Device ID
  'pulse_oximeter',         // Device type
  'PT-2025-001',           // Patient ID
  98,                       // Value
  '%',                      // Unit
  {
    location: 'Home',
    firmwareVersion: '2.1.0',
    batteryLevel: 85
  }
);

// Validate reading
const validation = sdk.validateReading(reading);
console.log(validation.valid ? 'Valid!' : validation.errors);

// Send to server
await sdk.sendReading(reading);
```

## Data Format

```json
{
  "format": "WIA-MEDICAL-IOT-v1.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "device_id": "DEV-SPO2-001",
  "device_type": "pulse_oximeter",
  "patient_id": "PT-2025-001",
  "measurement": {
    "value": 98,
    "unit": "%",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "location": "Home",
  "metadata": {
    "firmware_version": "2.1.0",
    "battery_level": 85,
    "signal_strength": -45
  }
}
```

## Vital Signs Monitoring

```typescript
import { VitalSigns } from '@wia/medical-iot';

const vitals: VitalSigns = {
  heart_rate: { value: 72, unit: 'bpm', timestamp: new Date().toISOString() },
  spo2: { value: 98, unit: '%', timestamp: new Date().toISOString() },
  temperature: { value: 36.8, unit: '°C', timestamp: new Date().toISOString() }
};

// Check for abnormal readings
const alerts = sdk.checkVitalSigns(vitals);
alerts.forEach(alert => {
  console.log(`${alert.level}: ${alert.message}`);
});
```

## Alert System

```typescript
// Automatic alerts for abnormal readings
const alert = sdk.createAlert(
  'critical',
  'Low oxygen saturation detected',
  'DEV-SPO2-001'
);

// Alert levels: normal, warning, critical
```

## Device Registration

```typescript
const registration = {
  serial_number: 'SN-MED-2025-001234',
  device_type: 'pulse_oximeter',
  manufacturer: 'Medical Devices Inc.',
  model: 'PulseOx Pro 3000',
  registration_date: new Date().toISOString(),
  certification: {
    wia_certified: true,
    certification_id: 'WIA-CERT-2025-001',
    expiry_date: '2026-12-31T23:59:59Z'
  }
};

await sdk.registerDevice(registration);
```

## Integration Examples

### EHR Integration

```typescript
const integration = {
  integration_id: 'INT-EHR-001',
  source_device: 'DEV-SPO2-001',
  destination_system: 'ehr',
  endpoint: 'https://ehr.hospital.com/api/v1',
  protocol: 'http',
  authentication: {
    method: 'oauth2',
    credentials: {
      client_id: 'your_client_id',
      client_secret: 'your_secret'
    }
  }
};
```

### Real-time Monitoring

```typescript
// Stream vital signs in real-time
const stream = {
  stream_id: 'STREAM-001',
  device_id: 'DEV-SPO2-001',
  start_time: new Date().toISOString(),
  sample_rate: 1, // 1 Hz
  data_points: []
};
```

## API Reference

### MedicalIoTSDK

| Method | Description |
|--------|-------------|
| `createDeviceReading()` | Create a new device reading |
| `validateReading()` | Validate reading format |
| `sendReading()` | Send reading to server |
| `createAlert()` | Create health alert |
| `checkVitalSigns()` | Check vital signs against normal ranges |
| `calculateBMI()` | Calculate BMI from weight/height |
| `convertTemperature()` | Convert between °C and °F |
| `registerDevice()` | Register new device |
| `getDeviceStatus()` | Get device status |
| `getPatientReadings()` | Retrieve patient readings |
| `sendCommand()` | Send command to device |

## Security

- **End-to-end Encryption** - All data encrypted in transit and at rest
- **Authentication** - API key, OAuth2, certificate-based auth
- **HIPAA Compliant** - Meets healthcare data privacy standards
- **Audit Logging** - Complete audit trail for all operations

## Protocol Support

- **MQTT** - Lightweight messaging for IoT devices
- **CoAP** - Constrained Application Protocol for resource-limited devices
- **HTTP/REST** - Standard web API
- **WebSocket** - Real-time bidirectional communication

## Certification

Devices implementing this standard can obtain **WIA Medical IoT Certification**:

1. Implement the WIA Medical IoT data format
2. Pass security and interoperability tests
3. Submit for certification review
4. Receive WIA certification badge

Learn more: https://cert.wiastandards.com

## Resources

- **Simulator**: https://wiastandards.com/medical-iot/simulator
- **Ebook**: https://wiabook.com/medical-iot
- **Specification**: https://wiastandards.com/medical-iot/spec
- **GitHub**: https://github.com/WIA-Official/wia-standards

## License

MIT License - See LICENSE file for details

## Support

- **Documentation**: https://docs.wiastandards.com
- **Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Email**: support@wiastandards.com

---

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
© 2025 MIT License
