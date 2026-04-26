# WIA Vital Sign Streaming Standard

> **Vital Sign Streaming Standard for Real-time Patient Monitoring**

## Overview

The WIA Vital Sign Streaming Standard provides a unified framework for real-time streaming of vital signs from medical devices to monitoring systems, supporting multiple protocols and high-frequency data transmission.

## Features

- 📈 **Real-time Streaming** - Low-latency vital sign transmission
- 📡 **Multi-Protocol** - WebSocket, MQTT, gRPC, Server-Sent Events
- 🔒 **Encrypted Streams** - End-to-end encryption for patient data
- 📊 **High Frequency** - Support for high sample rates (up to 1000 Hz)
- 🎯 **Low Latency** - Sub-100ms transmission delays
- 📦 **Compression** - Efficient data compression for bandwidth optimization

## Quick Start

### Installation

```bash
npm install @wia/vital-sign-streaming
```

### Usage

```typescript
import { VitalSignStreamingSDK } from '@wia/vital-sign-streaming';

const sdk = new VitalSignStreamingSDK('https://api.wia.streaming', 'YOUR_API_KEY');

// Start vital sign stream
const result = await sdk.startStream({
  stream_id: 'STREAM-001',
  patient_id: 'PT-2025-001',
  vital_sign_types: ['ecg', 'spo2'],
  protocol: 'websocket',
  sample_rate_hz: 250,
  buffer_size: 1000,
  compression_enabled: true,
  encryption_enabled: true
});

// Connect WebSocket for real-time data
sdk.connectWebSocket(result.stream_id, (data) => {
  console.log('Vital sign data:', data);
});
```

## Supported Vital Signs

- ECG (Electrocardiogram)
- PPG (Photoplethysmogram)
- EEG (Electroencephalogram)
- EMG (Electromyogram)
- SpO2 (Oxygen Saturation)
- Blood Pressure

## Resources

- **Simulator**: https://wiastandards.com/vital-sign-streaming/simulator
- **Ebook**: https://wiabook.com/vital-sign-streaming
- **Specification**: https://wiastandards.com/vital-sign-streaming/spec
- **CLI**: `cli/vital-sign-streaming.sh` for stream validation and replay
- **Press**: `press/release-notes.md`

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 signal format, Phase 2 stream subscribe API |
| Core | Plus Phase 3 federation, encryption, replay defence |
| Full | Plus Phase 4 EHR/HL7 FHIR bridge, alert routing |

## Companion standards

* IEEE 11073 — point-of-care medical device communication family
* HL7 FHIR R5 — for the optional EHR bridge in Phase 4
* IETF RFC 8446 (TLS 1.3) — required transport security
* WIA-OMNI-API — credential storage for clinician identities
* WIA-ACCESSIBILITY — patient and clinician accommodations

弘益人間 — Benefit All Humanity.

## License

MIT License

---

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
© 2025 MIT License
