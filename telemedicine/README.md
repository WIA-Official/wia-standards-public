# WIA Telemedicine Standard

> **Telemedicine Standard for Remote Healthcare Consultations**

## Overview

The WIA Telemedicine Standard provides a unified framework for remote healthcare consultations, enabling video, audio, and chat-based medical appointments with full EHR integration.

## Features

- 📹 **Multi-Modal Consultations** - Video, audio, chat, and async messaging
- 🔒 **HIPAA Compliant** - End-to-end encryption and secure communications
- 📊 **EHR Integration** - Direct integration with electronic health records
- 💊 **E-Prescribing** - Digital prescription management
- 📅 **Scheduling** - Automated appointment scheduling and reminders
- 🎥 **HD Video** - High-quality video streaming with adaptive bitrate

## Quick Start

### Installation

```bash
npm install @wia/telemedicine
```

### Usage

```typescript
import { TelemedicineSDK } from '@wia/telemedicine';

const sdk = new TelemedicineSDK('https://api.wia.telemedicine', 'YOUR_API_KEY');

// Create consultation session
const session = await sdk.createSession(
  'PT-2025-001',      // Patient ID
  'DR-2025-001',      // Provider ID
  'video'             // Consultation type
);

console.log('Session ID:', session.session_id);
```

## Resources

- **Simulator**: https://wiastandards.com/telemedicine/simulator
- **Ebook**: https://wiabook.com/telemedicine
- **Specification**: https://wiastandards.com/telemedicine/spec

## License

MIT License

---

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
© 2025 MIT License

## Standard architecture

The WIA Telemedicine Standard follows a four-phase architecture
aligned with the broader WIA family:

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Consultation session, encounter, prescription, scheduling envelopes |
| 2 — API Interface | HTTPS surface for session establishment, signalling, EHR query |
| 3 — Federation Protocol | Cross-jurisdiction provider trust, replay defence, patient consent flow |
| 4 — Integration | HL7 FHIR R5, IHE, ICD-11, SNOMED CT, RxNorm, eIDAS, KFTC OpenBanking-style claims |

## Companion standards

* **HL7 FHIR R5** — patient and encounter modelling
* **IHE ITI** — cross-enterprise document sharing
* **WHO ICD-11** — diagnostic coding
* **SNOMED CT International** — clinical terminology
* **RxNorm / WHO ATC/DDD** — medication identification
* **eIDAS / KFTC** — patient identity assurance
* **WIA-OMNI-API** — credential storage for provider identities

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + signalling |
| Core | Plus Phase 3 federation, eIDAS / KFTC identity binding |
| Full | Plus Phase 4 HL7 FHIR R5 + IHE + RxNorm bridges |

弘益人間 — Benefit All Humanity.
