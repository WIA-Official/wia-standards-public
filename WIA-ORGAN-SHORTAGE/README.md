# WIA-ORGAN-SHORTAGE Standard

🫀 **Unlimited Organ Supply Platform: Xenotransplantation and Bioprinting Interoperability**

[![WIA Standard](https://img.shields.io/badge/WIA-Official%20Standard-14B8A6)](https://wiastandards.com/organ-shortage)
[![Version](https://img.shields.io/badge/version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

> **홍익인간 (弘益人間) - Benefit All Humanity** - "Benefit All Humanity"
>
> The WIA-ORGAN-SHORTAGE Standard unifies fragmented organ transplantation research to create a standard for unlimited organ supply through xenotransplantation-bioprinting fusion.

---

## Overview

The WIA-ORGAN-SHORTAGE Standard provides a comprehensive framework for:

- **Waitlist Management** - Standardized patient profiles with urgency scoring and real-time status
- **Xenotransplantation** - Gene-edited pig organ protocols with immunological compatibility
- **3D Bioprinting** - Patient-specific organ fabrication with vascularization tracking
- **Matching Algorithms** - Multi-source organ matching integrating traditional and alternative supplies
- **Registry Integration** - Seamless data exchange with UNOS, Eurotransplant, and research platforms

## The Crisis

| Statistic | Value | Impact |
|-----------|-------|--------|
| US Waitlist | 103,000+ | Growing faster than supply |
| Daily Deaths | 17 | Preventable with adequate supply |
| Supply Gap | 40,000+ | Annual transplants vs demand |
| Wait Time | 3-5 years | Average for kidney transplant |

## Quick Start

### Installation

```bash
npm install @wia/organ-shortage-sdk
```

### Usage

```typescript
import { WiaOrganShortageClient } from '@wia/organ-shortage-sdk';

const client = new WiaOrganShortageClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create a patient profile
const patient = await client.patients.create({
  organNeeded: 'kidney',
  urgency: 'high',
  bloodType: 'O',
  hlaTyping: {
    classI: { A: ['02:01', '03:01'], B: ['07:02', '44:02'], C: ['07:02', '05:01'] },
    classII: { DR: ['15:01', '04:01'], DQ: ['06:02', '03:01'] }
  }
});

// Check alternative options
const options = await client.alternatives.evaluate(patient.id);
console.log(`Xenotransplant eligible: ${options.xenotransplant.eligible}`);
console.log(`Bioprinted organ eligible: ${options.bioprinted.eligible}`);
```

## Standard Components

### 📊 Phase 1: Data Format
Standardized JSON schemas for patient profiles, organ specifications, and matching data.

```json
{
  "$schema": "https://wia.live/schemas/organ-shortage/v1.0.0",
  "organ_profile": {
    "patient_id": "uuid",
    "organ_needed": "kidney",
    "urgency": "high",
    "waitlist_status": {
      "registered_date": "2024-01-15T00:00:00Z",
      "unos_status": "1A",
      "waiting_time_days": 847
    },
    "immunological": {
      "blood_type": "O",
      "hla_typing": {},
      "pra_percent": 45
    },
    "alternative_options": {
      "xenotransplant_eligible": true,
      "bioprinted_organ_eligible": true
    }
  }
}
```

### 🔌 Phase 2: API Interface
RESTful APIs with OAuth 2.0 authentication for secure data exchange.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/waitlist/status` | GET | Get waitlist current status |
| `/xenotransplant/eligibility` | POST | Evaluate xenotransplant eligibility |
| `/bioprint/order` | POST | Order bioprinted organ |
| `/match` | POST | Run optimal matching algorithm |

### 📡 Phase 3: Protocol
Real-time streaming protocols for organ availability and urgent matching.

- WebSocket support for real-time organ availability alerts
- gRPC for high-throughput server-to-server communication
- TLS 1.3 encryption with end-to-end encryption for PHI

### 🔗 Phase 4: Integration
Seamless integration with transplant ecosystems.

- **UNOS/OPTN** - United Network for Organ Sharing
- **Eurotransplant** - European transplant organization
- **Research platforms** - Clinical trial registries
- **EHR systems** - Epic, Cerner integration

## Directory Structure

```
WIA-ORGAN-SHORTAGE/
├── index.html              # Landing page
├── simulator/
│   └── index.html          # Interactive simulator (5 tabs)
├── ebook/
│   ├── en/                 # English ebook (8 chapters)
│   └── ko/                 # Korean ebook (8 chapters)
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # Type definitions
│       │   └── index.ts    # SDK implementation
│       └── package.json
└── README.md
```

## Organ Types Supported

| Code | Organ | Xeno Status | Bioprint Status |
|------|-------|-------------|-----------------|
| WIA-ORG-001 | Kidney | Clinical Trials | Research |
| WIA-ORG-002 | Liver | Research | Research |
| WIA-ORG-003 | Heart | Clinical Trials | Early Research |
| WIA-ORG-004 | Lung | Research | Early Research |
| WIA-ORG-005 | Pancreas | Research | Research |
| WIA-ORG-006 | Intestine | Preclinical | Early Research |

## Xenotransplantation Advances (2024-2025)

| Organization | Achievement | Timeline |
|--------------|-------------|----------|
| eGenesis | Pig kidney clinical trial approved | Dec 2024 |
| United Therapeutics | UKidney first transplant | Mid 2025 |
| NYU Langone | Pig heart function 60+ days | 2024 |
| Mass General | Pig kidney in living patient | 2024 |

## Bioprinting Milestones

| Organization | Achievement | Status |
|--------------|-------------|--------|
| Organovo | Liver patch mouse implant | 28-day vascularization |
| 3D Systems | Kidney scaffold printing | Preclinical |
| Aspect Biosystems | Liver tissue fabrication | Research |

## WIA Certification

Implement the WIA-ORGAN-SHORTAGE standard and get certified:

| Level | Requirements | Benefits |
|-------|--------------|----------|
| 🥉 Bronze | Phase 1 compliance | WIA Registry listing |
| 🥈 Silver | Phase 1-2 compliance | Use WIA logo |
| 🥇 Gold | Phase 1-3 compliance | Priority support |
| 💎 Platinum | Full compliance + audit | Enterprise features |

[Apply for certification →](https://cert.wiastandards.com)

## Resources

- 🏠 [Landing Page](https://organ-shortage.wiastandards.com)
- 🎮 [Interactive Simulator](https://organ-shortage.wiastandards.com/simulator/)
- 📚 [Ebook (EN)](https://wiabooks.store/tag/wia-organ-shortage/)
- 📚 [Ebook (KO)](https://wiabooks.store/tag/wia-organ-shortage/)
- 📋 [Full Specification](spec/)
- 💻 [TypeScript SDK](api/typescript/)

## Contributing

We welcome contributions! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/blob/main/CONTRIBUTING.md).

## License

MIT License - see [LICENSE](LICENSE) for details.

---

<p align="center">
  <strong>홍익인간 (弘益人間) · Benefit All Humanity</strong><br>
  <em>WIA - World Certification Industry Association</em><br>
  © 2025 MIT License
</p>

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
