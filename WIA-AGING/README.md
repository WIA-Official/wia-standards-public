# WIA-AGING Standard

🧓 **Healthy Aging, Longevity Biomarkers, and Geriatric Care Interoperability**

[![WIA Standard](https://img.shields.io/badge/WIA-Official%20Standard-06B6D4)](https://wiastandards.com/aging)
[![Version](https://img.shields.io/badge/version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

> **홍익인간 (弘益人間) - Benefit All Humanity** - "Benefit All Humanity"
>
> The WIA-AGING Standard enables dignified aging through technology by standardizing data formats, APIs, and protocols for aging-related health information.

---

## Overview

The WIA-AGING Standard provides a comprehensive framework for:

- **Biological Age Assessment** - Standardized calculation methods (epigenetic clocks, phenotypic age, telomere length)
- **Biomarker Data** - Interoperable format for 150+ aging biomarkers
- **Health Metrics** - Integration with wearables and health monitoring devices
- **Longevity Interventions** - Tracking supplements, lifestyle changes, and therapies
- **Geriatric Care Coordination** - Seamless data exchange between healthcare providers

## Quick Start

### Installation

```bash
npm install @wia/aging-sdk
```

### Usage

```typescript
import { WiaAgingClient } from '@wia/aging-sdk';

const client = new WiaAgingClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create a biological age assessment
const assessment = await client.assessments.create('profile_abc123', {
  method: 'phenotypic-levine',
  biomarkers: [
    { code: 'WIA-AGE-001', value: 0.8, unit: 'mg/L', timestamp: new Date().toISOString() },
    { code: 'WIA-AGE-003', value: 4.5, unit: 'g/dL', timestamp: new Date().toISOString() },
    { code: 'WIA-AGE-005', value: 92, unit: 'mg/dL', timestamp: new Date().toISOString() }
  ]
});

console.log(`Biological Age: ${assessment.biologicalAge.value}`);
console.log(`Age Difference: ${assessment.biologicalAge.ageDifference}`);
console.log(`Health Score: ${assessment.healthScore}/100`);
```

### Rust SDK

```bash
cargo add wia-aging-sdk
```

```rust
use wia_aging_sdk::{WiaAgingClient, Config, Environment, CreateAssessmentRequest, BiologicalAgeMethod};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = WiaAgingClient::new(Config {
        api_key: "your-api-key".to_string(),
        environment: Environment::Production,
        ..Default::default()
    });

    let assessment = client.assessments()
        .create("profile_abc123", CreateAssessmentRequest {
            method: BiologicalAgeMethod::PhenotypicLevine,
            biomarkers: vec![
                Biomarker::new("WIA-AGE-001", 0.8, "mg/L"),
            ],
        })
        .await?;

    println!("Biological Age: {}", assessment.biological_age.value);
    Ok(())
}
```

## Standard Components

### 📊 Phase 1: Data Format
Standardized JSON schemas for aging profiles, biomarkers, and health metrics.

```json
{
  "@context": ["https://wia.org/aging/v1"],
  "type": ["AgingProfile"],
  "subject": {
    "id": "did:wia:subject:001",
    "chronologicalAge": 55
  },
  "biologicalAge": {
    "value": 48.3,
    "method": "phenotypic-levine",
    "ageDifference": -6.7
  }
}
```

### 🔌 Phase 2: API Interface
RESTful APIs with OAuth 2.0 authentication for secure data exchange.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/profiles` | GET/POST | Manage aging profiles |
| `/profiles/{id}/assessments` | POST | Create biological age assessment |
| `/biomarkers/batch` | POST | Submit biomarker data |
| `/profiles/{id}/interventions` | GET/POST | Track interventions |

### 📡 Phase 3: Protocol
Real-time streaming protocols for wearable devices and continuous monitoring.

- WebSocket support for real-time biomarker streaming
- gRPC for high-throughput server-to-server communication
- TLS 1.3 encryption with optional end-to-end encryption

### 🔗 Phase 4: Integration
Seamless integration with healthcare ecosystems.

- **HL7 FHIR** compatibility
- **Wearable platforms**: Apple HealthKit, Google Fit, Oura, Whoop
- **EHR systems**: Epic, Cerner, and more
- **Research platforms**: Longevity research databases

## Directory Structure

```
WIA-AGING/
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
│   ├── typescript/
│   │   ├── src/
│   │   │   ├── types.ts    # Type definitions
│   │   │   └── index.ts    # SDK implementation
│   │   └── package.json
│   └── rust/
│       ├── src/
│       │   ├── lib.rs      # Library entry
│       │   ├── types.rs    # Type definitions
│       │   ├── client.rs   # HTTP client
│       │   ├── error.rs    # Error types
│       │   └── streaming.rs # WebSocket streaming
│       ├── Cargo.toml
│       └── README.md
└── README.md
```

## Core Biomarkers

| Code | Name | Unit | Category |
|------|------|------|----------|
| WIA-AGE-001 | C-Reactive Protein | mg/L | Inflammatory |
| WIA-AGE-002 | Interleukin-6 | pg/mL | Inflammatory |
| WIA-AGE-003 | Albumin | g/dL | Organ Function |
| WIA-AGE-004 | Creatinine | mg/dL | Organ Function |
| WIA-AGE-005 | Fasting Glucose | mg/dL | Metabolic |
| WIA-AGE-006 | HbA1c | % | Metabolic |
| WIA-AGE-007 | Lymphocyte % | % | Hematological |

[View full biomarker catalog →](spec/PHASE-1-DATA-FORMAT.md#4-standard-biomarker-catalog)

## Biological Age Methods

| Method | Description | Accuracy |
|--------|-------------|----------|
| Epigenetic (Horvath) | First-generation DNA methylation clock | ±3.6 years |
| Epigenetic (GrimAge) | Mortality-predicting clock | ±2.8 years |
| Phenotypic (Levine) | Blood biomarker-based | ±4.5 years |
| Telomere Length | Cellular replication history | ±5.0 years |

## WIA Certification

Implement the WIA-AGING standard and get certified:

| Level | Requirements | Benefits |
|-------|--------------|----------|
| 🥉 Bronze | Phase 1 compliance | WIA Registry listing |
| 🥈 Silver | Phase 1-2 compliance | Use WIA logo |
| 🥇 Gold | Phase 1-3 compliance | Priority support |
| 💎 Platinum | Full compliance + audit | Enterprise features |

[Apply for certification →](https://cert.wiastandards.com)

## Resources

- 🏠 [Landing Page](https://aging.wiastandards.com)
- 🎮 [Interactive Simulator](https://aging.wiastandards.com/simulator/)
- 📚 [Ebook (EN)](https://wiabooks.store/tag/wia-aging/)
- 📚 [Ebook (KO)](https://wiabooks.store/tag/wia-aging/)
- 📋 [Full Specification](spec/)
- 💻 [TypeScript SDK](api/typescript/)
- 🦀 [Rust SDK](api/rust/)

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
