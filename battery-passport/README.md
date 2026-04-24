# WIA Battery Passport

**EU 배터리 여권 - EU Battery Regulation 2027 Compliant**
*Comprehensive battery lifecycle tracking for EV and industrial batteries*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA Battery Passport implements the EU Battery Regulation (2023/1542) requirements for digital battery passports, mandatory from 2027 for:
- Electric Vehicle (EV) batteries > 2 kWh
- Industrial batteries > 2 kWh
- Light Means of Transport (LMT) batteries (from 2028)

### Key Features

- **Lifecycle Tracking**: From raw material extraction to recycling
- **State of Health (SOH)**: Real-time battery health monitoring
- **Carbon Footprint**: ISO 14067 / EU PEF compliant carbon tracking
- **Responsible Sourcing**: Supply chain due diligence verification
- **Second-Life Assessment**: Eligibility for repurposing (EV → stationary)
- **Recycling Efficiency**: EU material recovery target compliance
- **Interoperability**: Catena-X and GBA compatible

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures (identity, materials, carbon) |
| 2 | [PHASE-2-ALGORITHMS.md](spec/PHASE-2-ALGORITHMS.md) | SOH, RUL, carbon, second-life algorithms |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | REST/WebSocket API, BMS integration |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | EU, automotive, grid, recycling integrations |

---

## Quick Start

### Rust API

```bash
cd api/rust

# Build
cargo build --release

# Run server
cargo run

# Run tests
cargo test
```

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/version` | API version & EU regulation info |
| POST | `/api/v1/passports` | Create battery passport |
| GET | `/api/v1/passports/:id` | Get passport by ID |
| GET | `/api/v1/passports/qr/:code` | Get passport by QR code |
| GET | `/api/v1/passports/:id/soh/calculate` | Calculate SOH |
| GET | `/api/v1/passports/:id/rul/predict` | Predict remaining useful life |
| POST | `/api/v1/passports/:id/carbon/calculate` | Calculate carbon footprint |
| GET | `/api/v1/passports/:id/second-life/assess` | Assess second-life eligibility |
| POST | `/api/v1/passports/:id/supply-chain/verify` | Verify supply chain compliance |
| WS | `/ws` | Real-time updates |

### Example: Create Battery Passport

```bash
curl -X POST http://localhost:8080/api/v1/passports \
  -H "Content-Type: application/json" \
  -d '{
    "id": "01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b",
    "qr_code": "WIABAT:ABC123DEF456GHI7",
    "version": "1.0.0",
    "identity": {
      "unique_identifier": "EU-BAT-2024-0001",
      "category": "ev",
      "chemistry": "nmc",
      "model": "NMC811-75",
      "serial_number": "SN12345",
      "batch_number": "BATCH001",
      "weight_kg": 450.0,
      "production_date": "2024-06-15T00:00:00Z",
      "production_country": "CN"
    },
    "manufacturer": {
      "name": "CATL",
      "legal_name": "Contemporary Amperex Technology Co. Limited"
    },
    "specifications": {
      "nominal_voltage_v": 400.0,
      "rated_capacity_wh": 75000,
      "expected_cycle_life": 2000
    }
  }'
```

### Example: Calculate SOH

```bash
curl http://localhost:8080/api/v1/passports/01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b/soh/calculate

# Response:
{
  "success": true,
  "data": {
    "soh_percent": 94.5,
    "capacity_soh": 94.5,
    "resistance_soh": 95.0,
    "cycle_soh": 92.0,
    "confidence": "high"
  }
}
```

### Example: Assess Second-Life Eligibility

```bash
curl http://localhost:8080/api/v1/passports/01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b/second-life/assess

# Response:
{
  "success": true,
  "data": {
    "eligible": true,
    "score": 85,
    "suggested_applications": [
      "Grid-scale energy storage",
      "Residential energy storage"
    ],
    "estimated_remaining_value_usd": 4500.00
  }
}
```

---

## Interactive Simulator

Try the Battery Passport simulator to see how the system works:

- **[Simulator](simulator/index.html)** - Interactive demo with en/ko language support
  - Create virtual battery passports
  - Calculate carbon footprint
  - Check EU compliance status
  - Analyze battery lifecycle
  - Generate QR codes

---

## Ebook - Complete Implementation Guide

Learn how to implement the WIA Battery Passport standard with EU Regulation 2023/1542 compliance:

**[Ebook - $99](ebook/index.html)** - Complete 10-chapter guide

| Chapter | Title | Content |
|---------|-------|---------|
| 1 | [Introduction](ebook/chapter-01.html) | EU Battery Regulation overview (FREE) |
| 2 | [Data Format](ebook/chapter-02.html) | JSON-LD schemas and DIDs |
| 3 | [Algorithms](ebook/chapter-03.html) | Carbon footprint, SoH calculation |
| 4 | [Protocol](ebook/chapter-04.html) | REST API and QR encoding |
| 5 | [Integration](ebook/chapter-05.html) | EU ESPR, supply chain, recycling |
| 6 | Rust Implementation | Production code walkthrough |
| 7 | Carbon Footprint | PEF methodology and verification |
| 8 | Supply Chain | Due diligence and traceability |
| 9 | Second-Life & Recycling | End-of-life management |
| 10 | Case Studies | EV manufacturer implementations |

**[Get Certified](https://wia.org/certification/battery-passport)** - WIA Implementation Specialist certification

---

## Data Model

```
BatteryPassport
├── BatteryIdentity
│   ├── unique_identifier (EU Battery Passport ID)
│   ├── category (ev, lmt, industrial, stationary)
│   ├── chemistry (lfp, nmc, nca, lto, solid_state)
│   └── dimensions, weight, production info
├── ManufacturerInfo
│   ├── name, legal_name, registration
│   ├── EU representative (for non-EU)
│   └── production_facilities[]
├── BatterySpecifications
│   ├── voltage, capacity, energy_density
│   ├── c_rate, efficiency, temperature limits
│   └── cycle_life, warranty
├── MaterialComposition
│   ├── cobalt, lithium, nickel, manganese, graphite
│   ├── recycled_content percentages
│   ├── hazardous_substances[]
│   └── responsible_sourcing certifications
├── CarbonFootprint
│   ├── total_kg_co2e, per_kwh
│   ├── breakdown (materials, manufacturing, transport)
│   ├── performance_class (A-E)
│   └── verification status
├── BatteryHealth
│   ├── state_of_health_percent (SOH)
│   ├── state_of_charge_percent (SOC)
│   ├── capacity_fade, resistance_increase
│   ├── cycle_count, temperature history
│   └── remaining_useful_life prediction
├── LifecycleEvent[]
│   ├── manufactured, shipped, installed
│   ├── maintained, repaired, repurposed
│   └── decommissioned, recycled
├── RecyclingInfo
│   ├── recyclability_score
│   ├── recovery_targets (Co, Li, Ni, Cu)
│   └── hazard_classification
└── Certification[]
    ├── UN38.3, IEC62619, CE marking
    └── RMI, IRMA responsible sourcing
```

---

## EU Compliance

### Carbon Performance Classes

| Class | kg CO2e/kWh | Description |
|-------|-------------|-------------|
| A | < 50 | Excellent |
| B | 50-65 | Good |
| C | 65-80 | Average |
| D | 80-95 | Below average |
| E | > 95 | Poor |

### Recycled Content Targets (2031)

| Material | Target |
|----------|--------|
| Cobalt | 16% |
| Lithium | 6% |
| Nickel | 6% |
| Lead | 85% |

### Material Recovery Targets (2031)

| Material | Target |
|----------|--------|
| Cobalt | 95% |
| Lithium | 80% |
| Nickel | 95% |
| Copper | 95% |

---

## Battery Chemistries Supported

| Chemistry | Code | Description |
|-----------|------|-------------|
| LFP | `lfp` | Lithium Iron Phosphate |
| NMC | `nmc` | Nickel Manganese Cobalt |
| NCA | `nca` | Nickel Cobalt Aluminum |
| LCO | `lco` | Lithium Cobalt Oxide |
| LTO | `lto` | Lithium Titanate |
| Solid State | `solid_state` | Solid-state battery |
| Sodium-ion | `sodium_ion` | Sodium-ion battery |

---

## Integrations

- **EU Central Registry**: EU Battery Passport registration
- **Catena-X**: Automotive supply chain data exchange
- **Global Battery Alliance (GBA)**: Sustainability metrics
- **BMS Integration**: Real-time health data from Battery Management Systems
- **V2G Systems**: Vehicle-to-Grid bidirectional charging
- **Recycling Networks**: End-of-life material recovery

---

## Security

- TLS 1.3 transport encryption
- OAuth 2.0 / DID authentication
- Digital signatures (Ed25519/ECDSA)
- Audit logging for all operations
- GDPR compliant data handling

---

## License

MIT License

Copyright (c) 2025 WIA - World Certification Industry Association

---

**Document ID**: WIA-BATTERY-PASSPORT
**Version**: 1.0.0
**Date**: 2025-12-16
**EU Regulation**: 2023/1542
