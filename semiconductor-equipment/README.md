# WIA-SEMI-019: Semiconductor Equipment Standard

> **홍익인간 (弘益人間) - Benefit All Humanity**

[![WIA Standard](https://img.shields.io/badge/WIA-SEMI--019-06B6D4)](https://wiastandards.com)
[![Version](https://img.shields.io/badge/version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

## Overview

WIA-SEMI-019 is a comprehensive standard for semiconductor manufacturing equipment covering lithography, etch, deposition, inspection, and other critical fab tools. The standard provides unified data formats, communication APIs, SECS/GEM protocol extensions, and fab integration specifications.

### Key Features

- 🏭 **Equipment Types**: Lithography (EUV/DUV), Etch (Plasma/RIE), Deposition (CVD/ALD/PVD), Inspection (E-beam/Optical), CMP
- ⚡ **Performance Metrics**: Throughput (WPH), Overlay accuracy, Defect detection, Yield optimization
- 📡 **SECS/GEM**: SEMI E5 (SECS-II), SEMI E30 (GEM), SEMI E37 (HSMS) with WIA extensions
- 🔗 **Fab Automation**: MES integration, Recipe management, SPC/FDC, Predictive maintenance
- 🔍 **Process Control**: Real-time monitoring, Chamber matching, APC, Equipment qualification
- 📊 **Analytics**: Equipment data collection, AI/ML optimization, Digital twin modeling

## Quick Start

### Try the Simulator

Visit the [online simulator](simulator/index.html) to explore equipment specifications, throughput calculations, SECS/GEM protocols, and fab integration scenarios in 99 languages.

### Read the Ebook

- [English Version](ebook/en/index.html) - Complete 8-chapter guide
- [Korean Version](ebook/ko/index.html) - 한국어 완전 가이드

### Install the SDK

```bash
npm install @wia/semiconductor-equipment
```

```typescript
import { WIAEquipmentClient } from '@wia/semiconductor-equipment';

const client = new WIAEquipmentClient({
  baseURL: 'https://equipment-001.fab.com',
  apiKey: process.env.WIA_API_KEY
});

// Get equipment specification
const spec = await client.getEquipmentInfo();
console.log(`${spec.equipment.manufacturer} ${spec.equipment.model}`);

// Get current status
const status = await client.getEquipmentStatus();
console.log(`State: ${status.state}, Health: ${status.health.status}`);

// Stream real-time parameters
const ws = client.streamParameters(
  ['substrate_temperature_celsius', 'chamber_pressure_pascal'],
  100, // 100 Hz
  (data) => console.log(data)
);
```

## Four-Phase Architecture

### Phase 1: Equipment Data Format

Standardized JSON schemas for equipment specifications and parameter definitions.

**Spec**: [PHASE-1-EQUIPMENT-FORMAT.md](spec/PHASE-1-EQUIPMENT-FORMAT.md)

**Key Features**:
- Equipment specification schema
- Parameter naming conventions (`category_parameter_unit`)
- Data type definitions (float, integer, boolean, string, timestamp)
- Unit standardization (celsius, pascal, sccm, watts, etc.)
- Equipment-specific extensions (lithography, etch, deposition, inspection)

**Certification**: Bronze level

### Phase 2: Communication API

Modern HTTP/REST and WebSocket APIs for equipment interaction.

**Spec**: [PHASE-2-COMMUNICATION-API.md](spec/PHASE-2-COMMUNICATION-API.md)

**Key Features**:
- REST API endpoints (equipment info, status, parameters, commands, alarms, events, recipes)
- WebSocket streaming (10-1000Hz real-time data)
- Authentication (API keys, OAuth 2.0)
- Role-based access control (Viewer, Operator, Engineer, Administrator)
- Performance requirements (<200ms response time, <10ms WebSocket latency)

**Certification**: Silver level

### Phase 3: SECS/GEM Protocol

Enhanced SECS/GEM with WIA-specific messages and security.

**Spec**: [PHASE-3-SECSGEM-PROTOCOL.md](spec/PHASE-3-SECSGEM-PROTOCOL.md)

**Key Features**:
- WIA SECS messages (S9F100-S9F199)
- Standardized SVID ranges (4000-9999)
- Enhanced event reporting with rich context
- HSMS-TLS with certificate authentication
- Process job management extensions
- Enhanced alarm reporting with diagnostics

**Certification**: Gold level

### Phase 4: Fab Integration

Complete fab ecosystem integration specifications.

**Spec**: [PHASE-4-FAB-INTEGRATION.md](spec/PHASE-4-FAB-INTEGRATION.md)

**Key Features**:
- MES integration (lot tracking, recipe management, state machine coordination)
- SPC/FDC connectors (real-time monitoring, excursion detection)
- Predictive maintenance (equipment health, component lifetime tracking)
- AMHS coordination (E84 load port protocol)
- Cloud integration (hybrid architecture, secure data export)
- Advanced analytics (digital twin, AI/ML integration)

**Certification**: Platinum level

## Equipment Type Coverage

### Lithography Equipment

**Vendors**: ASML (EUV exclusive), Nikon, Canon

**Key Parameters**:
- Wavelength: 13.5nm (EUV), 193nm (ArF), 248nm (KrF)
- Resolution: 13nm to 350nm depending on technology
- Overlay accuracy: <2nm for advanced nodes
- Throughput: 125-275 WPH

**Specific Extensions**: Exposure control, overlay measurement, focus management, reticle handling

### Etch Equipment

**Vendors**: Lam Research, Applied Materials, Tokyo Electron

**Key Parameters**:
- Etch rate: 100-500 nm/min
- Selectivity: 20:1 to 100:1
- Uniformity: <2% across 300mm wafers
- Aspect ratio: >50:1 for advanced 3D structures

**Specific Extensions**: Plasma parameters (RF power, DC bias), gas chemistry, etch rate control

### Deposition Equipment

**Vendors**: Applied Materials, Lam Research, Tokyo Electron

**Key Parameters**:
- Deposition rate: 0.5-100 nm/min depending on method
- Uniformity: <2% for CVD/PVD, <1% for ALD
- Step coverage: >95% for ALD, 70-90% for CVD
- Temperature: 100-800°C depending on process

**Specific Extensions**: Film properties (thickness, stress, refractive index), deposition method (CVD, ALD, PVD)

### Inspection Equipment

**Vendors**: KLA, Applied Materials, Hitachi, ASML

**Key Parameters**:
- Resolution: 5-30nm depending on method
- Defect sensitivity: 10-50nm
- Throughput: 50-200 WPH
- Capture rate: >99%

**Specific Extensions**: Inspection method (e-beam, optical), defect classification, defect maps

### CMP Equipment

**Vendors**: Applied Materials, Ebara, ACCRETECH

**Key Parameters**:
- Removal rate: 100-300 nm/min
- Uniformity: <5% across wafer
- Dishing/erosion: <10nm for Cu damascene
- Throughput: 60-100 WPH

**Specific Extensions**: Platen/carrier speed, downforce, slurry flow, pad conditioning

## API Reference

### REST API

Base URL: `https://{equipment-host}/api/v1/`

#### Equipment Information

```http
GET /equipment/info
GET /equipment/status
```

#### Parameters

```http
GET /parameters
GET /parameters/{id}
POST /parameters/{id}/subscribe
DELETE /parameters/{id}/subscribe
```

#### Commands

```http
POST /command
GET /command/{id}/status
```

#### Alarms

```http
GET /alarms?status=active
GET /alarms/{id}
POST /alarms/{id}/acknowledge
```

#### Events

```http
GET /events?start_time={iso8601}&end_time={iso8601}&limit={n}
GET /events/{id}
```

#### Recipes

```http
GET /recipes
GET /recipes/{id}
POST /recipes
PUT /recipes/{id}
DELETE /recipes/{id}
```

### WebSocket Streaming

```javascript
ws://equipment-host/api/v1/stream
wss://equipment-host/api/v1/stream  // Secure

// Subscribe message
{
  "action": "subscribe",
  "parameters": ["substrate_temp", "pressure"],
  "frequency_hz": 100,
  "format": "json"
}

// Data message
{
  "timestamp": "2025-12-26T15:30:45.123Z",
  "sequence": 12345,
  "data": {
    "substrate_temp_celsius": 425.3,
    "chamber_pressure_pascal": 133.2
  }
}
```

## Certification Levels

| Level | Requirements | Suitable For |
|-------|-------------|--------------|
| **Bronze** | Phase 1 (Data Format) | Legacy equipment upgrades |
| **Silver** | Phases 1-2 (Data + API) | Modern equipment with REST API |
| **Gold** | Phases 1-3 (Data + API + SECS/GEM) | Full-featured equipment |
| **Platinum** | All phases + advanced features | Next-generation equipment |

### Certification Process

1. Self-assessment using WIA checklist
2. Documentation review
3. Automated functional testing
4. Interoperability testing
5. Security audit
6. Official certification and logo license
7. Annual recertification

## Industry Adoption

### Leading Equipment Manufacturers

- **ASML** (Lithography) - EUV market leader, >$150M per system
- **Applied Materials** (Diversified) - Largest overall equipment vendor
- **Lam Research** (Etch) - Market leader in plasma etch
- **Tokyo Electron** (Etch/Deposition) - Strong in Asian markets
- **KLA** (Inspection) - Dominant in process control and metrology
- **Screen** (Wet Processing) - Leader in cleaning and coater/developer

### Benefits

**For Equipment Vendors**:
- Competitive differentiation through WIA certification
- Reduced support costs via standardized interfaces
- Broader market access
- Future-proofing for cloud and AI/ML trends

**For Fabs**:
- 50-70% reduction in custom integration engineering
- 6-month equipment qualification vs. 12-18 months
- Improved yield through real-time FDC
- Enhanced security and compliance
- Vendor flexibility and multi-sourcing

**For the Industry**:
- $25-45B annual savings across global semiconductor industry
- Accelerated innovation through resource reallocation
- Improved sustainability via optimized operations
- Greater accessibility to advanced manufacturing

## 弘益人間 Philosophy

The Korean concept of **홍익인간 (弘益人間)** - "Benefit All Humanity" - guides WIA-SEMI-019 development. By standardizing semiconductor equipment interfaces:

- **Healthcare**: Enable medical imaging, genomics, personalized medicine
- **Communication**: Connect billions through smartphones and networks
- **Transportation**: Autonomous vehicles saving lives
- **Energy**: Efficient power management for renewable adoption
- **AI**: Solve complex problems from climate to drug discovery
- **Accessibility**: Empower people with disabilities through assistive technology

Every 1% improvement in fab efficiency translates to lower chip costs, accelerating technology adoption worldwide.

## Contributing

We welcome contributions from equipment vendors, fab engineers, and industry experts. Please see [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

## Support

- **Documentation**: [https://wiastandards.com/semi-019](https://wiastandards.com/semi-019)
- **Ebook**: [https://wiabooks.store/tag/wia-semiconductor-equipment/](https://wiabooks.store/tag/wia-semiconductor-equipment/)
- **Simulator**: [https://wiabooks.store/reader/simulators/](https://wiabooks.store/reader/simulators/)
- **Certification**: [https://cert.wiastandards.com](https://cert.wiastandards.com)
- **GitHub**: [https://github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)

## Related Standards

- **SEMI E5**: SECS-II Message Content
- **SEMI E30**: Generic Equipment Model (GEM)
- **SEMI E37**: High-Speed SECS Message Services (HSMS)
- **SEMI E40**: Processing Job Management (PJM)
- **SEMI E84**: Parallel I/O Protocol for Load Port
- **SEMI E90**: Substrate Tracking
- **SEMI E94**: Control Job Management (CMS)

## License

MIT License - See [LICENSE](LICENSE) for details

---

**홍익인간 (弘益人間) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*

---

**For more information**: [wiastandards.com](https://wiastandards.com) | [wiabooks.store](https://wiabooks.store)
