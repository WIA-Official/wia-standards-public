# WIA-PLASTIC-ENZYME

> **Enzyme-Based Plastic Degradation Standard for Circular Economy**

[![WIA Standard](https://img.shields.io/badge/WIA-PLASTIC--ENZYME-10B981?style=for-the-badge)](https://wiastandards.com/plastic-enzyme)
[![Version](https://img.shields.io/badge/version-1.0.0-blue?style=for-the-badge)](./spec/)
[![License](https://img.shields.io/badge/license-MIT-green?style=for-the-badge)](LICENSE)

**홍익인간 (弘益人間) - Benefit All Humanity**

---

## Overview

WIA-PLASTIC-ENZYME is a comprehensive standard for enzymatic plastic degradation, enabling true circular economy through biotechnology. This standard provides specifications for enzyme profiles, degradation processes, quality metrics, and supply chain integration.

### Key Features

- **Enzyme Database**: Standardized profiles for PETase, MHETase, and engineered variants
- **Process Optimization**: AI-powered enzyme matching and condition optimization
- **Quality Assurance**: Food-contact grade certification pathways
- **Supply Chain Integration**: End-to-end traceability from waste to monomers
- **Carbon Tracking**: Lifecycle assessment and credit generation

---

## Quick Start

### Installation

```bash
npm install @wia/plastic-enzyme-sdk
```

### Basic Usage

```typescript
import { WiaPlasticEnzyme } from '@wia/plastic-enzyme-sdk';

const client = new WiaPlasticEnzyme({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Match optimal enzymes for PET degradation
const match = await client.matchEnzymes(
  { type: 'PET', weight_kg: 100, crystallinity_percent: 25 },
  { temperature: 50, ph: 8.0, target_efficiency: 0.95 }
);

console.log('Top enzyme:', match.recommendations[0].name);
console.log('Predicted efficiency:', match.recommendations[0].predicted_efficiency);

// Predict degradation outcomes
const prediction = await client.predictDegradation({
  process_id: 'test-001',
  plastic_input: { type: 'PET', weight_kg: 100 },
  enzyme_cocktail: [
    { enzyme_id: 'turbopetase-v2', concentration_mg_g: 3 },
    { enzyme_id: 'mhetase-is', concentration_mg_g: 1.5 }
  ],
  conditions: {
    temperature_c: 50,
    ph: 8.0,
    duration_hours: 48
  }
});

console.log(`Expected: ${prediction.predictions.degradation_percent.mean}% degradation`);
```

---

## Standard Phases

### Phase 1: Data Format

Defines JSON schemas for:
- Enzyme profiles with kinetic parameters
- Degradation process records
- Quality metrics and certifications

See: [spec/PHASE-1-DATA-FORMAT.md](./spec/PHASE-1-DATA-FORMAT.md)

### Phase 2: API Interface

RESTful API specification:
- Enzyme library queries
- Plastic identification
- Degradation prediction
- Process optimization

See: [spec/PHASE-2-API-INTERFACE.md](./spec/PHASE-2-API-INTERFACE.md)

### Phase 3: Protocol

Operational protocols:
- Pre-treatment procedures
- Enzymatic reaction conditions
- Monomer recovery methods
- Quality control standards

See: [spec/PHASE-3-PROTOCOL.md](./spec/PHASE-3-PROTOCOL.md)

### Phase 4: Integration

System integration:
- Waste collection connectivity
- Chemical supply chain
- Carbon footprint tracking
- Digital twin support

See: [spec/PHASE-4-INTEGRATION.md](./spec/PHASE-4-INTEGRATION.md)

---

## Supported Enzymes

| Enzyme | Source | Optimal Temp | Target |
|--------|--------|--------------|--------|
| PETase (IsPETase) | *Ideonella sakaiensis* | 30-37°C | PET |
| TurboPETase | Engineered | 50-55°C | PET |
| GuaPA | Engineered | 50-70°C | PET |
| DIEGO | Engineered | 55-60°C | PET |
| MHETase | *Ideonella sakaiensis* | 30-37°C | MHET |
| Cutinase | Various fungi | 40-60°C | PET, PCL |

---

## Supported Plastics

| Code | Name | Degradation Pathway |
|------|------|---------------------|
| PET | Polyethylene Terephthalate | PET → MHET → TPA + EG |
| PLA | Polylactic Acid | PLA → Lactic Acid |
| PBAT | Polybutylene Adipate Terephthalate | PBAT → Monomers |
| PCL | Polycaprolactone | PCL → 6-HHA |
| PHA | Polyhydroxyalkanoates | PHA → 3-HB |

---

## Directory Structure

```
WIA-PLASTIC-ENZYME/
├── index.html              # Landing page
├── README.md               # This file
├── spec/                   # Technical specifications
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/
│   └── typescript/         # TypeScript SDK
│       ├── src/
│       │   ├── index.ts    # Main client
│       │   └── types.ts    # Type definitions
│       └── package.json
├── simulator/              # Interactive simulator
│   └── index.html
└── ebook/                  # Educational content
    ├── en/                 # English version
    └── ko/                 # Korean version
```

---

## API Reference

### Enzyme Operations

```typescript
// List enzymes with filtering
const enzymes = await client.listEnzymes({
  classification: 'PETase',
  min_temperature: 40,
  max_temperature: 60
});

// Get enzyme details
const enzyme = await client.getEnzyme('turbopetase-v2');

// Search enzymes
const results = await client.searchEnzymes('thermostable PETase');
```

### Plastic Identification

```typescript
// Identify from FTIR spectrum
const plastic = await client.identifyPlastic({
  method: 'ftir_spectrum',
  data: {
    wavenumbers: [1720, 1410, 1240, ...],
    absorbance: [0.85, 0.42, 0.68, ...]
  }
});

console.log(`Type: ${plastic.plastic_type}, Confidence: ${plastic.confidence}`);
```

### Process Optimization

```typescript
// Optimize for cost/speed/efficiency
const optimized = await client.optimizeConditions({
  plastic: { type: 'PET', weight_kg: 1000, crystallinity_percent: 30 },
  constraints: {
    max_temperature_c: 60,
    max_time_hours: 48,
    target_efficiency: 0.95
  },
  optimization_goal: 'cost'
});
```

### Quality Certification

```typescript
// Submit quality metrics
const quality = await client.submitQualityMetrics({
  batch_id: 'BATCH-2025-001',
  measurement_date: '2025-01-15',
  monomer_analysis: {
    tpa: { concentration_mM: 850, purity_percent: 99.2 },
    eg: { concentration_mM: 420, purity_percent: 99.5 }
  }
});

// Request certification
const cert = await client.requestCertification('BATCH-2025-001', 'food-contact');
```

---

## Interactive Simulator

Try the hands-on simulator to experiment with enzyme-based degradation:

- **Data Format Tab**: Explore enzyme and process schemas
- **Algorithms Tab**: Match enzymes to plastics
- **Protocol Tab**: Simulate degradation processes
- **Integration Tab**: Test API endpoints
- **Test Tab**: Run validation tests

[Launch Simulator](./simulator/)

---

## Educational Ebook

Comprehensive guide covering:

1. The Plastic Crisis
2. Enzyme Revolution
3. PETase & Beyond
4. Phase 1: Data Format
5. Phase 2: API Interface
6. Phase 3: Protocols
7. Phase 4: Integration
8. Circular Economy Future

Available in:
- [English](./ebook/en/)
- [Korean](./ebook/ko/)

---

## Environmental Impact

Enzymatic recycling vs. traditional methods:

| Metric | Virgin PET | Mechanical | Enzymatic |
|--------|-----------|------------|-----------|
| CO2 (kg/ton) | 2,100 | 450 | 75 |
| Energy (kWh/ton) | 850 | 320 | 50 |
| Quality | New | Degraded | Like-new |
| Cycles | - | 2-3 | Infinite |

---

## Contributing

We welcome contributions! Please see our contribution guidelines:

1. Fork the repository
2. Create a feature branch
3. Follow the WIA coding standards
4. Submit a pull request

---

## License

MIT License - see [LICENSE](LICENSE) for details.

---

## Links

- [WIA Standards](https://wiastandards.com)
- [GitHub Repository](https://github.com/WIA-Official/wia-standards)
- [API Documentation](https://api.wiastandards.com/plastic-enzyme/docs)

---

## Philosophy

**홍익인간 (弘益人間) - Benefit All Humanity** - Benefit All Humanity

This standard embodies the principle that technology should serve humanity's greater good. By enabling true circular economy for plastics, we can:

- Eliminate plastic pollution from our oceans and landfills
- Reduce dependence on fossil fuel-based virgin plastics
- Create sustainable supply chains for future generations
- Democratize advanced recycling technology globally

---

**© 2025 WIA - World Certification Industry Association**

*홍익인간 (弘益人間) - Benefit All Humanity*
