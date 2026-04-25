# 🏦 WIA-BIO-019: Bio-Banking Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-019
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO / Biotechnology
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-019 standard defines the comprehensive framework for biological sample banking, including collection, processing, storage, retrieval, quality management, and regulatory compliance for biospecimens.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to ensure the highest quality preservation of biological materials, enabling medical research, diagnostics, and therapeutic development that benefits all of humanity.

## 🎯 Key Features

- **Sample Collection**: Standardized protocols for collecting blood, tissue, DNA, cells, and fluids
- **Storage Systems**: Multi-tier storage from liquid nitrogen (-196°C) to ambient conditions
- **Quality Management**: ISO 20387:2018 compliant quality systems
- **Sample Tracking**: Complete chain of custody and LIMS integration
- **Consent Management**: Ethical framework for donor consent and privacy
- **Retrieval Protocols**: Rapid, quality-preserved sample distribution

## 📊 Core Concepts

### 1. Sample Integrity Score

```
SIS = (Vt / V0) × (1 - ΔT/Tmax) × (1 - Δt/tmax)
```

Where:
- `SIS` = Sample Integrity Score (0-1)
- `Vt` = Viability at time t
- `V0` = Initial viability
- `ΔT` = Temperature deviation from optimal
- `Tmax` = Maximum acceptable temperature deviation
- `Δt` = Time since collection
- `tmax` = Maximum storage duration

### 2. Storage Quality Index

```
SQI = exp(-k × t × exp(Ea/RT))
```

Where:
- `SQI` = Storage Quality Index
- `k` = Degradation rate constant
- `t` = Storage time
- `Ea` = Activation energy (kJ/mol)
- `R` = Gas constant (8.314 J/mol·K)
- `T` = Storage temperature (Kelvin)

### 3. Chain of Custody Verification

```
COC = H(S0) → H(S1) → H(S2) → ... → H(Sn)
```

Where:
- `COC` = Chain of custody hash chain
- `H()` = Cryptographic hash function
- `Sn` = Sample state at checkpoint n

## 🔧 Components

### TypeScript SDK

```typescript
import {
  BioBankingSDK,
  SampleType,
  StorageCondition,
  calculateSampleIntegrity
} from '@wia/bio-019';

const biobank = new BioBankingSDK();

// Register a new sample
const sample = await biobank.registerSample({
  type: SampleType.BLOOD,
  donorId: 'D-12345',
  collectionDate: new Date(),
  volume: 10, // mL
  storageTemp: -80 // °C
});

// Calculate integrity score
const integrity = calculateSampleIntegrity({
  initialViability: 0.98,
  currentViability: 0.95,
  tempDeviation: 2, // °C
  maxTempDeviation: 5,
  storageTime: 365, // days
  maxStorageTime: 3650
});

console.log(`Sample integrity: ${(integrity * 100).toFixed(1)}%`);
```

### CLI Tool

```bash
# Register new sample
wia-bio-019 register --type blood --donor D-12345 --volume 10 --temp -80

# Check sample integrity
wia-bio-019 integrity --id SAM-001 --current-viability 0.95

# Retrieve sample
wia-bio-019 retrieve --id SAM-001 --purpose research

# Generate inventory report
wia-bio-019 inventory --storage-type liquid-nitrogen
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-019-v1.0.md](./spec/WIA-BIO-019-v1.0.md) | Complete specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-019.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/bio-banking

# Run installation script
./install.sh

# Verify installation
wia-bio-019 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-019

# Or yarn
yarn add @wia/bio-019
```

```typescript
import { BioBankingSDK, SampleType, StorageCondition } from '@wia/bio-019';

const sdk = new BioBankingSDK();

// Register tissue sample
const sample = sdk.registerSample({
  id: 'SAM-001',
  type: SampleType.TISSUE,
  donorId: 'D-12345',
  collectionDate: new Date(),
  volume: 5,
  storageCondition: StorageCondition.LIQUID_NITROGEN,
  consent: {
    consentId: 'CNS-001',
    purpose: ['research', 'diagnostics'],
    expiryDate: new Date('2030-12-31')
  }
});

console.log(`Sample ${sample.id} registered successfully`);
```

## 🧬 Sample Types

| Type | Storage Temp | Max Duration | Quality Metric |
|------|--------------|--------------|----------------|
| Blood (Whole) | -80°C | 10 years | Hemolysis index |
| Plasma/Serum | -80°C | 20 years | Protein stability |
| DNA | -20°C | 50+ years | Fragment integrity |
| RNA | -80°C | 5 years | RIN score |
| Tissue | -196°C (LN₂) | Indefinite | Cell viability |
| Cells (Live) | -196°C (LN₂) | Indefinite | Post-thaw viability |
| Urine | -80°C | 5 years | Metabolite stability |

## ❄️ Storage Conditions

| Condition | Temperature | Applications | Cost Factor |
|-----------|-------------|--------------|-------------|
| Liquid Nitrogen | -196°C | Cells, tissue, long-term | High (5×) |
| Ultra-Low Freezer | -80°C | Blood, DNA, RNA | Medium (3×) |
| Standard Freezer | -20°C | DNA, some proteins | Low (1×) |
| Refrigerated | 4°C | Short-term, some samples | Very Low (0.5×) |
| Ambient | 20-25°C | Fixed tissue, dried samples | Minimal (0.1×) |

## 🔬 Quality Standards

This standard complies with:
- **ISO 20387:2018**: Biotechnology - Biobanking - General requirements
- **ISO 15189**: Medical laboratories - Requirements for quality and competence
- **ISBER Best Practices**: International Society for Biological and Environmental Repositories
- **GDPR**: General Data Protection Regulation (for EU biobanks)
- **21 CFR Part 1271**: FDA regulations for human cells and tissues

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based sample queries
- **WIA-OMNI-API**: Universal biobanking API gateway
- **WIA-SOCIAL**: Collaborative research networks
- **WIA-HEALTH**: Healthcare data integration

## 📖 Use Cases

1. **Tissue Banks**: Long-term preservation of surgical specimens
2. **Cord Blood Banking**: Newborn stem cell storage for future therapy
3. **DNA Repositories**: Population genetics and disease studies
4. **Clinical Trial Biobanks**: Drug development sample management
5. **Reproductive Banking**: Sperm, egg, and embryo preservation
6. **Research Collections**: Academic and pharmaceutical research samples

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
