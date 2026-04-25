# 📡 WIA-BIO-011: Biosensor Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-011
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO (Biotechnology)
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-011 standard defines the comprehensive framework for biosensor technology, including sensor types, transduction mechanisms, biorecognition elements, signal processing, and performance metrics for biomedical and environmental applications.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to provide a scientifically-grounded foundation for biosensor development that enables accessible, accurate, and real-time biological detection for improved healthcare and environmental monitoring worldwide.

## 🎯 Key Features

- **Sensor Types**: Electrochemical, optical, piezoelectric, and thermal biosensors
- **Biorecognition Elements**: Enzymes, antibodies, aptamers, DNA/RNA probes
- **Transduction Mechanisms**: Signal conversion from biological to measurable output
- **Signal Processing**: Amplification, filtering, and calibration algorithms
- **Performance Metrics**: Sensitivity, specificity, LOD, dynamic range, response time
- **Point-of-Care Applications**: Rapid diagnostic testing and field deployment

## 📊 Core Concepts

### 1. Sensitivity

```
S = Δsignal / Δconcentration
```

Where:
- `S` = Sensitivity (signal/concentration)
- `Δsignal` = Change in sensor signal
- `Δconcentration` = Change in analyte concentration

### 2. Limit of Detection (LOD)

```
LOD = 3.3 × (σ / S)
```

Where:
- `LOD` = Minimum detectable concentration
- `σ` = Standard deviation of blank signal
- `S` = Sensitivity (slope of calibration curve)

### 3. Dynamic Range

```
DR = log₁₀(Cₘₐₓ / LOD)
```

Where:
- `DR` = Dynamic range (orders of magnitude)
- `Cₘₐₓ` = Maximum measurable concentration
- `LOD` = Limit of detection

### 4. Response Time

```
t₉₀ = time to reach 90% of steady-state signal
```

Where:
- `t₉₀` = Response time (seconds)
- Typical values: 1-60 seconds for rapid biosensors

## 🔧 Components

### TypeScript SDK

```typescript
import {
  BiosensorSDK,
  calculateSensitivity,
  calculateLOD,
  calibrateSensor,
  processBiosensorSignal
} from '@wia/bio-011';

// Create biosensor instance
const sdk = new BiosensorSDK();

// Configure electrochemical sensor
const sensor = sdk.createSensor({
  type: 'electrochemical',
  biorecognitionElement: 'glucose-oxidase',
  transducer: 'amperometric',
  workingElectrode: 'platinum',
  referenceElectrode: 'ag-agcl'
});

// Calibrate sensor
const calibration = sdk.calibrateSensor({
  sensorId: sensor.id,
  standardConcentrations: [0, 1, 5, 10, 50, 100], // mM
  measuredSignals: [0.05, 1.2, 5.8, 11.5, 58.2, 115.8] // μA
});

console.log(`Sensitivity: ${calibration.sensitivity} μA/mM`);
console.log(`LOD: ${calibration.lod} mM`);
console.log(`R²: ${calibration.rSquared}`);

// Measure sample
const measurement = sdk.measureAnalyte({
  sensorId: sensor.id,
  signal: 23.5, // μA
  temperature: 37, // °C
  calibration: calibration
});

console.log(`Glucose concentration: ${measurement.concentration} mM`);
console.log(`Confidence: ${measurement.confidence * 100}%`);
```

### CLI Tool

```bash
# Create and configure biosensor
wia-bio-011 create-sensor --type electrochemical --bioelement glucose-oxidase

# Calibrate sensor
wia-bio-011 calibrate --sensor-id BS-001 \
  --standards "0,1,5,10,50,100" \
  --signals "0.05,1.2,5.8,11.5,58.2,115.8"

# Measure sample
wia-bio-011 measure --sensor-id BS-001 --signal 23.5 --temp 37

# Calculate LOD
wia-bio-011 calc-lod --blank-stddev 0.015 --sensitivity 1.15

# Validate sensor performance
wia-bio-011 validate --sensor-id BS-001 --accuracy 95 --precision 3
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-011-v1.0.md](./spec/WIA-BIO-011-v1.0.md) | Complete specification with biosensor theory |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-011.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/biosensor

# Run installation script
./install.sh

# Verify installation
wia-bio-011 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-011

# Or yarn
yarn add @wia/bio-011
```

```typescript
import { BiosensorSDK } from '@wia/bio-011';

const sdk = new BiosensorSDK();

// Create glucose biosensor
const sensor = sdk.createSensor({
  type: 'electrochemical',
  biorecognitionElement: 'glucose-oxidase',
  targetAnalyte: 'glucose'
});

// Perform calibration
const calibration = sdk.calibrateSensor({
  sensorId: sensor.id,
  standardConcentrations: [0, 5, 10, 25, 50],
  measuredSignals: [0.1, 5.2, 10.5, 25.8, 51.2]
});

console.log(`Sensitivity: ${calibration.sensitivity}`);
console.log(`LOD: ${calibration.lod}`);
console.log(`Linear range: ${calibration.linearRange.min} - ${calibration.linearRange.max}`);
```

## 🔬 Biosensor Types

| Type | Transduction | Applications | Typical LOD |
|------|--------------|--------------|-------------|
| Electrochemical | Current/Voltage | Glucose, lactate, cholesterol | nM - μM |
| Optical | Light absorption/emission | DNA, proteins, pathogens | pM - nM |
| Piezoelectric | Mass change | Bacteria, viruses, toxins | ng/mL |
| Thermal | Heat generation | Enzyme activity, metabolism | μM - mM |
| Impedance | Electrical resistance | Cell counting, viability | 10³ - 10⁶ cells/mL |

## 📊 Performance Metrics

| Metric | Symbol | Typical Range | Unit |
|--------|--------|---------------|------|
| Sensitivity | S | 0.1 - 1000 | signal/concentration |
| Limit of Detection | LOD | pM - μM | molar |
| Dynamic Range | DR | 3 - 6 | orders of magnitude |
| Response Time | t₉₀ | 1 - 60 | seconds |
| Selectivity | α | >100:1 | ratio |
| Stability | t₁/₂ | 1 - 365 | days |

## ⚠️ Quality Assurance

1. **Calibration**: Use minimum 5-point calibration curve with R² > 0.99
2. **Validation**: Verify accuracy (±5%), precision (CV < 5%), linearity
3. **Quality Control**: Run blank, low, and high controls with each batch
4. **Temperature Control**: Maintain ±1°C for enzymatic biosensors
5. **Storage**: Follow manufacturer guidelines (typically 2-8°C)
6. **Shelf Life**: Monitor performance degradation, typical 3-12 months

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based biosensor configuration
- **WIA-OMNI-API**: Universal biosensor data API
- **WIA-SOCIAL**: Collaborative health monitoring networks
- **WIA-HEALTH**: Electronic health record integration

## 📖 Use Cases

1. **Glucose Monitoring**: Continuous glucose monitoring for diabetes management
2. **Pathogen Detection**: Rapid detection of bacteria and viruses (E. coli, COVID-19)
3. **Environmental Monitoring**: Water quality testing, toxin detection
4. **Point-of-Care Diagnostics**: Cardiac markers (troponin), pregnancy tests
5. **Drug Discovery**: High-throughput screening, enzyme assays
6. **Food Safety**: Detection of allergens, contaminants, spoilage

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
