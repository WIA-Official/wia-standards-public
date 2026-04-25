# WIA-CONSCIOUSNESS

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


## Scientific Consciousness Measurement Standard

**홍익인간 (弘益人間) - Benefit All Humanity**

<p align="center">
  <span style="font-size: 80px;">🧠</span>
  <br>
  <span style="font-size: 60px; color: #8B5CF6;">Φ</span>
</p>

---

## Overview

WIA-CONSCIOUSNESS provides a unified framework for measuring and understanding consciousness based on:

- **Integrated Information Theory (IIT 4.0)** - Measuring Φ (integrated information)
- **Global Neuronal Workspace Theory (GNW)** - Detecting global ignition and broadcast
- **Perturbational Complexity Index (PCI)** - Clinical consciousness measurement
- **AI Consciousness Assessment** - Evaluating consciousness indicators in AI systems

## Key Features

| Feature | Description |
|---------|-------------|
| **Φ Estimation** | Calculate integrated information using IIT 4.0 formalism |
| **PCI Measurement** | TMS-EEG based perturbational complexity index (threshold: 0.31) |
| **Global Workspace** | GNW metrics for ignition and broadcast detection |
| **AI Assessment** | Butlin et al. 14 indicators framework for AI consciousness |
| **Clinical Protocols** | Standardized VS/MCS diagnosis procedures |
| **Real-time Monitoring** | WebSocket-based continuous consciousness tracking |

## Quick Start

### Installation

```bash
npm install @wia/consciousness
```

### Usage

```typescript
import { ConsciousnessClient, classifyPCI, isConscious } from '@wia/consciousness';

// Initialize client
const client = new ConsciousnessClient({
  apiKey: 'your-api-key'
});

// Measure consciousness
const measurement = await client.measure({
  subject_id: 'patient-001',
  method: 'TMS-EEG',
  parameters: {
    tms_intensity: 80,
    pulse_count: 200
  }
});

// Get PCI value
const pci = await client.getPCI('patient-001');
console.log(`PCI: ${pci.value}`);

// Check consciousness
if (isConscious(pci.value)) {
  console.log(`State: ${classifyPCI(pci.value)}`);
}

// Real-time monitoring
const unsubscribe = client.monitor('patient-001', (update) => {
  console.log(`PCI: ${update.pci}, State: ${update.state}`);
  if (update.alert) {
    console.warn(`Alert: ${update.alert.message}`);
  }
});
```

## Clinical Classification

| State | PCI Range | Description |
|-------|-----------|-------------|
| VS/UWS | < 0.31 | Vegetative State / Unresponsive Wakefulness Syndrome |
| MCS- | 0.31 - 0.37 | Minimally Conscious State (Minus) |
| MCS+ | 0.37 - 0.49 | Minimally Conscious State (Plus) |
| EMCS | 0.49 - 0.52 | Emerged from MCS |
| LIS | 0.51 - 0.62 | Locked-in Syndrome |
| Conscious | > 0.50 | Full Consciousness |

## Project Structure

```
WIA-CONSCIOUSNESS/
├── index.html              # Landing page
├── simulator/
│   └── index.html          # Interactive simulator (5 tabs, 99 languages)
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

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/measure` | Initiate consciousness measurement |
| GET | `/measure/{id}` | Get measurement results |
| GET | `/state/{subject_id}` | Get consciousness state |
| GET | `/pci/{subject_id}` | Get PCI measurement |
| GET | `/phi/{subject_id}` | Get Φ estimate |
| POST | `/compare` | Compare IIT/GNW predictions |
| POST | `/ai/assess` | Assess AI consciousness indicators |
| WS | `/monitor/{subject_id}` | Real-time monitoring |

## Consciousness Theories

### Integrated Information Theory (IIT 4.0)

IIT proposes that consciousness IS integrated information (Φ). The five axioms:

1. **Intrinsicality** - Experience exists for the experiencer
2. **Information** - Experience is specific
3. **Integration** - Experience is unified
4. **Exclusion** - Experience has definite boundaries
5. **Composition** - Experience is structured

### Global Neuronal Workspace (GNW)

GNW proposes that consciousness arises when information is broadcast globally across brain regions through prefrontal-parietal networks.

## References

- [IIT 4.0 (PLOS Computational Biology, 2023)](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1011465)
- [IIT vs GNW Adversarial Study (Nature, 2025)](https://www.nature.com/articles/s41586-025-08888-1)
- [PCI for Consciousness Detection (PubMed)](https://pubmed.ncbi.nlm.nih.gov/23946194/)
- [IIT Wiki (Wisconsin)](https://centerforsleepandconsciousness.psychiatry.wisc.edu/integrated-information-theory/)

## Ethical Considerations

Consciousness measurement carries profound ethical implications:

- **Patient Rights**: Respect dignity in consciousness disorder patients
- **AI Moral Status**: Careful consideration of AI consciousness claims
- **Animal Welfare**: Implications for animal consciousness understanding
- **Data Privacy**: Protecting sensitive consciousness data

## License

MIT License

---

**弘益人間 (弘益人間) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
