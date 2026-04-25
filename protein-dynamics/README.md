# WIA-PROTEIN-DYNAMICS

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standardizing Protein Dynamics for the Post-AlphaFold Era**

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

AlphaFold revolutionized protein structure prediction, but proteins are not static. They breathe, flex, and dance through conformational landscapes. **WIA-PROTEIN-DYNAMICS** bridges this critical gap by standardizing how we represent, exchange, and analyze protein dynamics data.

```
Protein Function = Structure × Dynamics × Environment
```

### The Problem

Current protein dynamics research suffers from:

- **Fragmented Formats**: No universal standard for dynamics data
- **Lost Information**: Static structures miss 80% of functional mechanisms
- **Reproducibility Crisis**: Different labs, different methods, incompatible results
- **Drug Discovery Gaps**: Binding kinetics often more important than affinity

### The Solution

A unified framework that captures:

- **Conformational Ensembles**: Multiple states with populations
- **Dynamics Metrics**: RMSF, B-factors, order parameters
- **Free Energy Landscapes**: Energy basins and transition barriers
- **Drug Binding Dynamics**: kon, koff, residence time
- **Allosteric Networks**: Communication pathways

---

## Quick Start

### Installation

```bash
# TypeScript/JavaScript
npm install @wia/protein-dynamics

# Python
pip install wia-protein-dynamics

# CLI
curl -sSL https://wia-standards.org/protein-dynamics/install.sh | bash
```

### Usage

```typescript
import { WIAProteinDynamics } from '@wia/protein-dynamics';

const client = new WIAProteinDynamics({
  apiKey: 'your-api-key'
});

// Fetch protein dynamics profile
const profile = await client.getProteinDynamics('P00533');

// Access conformational ensemble
console.log(`States: ${profile.data?.conformational_ensemble?.states.length}`);
for (const state of profile.data?.conformational_ensemble?.states || []) {
  console.log(`  ${state.name}: ${(state.population * 100).toFixed(1)}%`);
}

// Generate new ensemble using AlphaFlow
const job = await client.generateEnsemble({
  protein_id: 'P00533',
  method: 'alphaflow'
});
```

```python
from wia_pd import fetch, analyze

# Fetch dynamics profile
dynamics = fetch("P00533")

# Analyze flexibility
flexibility = analyze.flexibility(dynamics)
print(f"Most flexible region: residues {flexibility.hotspots[0]}")

# Find cryptic binding sites
sites = analyze.find_cryptic_sites(dynamics)
for site in sites:
    print(f"Cryptic site: {site.residues}, druggability: {site.score:.2f}")
```

---

## Specification Phases

### Phase 1: Data Format

Standardized JSON schema for protein dynamics data:

```json
{
  "wia_version": "1.0",
  "protein_id": "P00533",
  "conformational_ensemble": {
    "method": "metadynamics",
    "states": [
      {
        "name": "active",
        "population": 0.65,
        "relative_energy_kJ_mol": 0
      },
      {
        "name": "inactive",
        "population": 0.35,
        "relative_energy_kJ_mol": 2.5
      }
    ]
  },
  "dynamics_metrics": {
    "flexibility": {
      "rmsf_angstrom": [0.5, 0.6, 1.2, ...],
      "flexible_regions": [[45, 52], [120, 135]]
    }
  }
}
```

### Phase 2: API Interface

RESTful API for dynamics data access and computation:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/proteins/{id}` | GET | Fetch dynamics profile |
| `/proteins/{id}/ensemble` | GET | Get conformational ensemble |
| `/ensemble/generate` | POST | Generate new ensemble |
| `/binding/predict` | POST | Predict drug binding |
| `/protocols` | GET | List simulation protocols |

### Phase 3: Protocols

Standardized simulation and analysis workflows:

| Protocol ID | Name | Purpose |
|-------------|------|---------|
| WIA-PD-ENS-001 | MD Ensemble | Standard MD simulation |
| WIA-PD-ENS-002 | Metadynamics | Enhanced sampling |
| WIA-PD-ENS-003 | AlphaFlow | ML-based ensemble |
| WIA-PD-FLEX-001 | Flexibility | RMSF, B-factors |
| WIA-PD-ALLO-001 | Allosteric | Network analysis |
| WIA-PD-BIND-001 | Binding | Drug binding pathway |

### Phase 4: Integration

Connections to external databases and tools:

- **PDB**: Structure retrieval and B-factor extraction
- **AlphaFold DB**: Predicted structures and confidence
- **UniProt**: Sequence and functional annotations
- **ChEMBL**: Bioactivity and binding data
- **BindingDB**: Kinetic parameters

---

## Key Features

### Conformational Ensembles

Represent proteins as dynamic ensembles, not single structures:

```
Active ⟷ Intermediate ⟷ Inactive
 65%        10%           25%
```

### Free Energy Landscapes

Quantify the energy barriers between states:

```
Energy (kJ/mol)
     │
  15 ┤     ╱╲
  10 ┤    ╱  ╲    ╱╲
   5 ┤   ╱    ╲  ╱  ╲
   0 ┼──●      ●●    ●──
     Active  TS  Inactive
```

### Drug Binding Kinetics

Beyond affinity - capture binding dynamics:

```
Drug A: Kd = 1 nM, koff = 0.01 s⁻¹  → τ = 100 s
Drug B: Kd = 1 nM, koff = 0.0001 s⁻¹ → τ = 10,000 s

Same affinity, 100× different duration!
```

### Allosteric Networks

Map communication pathways through proteins:

```
Allosteric Site → Pathway → Active Site
   [55-60]    →  [75,92,110] → [145-170]
```

---

## Directory Structure

```
protein-dynamics/
├── index.html                    # Landing page
├── README.md                     # This file
├── simulator/
│   └── index.html                # Interactive simulator (5 tabs)
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md    # JSON schema specification
│   ├── PHASE-2-API-INTERFACE.md  # RESTful API specification
│   ├── PHASE-3-PROTOCOL.md       # Simulation protocols
│   └── PHASE-4-INTEGRATION.md    # Database integrations
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts          # TypeScript type definitions
│       │   └── index.ts          # SDK implementation
│       ├── package.json
│       └── tsconfig.json
└── ebook/
    ├── en/                       # English documentation
    │   ├── 01-cover.md
    │   ├── 02-beyond-alphafold.md
    │   ├── 03-dynamics-fundamentals.md
    │   ├── 04-conformational-ensembles.md
    │   ├── 05-phase1-data-format.md
    │   ├── 06-phase2-api.md
    │   ├── 07-phase3-protocols.md
    │   ├── 08-phase4-integration.md
    │   └── 09-drug-discovery.md
    └── ko/                       # Korean documentation
        ├── 01-cover.md
        ├── 02-beyond-alphafold.md
        └── ... (matching English chapters)
```

---

## Applications

### Drug Discovery

- **Binding kinetics optimization**: Design drugs with optimal residence time
- **Cryptic site discovery**: Find hidden pockets in conformational ensembles
- **Resistance prediction**: Anticipate mutations affecting drug binding
- **Selectivity optimization**: Target unique conformational states

### Protein Engineering

- **Stability engineering**: Understand flexibility-stability tradeoffs
- **Allosteric design**: Engineer remote control sites
- **Enzyme optimization**: Balance dynamics and catalysis

### Disease Research

- **Conformational diseases**: Study misfolding in Alzheimer's, Parkinson's
- **Cancer mutations**: Understand oncogenic activation mechanisms
- **Intrinsically disordered proteins**: Characterize fuzzy interactions

---

## Timescale Coverage

```
fs        ps        ns        μs        ms        s
|---------|---------|---------|---------|---------|
 Bond vib  Side      Loop      Domain    Folding   Function
           chain     motion    motion

 ←─── Simulation ───→←─── Enhanced Sampling ───→←─ Experiment →
```

---

## Supported Methods

### Simulation Engines

- GROMACS
- AMBER
- OpenMM
- NAMD
- Desmond

### Enhanced Sampling

- Metadynamics
- Replica Exchange MD (REMD)
- Accelerated MD
- Steered MD

### ML Methods

- AlphaFlow
- Boltzmann Generators
- Flow Matching
- Diffusion Models

---

## Contributing

Contributions are welcome! Please see our [Contributing Guide](../../CONTRIBUTING.md).

### Development

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/protein-dynamics

# Install dependencies
cd api/typescript && npm install

# Run tests
npm test

# Build
npm run build
```

---

## Resources

- **Landing Page**: [https://wia-standards.org/protein-dynamics](https://wia-standards.org/protein-dynamics)
- **Interactive Simulator**: [https://wia-standards.org/protein-dynamics/simulator](https://wia-standards.org/protein-dynamics/simulator)
- **API Documentation**: [https://api.wia-standards.org/protein-dynamics/docs](https://api.wia-standards.org/protein-dynamics/docs)
- **Ebook (English)**: [./ebook/en/](./ebook/en/)
- **Ebook (Korean)**: [./ebook/ko/](./ebook/ko/)

---

## Related Standards

- **WIA-AI-CHIP**: AI accelerator standards (for dynamics computation)
- **WIA-FUSION**: Nuclear fusion standards (for high-performance computing)
- **WIA-HOME**: Smart home standards (IoT integration patterns)

---

## License

MIT License - see [LICENSE](../../LICENSE)

---

## Acknowledgments

This standard draws inspiration from:

- The AlphaFold team at DeepMind
- The molecular dynamics community (GROMACS, AMBER, OpenMM)
- D.E. Shaw Research for long-timescale simulations
- The PDB, UniProt, and ChEMBL databases

---

**弘益人間 - Benefit All Humanity**

*Proteins move. Now we can standardize how we describe it.*

© 2025 WIA - World Certification Industry Association

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
