# 🔬 WIA-BIO-008: Protein Structure Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-008
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO / Bioinformatics & Life Sciences
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-008 standard defines the computational framework for protein structure prediction, analysis, and molecular dynamics simulations, including structure determination, AlphaFold integration, protein-ligand docking, and quality assessment.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to accelerate drug discovery, enzyme engineering, and our understanding of disease mechanisms by providing a unified framework for protein structure analysis.

## 🎯 Key Features

- **Structure Prediction**: AlphaFold, RoseTTAFold, and traditional methods
- **Quality Assessment**: RMSD, TM-score, and validation metrics
- **Molecular Dynamics**: Simulation of protein motion and stability
- **Protein-Ligand Docking**: Binding site prediction and affinity calculation
- **Structural Comparison**: Alignment and similarity analysis
- **Data Integration**: PDB, AlphaFold DB, and UniProt compatibility

## 📊 Core Concepts

### 1. Root Mean Square Deviation (RMSD)

```
RMSD = √(1/N ∑ᵢ₌₁ᴺ ||rᵢ - r'ᵢ||²)
```

Where:
- `N` = Number of atoms
- `rᵢ` = Position of atom i in structure 1
- `r'ᵢ` = Position of atom i in structure 2

### 2. TM-Score (Template Modeling Score)

```
TM-score = max[1/Lₜ ∑ᵢ₌₁ᴸᵃˡⁱᵍⁿ 1/(1 + (dᵢ/d₀)²)]
```

Where:
- `Lₜ` = Target protein length
- `Lₐₗᵢgₙ` = Number of aligned residues
- `dᵢ` = Distance between aligned residues
- `d₀` = Normalization factor (1.24 ∛(Lₜ - 15) - 1.8)

### 3. Binding Affinity (ΔG)

```
ΔG = ΔH - TΔS
```

Where:
- `ΔG` = Gibbs free energy (kcal/mol)
- `ΔH` = Enthalpy change
- `T` = Temperature (K)
- `ΔS` = Entropy change

### 4. Molecular Dynamics Energy

```
E_total = E_bonded + E_non-bonded
E_bonded = E_bond + E_angle + E_dihedral
E_non-bonded = E_vdw + E_electrostatic
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  predictStructure,
  calculateRMSD,
  runDocking,
  simulateDynamics,
  assessQuality
} from '@wia/bio-008';

// Predict protein structure using AlphaFold
const structure = await predictStructure({
  sequence: 'MKTAYIAKQRQISFVKSHFSRQLEERLGLIEVQAPILSRVGDGTQDNLSGAEKAVQVKVKALPDAQFEVVHSLAKWKRQTLGQHDFSAGEGLYTHMKALRPDEDRLSPLHSVYVDQWDWERVMGDGERQFSTLKSTVEAIWAGIKATEAAVSEEFGLAPFLPDQIHFVHSQELLSRYPDLDAKGRERAIAKDLGAVFLVGIGGKLSDGHRHDVRAPDYDDWSTPSELGHAGLNGDILVWNPVLEDAFELSSMGIRVDADTLKHQLALTGDEDRLELEWHQALLRGEMPQTIGGGIGQSRLTMLLLQLPHIGQVQAGVWPAAVRESVPSLL',
  method: 'alphafold',
  templates: []
});

// Calculate RMSD between two structures
const rmsd = calculateRMSD({
  structure1: structure,
  structure2: referenceStructure,
  atoms: 'CA' // Compare alpha carbons
});

console.log(`RMSD: ${rmsd.value.toFixed(2)} Å`);

// Perform protein-ligand docking
const docking = await runDocking({
  protein: structure,
  ligand: 'CC(C)Cc1ccc(cc1)C(C)C(O)=O', // Ibuprofen SMILES
  bindingSite: { center: { x: 10, y: 15, z: 20 }, radius: 10 },
  exhaustiveness: 8
});

console.log(`Binding affinity: ${docking.bindingAffinity} kcal/mol`);
```

### CLI Tool

```bash
# Predict protein structure
wia-bio-008 predict --sequence "MKTAYIAK..." --method alphafold

# Calculate RMSD
wia-bio-008 rmsd --structure1 protein1.pdb --structure2 protein2.pdb

# Perform docking
wia-bio-008 dock --protein target.pdb --ligand ligand.sdf --site "10,15,20"

# Run molecular dynamics
wia-bio-008 simulate --structure protein.pdb --duration 100 --temp 310

# Assess structure quality
wia-bio-008 validate --structure predicted.pdb --native native.pdb
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-008-v1.0.md](./spec/WIA-BIO-008-v1.0.md) | Complete specification with algorithms |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-008.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/protein-structure

# Run installation script
./install.sh

# Verify installation
wia-bio-008 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-008

# Or yarn
yarn add @wia/bio-008
```

```typescript
import { ProteinStructureSDK } from '@wia/bio-008';

const sdk = new ProteinStructureSDK();

// Predict structure
const prediction = await sdk.predictStructure({
  sequence: 'MKTAYIAKQRQISFVK...',
  method: 'alphafold'
});

console.log(`Predicted structure with ${prediction.residues.length} residues`);
console.log(`Confidence: ${prediction.confidence.mean.toFixed(2)}`);
console.log(`Secondary structure: ${prediction.secondaryStructure.helixRatio.toFixed(1)}% helix`);
```

## 🧬 Structure Determination Methods

| Method | Resolution | Time | Advantages |
|--------|-----------|------|------------|
| X-ray Crystallography | 1-3 Å | Weeks | High resolution, established |
| NMR Spectroscopy | 2-5 Å | Weeks | Solution state, dynamics |
| Cryo-EM | 2-10 Å | Days | Large complexes, native state |
| AlphaFold | N/A | Minutes | Fast, no experimental data needed |
| Homology Modeling | 2-5 Å | Hours | Template-based, reliable |

## 📈 Quality Metrics

| Metric | Range | Good | Excellent | Description |
|--------|-------|------|-----------|-------------|
| RMSD | 0+ Å | <2 Å | <1 Å | Structural similarity |
| TM-score | 0-1 | >0.5 | >0.8 | Fold similarity |
| GDT-TS | 0-100 | >50 | >80 | Global distance test |
| Ramachandran | 0-100% | >90% | >98% | Backbone dihedral angles |
| Clash Score | 0+ | <10 | <5 | Atomic overlaps |
| MolProbity | 0+ | <2 | <1 | Overall quality |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based structure queries
- **WIA-OMNI-API**: Universal bioinformatics API gateway
- **WIA-SOCIAL**: Collaborative research networks
- **WIA-CLOUD**: Distributed computation for predictions

## 📖 Use Cases

1. **Drug Discovery**: Identify binding sites and predict drug-protein interactions
2. **Enzyme Engineering**: Design proteins with enhanced catalytic activity
3. **Disease Mechanisms**: Understand pathogenic mutations and protein misfolding
4. **Protein Engineering**: Create novel proteins for industrial applications
5. **Vaccine Development**: Analyze viral protein structures for epitope prediction
6. **Structural Genomics**: High-throughput structure determination

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
