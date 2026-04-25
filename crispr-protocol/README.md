# ✂️ WIA-BIO-014: CRISPR Protocol Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-014
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO (Biotechnology / Life Sciences)
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-014 standard defines a comprehensive framework for CRISPR (Clustered Regularly Interspaced Short Palindromic Repeats) gene editing protocols, including guide RNA design, delivery methods, off-target analysis, and ethical guidelines for genome engineering.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to provide a scientifically rigorous and ethically responsible framework for CRISPR technology that benefits humanity through precise genome editing while ensuring safety and minimizing unintended consequences.

## 🎯 Key Features

- **CRISPR Systems**: Support for Cas9, Cas12a (Cpf1), Cas13, base editors, and prime editors
- **Guide RNA Design**: Algorithms for optimal gRNA design with on-target and off-target scoring
- **Delivery Methods**: RNP complexes, plasmids, viral vectors, and nanoparticles
- **Off-Target Analysis**: Integration with GUIDE-seq, CIRCLE-seq, and computational prediction
- **Editing Validation**: T7E1 assay, Sanger sequencing, and NGS validation methods
- **Safety Protocols**: Comprehensive ethical guidelines and regulatory compliance

## 📊 Core Concepts

### 1. On-Target Score

```
On-Target = (1 - d_mismatch) × GC_content × accessibility
```

Where:
- `d_mismatch` = Distance-weighted mismatch penalty (0-1)
- `GC_content` = GC content factor (optimal: 40-60%)
- `accessibility` = Chromatin accessibility score (0-1)

### 2. Off-Target Score

```
Off-Target = Σ(1 / (1 + mismatches + bulges)) × CFD_score
```

Where:
- `mismatches` = Number of mismatched bases
- `bulges` = Number of DNA/RNA bulges
- `CFD_score` = Cutting Frequency Determination score

### 3. Editing Efficiency

```
Efficiency = (edited_reads / total_reads) × 100%
```

Typical efficiency ranges:
- **HDR (Homology-Directed Repair)**: 1-20%
- **NHEJ (Non-Homologous End Joining)**: 30-90%
- **Base Editing**: 20-60%
- **Prime Editing**: 10-40%

## 🔧 Components

### TypeScript SDK

```typescript
import {
  designGuideRNA,
  predictOffTargets,
  calculateEditingEfficiency,
  validateCRISPRProtocol
} from '@wia/bio-014';

// Design guide RNA for target sequence
const gRNA = designGuideRNA({
  targetSequence: 'ATCGATCGATCGATCGATCGGG', // 23bp (20bp + PAM)
  pamType: 'NGG', // SpCas9 PAM
  organism: 'human',
  chromosome: 'chr7',
  position: 117559590
});

// Predict off-targets
const offTargets = predictOffTargets({
  guideRNA: gRNA.sequence,
  genome: 'hg38',
  maxMismatches: 3,
  includeBulges: true
});

console.log(gRNA.onTargetScore, offTargets.length);
```

### CLI Tool

```bash
# Design guide RNA
wia-bio-014 design-grna --target "ATCGATCGATCGATCGATCGGG" --pam NGG

# Predict off-targets
wia-bio-014 predict-offtargets --grna "ATCGATCGATCGATCGATC" --genome hg38

# Calculate editing efficiency
wia-bio-014 calc-efficiency --edited 450 --total 1000

# Validate protocol
wia-bio-014 validate --protocol protocol.json --check-ethics
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-014-v1.0.md](./spec/WIA-BIO-014-v1.0.md) | Complete specification with detailed protocols |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-014.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/crispr-protocol

# Run installation script
./install.sh

# Verify installation
wia-bio-014 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-014

# Or yarn
yarn add @wia/bio-014
```

```typescript
import { CRISPRProtocolSDK } from '@wia/bio-014';

const sdk = new CRISPRProtocolSDK();

// Design guide RNA
const result = sdk.designGuideRNA({
  targetSequence: 'ATCGATCGATCGATCGATCGGG',
  pamType: 'NGG',
  organism: 'human'
});

console.log(`Guide RNA: ${result.sequence}`);
console.log(`On-target score: ${result.onTargetScore}`);
console.log(`Predicted efficiency: ${result.predictedEfficiency}%`);
```

## 🧬 CRISPR Systems

| System | PAM | Length | Application | Specificity |
|--------|-----|--------|-------------|-------------|
| SpCas9 | NGG | 20 bp | General editing | High |
| SaCas9 | NNGRRT | 21 bp | Compact delivery | Medium |
| Cas12a | TTTV | 23 bp | AT-rich regions | High |
| Cas13 | N/A | 28 bp | RNA editing | Medium |
| Base Editor | NGG | 20 bp | Single base changes | Very High |
| Prime Editor | NGG | 13 bp (PBS) | Precise insertions | Very High |

## ⚠️ Safety Considerations

1. **Off-Target Effects**: Always perform comprehensive off-target analysis
2. **Mosaicism**: Screen for mosaic editing in organisms
3. **Germline Editing**: Follow strict ethical guidelines for germline modifications
4. **Biosafety Level**: Use appropriate containment (BSL-2 minimum)
5. **Regulatory Compliance**: Adhere to local and international regulations
6. **Informed Consent**: Required for all therapeutic applications

## 🌐 WIA Integration

This standard integrates with:
- **WIA-BIOBANK**: Biological sample management
- **WIA-GENOMICS**: Genomic data standards
- **WIA-LAB-AUTOMATION**: Automated protocol execution
- **WIA-INTENT**: Intent-based CRISPR design

## 📖 Use Cases

1. **Gene Knockout**: Disrupt gene function for research
2. **Disease Modeling**: Create cell and animal models of genetic diseases
3. **Therapeutics**: Develop treatments for genetic disorders
4. **Agriculture**: Improve crop traits and disease resistance
5. **Synthetic Biology**: Engineer novel biological systems
6. **Drug Discovery**: Target validation and screening

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
