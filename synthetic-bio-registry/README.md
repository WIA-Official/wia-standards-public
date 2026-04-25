# 📋 WIA-BIO-021: Synthetic Biology Registry

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-021
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO / Biotechnology
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-021 standard defines a comprehensive framework for synthetic biology part registration, sequence sharing, metadata standardization, and version control. It provides interoperability with existing registries like iGEM, Addgene, and SynBioHub.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to accelerate synthetic biology research by creating a unified, accessible registry system that enables global collaboration and part reusability.

## 🎯 Key Features

- **Part Registration**: Standardized registration for BioBrick parts (promoters, RBS, CDS, terminators)
- **Sequence Sharing**: DNA/RNA sequence storage with format interoperability
- **Metadata Standards**: SBOL and GenBank compatible metadata
- **Version Control**: Track part evolution and modifications
- **Characterization Data**: Store and retrieve experimental characterization
- **Access Control**: Flexible licensing and permission management

## 📊 Core Concepts

### 1. BioBrick Standard Parts

Parts are categorized into functional types:

```
Part Types:
- Promoters (BBa_P*)
- Ribosome Binding Sites (BBa_R*)
- Coding Sequences (BBa_C*)
- Terminators (BBa_T*)
- Plasmid Backbones (BBa_V*)
- Composite Parts (BBa_K*)
```

### 2. Part Identification

Standard part naming convention:

```
BBa_[Type][Number]
Example: BBa_K123456 (Composite part from team K)
```

### 3. Metadata Fields

Required metadata for each part:

- **Part ID**: Unique identifier (BBa_XXXXXX)
- **Part Name**: Human-readable name
- **Type**: Functional category
- **Sequence**: DNA/RNA sequence
- **Author**: Creator information
- **Safety**: Biosafety level (BSL-1/2/3/4)
- **Status**: Available/In Progress/Deprecated

### 4. SBOL Compatibility

Parts are stored in Synthetic Biology Open Language (SBOL) format for maximum interoperability.

## 🔧 Components

### TypeScript SDK

```typescript
import {
  registerPart,
  searchParts,
  getPartDetails,
  updatePartCharacterization
} from '@wia/bio-021';

// Register a new part
const part = await registerPart({
  partId: 'BBa_K123456',
  name: 'Strong Constitutive Promoter',
  type: 'promoter',
  sequence: 'TTGACAGCTAGCTCAGTCCTAGGTATAATGCTAGC',
  author: 'Team iGEM 2024',
  description: 'High-strength constitutive promoter for E. coli',
  safetyLevel: 'BSL-1',
  license: 'CC-BY-4.0'
});

// Search for parts
const results = await searchParts({
  type: 'promoter',
  organism: 'E. coli',
  strength: 'high'
});

// Get detailed characterization
const details = await getPartDetails('BBa_K123456');
console.log(details.characterization.strength);
```

### CLI Tool

```bash
# Register a new part
wia-bio-021 register --id BBa_K123456 --type promoter --sequence-file promoter.fasta

# Search the registry
wia-bio-021 search --type promoter --organism "E. coli"

# Get part details
wia-bio-021 get BBa_K123456

# Export to SBOL format
wia-bio-021 export --id BBa_K123456 --format sbol --output part.xml

# Import from GenBank
wia-bio-021 import --file plasmid.gb --format genbank

# Update characterization data
wia-bio-021 characterize --id BBa_K123456 --strength 1500 --growth-rate 0.8
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-021-v1.0.md](./spec/WIA-BIO-021-v1.0.md) | Complete specification with BioBrick standards |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-021.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/synthetic-bio-registry

# Run installation script
./install.sh

# Verify installation
wia-bio-021 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-021

# Or yarn
yarn add @wia/bio-021
```

```typescript
import { SyntheticBioRegistrySDK } from '@wia/bio-021';

const sdk = new SyntheticBioRegistrySDK();

// Register a promoter part
const promoter = await sdk.registerPart({
  partId: 'BBa_J23100',
  name: 'Anderson Promoter Collection - J23100',
  type: 'promoter',
  sequence: 'TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC',
  organism: 'E. coli',
  description: 'Constitutive promoter from Anderson collection',
  safetyLevel: 'BSL-1'
});

console.log(`Part registered: ${promoter.partId}`);
console.log(`Registry URL: ${promoter.url}`);
```

## 🔬 Part Categories

| Category | Prefix | Description | Example |
|----------|--------|-------------|---------|
| Promoter | BBa_P | Transcription initiation | BBa_J23100 |
| RBS | BBa_R | Ribosome binding site | BBa_B0034 |
| CDS | BBa_C | Coding sequence | BBa_E0040 (GFP) |
| Terminator | BBa_T | Transcription termination | BBa_B0015 |
| Plasmid | BBa_V | Plasmid backbone | BBa_pSB1C3 |
| Composite | BBa_K | Multi-part device | BBa_K123456 |

## 🔐 Safety Levels

| Level | Description | Containment |
|-------|-------------|-------------|
| BSL-1 | Minimal risk | Standard lab practices |
| BSL-2 | Moderate risk | Limited access, biosafety cabinet |
| BSL-3 | Serious/lethal | Respiratory protection required |
| BSL-4 | Life-threatening | Maximum containment facility |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based part discovery
- **WIA-OMNI-API**: Universal biological API gateway
- **WIA-SOCIAL**: Collaboration and sharing features
- **WIA-QUANTUM**: Quantum simulation of biological systems

## 📖 Use Cases

1. **iGEM Registry**: Central repository for iGEM competition parts
2. **Research Labs**: Shared part libraries across institutions
3. **Collaborative Design**: Multi-team synthetic biology projects
4. **Part Characterization**: Standardized experimental data collection
5. **Educational Resources**: Teaching materials for synthetic biology
6. **Commercial Biotech**: IP-tracked part development

## 🔄 Version Control

Parts support semantic versioning:

```
v1.0.0 - Initial part submission
v1.1.0 - Improved characterization data
v1.2.0 - Sequence optimization
v2.0.0 - Breaking change (incompatible modification)
```

## 🤝 Registry Interoperability

Compatible with:
- **iGEM Registry**: Import/export iGEM parts
- **Addgene**: Plasmid repository integration
- **SynBioHub**: SBOL-based storage
- **NCBI GenBank**: Sequence format compatibility
- **UniProt**: Protein annotation links

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
