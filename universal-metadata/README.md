# 📋 WIA-CORE-008: Universal Metadata Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-CORE-008
> **Version:** 1.0.0
> **Status:** Active
> **Category:** CORE (Universal Integration Standards)
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-CORE-008 standard defines a comprehensive framework for universal metadata that can describe, enrich, and contextualize any type of data. This core standard enables consistent metadata management across all domains, systems, and data types.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to make all data universally understandable, discoverable, and interoperable, benefiting humanity through enhanced knowledge sharing and data accessibility.

## 🎯 Key Features

- **Universal Schema**: Flexible metadata structure for any data type
- **Multi-domain Support**: Healthcare, finance, science, IoT, media, and more
- **Semantic Relationships**: Rich linking between metadata elements
- **Versioning & Lineage**: Track metadata evolution and data provenance
- **Quality Metrics**: Automated quality scoring and validation
- **Privacy Controls**: Fine-grained access control and data classification
- **Extensibility**: Custom metadata fields for domain-specific needs

## 📊 Core Concepts

### 1. Metadata Structure

```
Metadata = Core Fields + Domain Fields + Custom Fields + Relationships
```

Where:
- `Core Fields` = Universal fields (id, title, creator, timestamp, etc.)
- `Domain Fields` = Domain-specific metadata (medical, financial, etc.)
- `Custom Fields` = User-defined extensions
- `Relationships` = Links to other data/metadata

### 2. Quality Score

```
Q = (Completeness × 0.4) + (Accuracy × 0.3) + (Consistency × 0.2) + (Timeliness × 0.1)
```

Where:
- `Q` = Overall quality score (0-100)
- `Completeness` = Percentage of required fields populated
- `Accuracy` = Validation pass rate
- `Consistency` = Cross-field coherence score
- `Timeliness` = Freshness relative to data currency

### 3. Discoverability Score

```
D = log₂(1 + Keywords) + log₂(1 + Tags) + (HasDescription ? 20 : 0)
```

Where:
- `D` = Discoverability score
- Target: D ≥ 25 for optimal searchability

## 🔧 Components

### TypeScript SDK

```typescript
import {
  createMetadata,
  validateMetadata,
  enrichMetadata,
  searchMetadata
} from '@wia/core-008';

// Create universal metadata
const metadata = createMetadata({
  title: 'Research Dataset: Gene Expression Analysis',
  description: 'RNA-seq data from cancer cell lines',
  creator: {
    name: 'Dr. Jane Smith',
    id: 'orcid:0000-0002-1825-0097'
  },
  domain: 'biomedical-research',
  dataType: 'dataset',
  format: 'CSV',
  size: 1024000000, // 1GB
  keywords: ['genomics', 'RNA-seq', 'cancer', 'transcriptomics'],
  license: 'CC-BY-4.0',
  privacy: 'public'
});

// Validate and enrich
const validation = validateMetadata(metadata);
const enriched = enrichMetadata(metadata, { autoClassify: true });

console.log(`Quality Score: ${validation.qualityScore}/100`);
console.log(`Discoverability: ${enriched.discoverabilityScore}`);
```

### CLI Tool

```bash
# Create metadata for a file
wia-core-008 create --file data.csv --title "Sales Data" --domain finance

# Validate existing metadata
wia-core-008 validate --metadata metadata.json

# Enrich metadata with auto-classification
wia-core-008 enrich --metadata metadata.json --auto-classify

# Search metadata
wia-core-008 search --query "genomics cancer" --domain biomedical

# Generate metadata template
wia-core-008 template --domain healthcare --format json

# Check quality score
wia-core-008 quality --metadata metadata.json
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-CORE-008-v1.0.md](./spec/WIA-CORE-008-v1.0.md) | Complete specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-core-008.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/universal-metadata

# Run installation script
./install.sh

# Verify installation
wia-core-008 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/core-008

# Or yarn
yarn add @wia/core-008
```

```typescript
import { UniversalMetadataSDK } from '@wia/core-008';

const sdk = new UniversalMetadataSDK();

// Create metadata for an image
const imageMetadata = sdk.createMetadata({
  title: 'Mars Rover Photo',
  description: 'High-resolution image from Perseverance rover',
  creator: { name: 'NASA JPL', id: 'nasa-jpl' },
  domain: 'space-science',
  dataType: 'image',
  format: 'JPEG',
  keywords: ['mars', 'rover', 'perseverance', 'space'],
  spatial: {
    location: 'Jezero Crater, Mars',
    coordinates: { lat: 18.85, lon: 77.52 }
  },
  temporal: {
    created: '2025-01-15T10:30:00Z',
    coverage: { start: '2025-01-15', end: '2025-01-15' }
  }
});

const result = sdk.validateMetadata(imageMetadata);
console.log(`Metadata quality: ${result.qualityScore}/100`);
```

## 📋 Metadata Domains

| Domain | Code | Fields | Use Cases |
|--------|------|--------|-----------|
| Healthcare | `healthcare` | Patient, diagnosis, treatment | Medical records, clinical trials |
| Finance | `finance` | Transaction, account, amount | Financial data, trading |
| Science | `science` | Experiment, method, results | Research datasets, publications |
| IoT | `iot` | Device, sensor, readings | Sensor data, telemetry |
| Media | `media` | Format, codec, duration | Images, videos, audio |
| Government | `government` | Classification, agency | Public records, regulations |
| Education | `education` | Course, institution, level | Learning materials, credentials |
| Geospatial | `geospatial` | Coordinates, projection | Maps, GIS data |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language metadata queries
- **WIA-OMNI-API**: Universal data access with metadata
- **WIA-DATA**: Data management and cataloging
- **WIA-SEARCH**: Metadata-driven search

## 📖 Use Cases

1. **Data Catalogs**: Build comprehensive data discovery platforms
2. **Research Data Management**: Track scientific datasets with provenance
3. **Digital Libraries**: Organize and discover documents/media
4. **IoT Data Streams**: Contextualize sensor data with rich metadata
5. **Healthcare Records**: Standardize medical data descriptions
6. **Enterprise Data Governance**: Ensure data quality and compliance
7. **Open Data Portals**: Enable public data discovery
8. **AI Training Data**: Document ML datasets comprehensively

## ✨ Features

### Core Metadata Fields

- **Identification**: id, title, alternateTitle, identifier
- **Description**: description, abstract, keywords, tags
- **Provenance**: creator, contributor, publisher, source
- **Temporal**: created, modified, published, coverage
- **Rights**: license, copyright, accessRights, privacy
- **Technical**: format, size, version, checksum
- **Quality**: qualityScore, validationStatus, completeness

### Advanced Features

- **Semantic Links**: Relations between datasets (derivedFrom, partOf, references)
- **Versioning**: Full version history and change tracking
- **Multilingual**: Support for multiple languages
- **Custom Extensions**: Domain-specific metadata schemas
- **Auto-enrichment**: ML-based classification and tagging
- **Quality Validation**: Automated quality checks

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
