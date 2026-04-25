# 🔄 WIA-CORE-003: Universal Data Exchange Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-CORE-003
> **Version:** 1.0.0
> **Status:** Active
> **Category:** CORE / Universal Integration Standards
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-CORE-003 standard defines a universal data exchange format and protocol for seamless interoperability between heterogeneous systems, platforms, and technologies. It enables bi-directional data transfer with schema validation, transformation, versioning, and integrity verification.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to eliminate data silos and enable universal connectivity, ensuring that all systems can communicate effectively to benefit humanity through seamless information exchange.

## 🎯 Key Features

- **Universal Format**: JSON-LD based schema with semantic annotations
- **Schema Evolution**: Backward and forward compatibility through versioning
- **Data Transformation**: Built-in mapping and conversion between formats
- **Integrity Verification**: Cryptographic hash chains for tamper detection
- **Protocol Agnostic**: Supports HTTP, WebSocket, gRPC, MQTT, and custom protocols
- **Type Safety**: Strong typing with runtime validation
- **Compression**: Efficient encoding with optional compression (gzip, brotli, zstd)
- **Streaming**: Support for large datasets with chunked transfer

## 📊 Core Concepts

### 1. Universal Data Envelope

```
UDE = {
  meta: {id, version, timestamp, schema},
  data: {payload},
  integrity: hash(meta + data)
}
```

Where:
- `UDE` = Universal Data Envelope
- `meta` = Metadata describing the payload
- `data` = Actual data payload
- `integrity` = SHA-256 hash for verification

### 2. Schema Compatibility Score

```
SCS = (Cm / Ct) × (1 - |Vt - Vs| / Vmax)
```

Where:
- `SCS` = Schema Compatibility Score (0-1)
- `Cm` = Number of matching fields
- `Ct` = Total fields in target schema
- `Vt` = Target version
- `Vs` = Source version
- `Vmax` = Maximum version difference

### 3. Data Transformation Pipeline

```
T(D) = T₁(T₂(...Tₙ(D)))
```

Where:
- `T(D)` = Transformed data
- `Tₙ` = Individual transformation function
- `D` = Source data

## 🔧 Components

### TypeScript SDK

```typescript
import {
  UniversalDataExchange,
  DataEnvelope,
  DataFormat,
  TransformationPipeline,
  validateSchema
} from '@wia/core-003';

const ude = new UniversalDataExchange();

// Create a data envelope
const envelope: DataEnvelope = ude.createEnvelope({
  schema: 'https://schema.org/Person',
  version: '1.0.0',
  data: {
    name: 'John Doe',
    email: 'john@example.com',
    age: 30
  }
});

// Transform data between formats
const transformed = await ude.transform(
  envelope,
  DataFormat.JSON,
  DataFormat.XML
);

// Verify integrity
const isValid = ude.verifyIntegrity(envelope);
console.log(`Envelope integrity: ${isValid ? 'Valid' : 'Invalid'}`);

// Create transformation pipeline
const pipeline = new TransformationPipeline()
  .addStep('normalize', (data) => normalizeKeys(data))
  .addStep('validate', (data) => validateSchema(data))
  .addStep('enrich', (data) => enrichWithDefaults(data));

const result = await pipeline.execute(envelope);
```

### CLI Tool

```bash
# Create a universal data envelope
wia-core-003 create --schema person --data person.json --output envelope.json

# Validate schema compatibility
wia-core-003 validate --envelope envelope.json --schema v2.0.0

# Transform data format
wia-core-003 transform --input envelope.json --from json --to xml --output data.xml

# Verify data integrity
wia-core-003 verify --envelope envelope.json

# Calculate compatibility score
wia-core-003 compatibility --source v1.0.0 --target v2.0.0
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-CORE-003-v1.0.md](./spec/WIA-CORE-003-v1.0.md) | Complete specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-core-003.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/universal-data-exchange

# Run installation script
./install.sh

# Verify installation
wia-core-003 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/core-003

# Or yarn
yarn add @wia/core-003
```

```typescript
import { UniversalDataExchange, DataEnvelope } from '@wia/core-003';

const ude = new UniversalDataExchange({
  defaultVersion: '1.0.0',
  validateOnCreate: true,
  strictMode: false
});

// Simple data exchange
const envelope = ude.createEnvelope({
  schema: 'https://schema.org/Product',
  version: '1.0.0',
  data: {
    name: 'Laptop',
    price: 999.99,
    currency: 'USD',
    inStock: true
  }
});

// Send via HTTP
const response = await fetch('https://api.example.com/exchange', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'X-WIA-Schema': envelope.meta.schema,
    'X-WIA-Version': envelope.meta.version
  },
  body: JSON.stringify(envelope)
});

// Receive and validate
const received: DataEnvelope = await response.json();
const isValid = ude.verifyIntegrity(received);

if (isValid) {
  console.log('Data received successfully:', received.data);
} else {
  console.error('Integrity verification failed');
}
```

## 🌐 Supported Formats

| Format | Extension | MIME Type | Encoding |
|--------|-----------|-----------|----------|
| JSON | .json | application/json | UTF-8 |
| JSON-LD | .jsonld | application/ld+json | UTF-8 |
| XML | .xml | application/xml | UTF-8 |
| YAML | .yaml | application/x-yaml | UTF-8 |
| Protocol Buffers | .pb | application/protobuf | Binary |
| MessagePack | .msgpack | application/msgpack | Binary |
| CBOR | .cbor | application/cbor | Binary |
| Avro | .avro | application/avro | Binary |

## 🔐 Security Features

| Feature | Description | Implementation |
|---------|-------------|----------------|
| Integrity Hash | SHA-256 checksum | Automatic on envelope creation |
| Signature | Digital signature support | Optional with private key |
| Encryption | AES-256-GCM | Optional end-to-end encryption |
| Compression | gzip/brotli/zstd | Transparent compression layer |
| Rate Limiting | Token bucket algorithm | Configurable limits |
| Schema Validation | JSON Schema/XSD | Runtime validation |

## 📈 Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Throughput | 10,000+ msg/sec | Single thread, JSON format |
| Latency | < 1ms | Local transformation |
| Memory Overhead | ~5% | Compared to raw data |
| Compression Ratio | 60-80% | Typical with gzip |
| Max Envelope Size | 100MB | Configurable, streaming for larger |
| Schema Cache | 1000 schemas | LRU cache |

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based data queries and transformations
- **WIA-OMNI-API**: Universal API gateway for all protocols
- **WIA-SOCIAL**: Decentralized data sharing networks
- **WIA-AIR-POWER**: Distributed computing for large transformations
- **WIA-AIR-SHIELD**: Security and encryption layer

## 📖 Use Cases

1. **IoT Data Exchange**: Standardized sensor data from heterogeneous devices
2. **Healthcare Interoperability**: Patient data exchange between EHR systems
3. **Financial Transactions**: Cross-platform payment and settlement data
4. **Supply Chain**: Product tracking across multiple vendors and systems
5. **API Integration**: Universal adapter between REST, GraphQL, SOAP, gRPC
6. **Data Migration**: Safe transfer between legacy and modern systems
7. **Microservices**: Inter-service communication with schema evolution

## 🔄 Schema Evolution Example

```typescript
// Version 1.0.0
interface PersonV1 {
  name: string;
  email: string;
}

// Version 2.0.0 (backward compatible)
interface PersonV2 {
  name: string;
  email: string;
  phone?: string; // Optional field added
  age?: number;   // Optional field added
}

// Automatic upgrade
const v1Data = { name: 'John', email: 'john@example.com' };
const v2Data = ude.upgradeSchema(v1Data, '1.0.0', '2.0.0');
// Result: { name: 'John', email: 'john@example.com', phone: null, age: null }

// Calculate compatibility
const score = ude.calculateCompatibility('1.0.0', '2.0.0');
console.log(`Compatibility: ${(score * 100).toFixed(1)}%`); // 100%
```

## 🛠️ Advanced Features

### Custom Transformations

```typescript
// Register custom transformation
ude.registerTransformation('uppercase-names', (data) => {
  return {
    ...data,
    name: data.name?.toUpperCase()
  };
});

// Use in pipeline
const pipeline = new TransformationPipeline()
  .addStep('uppercase-names')
  .addStep('validate');

const result = await pipeline.execute(envelope);
```

### Streaming Support

```typescript
// Stream large dataset
const stream = ude.createStream({
  schema: 'https://schema.org/Dataset',
  version: '1.0.0',
  chunkSize: 1024 * 1024 // 1MB chunks
});

for await (const chunk of stream) {
  // Process chunk
  await processChunk(chunk);
}
```

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
