# WIA-CORE-007 PHASE 3: Encoding and Serialization Specification

**Version:** 1.0
**Status:** APPROVED
**Last Updated:** 2025-12-27
**Authors:** WIA Standards Committee

## 1. Introduction

PHASE 3 defines the encoding and serialization specifications for WIA-CORE-007 Universal Protocol. This document establishes requirements for data encoding, serialization formats, and interoperability between different encoding mechanisms.

## 2. Encoder Interface

### 2.1 Base Encoder Interface

All encoder implementations MUST implement:

```typescript
interface Encoder {
  // Encoding operations
  encode(data: any): Buffer | Uint8Array | string;
  decode(encoded: Buffer | Uint8Array | string): any;

  // Metadata
  getName(): string;
  getMimeType(): string;
  getCapabilities(): EncoderCapabilities;
}

interface EncoderCapabilities {
  supportsBinary: boolean;
  supportsStreaming: boolean;
  supportsSchema: boolean;
  compressionBuiltIn: boolean;
  typePreservation: TypePreservation[];
}

enum TypePreservation {
  NULL = 'null',
  BOOLEAN = 'boolean',
  INTEGER = 'integer',
  FLOAT = 'float',
  STRING = 'string',
  BINARY = 'binary',
  ARRAY = 'array',
  OBJECT = 'object',
  DATE = 'date',
  BIGINT = 'bigint',
  UNDEFINED = 'undefined'
}
```

## 3. JSON Encoding

### 3.1 Standard JSON

**MIME Type:** `application/json`

**Features:**
- Human-readable
- Universal support
- No schema required
- Text-based

**Limitations:**
- Larger message size
- Limited type support
- No native binary support

**Configuration:**
```typescript
interface JSONEncoderOptions {
  pretty?: boolean;          // Pretty-print output
  dateFormat?: 'iso8601' | 'timestamp';
  bigintSupport?: boolean;   // Support BigInt
  undefinedHandling?: 'null' | 'omit' | 'error';
  replacer?: (key: string, value: any) => any;
  reviver?: (key: string, value: any) => any;
}
```

**Example:**
```typescript
const encoder = new JSONEncoder({
  pretty: false,
  dateFormat: 'iso8601',
  bigintSupport: true
});

const encoded = encoder.encode({
  id: 123,
  name: "John Doe",
  timestamp: new Date(),
  bigNumber: BigInt('9007199254740991')
});

// Output:
// {"id":123,"name":"John Doe","timestamp":"2025-12-27T10:00:00.000Z","bigNumber":"9007199254740991"}
```

### 3.2 JSON Extensions

**Support for additional types:**
```typescript
// Date handling
{
  "timestamp": {
    "$date": "2025-12-27T10:00:00.000Z"
  }
}

// BigInt handling
{
  "largeNumber": {
    "$bigint": "9007199254740991"
  }
}

// Binary handling (base64)
{
  "data": {
    "$binary": "SGVsbG8gV29ybGQ="
  }
}

// Undefined handling
{
  "optional": {
    "$undefined": true
  }
}
```

## 4. Protocol Buffers

### 4.1 Protobuf v3

**MIME Type:** `application/x-protobuf`

**Features:**
- Compact binary format
- Strong typing
- Schema required
- Fast serialization
- Backward/forward compatible

**Schema Definition:**
```protobuf
syntax = "proto3";

package wia.universal_protocol;

message UniversalMessage {
  string id = 1;
  MessageType type = 2;
  string method = 3;
  bytes payload = 4;
  map<string, string> metadata = 5;
  Error error = 6;
  uint64 sequence = 7;
}

enum MessageType {
  REQUEST = 0;
  RESPONSE = 1;
  STREAM = 2;
  EVENT = 3;
}

message Error {
  string code = 1;
  string message = 2;
  bytes details = 3;
  bool retryable = 4;
}
```

**Usage:**
```typescript
import { UniversalMessage } from './generated/universal_pb';

const encoder = new ProtobufEncoder({
  schemas: {
    'users.get': UserGetRequest,
    'users.create': UserCreateRequest
  }
});

const message = new UniversalMessage();
message.setId('msg-123');
message.setType(MessageType.REQUEST);
message.setMethod('users.get');

const encoded = encoder.encode(message);
const decoded = encoder.decode(encoded);
```

### 4.2 Schema Evolution

**Compatible changes:**
- Adding new fields
- Adding new enum values
- Changing field names (with field number preservation)

**Incompatible changes:**
- Changing field types
- Removing fields
- Changing field numbers

**Best practices:**
```protobuf
// Reserve removed field numbers
message User {
  reserved 2, 15, 9 to 11;
  reserved "old_field", "deprecated_field";

  string id = 1;
  string name = 3;
  // Field 2 was email (removed)
}
```

## 5. MessagePack

### 5.1 MessagePack Encoding

**MIME Type:** `application/msgpack`

**Features:**
- Binary format (smaller than JSON)
- No schema required
- Fast serialization
- Native binary support

**Type Mapping:**
```typescript
MessagePack Type → JavaScript Type
-----------------------------------
nil          → null
boolean      → boolean
integer      → number
float        → number
string       → string
binary       → Buffer/Uint8Array
array        → Array
map          → Object
ext          → Custom extensions
```

**Configuration:**
```typescript
interface MessagePackEncoderOptions {
  forceFloat64?: boolean;
  sortKeys?: boolean;
  extensionCodec?: ExtensionCodec;
  ignoreUndefined?: boolean;
  maxDepth?: number;
}
```

**Custom Extensions:**
```typescript
const extensionCodec = new ExtensionCodec();

// Register Date extension
extensionCodec.register({
  type: 0,
  encode: (object: unknown): Uint8Array => {
    if (object instanceof Date) {
      return encodeDate(object);
    }
    return null;
  },
  decode: (data: Uint8Array): Date => {
    return decodeDate(data);
  }
});

const encoder = new MessagePackEncoder({
  extensionCodec
});
```

## 6. CBOR

### 6.1 CBOR Encoding

**MIME Type:** `application/cbor`

**Features:**
- RFC 7049 standard
- Self-describing binary format
- Tagged data types
- Good for IoT

**Tagged Types:**
```typescript
// Date/Time (tag 0)
{
  tag: 0,
  value: "2025-12-27T10:00:00Z"
}

// Epoch timestamp (tag 1)
{
  tag: 1,
  value: 1735344000
}

// BigInt (tag 2/3)
{
  tag: 2,
  value: Buffer.from([0x01, 0xff, ...])
}

// UUID (tag 37)
{
  tag: 37,
  value: Buffer.from('550e8400e29b41d4a716446655440000', 'hex')
}
```

**Configuration:**
```typescript
interface CBOREncoderOptions {
  useTimestamps?: boolean;
  useRecords?: boolean;
  useBigIntExtension?: boolean;
  maxDepth?: number;
}
```

## 7. Apache Avro

### 7.1 Avro Encoding

**MIME Type:** `application/avro`

**Features:**
- Rich schema support
- Excellent schema evolution
- Compact encoding
- Self-describing data

**Schema Definition:**
```json
{
  "type": "record",
  "name": "UniversalMessage",
  "namespace": "wia.universal_protocol",
  "fields": [
    {
      "name": "id",
      "type": "string"
    },
    {
      "name": "type",
      "type": {
        "type": "enum",
        "name": "MessageType",
        "symbols": ["REQUEST", "RESPONSE", "STREAM", "EVENT"]
      }
    },
    {
      "name": "method",
      "type": ["null", "string"],
      "default": null
    },
    {
      "name": "payload",
      "type": ["null", "bytes"],
      "default": null
    },
    {
      "name": "metadata",
      "type": {
        "type": "map",
        "values": "string"
      }
    }
  ]
}
```

**Schema Evolution:**
```typescript
// Reader's schema (newer)
const readerSchema = {
  type: "record",
  name: "User",
  fields: [
    { name: "id", type: "string" },
    { name: "name", type: "string" },
    { name: "email", type: ["null", "string"], default: null }, // New field
    { name: "premium", type: "boolean", default: false }        // New field
  ]
};

// Writer's schema (older)
const writerSchema = {
  type: "record",
  name: "User",
  fields: [
    { name: "id", type: "string" },
    { name: "name", type: "string" }
  ]
};

// Avro handles evolution automatically
const data = avro.decode(writerSchema, readerSchema, buffer);
// Result: { id: "123", name: "John", email: null, premium: false }
```

## 8. XML Encoding

### 8.1 XML Support

**MIME Type:** `application/xml` or `text/xml`

**Use Cases:**
- Legacy system integration
- SOAP compatibility
- Document-centric data

**Example:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<UniversalMessage xmlns="https://wia.org/universal-protocol/v1">
  <id>550e8400-e29b-41d4-a716-446655440000</id>
  <type>request</type>
  <method>users.get</method>
  <payload>
    <userId>12345</userId>
    <fields>
      <field>name</field>
      <field>email</field>
    </fields>
  </payload>
  <metadata>
    <entry key="timestamp">2025-12-27T10:00:00Z</entry>
    <entry key="version">1.0</entry>
  </metadata>
</UniversalMessage>
```

**Configuration:**
```typescript
interface XMLEncoderOptions {
  declaration?: boolean;
  formatting?: 'compact' | 'pretty';
  rootElement?: string;
  arrayElement?: string;
  namespaces?: Record<string, string>;
}
```

## 9. Encoding Selection

### 9.1 Comparison Matrix

| Encoding     | Size | Speed | Schema | Human-Readable | Binary Support |
|--------------|------|-------|--------|----------------|----------------|
| JSON         | 100% | 100%  | No     | Yes            | Base64         |
| MessagePack  | 50%  | 150%  | No     | No             | Native         |
| Protobuf     | 30%  | 200%  | Yes    | No             | Native         |
| CBOR         | 45%  | 140%  | No     | No             | Native         |
| Avro         | 35%  | 180%  | Yes    | No             | Native         |
| XML          | 150% | 60%   | Yes    | Yes            | Base64         |

*(Percentages relative to JSON)*

### 9.2 Selection Guide

```
Choose JSON if:
  - Debugging is important
  - Browser compatibility required
  - Schema flexibility needed

Choose MessagePack if:
  - Smaller payload desired
  - No schema available
  - Good performance needed

Choose Protobuf if:
  - Best performance required
  - Strong typing needed
  - Schema evolution important

Choose CBOR if:
  - IoT/embedded systems
  - Self-describing format desired
  - Standards compliance needed

Choose Avro if:
  - Complex schema evolution
  - Data warehouse integration
  - Kafka/Hadoop ecosystem

Choose XML if:
  - Legacy system integration
  - SOAP/WS-* compatibility
  - Document-centric data
```

## 10. Compression

### 10.1 Compression Algorithms

**Supported algorithms:**
```typescript
enum CompressionAlgorithm {
  NONE = 'none',
  GZIP = 'gzip',
  DEFLATE = 'deflate',
  BROTLI = 'brotli',
  LZ4 = 'lz4',
  ZSTD = 'zstd'
}
```

**Configuration:**
```typescript
interface CompressionOptions {
  algorithm: CompressionAlgorithm;
  level?: number;              // 1-9 (algorithm-dependent)
  threshold?: number;          // Minimum size to compress
  dictionary?: Buffer;         // Compression dictionary
}
```

### 10.2 Compression Strategy

```typescript
class CompressionEncoder implements Encoder {
  async encode(data: any): Promise<Buffer> {
    const raw = this.baseEncoder.encode(data);

    // Skip compression if below threshold
    if (raw.length < this.options.threshold) {
      return raw;
    }

    // Compress
    const compressed = await compress(
      raw,
      this.options.algorithm,
      this.options.level
    );

    // Only use if compression ratio > 20%
    if (compressed.length < raw.length * 0.8) {
      return this.addCompressionHeader(compressed);
    }

    return raw;
  }
}
```

## 11. Polyglot Encoding

### 11.1 Multiple Encoding Support

```typescript
interface PolyglotEncoder {
  encoders: Map<string, Encoder>;
  defaultEncoding: string;

  encode(data: any, encoding?: string): Buffer;
  decode(buffer: Buffer, encoding?: string): any;
  negotiate(acceptedEncodings: string[]): string;
}
```

**Content Negotiation:**
```typescript
// Client request
metadata: {
  'accept-encoding': ['protobuf', 'messagepack', 'json']
}

// Server response
metadata: {
  'content-encoding': 'protobuf'
}
```

### 11.2 Automatic Encoding Detection

```typescript
function detectEncoding(buffer: Buffer): string {
  // Magic bytes detection
  if (buffer[0] === 0x08) return 'protobuf';
  if (buffer[0] === 0xD9) return 'messagepack';
  if (buffer[0] === 0x7B) return 'json';  // {
  if (buffer[0] === 0x5B) return 'json';  // [
  if (buffer[0] === 0x3C) return 'xml';   // <

  throw new Error('Unknown encoding');
}
```

## 12. Performance Optimization

### 12.1 Pooling

```typescript
class EncoderPool {
  private pool: Encoder[] = [];

  acquire(): Encoder {
    return this.pool.pop() || this.createEncoder();
  }

  release(encoder: Encoder): void {
    encoder.reset();
    this.pool.push(encoder);
  }
}
```

### 12.2 Streaming

```typescript
interface StreamingEncoder {
  createEncoder(): WritableStream;
  createDecoder(): ReadableStream;
}

// Usage
const encoder = streamingEncoder.createEncoder();
for (const chunk of largeData) {
  await encoder.write(chunk);
}
await encoder.end();
```

## 13. Security Considerations

### 13.1 Size Limits

Implementations MUST enforce:
- Maximum message size (default: 10MB)
- Maximum nesting depth (default: 32)
- Maximum string length (default: 1MB)
- Maximum array length (default: 10000)

### 13.2 Validation

```typescript
function validateMessage(message: any): void {
  if (getSize(message) > MAX_MESSAGE_SIZE) {
    throw new Error('Message too large');
  }

  if (getDepth(message) > MAX_DEPTH) {
    throw new Error('Message too deeply nested');
  }

  sanitizeMessage(message);
}
```

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
