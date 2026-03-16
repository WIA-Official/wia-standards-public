# WIA-CORE-003: Universal Data Exchange
## PHASE 2 - Transformation

**Version:** 1.0
**Status:** Stable
**Category:** CORE
**Color:** Gray (#6B7280)

---

## Overview

PHASE 2 defines format conversion engines, schema mapping algorithms, and bidirectional transformers that enable lossless conversion between 99+ data formats while preserving semantic integrity.

## Transformation Engine Architecture

### Core Components

```typescript
interface TransformationEngine {
    // Single transformation
    transform(
        sourceData: Bytes,
        sourceFormat: FormatId,
        targetFormat: FormatId,
        options?: TransformOptions
    ): Result<Bytes>;

    // With schema mapping
    transformWithMapping(
        sourceData: Bytes,
        sourceSchema: SchemaId,
        targetSchema: SchemaId,
        mapping: MappingId,
        options?: TransformOptions
    ): Result<Bytes>;

    // Batch transformation
    transformBatch(requests: TransformRequest[]): TransformResult[];

    // Streaming transformation
    transformStream(
        sourceStream: Stream<Bytes>,
        sourceFormat: FormatId,
        targetFormat: FormatId
    ): Stream<Result<Bytes>>;
}
```

### Transformation Strategies

**1. Direct Codec-to-Codec**
- For lossless conversions between similar formats (JSON ↔ YAML)
- Bypasses universal representation for performance
- Automatically selected when safe

**2. Universal Representation Bridge**
- Source → Universal → Target
- Ensures semantic preservation
- Enables complex schema mappings

**3. Multi-Hop Transformation**
- Chain of transformations for complex conversions
- Example: CSV → JSON → Protobuf
- Optimization: minimize intermediate steps

## Format Support Matrix

### Text Formats
- JSON, JSON5, JSONC, HJSON
- XML, SOAP
- YAML, TOML
- CSV, TSV
- INI, Properties
- Markdown, HTML, LaTeX

### Binary Formats
- Protocol Buffers
- Apache Avro
- Apache Parquet
- MessagePack
- CBOR, BSON
- Apache Thrift

### Database Formats
- SQL (INSERT statements)
- MongoDB documents
- Redis commands
- Cassandra CQL
- DynamoDB JSON

### Enterprise Formats
- EDI (X12, EDIFACT)
- HL7 v2, HL7 FHIR
- XBRL
- SAP IDoc
- FIX Protocol

## Mapping Definition Language

### MDL (Mapping Definition Language)

```mdl
mapping OrderV1_to_OrderV2 {
    source: schema OrderV1@1.0
    target: schema OrderV2@2.0

    // Simple field mapping
    map order_id -> orderId

    // Type conversion
    map customer_id -> customerId {
        transform: parse_int
    }

    // Restructuring
    map {first_name, last_name} -> customer.name {
        transform: (first, last) => {
            full_name: concat(first, " ", last),
            given_name: first,
            family_name: last
        }
    }

    // Conditional mapping
    map status -> orderStatus {
        transform: match value {
            "pending" | "new" => "PENDING",
            "shipped" => "IN_TRANSIT",
            "delivered" => "COMPLETED",
            _ => "UNKNOWN"
        }
    }

    // Array transformation
    map items -> lineItems {
        transform: items.map(item => {
            productId: item.product_id,
            quantity: item.qty,
            price: item.unit_price
        })
    }

    // Computed fields
    derive total_with_tax {
        compute: source.subtotal * (1 + source.tax_rate)
    }

    // Validation
    validate {
        require: target.customerId > 0
        require: target.lineItems.length > 0
    }
}
```

### Built-in Transformation Functions

**String Operations:**
- `uppercase()`, `lowercase()`, `trim()`
- `concat(...strings)`, `split(delimiter)`
- `substring(start, end)`, `replace(pattern, replacement)`
- `hash(algorithm)`, `base64_encode()`, `base64_decode()`

**Numeric Operations:**
- `round(precision)`, `floor()`, `ceil()`
- `abs()`, `min(a, b)`, `max(a, b)`
- `to_int()`, `to_float()`, `to_decimal(precision, scale)`

**Date/Time Operations:**
- `parse_date(format)`, `format_date(format)`
- `add_duration(duration)`, `diff(other_date)`
- `to_timezone(timezone)`, `to_epoch()`

**Collection Operations:**
- `map(fn)`, `filter(fn)`, `reduce(fn, init)`
- `flatten()`, `distinct()`, `sort(comparator)`
- `first()`, `last()`, `take(n)`, `skip(n)`

**Type Conversions:**
- `to_string()`, `to_json()`, `to_xml()`
- `parse_int(base)`, `parse_float()`
- `to_bytes(encoding)`, `from_bytes(encoding)`

## Schema Mapping Patterns

### Field Renaming

```mdl
map user_id -> userId
map email_address -> email
map phone_num -> phoneNumber
```

### Type Coercion

```mdl
map age -> age {
    transform: to_int()  // String "25" → Integer 25
}

map price -> amount {
    transform: to_decimal(10, 2)  // Float → Decimal
}
```

### Structural Flattening

```mdl
// Nested → Flat
map address.street -> street
map address.city -> city
map address.postal_code -> zip
```

### Structural Nesting

```mdl
// Flat → Nested
map {street, city, state, zip} -> address {
    street_line: street,
    city: city,
    state: state,
    postal_code: zip
}
```

### Value Mapping

```mdl
map gender -> sex {
    transform: match value {
        "M" | "Male" | "male" => "MALE",
        "F" | "Female" | "female" => "FEMALE",
        _ => "OTHER"
    }
}
```

## Type System Mapping

### Cross-Format Type Mapping

| Universal Type | JSON | XML | Protobuf | Avro | SQL |
|----------------|------|-----|----------|------|-----|
| Integer | number | text | int32/int64 | int/long | INTEGER |
| Float | number | text | float/double | float/double | FLOAT |
| String | string | text | string | string | VARCHAR |
| Boolean | boolean | text | bool | boolean | BOOLEAN |
| Timestamp | string | text | int64 | long | TIMESTAMP |
| Bytes | base64 | base64 | bytes | bytes | BLOB |
| Array | array | elements | repeated | array | JSON |
| Map | object | elements | map | map | JSON |

### Handling Type Mismatches

**Widening (safe):**
- int32 → int64
- float → double
- optional → required (with default)

**Narrowing (requires validation):**
- int64 → int32 (check range)
- double → float (check precision)
- required → optional

**Incompatible (requires transformation):**
- String ↔ Integer (parse/format)
- Timestamp ↔ String (parse/format)
- Array ↔ Map (structural change)

## Performance Optimization

### Zero-Copy Transformations

```rust
// Avoid copying data when possible
let json_bytes = source_data;
let json_value = serde_json::from_slice(&json_bytes)?; // Parse in-place

// Create target without copying string data
let target = transform_zero_copy(json_value);
```

### Streaming Transformations

```typescript
// Process large datasets in chunks
const transformStream = createTransformStream({
    sourceFormat: 'json-lines',
    targetFormat: 'parquet',
    chunkSize: 10000
});

inputStream
    .pipe(transformStream)
    .pipe(outputStream);
```

### Parallel Processing

```typescript
// Distribute transformation across cores
const results = await transformBatch(records, {
    parallelism: os.cpus().length,
    batchSize: 1000
});
```

## Error Handling

### Error Types

```typescript
type TransformationError =
    | CodecError          // Format encoding/decoding failed
    | MappingError        // Schema mapping failed
    | ValidationError     // Output validation failed
    | TypeMismatchError   // Type conversion failed
    | DataLossError       // Lossy transformation detected
```

### Error Recovery

```typescript
const result = await transform(data, {
    errorStrategy: {
        onCodecError: 'fail',           // Stop immediately
        onMappingError: 'skip-field',   // Omit problematic field
        onValidationError: 'warn',      // Log warning, continue
        onTypeMismatch: 'coerce',       // Attempt type coercion
        onDataLoss: 'prevent'           // Fail rather than lose data
    }
});
```

## Quality Assurance

### Transformation Testing

```typescript
// Round-trip test
const roundTrip = await testRoundTrip({
    sourceFormat: 'json',
    targetFormat: 'protobuf',
    samples: testData,
    expectLossless: true
});

// Equivalence test
const equivalent = await testEquivalence({
    sourceData: originalData,
    transformedData: result,
    ignoreFields: ['metadata.timestamp']
});
```

### Performance Benchmarking

```typescript
const benchmark = await benchmarkTransformation({
    sourceFormat: 'json',
    targetFormat: 'avro',
    dataSize: '1GB',
    metrics: ['throughput', 'latency', 'memory']
});

// Results:
// - Throughput: 150 MB/s
// - P50 latency: 6.7ms
// - P99 latency: 23.4ms
// - Peak memory: 512MB
```

## Advanced Features

### Conditional Transformations

```mdl
map user_type -> accountType {
    transform: if (source.premium) {
        "PREMIUM"
    } else if (source.enterprise) {
        "ENTERPRISE"
    } else {
        "STANDARD"
    }
}
```

### Lookups and Enrichment

```mdl
map product_id -> product_details {
    transform: async_lookup("redis://products/{value}")
}

map ip_address -> geolocation {
    transform: geoip_lookup(value)
}
```

### Custom Transformations

```typescript
registerTransformFunction('custom_hash', (value: string) => {
    return crypto.createHash('sha256').update(value).digest('hex');
});

// Use in mapping
map password -> password_hash {
    transform: custom_hash()
}
```

---

**Previous Phase:** [PHASE 1 - Foundation](PHASE-1-Foundation.md)
**Next Phase:** [PHASE 3 - Integration](PHASE-3-Integration.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
