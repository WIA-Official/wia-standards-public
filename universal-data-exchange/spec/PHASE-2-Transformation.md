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

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of universal-data-exchange so that conformance claims at any
Phase remain unambiguous.*

