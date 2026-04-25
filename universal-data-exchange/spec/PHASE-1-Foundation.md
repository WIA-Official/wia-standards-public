# WIA-CORE-003: Universal Data Exchange
## PHASE 1 - Foundation

**Version:** 1.0
**Status:** Stable
**Category:** CORE
**Color:** Gray (#6B7280)

---

## Overview

PHASE 1 establishes the foundational architecture for universal data exchange, defining the core data model abstraction, schema definition languages, and fundamental type systems that enable interoperability across 99+ data formats.

## Core Data Model

### Universal Data Representation

The foundation of WIA-CORE-003 is a format-independent data model that can represent information from any serialization format without loss of fidelity.

```typescript
interface UniversalData {
    // Core data value
    value: Value;

    // Schema reference
    schema: SchemaReference;

    // Metadata
    metadata: Metadata;

    // Semantic annotations
    annotations: Map<string, AnnotationValue>;
}

type Value =
    | PrimitiveValue
    | CompositeValue
    | CustomValue;

interface PrimitiveValue {
    type: 'null' | 'boolean' | 'integer' | 'float' | 'string' | 'bytes' | 'timestamp';
    value: any;
    constraints?: Constraints;
}

interface CompositeValue {
    type: 'array' | 'map' | 'record';
    elements: Value[] | Map<string, Value>;
    schema?: Schema;
}

interface CustomValue {
    type: string;
    representation: Value;
    semantic_type: SemanticType;
}
```

## Type System

### Primitive Types

| Type | Description | Examples |
|------|-------------|----------|
| `Null` | Absence of value | null, nil, None |
| `Boolean` | True/false | true, false |
| `Integer` | Signed integers | -42, 0, 12345 |
| `Float` | IEEE 754 floating point | 3.14, -0.001, 1.23e-10 |
| `String` | UTF-8 text | "hello", "안녕하세요" |
| `Bytes` | Binary data | base64 encoded |
| `Timestamp` | ISO 8601 datetime | "2024-12-27T10:00:00Z" |

### Composite Types

**Array**: Ordered collection of values
```
["apple", "banana", "cherry"]
```

**Map**: Key-value pairs
```
{"name": "Alice", "age": 30}
```

**Record**: Structured data with schema
```
User {
    id: 12345,
    name: "Alice",
    email: "alice@example.com"
}
```

### Semantic Types

Semantic types add domain-specific meaning to primitive types:

- **Email**: String conforming to email format
- **URL**: Valid URI/URL
- **UUID**: RFC 4122 UUID
- **PhoneNumber**: E.164 format
- **CountryCode**: ISO 3166-1 alpha-2
- **CurrencyCode**: ISO 4217
- **PostalCode**: Country-specific postal code
- **IPAddress**: IPv4 or IPv6
- **MACAddress**: Hardware address

## Schema Definition Language

### USDL (Universal Schema Definition Language)

```usdl
// Basic schema definition
schema User {
    // Primitive fields with constraints
    id: Integer(min: 1, max: 9999999999)
    username: String(min_length: 3, max_length: 20, pattern: "^[a-zA-Z0-9_]+$")
    email: Email
    age: Integer(min: 0, max: 150, optional: true)

    // Timestamp with timezone
    created_at: Timestamp(timezone: "UTC")
    updated_at: Timestamp(optional: true)

    // Enumeration
    status: Enum("active", "suspended", "deleted")

    // Nested record
    profile: Record {
        first_name: String
        last_name: String
        bio: String(max_length: 500, optional: true)
    }

    // Array
    tags: Array<String>

    // Map
    preferences: Map<String, String>

    // Annotations
    @unique(field: id)
    @index(fields: [email, username])
    @pii(fields: [email, profile.first_name, profile.last_name])
}
```

### Schema Composition

```usdl
// Import and reuse
import "wia://schemas/common/address.usdl" as Address
import "wia://schemas/common/contact.usdl" as Contact

// Inheritance
schema PremiumUser extends User {
    subscription_tier: Enum("basic", "pro", "enterprise")
    account_manager: String(optional: true)

    // Override parent field
    override status: Enum("active", "trial", "suspended", "deleted")
}

// Composition
schema Order {
    order_id: UUID
    customer: User
    shipping_address: Address.StandardAddress
    billing_address: Address.StandardAddress
    contact_info: Contact.ContactDetails

    items: Array<OrderItem>
    total: Decimal(precision: 10, scale: 2)
    currency: CurrencyCode
}
```

## Metadata Model

Every piece of data carries rich metadata for lineage, quality, and governance:

```typescript
interface Metadata {
    // Version information
    schema_version: SemanticVersion;
    data_version?: string;

    // Provenance
    source_system: string;
    source_format: FormatIdentifier;
    created_at: Timestamp;
    created_by?: string;

    // Quality indicators
    quality_score?: number; // 0.0 to 1.0
    validation_status: 'valid' | 'warning' | 'error';
    validation_messages?: string[];

    // Lineage
    parent_id?: string;
    transformation_history?: TransformationRecord[];

    // Governance
    data_classification?: 'public' | 'internal' | 'confidential' | 'restricted';
    retention_policy?: string;
    compliance_tags?: string[];
}

interface TransformationRecord {
    timestamp: Timestamp;
    source_format: FormatIdentifier;
    target_format: FormatIdentifier;
    transformation_id: string;
    mapping_version?: string;
}
```

## Schema Registry

### Registry Architecture

The schema registry serves as the central authority for all data schemas in the organization.

**Core Functions:**
- Schema storage and versioning
- Compatibility checking
- Schema discovery and search
- Access control and permissions
- Audit logging

**API Operations:**

```typescript
interface SchemaRegistry {
    // Register new schema
    register(schema: Schema, metadata: SchemaMetadata): SchemaId;

    // Retrieve schema
    getSchema(id: SchemaId): Schema;
    getSchemaByName(name: string, version?: SemanticVersion): Schema;

    // Versioning
    listVersions(schemaName: string): SemanticVersion[];
    getLatestVersion(schemaName: string): Schema;

    // Compatibility
    checkCompatibility(
        schemaName: string,
        newSchema: Schema,
        mode: CompatibilityMode
    ): CompatibilityResult;

    // Search
    search(query: SearchQuery): Schema[];

    // Governance
    setCompatibilityMode(schemaName: string, mode: CompatibilityMode): void;
    deprecateSchema(id: SchemaId, reason: string): void;
    deleteSchema(id: SchemaId, force?: boolean): void;
}
```

## Validation Framework

### Schema Validation

```typescript
interface Validator {
    // Validate data against schema
    validate(data: UniversalData, schema: Schema): ValidationResult;

    // Streaming validation
    validateStream(
        dataStream: Stream<UniversalData>,
        schema: Schema
    ): Stream<ValidationResult>;

    // Custom validation rules
    addCustomRule(rule: ValidationRule): void;
}

interface ValidationResult {
    valid: boolean;
    errors: ValidationError[];
    warnings: ValidationWarning[];
    quality_score: number;
}

interface ValidationError {
    path: string;
    message: string;
    constraint: string;
    actual_value: any;
    expected: any;
}
```

### Constraint Types

- **Type constraints**: Ensure values match declared types
- **Range constraints**: min/max for numbers, min_length/max_length for strings
- **Pattern constraints**: Regular expression matching
- **Enumeration constraints**: Value must be in allowed set
- **Uniqueness constraints**: No duplicate values
- **Reference constraints**: Foreign key validation
- **Custom constraints**: User-defined business rules

## Data Serialization

### Codec Interface

```typescript
interface Codec<F extends Format> {
    // Deserialize from format to universal representation
    decode(input: Bytes, schema: Schema): Result<UniversalData>;

    // Serialize from universal representation to format
    encode(data: UniversalData, schema: Schema): Result<Bytes>;

    // Validate format-specific rules
    validate(input: Bytes, schema: Schema): ValidationResult;

    // Infer schema from samples (optional)
    inferSchema(samples: Bytes[]): Option<Schema>;

    // Codec capabilities
    capabilities(): CodecCapabilities;
}

interface CodecCapabilities {
    format: FormatIdentifier;
    supports_streaming: boolean;
    supports_random_access: boolean;
    supports_schema_evolution: boolean;
    max_nesting_depth: number;
    supported_types: Set<TypeIdentifier>;
}
```

## Error Handling

### Error Types

```typescript
type DataExchangeError =
    | SchemaError
    | ValidationError
    | TransformationError
    | SerializationError
    | NetworkError
    | SystemError;

interface SchemaError {
    type: 'schema_not_found' | 'schema_invalid' | 'version_mismatch';
    schema_id?: SchemaId;
    message: string;
    details?: any;
}

interface TransformationError {
    type: 'mapping_failed' | 'type_mismatch' | 'data_loss';
    source_path?: string;
    target_path?: string;
    message: string;
    recoverable: boolean;
}
```

### Error Recovery Strategies

1. **Fail-fast**: Stop on first error (default for critical data)
2. **Skip-invalid**: Log error and continue with next record
3. **Best-effort**: Partial transformation with missing fields marked
4. **Fallback**: Use default values for missing/invalid fields
5. **Dead-letter-queue**: Route failed transformations for manual review

## Performance Considerations

### Memory Management

- **Streaming processing**: Process data in chunks, not all-at-once
- **Zero-copy**: Reference data in-place when possible
- **Lazy evaluation**: Defer computation until needed
- **Pool allocation**: Reuse buffers and objects

### Optimization Strategies

- **Schema caching**: Cache frequently used schemas
- **Validation caching**: Skip re-validation of known-good data
- **Codec selection**: Choose most efficient codec for use case
- **Parallel processing**: Distribute work across CPU cores

## Security Model

### Data Protection

- **Encryption at rest**: AES-256-GCM for stored data
- **Encryption in transit**: TLS 1.3 for all network communication
- **Field-level encryption**: Selective encryption of sensitive fields
- **Homomorphic operations**: Compute on encrypted data

### Access Control

```typescript
interface AccessControl {
    // Permission checks
    canRead(principal: Principal, schema: SchemaId): boolean;
    canWrite(principal: Principal, schema: SchemaId): boolean;
    canModifySchema(principal: Principal, schema: SchemaId): boolean;

    // Row-level security
    filterData(principal: Principal, data: UniversalData[]): UniversalData[];

    // Field-level security
    maskFields(principal: Principal, data: UniversalData): UniversalData;
}
```

### Audit Logging

All operations are logged for compliance and forensics:

```typescript
interface AuditLog {
    timestamp: Timestamp;
    principal: Principal;
    operation: Operation;
    resource: ResourceIdentifier;
    result: 'success' | 'failure';
    metadata: Map<string, any>;
}
```

## Standards Compliance

WIA-CORE-003 PHASE 1 complies with:

- **ISO/IEC 11179**: Metadata registries
- **ISO 8601**: Date and time formats
- **RFC 3986**: URI syntax
- **RFC 4122**: UUID specification
- **IEEE 754**: Floating-point arithmetic
- **Unicode 15.0**: Character encoding

---

**Next Phase:** [PHASE 2 - Transformation](PHASE-2-Transformation.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


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

