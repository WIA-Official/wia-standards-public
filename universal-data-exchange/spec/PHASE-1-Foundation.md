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
