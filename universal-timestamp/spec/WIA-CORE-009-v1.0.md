# WIA-CORE-009: Universal Timestamp Specification v1.0

> **Standard ID:** WIA-CORE-009
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Core Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Timestamp Format](#2-timestamp-format)
3. [Precision Levels](#3-precision-levels)
4. [Timezone Handling](#4-timezone-handling)
5. [Temporal Context](#5-temporal-context)
6. [Cryptographic Signatures](#6-cryptographic-signatures)
7. [Encoding & Serialization](#7-encoding--serialization)
8. [Validation Rules](#8-validation-rules)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a universal timestamp format that provides:
- Cross-system interoperability
- Timezone-independent time representation
- High-precision temporal measurements
- Support for temporal displacement (time-travel)
- Cryptographic verification
- Human-readable formats

### 1.2 Scope

The standard covers:
- Timestamp format and syntax
- Precision levels from seconds to Planck time
- Timezone and calendar systems
- Temporal context for time-travel scenarios
- Cryptographic signing and verification
- Encoding, parsing, and validation

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard unifies temporal representation across all human systems, enabling seamless coordination and future compatibility with advanced technologies including time travel.

### 1.4 Terminology

- **Epoch**: Seconds since Unix epoch (1970-01-01T00:00:00Z)
- **Precision**: Sub-second time resolution
- **Temporal Context**: Metadata for temporal displacement
- **Worldline**: Path through spacetime
- **Timeline**: Specific branch of reality
- **Planck Time**: Smallest meaningful unit of time (5.391×10⁻⁴⁴s)

---

## 2. Timestamp Format

### 2.1 Basic Format

```
WIA-TS: <epoch>.<precision>@<timezone>
```

**Components:**

1. **Prefix**: `WIA-TS:` (required)
2. **Epoch**: Integer seconds since 1970-01-01T00:00:00Z
3. **Precision**: Nanosecond fraction (0-999999999)
4. **Timezone**: IANA timezone or UTC offset

**Example:**
```
WIA-TS: 1735257600.123456789@UTC
```

### 2.2 Extended Format

```
WIA-TS: <epoch>.<precision>@<timezone>[<temporal-context>]{<signature>}
```

**Additional Components:**

4. **Temporal Context**: Optional, enclosed in `[]`
5. **Signature**: Optional, enclosed in `{}`

**Example:**
```
WIA-TS: 1735257600.123456789@UTC[origin=1704067200,displacement=-31536000]{sig=ed25519:a1b2...}
```

### 2.3 Format Variations

#### Minimal Format
```
WIA-TS: 1735257600@UTC
```

#### Millisecond Precision
```
WIA-TS: 1735257600.123@UTC
```

#### Microsecond Precision
```
WIA-TS: 1735257600.123456@UTC
```

#### Nanosecond Precision
```
WIA-TS: 1735257600.123456789@UTC
```

#### With Timezone Offset
```
WIA-TS: 1735257600.123456789@+09:00
WIA-TS: 1735257600.123456789@America/New_York
```

---

## 3. Precision Levels

### 3.1 Precision Hierarchy

| Level | Name | Unit | Precision | Digits | Use Case |
|-------|------|------|-----------|--------|----------|
| 0 | Second | s | 1 | 0 | General use |
| 1 | Millisecond | ms | 10⁻³ | 3 | Web apps |
| 2 | Microsecond | μs | 10⁻⁶ | 6 | HFT, databases |
| 3 | Nanosecond | ns | 10⁻⁹ | 9 | Scientific |
| 4 | Planck | tₚ | 5.391×10⁻⁴⁴ | variable | Quantum/time-travel |

### 3.2 Precision Encoding

The precision field must be:
- Zero-padded to appropriate length (3, 6, or 9 digits)
- Right-truncated if lower precision is sufficient
- Extended with metadata for Planck-level precision

**Examples:**
```
Level 0:  1735257600@UTC           (no fraction)
Level 1:  1735257600.123@UTC       (3 digits)
Level 2:  1735257600.123456@UTC    (6 digits)
Level 3:  1735257600.123456789@UTC (9 digits)
Level 4:  1735257600.123456789@UTC[planck=1.234e-44]
```

### 3.3 Planck-Level Precision

For quantum and time-travel applications:

```
WIA-TS: <epoch>.<nano>@<tz>[planck=<value>]
```

Where `<value>` is in scientific notation representing additional precision beyond nanoseconds.

**Example:**
```
WIA-TS: 1735257600.123456789@UTC[planck=5.391e-44]
```

---

## 4. Timezone Handling

### 4.1 Timezone Formats

Three formats are supported:

1. **UTC**: Universal Coordinated Time
   ```
   WIA-TS: 1735257600@UTC
   ```

2. **UTC Offset**: `±HH:MM` format
   ```
   WIA-TS: 1735257600@+09:00
   WIA-TS: 1735257600@-05:00
   ```

3. **IANA Timezone**: Named timezone database
   ```
   WIA-TS: 1735257600@America/New_York
   WIA-TS: 1735257600@Asia/Tokyo
   WIA-TS: 1735257600@Europe/London
   ```

### 4.2 Timezone Conversion

All timestamps MUST be convertible to UTC:
1. Store epoch in UTC
2. Record timezone as metadata
3. Convert for display/local use

**Conversion Algorithm:**
```
UTC_epoch = local_epoch - (offset_hours × 3600)
```

### 4.3 Daylight Saving Time

For IANA timezones:
- Use timezone database rules
- Handle DST transitions automatically
- Ambiguous times use standard time

---

## 5. Temporal Context

### 5.1 Purpose

Temporal context enables:
- Time-travel timestamp tracking
- Timeline branching
- Worldline identification
- Causality preservation

### 5.2 Temporal Context Format

```
[origin=<epoch>,displacement=<seconds>,worldline=<id>,timeline=<id>,ref=<standard>]
```

**Fields:**

- `origin`: Original timeline timestamp (epoch)
- `displacement`: Temporal displacement in seconds (negative for past)
- `worldline`: Worldline identifier
- `timeline`: Timeline branch identifier
- `ref`: Reference WIA-TIME standard

**Example:**
```
WIA-TS: 1735257600@UTC[origin=1704067200,displacement=-31536000,worldline=alpha-01,timeline=main,ref=WIA-TIME-001]
```

### 5.3 Time-Travel Integration

When integrated with WIA-TIME standards:

1. **Current Time**: Timestamp at current location in spacetime
2. **Origin Time**: Timestamp at departure point
3. **Displacement**: Temporal offset from origin
4. **Worldline**: Path through spacetime
5. **Timeline**: Specific reality branch

**Flow:**
```
Origin → [Displacement] → Current
  |                          |
  +---- Worldline: alpha-01--+
  +---- Timeline: main ------+
```

### 5.4 Causality Tracking

Temporal context enables:
- Paradox detection
- Novikov consistency validation
- Timeline integrity verification
- Event ordering across timelines

---

## 6. Cryptographic Signatures

### 6.1 Purpose

Cryptographic signatures provide:
- Timestamp authenticity
- Tamper detection
- Non-repudiation
- Blockchain compatibility

### 6.2 Signature Format

```
{sig=<algorithm>:<signature>,hash=<algorithm>:<hash>,nonce=<value>}
```

**Fields:**

- `sig`: Digital signature (algorithm:value)
- `hash`: Cryptographic hash (algorithm:value)
- `nonce`: Optional nonce for uniqueness

**Example:**
```
WIA-TS: 1735257600@UTC{sig=ed25519:a1b2c3d4...,hash=sha256:e5f6g7h8...}
```

### 6.3 Supported Algorithms

#### Signature Algorithms

| Algorithm | Type | Key Size | Quantum Safe |
|-----------|------|----------|--------------|
| Ed25519 | EdDSA | 256-bit | No |
| RSA | RSA | 2048/4096-bit | No |
| ECDSA | ECDSA | 256-bit | No |
| Dilithium | PQC | 2048-bit | Yes |
| SPHINCS+ | PQC | variable | Yes |

#### Hash Algorithms

| Algorithm | Output | Quantum Safe |
|-----------|--------|--------------|
| SHA-256 | 256-bit | No |
| SHA-3 | 256/512-bit | Partial |
| Blake3 | 256-bit | Partial |

### 6.4 Signing Process

1. **Create Base Timestamp**
   ```
   base = "WIA-TS: 1735257600.123456789@UTC"
   ```

2. **Hash Timestamp**
   ```
   hash = SHA256(base)
   ```

3. **Sign Hash**
   ```
   signature = Sign(private_key, hash)
   ```

4. **Encode Signature**
   ```
   encoded = algorithm + ":" + base64(signature)
   ```

5. **Append to Timestamp**
   ```
   final = base + "{sig=" + encoded + ",hash=sha256:" + hex(hash) + "}"
   ```

### 6.5 Verification Process

1. **Parse Timestamp**
2. **Extract Base (without signature)**
3. **Recompute Hash**
4. **Verify Signature**
5. **Compare Hashes**
6. **Return Validation Result**

---

## 7. Encoding & Serialization

### 7.1 String Format (Primary)

```
WIA-TS: 1735257600.123456789@UTC
```

### 7.2 JSON Format

```json
{
  "format": "WIA-TS",
  "version": "1.0",
  "epoch": 1735257600,
  "precision": 123456789,
  "timezone": "UTC",
  "temporalContext": {
    "origin": 1704067200,
    "displacement": -31536000,
    "worldline": "alpha-01",
    "timeline": "main",
    "reference": "WIA-TIME-001"
  },
  "signature": {
    "algorithm": "ed25519",
    "value": "a1b2c3d4...",
    "hash": "sha256:e5f6g7h8..."
  }
}
```

### 7.3 Binary Format

For efficient storage and transmission:

```
[Header: 8 bytes]
[Epoch: 8 bytes]
[Precision: 4 bytes]
[Timezone: variable, null-terminated]
[Temporal Context: variable, optional]
[Signature: variable, optional]
```

**Header Format:**
```
Byte 0-3: Magic number (0x57494154 = "WIAT")
Byte 4:   Version (0x01)
Byte 5:   Flags (bit 0=temporal, bit 1=signed)
Byte 6-7: Reserved
```

### 7.4 URL-Safe Format

For use in URLs and query parameters:

```
WIA-TS_1735257600-123456789_UTC
```

Replace:
- Space with underscore
- Colon with dash
- `@` with underscore
- Remove brackets/braces

---

## 8. Validation Rules

### 8.1 Epoch Validation

- **Range**: -2147483648 to 2147483647 (32-bit signed)
- **Extended**: -9223372036854775808 to 9223372036854775807 (64-bit signed)
- **Negative epochs**: Before 1970-01-01 (valid for historical dates)

### 8.2 Precision Validation

- **Format**: Integer 0-999999999
- **Zero-padding**: Must match precision level
- **No leading zeros**: Except for padding

### 8.3 Timezone Validation

- **UTC**: Must be uppercase "UTC"
- **Offset**: Must match `±HH:MM` format
- **IANA**: Must exist in IANA timezone database

### 8.4 Temporal Context Validation

- **Origin**: Must be valid epoch
- **Displacement**: Any integer (negative for past)
- **Worldline**: Alphanumeric with hyphens
- **Timeline**: Alphanumeric with hyphens
- **Reference**: Must be valid WIA-TIME standard ID

### 8.5 Signature Validation

- **Algorithm**: Must be supported
- **Value**: Must be valid base64
- **Hash**: Must match recomputed hash

### 8.6 Format Validation

Regular expression for basic format:
```regex
^WIA-TS:\s*(-?\d+)(?:\.(\d{1,9}))?@([A-Za-z/_+-:]+)(?:\[([^\]]+)\])?(?:\{([^}]+)\})?$
```

---

## 9. Implementation Guidelines

### 9.1 Creating Timestamps

```typescript
function createTimestamp(options: {
  date?: Date;
  precision?: 0 | 1 | 2 | 3 | 4;
  timezone?: string;
  temporalContext?: TemporalContext;
  sign?: boolean;
  privateKey?: CryptoKey;
}): UniversalTimestamp
```

### 9.2 Parsing Timestamps

```typescript
function parseTimestamp(input: string): UniversalTimestamp | null
```

Must handle:
- All format variations
- Invalid input gracefully
- Partial timestamps
- Legacy formats

### 9.3 Converting Timezones

```typescript
function convertTimezone(
  timestamp: UniversalTimestamp,
  targetTimezone: string
): UniversalTimestamp
```

### 9.4 Comparing Timestamps

```typescript
function compareTimestamps(
  a: UniversalTimestamp,
  b: UniversalTimestamp
): -1 | 0 | 1
```

Comparison logic:
1. Compare epochs
2. If equal, compare precision
3. Consider temporal displacement if present

### 9.5 Formatting Timestamps

Support multiple output formats:
- WIA-TS (canonical)
- ISO 8601
- RFC 3339
- Unix epoch
- Human readable

```typescript
function formatTimestamp(
  timestamp: UniversalTimestamp,
  format: 'wia' | 'iso' | 'rfc3339' | 'unix' | 'human'
): string
```

### 9.6 Performance Considerations

- Cache timezone conversions
- Precompile validation regex
- Use native Date objects when possible
- Lazy-load cryptographic modules
- Optimize binary encoding for transmission

---

## 10. References

### 10.1 Standards

- **ISO 8601**: Date and time format
- **RFC 3339**: Internet date/time format
- **IANA TZ Database**: Timezone data
- **IEEE 1588**: Precision Time Protocol
- **RFC 5905**: Network Time Protocol (NTP)

### 10.2 WIA Standards

- **WIA-TIME-001**: Time Travel Physics
- **WIA-TIME-002** to **WIA-TIME-035**: Temporal mechanics standards
- **WIA-INTENT**: Intent-based interfaces
- **WIA-OMNI-API**: Universal API gateway

### 10.3 Cryptography

- **RFC 8032**: EdDSA signatures (Ed25519)
- **FIPS 186-4**: Digital Signature Standard
- **NIST PQC**: Post-Quantum Cryptography

### 10.4 Physical Constants

- **Planck Time**: 5.391247×10⁻⁴⁴ seconds
- **Speed of Light**: 299,792,458 m/s
- **Unix Epoch**: 1970-01-01T00:00:00Z

---

## Appendix A: Examples

### A.1 Current Timestamp
```
WIA-TS: 1735257600.123456789@UTC
```

### A.2 Historical Date
```
WIA-TS: -2208988800@UTC  // 1900-01-01
```

### A.3 Future Date
```
WIA-TS: 4102444800@UTC  // 2100-01-01
```

### A.4 Time-Travel Timestamp
```
WIA-TS: 1735257600@UTC[origin=1704067200,displacement=-31536000,worldline=alpha-01]
```

### A.5 Signed Timestamp
```
WIA-TS: 1735257600@UTC{sig=ed25519:a1b2c3d4e5f6...,hash=sha256:123456789abc...}
```

### A.6 Complete Example
```
WIA-TS: 1735257600.123456789@America/New_York[origin=1704067200,displacement=-31536000,worldline=alpha-01,timeline=main,ref=WIA-TIME-001]{sig=ed25519:a1b2c3d4...,hash=sha256:e5f6g7h8...,nonce=9876543210}
```

---

## Appendix B: Migration Guide

### B.1 From Unix Timestamp

```typescript
const unix = 1735257600;
const wia = `WIA-TS: ${unix}@UTC`;
```

### B.2 From ISO 8601

```typescript
const iso = "2025-12-27T00:00:00Z";
const date = new Date(iso);
const epoch = Math.floor(date.getTime() / 1000);
const wia = `WIA-TS: ${epoch}@UTC`;
```

### B.3 From JavaScript Date

```typescript
const date = new Date();
const epoch = Math.floor(date.getTime() / 1000);
const nano = (date.getTime() % 1000) * 1000000;
const wia = `WIA-TS: ${epoch}.${nano.toString().padStart(9, '0')}@UTC`;
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
