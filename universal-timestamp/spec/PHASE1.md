# WIA-CORE-009: Universal Timestamp Standard - PHASE 1

## Core Format Specification

**Version:** 1.0.0
**Status:** Stable
**Date:** 2025-12-27

---

## 1. Overview

PHASE 1 defines the foundational format specification for WIA Universal Timestamps. This phase establishes the core structure, required components, and basic parsing rules that all implementations MUST support.

## 2. Format Structure

### 2.1 Complete Format

```
YYYY-MM-DDTHH:MM:SS[.F...]Z[TIMEZONE]{context:CONTEXT}[#HASH]
```

### 2.2 Component Breakdown

| Component | Required | Description | Example |
|-----------|----------|-------------|---------|
| YYYY-MM-DD | Yes | ISO 8601 date | 2025-12-27 |
| T | Yes | Date/time separator | T |
| HH:MM:SS | Yes | ISO 8601 time (UTC) | 14:30:00 |
| .F... | No | Fractional seconds (variable length) | .789123456 |
| Z | Yes | UTC indicator | Z |
| [TIMEZONE] | Yes | IANA timezone in brackets | [America/New_York] |
| {context:CONTEXT} | Yes | Temporal context in braces | {context:present} |
| #HASH | No | SHA-256 hash (hex) | #a1b2c3d4... |

### 2.3 Regular Expression

```regex
^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?Z\[([^\]]+)\]\{context:([^,}]+)(?:,([^}]+))?\}(?:#([0-9a-f]+))?$
```

## 3. Precision Levels

### 3.1 Supported Precisions

| Level | Name | Scale | Digits | Example Fractional |
|-------|------|-------|--------|-------------------|
| 0 | Second | 10⁰ s | 0 | (none) |
| 1 | Millisecond | 10⁻³ s | 3 | .789 |
| 2 | Microsecond | 10⁻⁶ s | 6 | .789123 |
| 3 | Nanosecond | 10⁻⁹ s | 9 | .789123456 |
| 4 | Picosecond | 10⁻¹² s | 12 | .789123456789 |
| 5 | Femtosecond | 10⁻¹⁵ s | 15 | .789123456789012 |
| 6 | Attosecond | 10⁻¹⁸ s | 18 | .789123456789012345 |
| 7 | Planck Time | 10⁻⁴⁴ s | 44 | .78912345678901234567890123456789012345678901234 |

### 3.2 Precision Rules

1. Fractional seconds MAY have any length from 0 to 44 digits
2. Trailing zeros MUST NOT be removed (preserve precision)
3. Leading zeros after decimal point MUST be included
4. Implementations MUST support at least nanosecond precision (9 digits)
5. Higher precision is OPTIONAL but RECOMMENDED

## 4. Timezone Specification

### 4.1 IANA Timezone Names

All Universal Timestamps MUST use IANA Time Zone Database names.

**Valid Examples:**
- `[UTC]`
- `[America/New_York]`
- `[Europe/London]`
- `[Asia/Tokyo]`
- `[Australia/Sydney]`
- `[Etc/GMT+5]` (fixed offset: UTC-5)

**Invalid Examples:**
- `[EST]` - Ambiguous abbreviation
- `[GMT-5]` - Not IANA format
- `[New York]` - Spaces and wrong format
- `[]` - Empty timezone

### 4.2 Timezone Database Version

Implementations SHOULD document which IANA timezone database version they use. The database version affects DST calculations and historical conversions.

**Recommended:** Use latest stable IANA timezone database (2025a or newer as of 2025)

## 5. Temporal Context

### 5.1 Standard Context Types

| Context | Meaning | Use Cases |
|---------|---------|-----------|
| present | Current real-time event | Live data, transactions |
| past | Historical record | Logs, archives, audit trails |
| future | Predicted or scheduled | Calendar, forecasts, schedules |
| simulated | Generated in simulation | Training data, games, models |
| quantum | Quantum superposition | Quantum computing, experiments |
| multiverse | Alternative timeline | Time-travel, parallel universes |

### 5.2 Context Rules

1. Context type MUST be specified (no default)
2. Context type MUST be one of the standard types or a custom extension
3. Custom context types MUST be documented
4. Context MAY include additional comma-separated metadata:
   ```
   {context:present,source:gps,accuracy:10ns}
   {context:multiverse,timeline:alpha-7,divergence:2025-01-01T00:00:00Z}
   ```

## 6. Cryptographic Hash

### 6.1 Hash Algorithm

- **Algorithm:** SHA-256
- **Input:** Complete timestamp string EXCLUDING the hash component and # symbol
- **Output:** Hexadecimal lowercase string
- **Encoding:** UTF-8 bytes of timestamp string

### 6.2 Hash Computation

```
1. Construct base timestamp: "YYYY-MM-DDTHH:MM:SS.FFF...Z[TZ]{context:CTX}"
2. Convert to UTF-8 bytes
3. Compute SHA-256 hash
4. Convert hash to hexadecimal lowercase
5. Optionally truncate (from left, keep first N hex digits)
6. Append #hash to timestamp
```

### 6.3 Hash Truncation Levels

| Truncation | Hex Digits | Bytes | Security Level |
|------------|------------|-------|----------------|
| Full | 64 | 32 | Cryptographic |
| Half | 32 | 16 | High |
| Quarter | 16 | 8 | Standard |
| Eighth | 8 | 4 | Basic |

### 6.4 Hash Verification

To verify a hash:
1. Split timestamp at # symbol
2. Recompute hash from base timestamp
3. Truncate computed hash to match provided hash length
4. Compare (case-insensitive)
5. Return true if match, false otherwise

## 7. Validation Rules

### 7.1 Date Validation

1. Year: MUST be 4 digits (0000-9999 supported, negative years use minus prefix)
2. Month: MUST be 01-12
3. Day: MUST be valid for the given year/month (account for leap years)
4. Date MUST be valid Gregorian calendar date

### 7.2 Time Validation

1. Hour: MUST be 00-23
2. Minute: MUST be 00-59
3. Second: MUST be 00-60 (60 for leap seconds)
4. Time MUST represent valid UTC time

### 7.3 Format Validation

1. All required components MUST be present
2. Component separators MUST be exact (T, Z, [ ], { }, :, -, .)
3. Timezone name MUST exist in IANA database
4. Context type MUST be valid
5. Hash (if present) MUST be valid hexadecimal

## 8. Parsing Algorithm

### 8.1 Basic Parsing

```python
def parse_universal_timestamp(timestamp_str):
    regex = r'^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?Z\[([^\]]+)\]\{context:([^}]+)\}(?:#([0-9a-f]+))?$'

    match = re.match(regex, timestamp_str)
    if not match:
        raise ValueError("Invalid Universal Timestamp format")

    return {
        'year': int(match.group(1)),
        'month': int(match.group(2)),
        'day': int(match.group(3)),
        'hour': int(match.group(4)),
        'minute': int(match.group(5)),
        'second': int(match.group(6)),
        'fractional': match.group(7) or '',
        'timezone': match.group(8),
        'context': match.group(9),
        'hash': match.group(10)
    }
```

### 8.2 Validation During Parsing

Implementations MUST validate:
1. Date is valid (including leap year handling)
2. Time is valid (including leap second support)
3. Timezone exists in IANA database
4. Context type is recognized or documented as custom
5. Hash (if present) is valid hex and optionally verify it

## 9. Generation Algorithm

### 9.1 Basic Generation

```python
def generate_universal_timestamp(
    dt,
    precision='millisecond',
    timezone='UTC',
    context='present',
    include_hash=False
):
    # Format datetime component
    timestamp = f"{dt.year:04d}-{dt.month:02d}-{dt.day:02d}T"
    timestamp += f"{dt.hour:02d}:{dt.minute:02d}:{dt.second:02d}"

    # Add fractional seconds based on precision
    if precision != 'second':
        fractional = format_fractional(dt, precision)
        timestamp += f".{fractional}"

    # Add timezone and context
    timestamp += f"Z[{timezone}]{{context:{context}}}"

    # Optionally add hash
    if include_hash:
        hash_value = compute_sha256_hash(timestamp)
        timestamp += f"#{hash_value}"

    return timestamp
```

## 10. Error Handling

### 10.1 Required Error Codes

| Code | Description | Example |
|------|-------------|---------|
| INVALID_FORMAT | Malformed timestamp string | Missing required component |
| INVALID_DATE | Invalid date component | 2025-02-30 (Feb 30th) |
| INVALID_TIME | Invalid time component | 25:00:00 (invalid hour) |
| INVALID_TIMEZONE | Unknown timezone | [Invalid/Timezone] |
| INVALID_CONTEXT | Unknown context type | {context:unknown} |
| INVALID_HASH | Hash verification failed | Hash mismatch |
| PRECISION_EXCEEDED | Too many fractional digits | >44 digits |

### 10.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_DATE",
    "message": "Invalid date: 2025-02-30 does not exist",
    "timestamp": "2025-02-30T12:00:00Z[UTC]{context:present}",
    "position": 8
  }
}
```

## 11. Implementation Requirements

### 11.1 MUST Support

1. Parsing all valid Universal Timestamp formats
2. Generating timestamps with at least millisecond precision
3. Validating date, time, timezone, and context
4. Converting to/from Unix time
5. Converting to/from ISO 8601
6. All standard context types (present, past, future, simulated, quantum, multiverse)

### 11.2 SHOULD Support

1. Generating timestamps with nanosecond precision
2. Hash generation and verification
3. Custom context types with metadata
4. Timezone database updates
5. Leap second handling

### 11.3 MAY Support

1. Precisions higher than nanosecond (pico, femto, atto, Planck)
2. Custom hash algorithms (SHA3, BLAKE3)
3. Binary encoding formats
4. Batch operations
5. Caching and optimization

## 12. Compatibility

### 12.1 Forward Compatibility

- New context types MAY be added in future versions
- New metadata fields in context MAY be added
- New hash algorithms MAY be supported
- Parsers MUST NOT fail on unknown context types if format is otherwise valid

### 12.2 Backward Compatibility

- Core format MUST NOT change
- Required components MUST remain required
- Component ordering MUST NOT change
- Existing context types MUST NOT be removed or redefined

## 13. Examples

### 13.1 Minimal Valid Timestamp

```
2025-12-27T14:30:00Z[UTC]{context:present}
```

### 13.2 Full Precision with Hash

```
2025-12-27T14:30:00.789123456789012345678901234567890123456789012345Z[UTC]{context:present}#a1b2c3d4e5f67890
```

### 13.3 With Custom Metadata

```
2025-12-27T14:30:00.789Z[America/New_York]{context:future,confidence:0.95,source:ml-model}
```

### 13.4 Multiverse Context

```
1969-07-20T20:17:40Z[UTC]{context:multiverse,timeline:alpha-537,divergence:1969-07-20T20:17:00Z}
```

---

**Next:** [PHASE2.md](PHASE2.md) - Conversion Algorithms and Interoperability

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) - Benefit All Humanity
