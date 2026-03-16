# WIA-CORE-009: Universal Timestamp Standard - PHASE 2

## Conversion Algorithms and Interoperability

**Version:** 1.0.0
**Status:** Stable
**Date:** 2025-12-27

---

## 1. Overview

PHASE 2 specifies conversion algorithms between Universal Timestamps and other common timestamp formats. This ensures seamless interoperability with existing systems and standards.

## 2. Unix Timestamp Conversion

### 2.1 Unix Time Definition

Unix time represents seconds (or milliseconds) since January 1, 1970, 00:00:00 UTC (the Unix epoch).

**Epoch:** 1970-01-01T00:00:00Z

### 2.2 Unix Seconds → Universal Timestamp

```python
def unix_to_universal(unix_seconds, precision='millisecond', timezone='UTC', context='present'):
    """Convert Unix timestamp (seconds) to Universal Timestamp"""
    # Handle both seconds and milliseconds
    if unix_seconds > 1e12:  # Likely milliseconds
        seconds = unix_seconds // 1000
        millis = unix_seconds % 1000
    else:
        seconds = unix_seconds
        millis = 0

    # Create datetime from Unix timestamp
    dt = datetime.utcfromtimestamp(seconds + millis/1000.0)

    # Format as Universal Timestamp
    return format_universal_timestamp(dt, precision, timezone, context)
```

### 2.3 Universal Timestamp → Unix Seconds

```python
def universal_to_unix(universal_timestamp, return_millis=False):
    """Convert Universal Timestamp to Unix time"""
    parsed = parse_universal_timestamp(universal_timestamp)

    # Create datetime object
    dt = datetime(
        parsed['year'], parsed['month'], parsed['day'],
        parsed['hour'], parsed['minute'], parsed['second']
    )

    # Convert to Unix timestamp
    unix_time = int(dt.timestamp())

    # Add fractional seconds if requested
    if return_millis and parsed['fractional']:
        millis = int(parsed['fractional'][:3].ljust(3, '0'))
        return unix_time * 1000 + millis

    return unix_time
```

### 2.4 Precision Mapping

| Unix Format | Precision | Universal Precision |
|-------------|-----------|---------------------|
| seconds (int) | 1 second | second |
| milliseconds (int) | 1 ms | millisecond |
| microseconds (int) | 1 μs | microsecond |
| nanoseconds (int) | 1 ns | nanosecond |

## 3. ISO 8601 / RFC 3339 Conversion

### 3.1 ISO 8601 → Universal Timestamp

```python
def iso8601_to_universal(iso_string, context='present'):
    """Convert ISO 8601 to Universal Timestamp"""
    # Parse ISO 8601 string
    dt = datetime.fromisoformat(iso_string.replace('Z', '+00:00'))

    # Determine precision from input
    precision = 'second'
    if '.' in iso_string:
        fractional = iso_string.split('.')[1].split('+')[0].split('Z')[0]
        if len(fractional) <= 3:
            precision = 'millisecond'
        elif len(fractional) <= 6:
            precision = 'microsecond'
        else:
            precision = 'nanosecond'

    # Extract timezone (ISO uses offsets, convert to IANA if possible)
    timezone = extract_timezone_from_iso(iso_string)

    return format_universal_timestamp(dt, precision, timezone, context)
```

### 3.2 Universal Timestamp → ISO 8601

```python
def universal_to_iso8601(universal_timestamp, include_timezone=True):
    """Convert Universal Timestamp to ISO 8601"""
    parsed = parse_universal_timestamp(universal_timestamp)

    # Build ISO 8601 string
    iso = f"{parsed['year']:04d}-{parsed['month']:02d}-{parsed['day']:02d}"
    iso += f"T{parsed['hour']:02d}:{parsed['minute']:02d}:{parsed['second']:02d}"

    if parsed['fractional']:
        iso += f".{parsed['fractional']}"

    if include_timezone:
        iso += 'Z'  # Always UTC in Universal Timestamps

    return iso
```

## 4. GPS Time Conversion

### 4.1 GPS Time Definition

GPS time is seconds since January 6, 1980, 00:00:00 UTC. Unlike UTC, GPS time does not include leap seconds.

**GPS Epoch:** 1980-01-06T00:00:00Z
**Leap Seconds (2025):** 18 seconds

### 4.2 GPS → Universal Timestamp

```python
GPS_EPOCH_UNIX = 315964800  # Unix time of GPS epoch

def gps_to_universal(gps_seconds, leap_seconds=18, precision='nanosecond'):
    """Convert GPS time to Universal Timestamp"""
    # GPS to Unix (account for leap seconds)
    unix_seconds = GPS_EPOCH_UNIX + gps_seconds - leap_seconds

    return unix_to_universal(unix_seconds, precision, 'UTC', 'present')
```

### 4.3 Universal Timestamp → GPS

```python
def universal_to_gps(universal_timestamp, leap_seconds=18):
    """Convert Universal Timestamp to GPS time"""
    unix_seconds = universal_to_unix(universal_timestamp)

    # Unix to GPS (reverse leap second adjustment)
    gps_seconds = unix_seconds - GPS_EPOCH_UNIX + leap_seconds

    return gps_seconds
```

### 4.4 Leap Second Table

Implementations MUST maintain current leap second count:

| Date | Leap Seconds Added | Total |
|------|-------------------|-------|
| 1980-01-06 | - | 0 |
| 1981-07-01 | +1 | 1 |
| ... | ... | ... |
| 2017-01-01 | +1 | 18 |
| (current) | - | 18 |

## 5. TAI (International Atomic Time) Conversion

### 5.1 TAI Definition

TAI is a high-precision time standard based on atomic clocks. TAI is currently 37 seconds ahead of UTC (as of 2025).

**TAI - UTC = 37 seconds** (2025)

### 5.2 TAI ↔ Universal Timestamp

```python
def tai_to_universal(tai_unix, leap_seconds=37):
    """Convert TAI to Universal Timestamp"""
    utc_unix = tai_unix - leap_seconds
    return unix_to_universal(utc_unix, 'nanosecond', 'UTC', 'present')

def universal_to_tai(universal_timestamp, leap_seconds=37):
    """Convert Universal Timestamp to TAI"""
    utc_unix = universal_to_unix(universal_timestamp)
    return utc_unix + leap_seconds
```

## 6. Julian Date Conversion

### 6.1 Julian Date Definition

Julian Date (JD) counts days since noon on January 1, 4713 BCE (Julian calendar). Modified Julian Date (MJD) = JD - 2400000.5.

### 6.2 Julian Date → Universal Timestamp

```python
def julian_to_universal(julian_date, precision='millisecond'):
    """Convert Julian Date to Universal Timestamp"""
    # JD to Unix conversion
    unix_seconds = (julian_date - 2440587.5) * 86400

    return unix_to_universal(unix_seconds, precision, 'UTC', 'past')
```

### 6.3 Universal Timestamp → Julian Date

```python
def universal_to_julian(universal_timestamp):
    """Convert Universal Timestamp to Julian Date"""
    unix_seconds = universal_to_unix(universal_timestamp)

    # Unix to JD conversion
    julian_date = (unix_seconds / 86400.0) + 2440587.5

    return julian_date
```

## 7. Database-Specific Conversions

### 7.1 PostgreSQL TIMESTAMP

```sql
-- PostgreSQL → Universal (in application layer)
SELECT
    to_char(timestamp_column, 'YYYY-MM-DD"T"HH24:MI:SS.MS"Z[UTC]{context:past}"')
FROM events;

-- Universal → PostgreSQL
INSERT INTO events (timestamp_column)
VALUES ('2025-12-27 14:30:00+00'::timestamptz);
```

### 7.2 MySQL DATETIME

```python
def mysql_to_universal(mysql_datetime, timezone='UTC'):
    """Convert MySQL DATETIME to Universal"""
    # MySQL format: "2025-12-27 14:30:00"
    iso = mysql_datetime.replace(' ', 'T') + 'Z'
    return iso8601_to_universal(iso)

def universal_to_mysql(universal_timestamp):
    """Convert Universal to MySQL DATETIME"""
    iso = universal_to_iso8601(universal_timestamp)
    return iso.replace('T', ' ').replace('Z', '').split('.')[0]
```

### 7.3 MongoDB ISODate

```javascript
// MongoDB → Universal
function mongoToUniversal(mongoDate) {
    const unixMillis = mongoDate.getTime();
    return unixToUniversal(unixMillis, 'millisecond');
}

// Universal → MongoDB
function universalToMongo(universalTimestamp) {
    const unixMillis = universalToUnix(universalTimestamp, true);
    return new Date(unixMillis);
}
```

## 8. Programming Language Native Formats

### 8.1 JavaScript Date

```javascript
function jsDateToUniversal(jsDate, timezone = 'UTC') {
    return formatUniversalTimestamp(jsDate, 'millisecond', timezone, 'present');
}

function universalToJSDate(universalTimestamp) {
    const unixMillis = universalToUnix(universalTimestamp, true);
    return new Date(unixMillis);
}
```

### 8.2 Python datetime

```python
from datetime import datetime, timezone

def python_to_universal(dt, precision='microsecond', tz='UTC', context='present'):
    """Convert Python datetime to Universal Timestamp"""
    return format_universal_timestamp(dt, precision, tz, context)

def universal_to_python(universal_timestamp):
    """Convert Universal Timestamp to Python datetime"""
    parsed = parse_universal_timestamp(universal_timestamp)

    dt = datetime(
        parsed['year'], parsed['month'], parsed['day'],
        parsed['hour'], parsed['minute'], parsed['second'],
        microsecond=int(parsed['fractional'][:6].ljust(6, '0')) if parsed['fractional'] else 0,
        tzinfo=timezone.utc
    )

    return dt
```

### 8.3 Java Instant

```java
import java.time.Instant;

public static String instantToUniversal(Instant instant, String timezone, String context) {
    String iso = instant.toString();
    return String.format("%s[%s]{context:%s}", iso, timezone, context);
}

public static Instant universalToInstant(String universal) {
    String isoComponent = universal.split("\\[")[0];
    return Instant.parse(isoComponent);
}
```

## 9. Conversion Matrix

### 9.1 Supported Conversions

| From / To | Unix | ISO 8601 | GPS | TAI | JD | Universal |
|-----------|------|----------|-----|-----|----|-----------|
| Unix | - | ✅ | ✅ | ✅ | ✅ | ✅ |
| ISO 8601 | ✅ | - | ✅ | ✅ | ✅ | ✅ |
| GPS | ✅ | ✅ | - | ✅ | ✅ | ✅ |
| TAI | ✅ | ✅ | ✅ | - | ✅ | ✅ |
| JD | ✅ | ✅ | ✅ | ✅ | - | ✅ |
| Universal | ✅ | ✅ | ✅ | ✅ | ✅ | - |

## 10. Precision Preservation

### 10.1 Lossless Conversions

| Source | Target | Data Loss |
|--------|--------|-----------|
| Universal (any) | Universal (same) | None |
| Unix milliseconds | Universal millisecond | None |
| ISO 8601 | Universal | Timezone name* |
| GPS nanosecond | Universal nanosecond | None |

*ISO 8601 uses timezone offsets; IANA timezone names must be inferred or defaulted

### 10.2 Lossy Conversions

| Source | Target | Loss |
|--------|--------|------|
| Universal nanosecond | Unix seconds | Sub-second precision |
| Universal with context | ISO 8601 | Context metadata |
| Universal with hash | Any other format | Hash verification |

### 10.3 Precision Downgrade Rules

When converting to format with lower precision:
1. MUST truncate, not round
2. MUST preserve as much precision as target format allows
3. SHOULD log or warn about precision loss
4. MAY store original precision in metadata

## 11. Batch Conversion Optimization

### 11.1 Batch Processing

```python
def batch_convert_to_universal(timestamps, source_format='unix'):
    """Convert multiple timestamps efficiently"""
    converter = get_converter(source_format, 'universal')

    return [converter(ts) for ts in timestamps]
```

### 11.2 Streaming Conversion

```python
def stream_convert(input_stream, source_format, target_format):
    """Convert timestamps in streaming fashion"""
    converter = get_converter(source_format, target_format)

    for timestamp in input_stream:
        yield converter(timestamp)
```

## 12. Error Handling in Conversions

### 12.1 Conversion Errors

| Error | Code | Description |
|-------|------|-------------|
| Out of range | RANGE_ERROR | Timestamp outside valid range for target format |
| Precision loss | PRECISION_WARNING | Target format cannot preserve source precision |
| Invalid source | INVALID_SOURCE | Source timestamp is malformed |
| Conversion failed | CONVERSION_ERROR | Unexpected error during conversion |

### 12.2 Error Response

```json
{
  "error": {
    "code": "RANGE_ERROR",
    "message": "Unix timestamp would overflow 32-bit integer",
    "source_timestamp": "2040-01-01T00:00:00Z[UTC]{context:future}",
    "target_format": "unix_32bit"
  }
}
```

## 13. Validation After Conversion

### 13.1 Round-Trip Testing

```python
def test_round_trip_conversion(original_timestamp):
    """Verify conversion preserves data"""
    # Convert: Universal → Unix → Universal
    unix = universal_to_unix(original_timestamp)
    converted_back = unix_to_universal(unix)

    assert timestamps_equal(original_timestamp, converted_back)
```

### 13.2 Precision Verification

```python
def verify_precision(original, converted, expected_precision):
    """Verify conversion maintains expected precision"""
    original_parsed = parse_universal_timestamp(original)
    converted_parsed = parse_universal_timestamp(converted)

    assert len(converted_parsed['fractional']) >= expected_precision
```

---

**Next:** [PHASE3.md](PHASE3.md) - Timezone Handling and DST

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) - Benefit All Humanity
