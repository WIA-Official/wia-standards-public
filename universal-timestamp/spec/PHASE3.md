# WIA-CORE-009: Universal Timestamp Standard - PHASE 3

## Timezone Handling and DST

**Version:** 1.0.0
**Status:** Stable
**Date:** 2025-12-27

---

## 1. Overview

PHASE 3 defines comprehensive timezone handling, including IANA timezone database integration, DST calculations, and historical timezone changes.

## 2. IANA Timezone Database

### 2.1 Database Requirement

All implementations MUST use the IANA Time Zone Database (also known as tz database or Olson database).

**Current Version:** 2025a or later
**Update Frequency:** At least quarterly
**Source:** https://www.iana.org/time-zones

### 2.2 Timezone Naming

Timezone names follow the pattern: `Area/Location`

**Examples:**
- `UTC` - Coordinated Universal Time
- `America/New_York` - Eastern Time (US & Canada)
- `Europe/London` - British Time
- `Asia/Tokyo` - Japan Standard Time
- `Australia/Sydney` - Australian Eastern Time
- `Etc/GMT+5` - Fixed offset UTC-5

### 2.3 Timezone Resolution

```python
def resolve_timezone(tz_name):
    """Resolve timezone name to IANA timezone object"""
    try:
        return pytz.timezone(tz_name)
    except pytz.UnknownTimeZoneError:
        raise ValueError(f"Unknown timezone: {tz_name}")
```

## 3. DST (Daylight Saving Time)

### 3.1 DST Transitions

DST creates two problematic scenarios:

**Spring Forward (Gap):**
- Clocks advance 1 hour
- Some local times don't exist
- Example: 2:30 AM on March 9, 2025 in New York doesn't exist

**Fall Back (Overlap):**
- Clocks retreat 1 hour
- Some local times occur twice
- Example: 1:30 AM on November 2, 2025 in New York occurs twice

### 3.2 Handling Non-Existent Times

When converting non-existent local time to UTC:

**Option 1: Advance to next valid time**
```
2025-03-09T02:30:00[America/New_York] → 2025-03-09T03:30:00[America/New_York] (EDT)
```

**Option 2: Reject as invalid**
```
Error: INVALID_TIME - 2:30 AM does not exist on March 9, 2025 in America/New_York
```

**Option 3: Use standard time interpretation**
```
2025-03-09T02:30:00[America/New_York] → interpret as 2025-03-09T07:30:00Z (EST)
```

### 3.3 Handling Ambiguous Times

When converting ambiguous local time to UTC:

**Option 1: Use earlier occurrence (default)**
```
2025-11-02T01:30:00[America/New_York] → 2025-11-02T05:30:00Z (EDT)
```

**Option 2: Use later occurrence**
```
2025-11-02T01:30:00[America/New_York] → 2025-11-02T06:30:00Z (EST)
```

**Option 3: Require disambiguation**
```
Error: AMBIGUOUS_TIME - 1:30 AM occurs twice on November 2, 2025 in America/New_York
```

### 3.4 WIA Solution

Universal Timestamps avoid DST ambiguity by using UTC as base time:

```
2025-11-02T05:30:00Z[America/New_York]{context:present}  # First 1:30 AM (EDT)
2025-11-02T06:30:00Z[America/New_York]{context:present}  # Second 1:30 AM (EST)
```

The UTC times are different, eliminating ambiguity.

## 4. Historical Timezone Changes

### 4.1 Tracking Changes

The IANA database includes historical timezone rule changes:

| Location | Change | Date | Impact |
|----------|--------|------|--------|
| Samoa | Crossed date line | 2011-12-30 | Skipped Dec 30, 2011 |
| Russia | Abolished DST | 2014 | Permanent standard time |
| North Korea | Created Pyongyang Time | 2015-2018 | UTC+8:30, then back to UTC+9 |
| Brazil | Ended DST | 2019 | No longer observes DST |
| Yukon | Permanent DST | 2020 | Stopped clock changes |

### 4.2 Historical Conversion

```python
def convert_with_historical_rules(utc_time, timezone_name):
    """Convert UTC to local time using historical rules"""
    tz = pytz.timezone(timezone_name)

    # Apply rules valid at the given time
    local_time = utc_time.astimezone(tz)

    return local_time
```

### 4.3 Future Changes

Timezone rules may change in the future. Implementations MUST:
1. Use latest database version
2. Document database version used
3. Support database updates without code changes
4. Revalidate future timestamps after database updates

## 5. Timezone Offset Calculation

### 5.1 UTC to Local Conversion

```python
def utc_to_local(utc_timestamp, timezone_name):
    """Convert UTC to local time in specified timezone"""
    tz = pytz.timezone(timezone_name)
    utc_dt = parse_utc_time(utc_timestamp)

    # Convert to target timezone
    local_dt = utc_dt.astimezone(tz)

    return {
        'local_time': local_dt,
        'offset_minutes': local_dt.utcoffset().total_seconds() / 60,
        'is_dst': bool(local_dt.dst()),
        'tzname': local_dt.tzname()
    }
```

### 5.2 Local to UTC Conversion

```python
def local_to_utc(local_timestamp, timezone_name, is_dst=None):
    """Convert local time to UTC"""
    tz = pytz.timezone(timezone_name)
    local_dt = parse_local_time(local_timestamp)

    # Localize (handles DST ambiguity)
    if is_dst is None:
        # Default: use earlier time in fall back
        local_dt = tz.localize(local_dt, is_dst=False)
    else:
        local_dt = tz.localize(local_dt, is_dst=is_dst)

    # Convert to UTC
    utc_dt = local_dt.astimezone(pytz.UTC)

    return utc_dt
```

## 6. Leap Seconds

### 6.1 Leap Second Definition

Leap seconds are occasional 1-second adjustments to UTC to keep it aligned with astronomical time (Earth's rotation).

**Leap Second Representation:**
```
2016-12-31T23:59:59Z[UTC]{context:past}  # Normal second
2016-12-31T23:59:60Z[UTC]{context:past}  # Leap second
2017-01-01T00:00:00Z[UTC]{context:past}  # Next day
```

### 6.2 Leap Second Table

Implementations SHOULD maintain leap second table:

```python
LEAP_SECONDS = [
    (datetime(1972, 6, 30, 23, 59, 60), +1),
    (datetime(1972, 12, 31, 23, 59, 60), +1),
    # ... (27 total through 2016) ...
    (datetime(2016, 12, 31, 23, 59, 60), +1),
]

TOTAL_LEAP_SECONDS = 27  # As of 2017
```

### 6.3 Leap Smearing

Some systems use "leap smearing" instead of leap seconds:

```python
def leap_smear(timestamp, smear_window_hours=24):
    """Distribute leap second over smear window"""
    # Google-style leap smearing
    # Gradually slow/speed clock over 24 hours
    pass
```

## 7. Timezone Validation

### 7.1 Validation Rules

```python
def validate_timezone(tz_name, utc_time):
    """Validate timezone name and rules"""
    # Check timezone exists
    try:
        tz = pytz.timezone(tz_name)
    except:
        raise ValueError(f"Invalid timezone: {tz_name}")

    # Check timezone has valid rules for given time
    try:
        offset = tz.utcoffset(utc_time)
    except:
        raise ValueError(f"No timezone rules for {utc_time} in {tz_name}")

    return True
```

## 8. Timezone Conversion Performance

### 8.1 Caching Strategy

```python
class TimezoneCache:
    def __init__(self, cache_size=1000):
        self.cache = {}
        self.max_size = cache_size

    def get_offset(self, tz_name, utc_hour):
        """Get timezone offset with hourly caching"""
        cache_key = f"{tz_name}:{utc_hour}"

        if cache_key not in self.cache:
            tz = pytz.timezone(tz_name)
            dt = datetime.fromtimestamp(utc_hour * 3600, tz=pytz.UTC)
            offset = tz.utcoffset(dt).total_seconds()

            # LRU eviction
            if len(self.cache) >= self.max_size:
                oldest_key = next(iter(self.cache))
                del self.cache[oldest_key]

            self.cache[cache_key] = offset

        return self.cache[cache_key]
```

## 9. Testing Requirements

### 9.1 DST Test Cases

All implementations MUST test:

1. **Spring Forward:**
   - Time before DST transition
   - Non-existent time during gap
   - Time after DST transition

2. **Fall Back:**
   - Time before DST transition
   - First occurrence of ambiguous time
   - Second occurrence of ambiguous time
   - Time after DST transition

3. **Historical Changes:**
   - Conversion before rule change
   - Conversion after rule change
   - Verify different results for same local time

### 9.2 Example Test Cases

```python
def test_spring_forward():
    # New York 2025 spring forward at 2:00 AM EST → 3:00 AM EDT
    before = "2025-03-09T06:59:59Z[America/New_York]{context:present}"
    # 1:59:59 AM EST
    assert local_time(before) == "01:59:59 EST"

    after = "2025-03-09T07:00:00Z[America/New_York]{context:present}"
    # 3:00:00 AM EDT (2:00:00 skipped)
    assert local_time(after) == "03:00:00 EDT"

def test_fall_back():
    # New York 2025 fall back at 2:00 AM EDT → 1:00 AM EST
    first = "2025-11-02T05:30:00Z[America/New_York]{context:present}"
    assert local_time(first) == "01:30:00 EDT"

    second = "2025-11-02T06:30:00Z[America/New_York]{context:present}"
    assert local_time(second) == "01:30:00 EST"
```

---

**Next:** [PHASE4.md](PHASE4.md) - Advanced Features and Extensions

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) - Benefit All Humanity
