# Phase 3: Pet Tracking Protocol Specification

## WIA-PET-TRACKING Protocol Standard

**Version**: 1.0.0  
**Date**: 2025-12-25  
**Status**: Active  
**Standard ID**: WIA-PET-008-PHASE3  
**Primary Color**: #F59E0B (Amber)

---

## 1. Communication Protocols

### 1.1 Supported Protocols

| Protocol | Transport | Use Case |
|----------|-----------|----------|
| HTTPS/REST | TCP | Standard API calls |
| MQTT | TCP | Real-time IoT updates |
| CoAP | UDP | Constrained devices |
| WebSocket | TCP | Live streaming |

### 1.2 MQTT Configuration

**Topics**:
```
wia/tracking/{trackerId}/location
wia/tracking/{trackerId}/battery
wia/tracking/{trackerId}/geofence
wia/alerts/{trackerId}/lost-pet
```

**QoS Levels**:
- QoS 0: Fire and forget (non-critical)
- QoS 1: At least once (standard updates)
- QoS 2: Exactly once (critical alerts)

---

## 2. Location Update Protocol

### 2.1 Update Frequency Modes

| Mode | Interval | Battery Life | Use Case |
|------|----------|--------------|----------|
| High Performance | 10 seconds | 12-18 hours | Lost pet, active search |
| Real-time | 30 seconds | 24-36 hours | Live tracking |
| Normal | 2 minutes | 48-60 hours | Daily monitoring |
| Eco | 5 minutes | 60-72 hours | Home monitoring |
| Sleep | 30 minutes | 5-7 days | Low activity |

### 2.2 Adaptive Updates

**Motion Detection Algorithm**:
```python
def determine_update_frequency(accelerometer_data):
    movement_magnitude = calculate_movement(accelerometer_data)
    
    if movement_magnitude < 0.1:  # Stationary
        return 30 * 60  # 30 minutes
    elif movement_magnitude < 0.3:  # Resting
        return 10 * 60  # 10 minutes
    elif movement_magnitude < 1.0:  # Walking
        return 2 * 60   # 2 minutes
    else:  # Running
        return 30       # 30 seconds
```

---

## 3. Geofence Detection

### 3.1 Point-in-Circle Algorithm

```python
def check_circular_geofence(location, geofence):
    distance = haversine(
        location.latitude, location.longitude,
        geofence.center.latitude, geofence.center.longitude
    )
    
    accuracy = location.accuracy
    radius = geofence.radius
    
    if distance + accuracy < radius:
        return "INSIDE"
    elif distance - accuracy > radius:
        return "OUTSIDE"
    else:
        return "UNCERTAIN"
```

### 3.2 Debouncing

**Time-based debouncing**:
- Require state change to persist for 30-60 seconds
- Prevent alert spam from GPS jitter

**Hysteresis**:
- Exit threshold: R + 10m
- Entry threshold: R - 10m

---

## 4. Power Management

### 4.1 PSM (Power Saving Mode)

**LTE Cat-M1 PSM Cycle**:
```
1. Sleep (30 seconds): 0.5 mA
2. Wake + GPS Fix (5 seconds): 40 mA
3. Transmit (2 seconds): 200 mA
4. Return to sleep

Average Power: ~8 mA (vs. 60 mA always-on)
Battery Life Extension: 7.5x
```

### 4.2 Battery Optimization Strategies

1. **Motion-Based GPS Sampling**
   - Only activate GPS when movement detected
   - Save 80% battery when stationary

2. **Assisted GPS (A-GPS)**
   - Reduce GPS fix time from 30s to 5s
   - Download ephemeris data via cellular

3. **Network Selection**
   - Prefer Wi-Fi over cellular when available
   - Use NB-IoT for periodic updates (lower power)

---

## 5. Network Failover

### 5.1 Connection Priority

```
1st: Wi-Fi (if available, trusted network)
2nd: Cellular LTE-M (primary)
3rd: LoRaWAN (rural areas)
4th: Satellite (emergency, expensive)
```

### 5.2 Offline Buffering

**Buffer Management**:
```typescript
class OfflineBuffer {
  maxSize: 7 * 24 * 60;  // 7 days of 1-min intervals
  
  priority_levels = {
    CRITICAL: 256,   // Geofence violations
    HIGH: 512,       // Movement periods
    NORMAL: 192,     // Regular updates
    LOW: 64          // Stationary
  };
  
  add(location: LocationUpdate, priority: Priority) {
    if (buffer.isFull()) {
      evict_low_priority_data();
    }
    buffer.store(location, priority);
  }
  
  sync_when_online() {
    upload_by_priority(CRITICAL_first);
    delete_uploaded_data();
  }
}
```

---

## 6. Data Transmission Optimization

### 6.1 Binary Encoding

**Protocol Buffers**:
```protobuf
message LocationUpdate {
  string tracker_id = 1;
  int64 timestamp = 2;
  double latitude = 3;
  double longitude = 4;
  float accuracy = 5;
  int32 battery = 6;
  int32 satellites = 7;
}
```

**Size Comparison**:
- JSON: 202 bytes
- CBOR: 65 bytes (68% reduction)
- Protobuf: 45 bytes (78% reduction)
- Custom binary: 23 bytes (89% reduction)

### 6.2 Compression

**GZIP for batch uploads**:
- Additional 30-50% size reduction
- Trade-off: CPU power vs. data cost
- Recommended for Wi-Fi uploads

---

## 7. Security Protocols

### 7.1 Transport Security

**TLS 1.3 Required**:
```
- Minimum cipher: TLS_AES_128_GCM_SHA256
- Certificate pinning recommended
- No fallback to TLS 1.2 or lower
```

### 7.2 Data Encryption

**At Rest**:
- AES-256-GCM
- Separate keys per user
- Key rotation every 90 days

**In Transit**:
- TLS 1.3 mandatory
- Perfect Forward Secrecy (PFS)

---

## 8. Quality of Service

### 8.1 Target Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Update Latency | < 5 seconds | GPS fix to client display |
| Location Accuracy | ≤ 10 meters | 95% confidence |
| Uptime | 99.5% | Monthly availability |
| Battery Life | ≥ 48 hours | Normal mode |
| Connection Success | > 95% | Update attempts |

### 8.2 Network Monitoring

**Real-time Diagnostics**:
```json
{
  "networkStatus": {
    "signalQuality": {
      "rssi": -75,
      "rsrq": -12,
      "bars": 3
    },
    "connectivity": {
      "latency": 85,
      "packetLoss": 0.5
    },
    "performance": {
      "updateSuccessRate": 0.98,
      "averageLatency": 92
    }
  }
}
```

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA - World Certification Industry Association | MIT License
