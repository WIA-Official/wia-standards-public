# PHASE 4 — Integration

> V2X integration with the broader vehicle, regulatory, and
> operational landscape: security and privacy regime,
> implementation guidelines for OEMs and infrastructure
> operators, and the references that anchor the standard to
> ISO / IEEE / SAE / ETSI base specifications.

## 9. Security and Privacy

### 9.1 Security Architecture

**Security Requirements:**
1. **Authentication**: Verify message sender identity
2. **Integrity**: Detect message tampering
3. **Non-repudiation**: Sender cannot deny sending
4. **Confidentiality**: Encrypt sensitive data (optional for safety)
5. **Availability**: Resist denial-of-service attacks
6. **Privacy**: Protect user location and identity

### 9.2 Public Key Infrastructure (PKI)

**Certificate Hierarchy:**
```
Root CA (Certificate Authority)
  ├── Enrollment CA (Long-term certificates)
  │     └── Vehicle Enrollment Certificates
  └── Authorization CA (Short-term certificates)
        └── Pseudonym Certificates (changed frequently)
```

**Certificate Format (IEEE 1609.2):**
```json
{
  "version": 3,
  "type": "explicit",
  "issuer": "AuthorizationCA-001",
  "toBeSigned": {
    "id": "CERT-789012",
    "cracaId": "ROOT-CA-001",
    "crlSeries": 5,
    "validityPeriod": {
      "start": "2025-12-26T00:00:00Z",
      "duration": "1 week"
    },
    "region": {
      "circularRegion": {
        "center": {"lat": 37.7749, "lon": -122.4194},
        "radius": 50000  // meters
      }
    },
    "assuranceLevel": "high",
    "appPermissions": [
      {
        "psid": 0x20,  // Basic Safety Message
        "ssp": "all"
      }
    ],
    "certIssuePermissions": [],
    "verifyKeyIndicator": {
      "verificationKey": {
        "algorithm": "ecdsaNistp256",
        "publicKey": "04a1b2c3..."
      }
    }
  },
  "signature": {
    "algorithm": "ecdsaNistp256",
    "r": "12345678...",
    "s": "87654321..."
  }
}
```

### 9.3 Message Authentication

**Signed Message Structure:**
```
┌─────────────────────────┐
│   Unsigned Payload      │  (BSM/CAM/DENM data)
├─────────────────────────┤
│   Header Info           │  (Protocol version, message type)
├─────────────────────────┤
│   Signer Info           │  (Certificate or digest)
├─────────────────────────┤
│   Generation Time       │  (Timestamp)
├─────────────────────────┤
│   Generation Location   │  (3D position)
├─────────────────────────┤
│   Digital Signature     │  (ECDSA P-256)
└─────────────────────────┘
```

**Verification Process:**
```python
def verify_v2x_message(signed_message):
    # 1. Extract certificate from message
    cert = extract_certificate(signed_message)

    # 2. Verify certificate validity
    if not verify_certificate(cert):
        return False, "Invalid certificate"

    # 3. Check certificate not revoked
    if is_revoked(cert):
        return False, "Certificate revoked"

    # 4. Verify message signature
    payload = signed_message.payload
    signature = signed_message.signature
    public_key = cert.public_key

    if not ecdsa_verify(public_key, payload, signature):
        return False, "Signature verification failed"

    # 5. Check message freshness (replay attack prevention)
    timestamp = signed_message.generation_time
    if abs(current_time() - timestamp) > 5000:  # 5 seconds
        return False, "Message too old"

    # 6. Verify geographic consistency
    claimed_location = signed_message.generation_location
    cert_region = cert.region

    if not is_in_region(claimed_location, cert_region):
        return False, "Location outside certificate region"

    return True, "Message authenticated"
```

### 9.4 Privacy Protection

**Pseudonym Certificates:**
- Changed every 5-10 minutes
- No linkability between pseudonyms
- Prevents long-term tracking
- Pool of 20+ certificates pre-loaded

**Privacy Zones:**
```
Home Location:
  - No V2X broadcast within 500m of registered home
  - Or use maximum privacy mode (encrypted)

Sensitive Locations:
  - Hospitals
  - Government buildings
  - Religious sites
  - Automatically detected and protected
```

**Location Obfuscation:**
```python
def apply_privacy_filter(position, privacy_level):
    if privacy_level == 'MAXIMUM':
        # Don't broadcast position at all
        return None

    elif privacy_level == 'HIGH':
        # Reduce precision to ~100m
        return {
            'latitude': round(position.latitude, 3),
            'longitude': round(position.longitude, 3)
        }

    elif privacy_level == 'MEDIUM':
        # Reduce precision to ~10m
        return {
            'latitude': round(position.latitude, 4),
            'longitude': round(position.longitude, 4)
        }

    else:  # LOW or NONE
        # Full precision (~1cm)
        return position
```

### 9.5 Misbehavior Detection

**Anomaly Detection:**
```python
def detect_misbehavior(message, history):
    anomalies = []

    # Check position consistency
    if history.last_position:
        max_distance = history.last_speed * time_delta * 1.5  # Allow 50% margin
        actual_distance = calculate_distance(history.last_position, message.position)

        if actual_distance > max_distance:
            anomalies.append({
                'type': 'POSITION_JUMP',
                'severity': 'HIGH',
                'description': f'Position jumped {actual_distance}m in {time_delta}s'
            })

    # Check speed consistency
    if message.speed > 200:  # km/h (unrealistic for most vehicles)
        anomalies.append({
            'type': 'EXCESSIVE_SPEED',
            'severity': 'MEDIUM',
            'description': f'Speed {message.speed} km/h exceeds realistic limit'
        })

    # Check acceleration consistency
    max_accel = 10  # m/s² (typical maximum)
    if abs(message.acceleration.longitudinal) > max_accel:
        anomalies.append({
            'type': 'EXCESSIVE_ACCELERATION',
            'severity': 'MEDIUM',
            'description': f'Acceleration {message.acceleration.longitudinal} m/s² unrealistic'
        })

    # Check message frequency
    if len(history.recent_messages) > 50:  # More than 50 messages in last 5 seconds
        anomalies.append({
            'type': 'MESSAGE_FLOODING',
            'severity': 'HIGH',
            'description': 'Excessive message rate detected'
        })

    return anomalies
```

---


## 13. Implementation Guidelines

### 13.1 Required Components

Any WIA-COMM-003 compliant system must include:

1. **V2X Radio**: DSRC (802.11p) or C-V2X transceiver
2. **GNSS Receiver**: GPS/GLONASS/Galileo for positioning
3. **Message Processor**: BSM/CAM/DENM generation and parsing
4. **Security Module**: Certificate management and cryptography
5. **Application Layer**: Collision detection, warnings, UI

### 13.2 Software Architecture

```
┌─────────────────────────────────────────────────┐
│              V2X Application Layer               │
│  (Collision Avoidance, Platooning, UI)          │
├─────────────────────────────────────────────────┤
│         V2X Middleware / SDK                     │
│  (Message Handling, Event Processing)            │
├─────────────────────────────────────────────────┤
│         Security & Privacy Layer                 │
│  (Signing, Verification, Anonymization)          │
├─────────────────────────────────────────────────┤
│       V2X Protocol Stack                         │
│  (IEEE 1609.x, ETSI ITS-G5, SAE J2735)          │
├─────────────────────────────────────────────────┤
│       Hardware Abstraction Layer                 │
│  (Radio Control, GNSS, CAN Bus)                  │
└─────────────────────────────────────────────────┘
```

### 13.3 Message Generation

**BSM Generation (10 Hz):**
```typescript
class BSMGenerator {
  private msgCount: number = 0;

  generateBSM(vehicleState: VehicleState): BSM {
    // Increment message counter (wraps at 127)
    this.msgCount = (this.msgCount + 1) % 128;

    return {
      messageType: 'BSM',
      msgCount: this.msgCount,
      id: vehicleState.id,
      timestamp: Date.now() % 65536,  // Milliseconds in minute

      position: {
        latitude: Math.round(vehicleState.position.lat * 1e7),
        longitude: Math.round(vehicleState.position.lon * 1e7),
        elevation: Math.round(vehicleState.position.alt * 10),
        accuracy: this.calculatePositionAccuracy(vehicleState.gnss)
      },

      speed: Math.round(vehicleState.speed / 0.02),  // 0.02 m/s units
      heading: Math.round(vehicleState.heading / 0.0125),  // 0.0125° units

      acceleration: {
        longitudinal: Math.round(vehicleState.acceleration.x / 0.01),
        lateral: Math.round(vehicleState.acceleration.y / 0.01),
        vertical: Math.round(vehicleState.acceleration.z / 0.02),
        yawRate: Math.round(vehicleState.yawRate / 0.01)
      },

      steeringAngle: Math.round(vehicleState.steeringAngle / 1.5),

      brakeStatus: {
        wheelBrakes: this.encodeWheelBrakes(vehicleState.brakes),
        tractionControl: vehicleState.tractionControl ? 'on' : 'off',
        abs: vehicleState.abs ? 'on' : 'off',
        stabilityControl: vehicleState.esc ? 'on' : 'off',
        brakeBoost: vehicleState.brakeBoost ? 'on' : 'off',
        auxBrakes: vehicleState.auxBrakes ? 'on' : 'off'
      },

      vehicleSize: {
        width: Math.round(vehicleState.width * 100),  // cm
        length: Math.round(vehicleState.length * 100)  // cm
      }
    };
  }
}
```

### 13.4 Performance Optimization

**Message Filtering:**
```typescript
class MessageFilter {
  filterRelevantMessages(
    messages: V2XMessage[],
    egoVehicle: VehicleState,
    maxDistance: number = 500
  ): V2XMessage[] {
    return messages.filter(msg => {
      // Distance filter
      const distance = this.calculateDistance(
        egoVehicle.position,
        msg.position
      );
      if (distance > maxDistance) return false;

      // Direction filter (only vehicles in front or nearby)
      const bearing = this.calculateBearing(
        egoVehicle.position,
        msg.position
      );
      const headingDiff = Math.abs(bearing - egoVehicle.heading);

      // Keep if within ±90° or very close (<50m)
      if (headingDiff > 90 && distance > 50) return false;

      // Keep message
      return true;
    });
  }
}
```

### 13.5 Testing and Validation

**Test Scenarios:**

1. **Basic Communication**
   - Message transmission and reception
   - Latency measurement
   - Packet delivery ratio

2. **Collision Avoidance**
   - Forward collision warning
   - Blind spot detection
   - Intersection collision

3. **Platooning**
   - Formation and maintenance
   - Lane change maneuvers
   - Emergency dissolution

4. **Security**
   - Message authentication
   - Certificate validation
   - Misbehavior detection

**Field Testing Requirements:**
```
Test Vehicles: Minimum 5 vehicles
Test Duration: 100+ hours
Test Scenarios: 50+ different scenarios
Environments: Urban, highway, rural
Weather Conditions: Clear, rain, fog
Traffic Densities: Light, medium, heavy
```

---


## 14. References

### 14.1 Standards Documents

1. **IEEE 1609.x** - Wireless Access in Vehicular Environments (WAVE)
   - 1609.1: Resource Manager
   - 1609.2: Security Services
   - 1609.3: Networking Services
   - 1609.4: Multi-Channel Operation

2. **SAE J2735** - Dedicated Short Range Communications Message Set Dictionary

3. **SAE J2945** - Dedicated Short Range Communications On-Board System Requirements

4. **ETSI EN 302 637** - Intelligent Transport Systems (ITS); Vehicular Communications
   - Part 2: Cooperative Awareness (CAM)
   - Part 3: Decentralized Environmental Notification (DENM)

5. **3GPP TS 22.185** - Service requirements for V2X services

6. **3GPP TS 23.285** - Architecture enhancements for V2X services

### 14.2 Technical Specifications

| Specification | Description |
|---------------|-------------|
| IEEE 802.11p | Wireless LAN for vehicular environments |
| IEEE 1609.2 | Security services for applications and management messages |
| SAE J2735 | Message set dictionary for DSRC |
| SAE J2945/1 | On-Board System Requirements for V2V Safety |
| ETSI TS 102 894-2 | Basic Set of Applications - Specification |
| ISO 21217 | Station and communication architecture |

### 14.3 WIA Standards

- **WIA-INTENT**: Intent-based vehicle control
- **WIA-OMNI-API**: Universal API gateway
- **WIA-SOCIAL**: Social coordination
- **WIA-QUANTUM**: Quantum-secure communication
- **WIA-EDGE**: Edge computing for V2X

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-COMM-003 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

