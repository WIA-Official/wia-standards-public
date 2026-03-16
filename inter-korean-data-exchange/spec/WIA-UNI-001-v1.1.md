# WIA-UNI-001 Specification v1.1

**Inter-Korean Data Exchange Standard**
**남북한 데이터 교환 표준**

---

## Document Information

- **Standard ID**: WIA-UNI-001
- **Version**: 1.1.0
- **Status**: Stable
- **Published**: 2024-06-20
- **Category**: UNI (Unification/Peace)
- **Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## Changes from v1.0

### New Features

1. **Group Messaging** - Support for family group chats (up to 20 members)
2. **Live Video Calls** - Real-time video conferencing for families
3. **Medical Records API** - Standardized format for cross-border patient data
4. **Cultural Heritage Protocol** - Special handling for historical documents

### Enhancements

- Increased video quality support (up to 4K for video calls)
- Improved Korean dialect translation (95% accuracy)
- Faster message delivery (average 847ms, down from 1.2s)
- Enhanced mobile support (iOS/Android native SDKs)

### Security Updates

- Added post-quantum cryptography preparedness (CRYSTALS-Kyber)
- Improved certificate pinning mechanisms
- Enhanced rate limiting and DDoS protection
- Biometric authentication support

---

## 1. Group Messaging

### 1.1 Group Creation

Family groups can include:
- Up to 20 members
- Members from both North and South
- Multi-generational families
- Red Cross facilitators (optional, read-only)

### 1.2 Group Management

```json
{
  "groupId": "uuid-v4",
  "name": "Kim Family Reunion",
  "members": [
    {"userId": "...", "region": "south", "role": "admin"},
    {"userId": "...", "region": "north", "role": "member"}
  ],
  "created": "2024-06-20T10:00:00Z",
  "humanitarian": true,
  "redCrossObserver": "optional-facilitator-id"
}
```

---

## 2. Live Video Calls

### 2.1 Technical Specifications

- **Protocol**: WebRTC over TURN/STUN servers
- **Codec**: VP9 or H.265
- **Resolution**: Up to 4K (3840x2160)
- **Frame Rate**: 30 fps standard, 60 fps for high-quality
- **Bitrate**: Adaptive (500 kbps - 20 Mbps)

### 2.2 Quality of Service

- Priority routing for family video calls
- Automatic quality adjustment based on bandwidth
- Recording capability (with consent)
- Screen sharing for document review

---

## 3. Medical Records API

### 3.1 Standardized Format

Based on HL7 FHIR (Fast Healthcare Interoperability Resources):

```json
{
  "resourceType": "Patient",
  "identifier": "encrypted-patient-id",
  "name": {"family": "Kim", "given": "Chol-su"},
  "birthDate": "1950-03-15",
  "medicalHistory": {
    "conditions": [...],
    "medications": [...],
    "allergies": [...]
  },
  "emergencyContact": {
    "name": "Kim Min-ji",
    "relationship": "sister",
    "region": "south",
    "phone": "encrypted"
  }
}
```

### 3.2 Use Cases

- Cross-border medical treatment
- Emergency medical information sharing
- Chronic disease management
- Family medical history compilation

---

## 4. Cultural Heritage Protocol

### 4.1 Document Types Supported

- Historical photographs (up to 500 MB)
- Ancient manuscripts (high-resolution scans)
- Audio recordings (traditional music, oral histories)
- Video documentation (cultural ceremonies, interviews)

### 4.2 Preservation Requirements

- Minimum 300 DPI for photographs
- Lossless compression (TIFF, FLAC)
- Metadata preservation (EXIF, XMP)
- Long-term archival (100+ years)

---

## 5. Performance Improvements

### 5.1 Metrics

| Metric | v1.0 | v1.1 | Improvement |
|--------|------|------|-------------|
| Avg latency | 1200ms | 847ms | 29% faster |
| Throughput | 100/sec | 250/sec | 150% increase |
| Uptime | 99.9% | 99.99% | 10x reduction in downtime |
| Mobile support | Basic | Native | Full platform support |

---

## 6. Security Enhancements

### 6.1 Post-Quantum Cryptography

Hybrid approach for quantum resistance:
- Classical: ECDH P-256
- Post-Quantum: CRYSTALS-Kyber-768
- Combined key derivation for double protection

### 6.2 Biometric Authentication

Support for:
- Fingerprint (Touch ID, Android Fingerprint)
- Face recognition (Face ID, Android Face Unlock)
- Voice verification (for elderly users)

---

## 7. Mobile SDKs

### 7.1 iOS

```swift
import WIAInterKoreanExchange

let exchange = WIAExchange(
    trustAnchors: [.rok, .dprk, .un, .icrc],
    environment: .production
)

// Send message
exchange.sendMessage(
    to: recipient,
    content: "Hello...",
    type: .familyReunification
) { result in
    // Handle result
}
```

### 7.2 Android

```kotlin
import org.wia.interkorean.Exchange

val exchange = Exchange.Builder()
    .trustAnchors(TrustAnchor.ALL)
    .environment(Environment.PRODUCTION)
    .build()

// Send message
exchange.sendMessage(
    recipient = recipient,
    content = "Hello...",
    type = MessageType.FAMILY_REUNIFICATION
)
```

---

## 8. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.1.0 | 2024-06-20 | Group messaging, live video, medical API, performance improvements |
| 1.0.0 | 2024-01-15 | Initial release |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
Published under Creative Commons Attribution 4.0 International License
