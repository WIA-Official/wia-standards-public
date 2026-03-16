# WIA-IND-001: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document defines the communication protocols for virtual fitting, data exchange, and real-time interactions in the WIA-IND-001 Fashion Technology Standard. These protocols ensure consistent behavior across implementations while maintaining security, privacy, and performance.

## 2. Virtual Fitting Protocol

### 2.1 Session Establishment

**Protocol Flow:**
```
Client → Server: FITTING_REQUEST
Server → Client: SESSION_CREATED
Server → Client: AVATAR_READY
Server → Client: GARMENT_LOADED
Server → Client: SIMULATION_READY
Client ↔ Server: INTERACTIVE_SESSION
```

**FITTING_REQUEST Message:**
```json
{
  "protocol": "WIA-IND-001-VirtualFitting/1.0",
  "messageType": "FITTING_REQUEST",
  "messageId": "uuid",
  "timestamp": "ISO-8601",
  "payload": {
    "userId": "string",
    "measurementId": "string",
    "productId": "string",
    "size": "string",
    "renderQuality": "enum: low|medium|high|ultra",
    "features": {
      "physics": "boolean",
      "rayTracing": "boolean",
      "animation": "boolean"
    }
  },
  "philosophy": "弘益人間"
}
```

**SESSION_CREATED Response:**
```json
{
  "protocol": "WIA-IND-001-VirtualFitting/1.0",
  "messageType": "SESSION_CREATED",
  "messageId": "uuid",
  "inReplyTo": "request-message-id",
  "timestamp": "ISO-8601",
  "payload": {
    "sessionId": "string",
    "wsEndpoint": "wss://fitting.example.com/session/{sessionId}",
    "httpEndpoint": "https://fitting.example.com/session/{sessionId}",
    "expiresAt": "ISO-8601",
    "capabilities": {
      "maxResolution": "4096x4096",
      "frameRate": 60,
      "formats": ["png", "jpg", "webp", "mp4"]
    }
  }
}
```

### 2.2 Real-Time Interaction Protocol (WebSocket)

**Connection:**
```
wss://fitting.example.com/session/{sessionId}?token={auth-token}
```

**Control Messages:**

**Rotate Avatar:**
```json
{
  "action": "ROTATE",
  "params": {
    "axis": "y",
    "angle": 45,
    "duration": 500
  }
}
```

**Change Size:**
```json
{
  "action": "CHANGE_SIZE",
  "params": {
    "size": "L"
  }
}
```

**Adjust Fit:**
```json
{
  "action": "ADJUST_FIT",
  "params": {
    "adjustment": "tighter|looser",
    "amount": 0.1
  }
}
```

**Change Pose:**
```json
{
  "action": "CHANGE_POSE",
  "params": {
    "pose": "enum: neutral|walking|arms-raised|sitting"
  }
}
```

**Capture Frame:**
```json
{
  "action": "CAPTURE",
  "params": {
    "format": "png",
    "quality": 95,
    "resolution": "1920x1080"
  }
}
```

**Server Updates:**
```json
{
  "updateType": "RENDER_COMPLETE",
  "timestamp": "ISO-8601",
  "data": {
    "imageUrl": "https://cdn.example.com/...",
    "fitAnalysis": {
      "shoulders": "perfect",
      "chest": "slightly-tight",
      "waist": "good"
    }
  }
}
```

### 2.3 Physics Simulation Protocol

**Cloth Simulation Parameters:**
```json
{
  "protocol": "WIA-IND-001-ClothPhysics/1.0",
  "material": {
    "type": "cotton",
    "stiffness": 0.3,
    "damping": 0.05,
    "friction": 0.4,
    "stretch": {
      "horizontal": 0.1,
      "vertical": 0.05
    }
  },
  "simulation": {
    "method": "mass-spring|finite-element",
    "timeStep": 0.016,
    "iterations": 10,
    "gravity": -9.81
  },
  "collision": {
    "bodyMesh": "avatar-mesh-id",
    "selfCollision": true,
    "collisionMargin": 0.5
  }
}
```

## 3. Data Exchange Protocol

### 3.1 Measurement Synchronization

**Sync Request:**
```json
{
  "protocol": "WIA-IND-001-Sync/1.0",
  "operation": "SYNC_MEASUREMENTS",
  "source": {
    "deviceId": "device-123",
    "timestamp": "ISO-8601",
    "version": "hash-or-version"
  },
  "measurements": [
    {
      "id": "meas-1",
      "operation": "CREATE|UPDATE|DELETE",
      "data": { ... },
      "timestamp": "ISO-8601"
    }
  ]
}
```

**Sync Response:**
```json
{
  "protocol": "WIA-IND-001-Sync/1.0",
  "status": "SUCCESS",
  "conflicts": [
    {
      "measurementId": "meas-1",
      "reason": "version-mismatch",
      "serverVersion": { ... },
      "clientVersion": { ... },
      "resolution": "server-wins|client-wins|merge"
    }
  ],
  "serverUpdates": [
    // Measurements newer on server
  ]
}
```

### 3.2 Encryption Protocol

**End-to-End Encryption:**
```
Algorithm: AES-256-GCM
Key Exchange: ECDH (Curve25519)
Authentication: HMAC-SHA256
```

**Encrypted Payload:**
```json
{
  "protocol": "WIA-IND-001-Secure/1.0",
  "encrypted": true,
  "algorithm": "AES-256-GCM",
  "keyId": "key-identifier",
  "iv": "base64-encoded-iv",
  "ciphertext": "base64-encoded-encrypted-data",
  "tag": "base64-encoded-auth-tag"
}
```

## 4. Privacy Protocol

### 4.1 Consent Management

**Consent Request:**
```json
{
  "protocol": "WIA-IND-001-Privacy/1.0",
  "requestType": "CONSENT",
  "purposes": [
    {
      "id": "size-recommendation",
      "description": "Use measurements for size recommendations",
      "required": true,
      "retention": "P365D"
    },
    {
      "id": "analytics-aggregate",
      "description": "Include anonymized data in aggregate analytics",
      "required": false,
      "retention": "P730D"
    }
  ]
}
```

**Consent Response:**
```json
{
  "protocol": "WIA-IND-001-Privacy/1.0",
  "consents": [
    {
      "purposeId": "size-recommendation",
      "granted": true,
      "timestamp": "ISO-8601",
      "signature": "digital-signature"
    },
    {
      "purposeId": "analytics-aggregate",
      "granted": false
    }
  ]
}
```

### 4.2 Data Portability

**Export Request:**
```json
{
  "protocol": "WIA-IND-001-Privacy/1.0",
  "operation": "DATA_EXPORT",
  "userId": "user-12345",
  "format": "json|xml|csv",
  "includeHistory": true
}
```

**Export Package:**
```json
{
  "exportDate": "ISO-8601",
  "userId": "user-12345",
  "measurements": [ ... ],
  "fitPreferences": { ... },
  "purchaseHistory": [ ... ],
  "recommendations": [ ... ],
  "philosophy": "弘益人間"
}
```

### 4.3 Right to Deletion

**Deletion Request:**
```json
{
  "protocol": "WIA-IND-001-Privacy/1.0",
  "operation": "DELETE_USER_DATA",
  "userId": "user-12345",
  "scope": "all|measurements|preferences|history",
  "confirmed": true
}
```

## 5. QR Code Protocol

### 5.1 QR Code Data Format

**Measurement QR Code:**
```
wia://ind-001/measurement/{encrypted-payload}

Decrypted Payload:
{
  "version": "1.0",
  "type": "measurement-profile",
  "data": {
    "height": 165.5,
    "chest": 88.0,
    "waist": 68.5,
    "hip": 94.0
  },
  "validUntil": "ISO-8601",
  "signature": "digital-signature"
}
```

**Product Link QR Code:**
```
wia://ind-001/product/{productId}?size={recommended-size}&confidence={score}
```

### 5.2 Verifiable Credentials

**Credential Structure (JSON-LD):**
```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/standards/IND-001/v1"
  ],
  "type": ["VerifiableCredential", "BodyMeasurementCredential"],
  "issuer": "did:wia:issuer-id",
  "issuanceDate": "ISO-8601",
  "expirationDate": "ISO-8601",
  "credentialSubject": {
    "id": "did:wia:user-12345",
    "measurements": {
      "height": 165.5,
      "chest": 88.0,
      "waist": 68.5,
      "hip": 94.0
    },
    "verificationMethod": "AI-3D-Scan",
    "accuracy": "±0.5cm"
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "ISO-8601",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:issuer-id#key-1",
    "proofValue": "base58-encoded-signature"
  },
  "philosophy": "弘益人間"
}
```

## 6. Analytics Protocol

### 6.1 Event Tracking

**Event Schema:**
```json
{
  "protocol": "WIA-IND-001-Analytics/1.0",
  "eventType": "size_recommendation_generated|virtual_fitting_started|purchase_completed",
  "timestamp": "ISO-8601",
  "sessionId": "string",
  "userId": "string (hashed)",
  "properties": {
    "productId": "string",
    "recommendedSize": "string",
    "confidence": 0.92,
    "accepted": true
  },
  "context": {
    "platform": "web|ios|android",
    "referrer": "string",
    "userAgent": "string"
  }
}
```

### 6.2 Privacy-Preserving Analytics

**Differential Privacy:**
- Add noise: Laplace(sensitivity/epsilon)
- Epsilon: 0.1 (strong privacy) to 1.0 (moderate privacy)
- K-anonymity: Minimum group size of 5

## 7. Performance Protocol

### 7.1 Quality of Service

**Latency Requirements:**
- API Response: < 200ms (p95)
- Size Recommendation: < 500ms
- Virtual Fitting Session Start: < 2s
- Render Generation: < 5s (high quality)
- WebSocket Message: < 50ms

**Throughput:**
- API: 1000 req/s per instance
- WebSocket: 100 concurrent sessions per instance
- Renders: 20 renders/s per GPU

### 7.2 Progressive Loading

**Multi-Resolution Strategy:**
```json
{
  "renders": {
    "thumbnail": "256x256 (loads in 100ms)",
    "preview": "512x512 (loads in 300ms)",
    "standard": "1920x1080 (loads in 1s)",
    "high": "3840x2160 (loads in 3s)"
  }
}
```

## 8. Error Recovery Protocol

### 8.1 Retry Strategy

**Exponential Backoff:**
```
Attempt 1: Immediate
Attempt 2: 1s delay
Attempt 3: 2s delay
Attempt 4: 4s delay
Attempt 5: 8s delay
Max Attempts: 5
```

### 8.2 Graceful Degradation

**Quality Levels:**
```
Level 1 (Preferred): Full 3D rendering with physics
Level 2 (Degraded): Static 3D renders without physics
Level 3 (Minimal): 2D overlay on photo
Level 4 (Fallback): Size recommendation only
```

## 9. Internationalization Protocol

### 9.1 Language Support

**Language Header:**
```http
Accept-Language: ko-KR,ko;q=0.9,en;q=0.8
```

**Response:**
```json
{
  "recommendation": {
    "reasoning": {
      "en": "Based on chest 88cm, waist 68.5cm",
      "ko": "가슴 88cm, 허리 68.5cm 기준",
      "ja": "胸囲88cm、ウエスト68.5cmに基づく"
    }
  }
}
```

### 9.2 Unit Preferences

**User Preferences:**
```json
{
  "units": {
    "length": "metric|imperial",
    "weight": "metric|imperial",
    "temperature": "celsius|fahrenheit"
  },
  "locale": "ko-KR",
  "timezone": "Asia/Seoul"
}
```

## 10. Compliance Protocol

### 10.1 GDPR Compliance

- Right to access: Data export API
- Right to rectification: Update endpoints
- Right to erasure: Deletion API
- Data portability: Export in machine-readable format
- Consent management: Granular consent tracking

### 10.2 Audit Logging

**Audit Log Entry:**
```json
{
  "timestamp": "ISO-8601",
  "actor": "user-id or system",
  "action": "READ|WRITE|DELETE",
  "resource": "measurement/meas-123",
  "outcome": "SUCCESS|FAILURE",
  "ipAddress": "hashed",
  "userAgent": "string"
}
```

---

**Copyright © 2025 SmileStory Inc. / WIA**
**License:** CC BY 4.0
**弘益人間 - Benefit All Humanity**
