# WIA Clinical Decision Support Standard - Phase 3: Communication Protocol Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-015

---

## 1. Overview

Phase 3 defines the communication protocols for Clinical Decision Support systems. This includes real-time alert delivery, EHR integration patterns, and secure message exchange between healthcare systems.

### 1.1 Design Principles

- **Real-time**: Minimal latency for clinical alerts
- **Reliable**: Guaranteed delivery for critical alerts
- **Secure**: End-to-end encryption for patient data
- **Interoperable**: Compatible with HL7 FHIR and CDS Hooks
- **Auditable**: Complete message tracking for compliance

---

## 2. Transport Protocols

### 2.1 Supported Protocols

| Protocol | Use Case | Latency | Reliability |
|----------|----------|---------|-------------|
| HTTPS REST | Standard API calls | Low | High |
| WebSocket | Real-time alerts | Very Low | High |
| HL7 FHIR | EHR integration | Medium | High |
| SMART on FHIR | App authorization | N/A | High |

### 2.2 WebSocket Connection

```javascript
// Connect to real-time CDS alert stream
const ws = new WebSocket('wss://api.provider.com/wia/cds/v1/ws');

ws.onopen = () => {
    // Subscribe to patient alerts
    ws.send(JSON.stringify({
        type: 'subscribe',
        channels: ['patient-alerts', 'critical-findings'],
        context: {
            institutionId: 'hospital-xyz',
            userId: 'physician-12345'
        }
    }));
};

ws.onmessage = (event) => {
    const alert = JSON.parse(event.data);
    
    if (alert.severity === 'critical') {
        // Display interruptive alert
        displayCriticalAlert(alert);
    }
};
```

### 2.3 Message Format

```json
{
    "messageId": "msg-12345-abcde",
    "messageType": "cds_alert",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "version": "1.0.0",
    "source": {
        "system": "wia-cds",
        "institution": "hospital-xyz"
    },
    "destination": {
        "userId": "physician-12345",
        "deviceId": "workstation-789"
    },
    "payload": {
        "alert": {...},
        "context": {...}
    },
    "security": {
        "encrypted": true,
        "signature": "sha256=..."
    }
}
```

---

## 3. CDS Hooks Protocol

### 3.1 Hook Lifecycle

```
┌─────────────────┐
│   EHR System    │
└────────┬────────┘
         │
         ▼ (1) Hook Event Triggered
┌─────────────────┐
│  CDS Hooks      │
│  Request        │
└────────┬────────┘
         │
         ▼ (2) POST to CDS Service
┌─────────────────┐
│  WIA CDS        │
│  Service        │
└────────┬────────┘
         │
         ▼ (3) Return Cards
┌─────────────────┐
│  CDS Hooks      │
│  Response       │
└────────┬────────┘
         │
         ▼ (4) Display Cards
┌─────────────────┐
│   EHR System    │
└─────────────────┘
```

### 3.2 Supported Hooks

| Hook | Trigger | Use Cases |
|------|---------|-----------|
| patient-view | Chart opened | Preventive care, health maintenance |
| medication-prescribe | Medication ordered | Drug interactions, dose checking |
| order-select | Order selected | Duplicate detection, appropriateness |
| order-sign | Order signed | Final safety check |
| encounter-start | Encounter begins | Relevant alerts, history reminders |
| encounter-discharge | Discharge initiated | Medication reconciliation |

### 3.3 Prefetch Templates

```json
{
    "prefetch": {
        "patient": "Patient/{{context.patientId}}",
        "conditions": "Condition?patient={{context.patientId}}&clinical-status=active",
        "medications": "MedicationRequest?patient={{context.patientId}}&status=active",
        "allergies": "AllergyIntolerance?patient={{context.patientId}}&clinical-status=active",
        "labs": "Observation?patient={{context.patientId}}&category=laboratory&_sort=-date&_count=20"
    }
}
```

---

## 4. FHIR Integration

### 4.1 Required FHIR Resources

| Resource | Purpose |
|----------|---------|
| Patient | Patient demographics |
| MedicationRequest | Current and ordered medications |
| AllergyIntolerance | Known allergies |
| Condition | Active diagnoses |
| Observation | Lab results, vitals |
| Practitioner | Ordering provider |
| Encounter | Current visit context |

### 4.2 FHIR Bundle Request

```json
{
    "resourceType": "Bundle",
    "type": "batch",
    "entry": [
        {
            "request": {
                "method": "GET",
                "url": "Patient/67890"
            }
        },
        {
            "request": {
                "method": "GET",
                "url": "MedicationRequest?patient=67890&status=active"
            }
        },
        {
            "request": {
                "method": "GET",
                "url": "AllergyIntolerance?patient=67890"
            }
        }
    ]
}
```

### 4.3 SMART on FHIR Authorization

```json
{
    "authorization_endpoint": "https://ehr.hospital.com/authorize",
    "token_endpoint": "https://ehr.hospital.com/token",
    "scopes": [
        "patient/*.read",
        "user/*.read",
        "launch/patient",
        "launch/encounter",
        "openid",
        "fhirUser"
    ],
    "capabilities": [
        "launch-ehr",
        "context-ehr-patient",
        "context-ehr-encounter",
        "sso-openid-connect"
    ]
}
```

---

## 5. Alert Delivery Protocol

### 5.1 Alert Priority Levels

| Priority | Delivery | Acknowledgment | Timeout |
|----------|----------|----------------|---------|
| P1 (Critical) | Immediate + Audio | Required | None |
| P2 (High) | Immediate | Required | 5 min |
| P3 (Medium) | Queue | Optional | 30 min |
| P4 (Low) | Queue | Optional | 4 hours |
| P5 (Info) | Background | None | 24 hours |

### 5.2 Alert Acknowledgment Protocol

```json
{
    "messageType": "alert_acknowledgment",
    "alertId": "alert-67890",
    "timestamp": "2025-01-15T14:30:15.000Z",
    "userId": "physician-12345",
    "action": "override",
    "reason": {
        "code": "patient-aware",
        "display": "Patient aware of risk and agrees to proceed",
        "freeText": "Discussed risks with patient, documented in note"
    },
    "signature": {
        "userId": "physician-12345",
        "timestamp": "2025-01-15T14:30:15.000Z",
        "method": "password"
    }
}
```

### 5.3 Alert Escalation

```
Time 0:    Alert delivered to ordering physician
Time 5m:   If P1/P2 not acknowledged → Escalate to supervisor
Time 15m:  If still not acknowledged → Escalate to department head
Time 30m:  If still not acknowledged → Escalate to CMO + auto-hold order
```

---

## 6. Security Protocol

### 6.1 Encryption Requirements

| Data Type | At Rest | In Transit |
|-----------|---------|------------|
| Patient identifiers | AES-256 | TLS 1.3 |
| Clinical data | AES-256 | TLS 1.3 |
| Alert content | AES-256 | TLS 1.3 + E2EE |
| Audit logs | AES-256 | TLS 1.3 |

### 6.2 Message Signing

```python
import hmac
import hashlib
import json

def sign_message(message: dict, secret: str) -> str:
    """Generate HMAC signature for message."""
    payload = json.dumps(message, sort_keys=True)
    signature = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return f"sha256={signature}"

def verify_signature(message: dict, signature: str, secret: str) -> bool:
    """Verify message signature."""
    expected = sign_message(message, secret)
    return hmac.compare_digest(expected, signature)
```

### 6.3 Access Control

```json
{
    "permissions": {
        "physician": [
            "view_alerts",
            "acknowledge_alerts",
            "override_alerts",
            "view_guidelines"
        ],
        "nurse": [
            "view_alerts",
            "acknowledge_alerts",
            "escalate_alerts"
        ],
        "pharmacist": [
            "view_medication_alerts",
            "acknowledge_medication_alerts",
            "override_medication_alerts"
        ],
        "administrator": [
            "configure_rules",
            "view_analytics",
            "manage_overrides"
        ]
    }
}
```

---

## 7. Audit Trail Protocol

### 7.1 Audit Event Schema

```json
{
    "auditEventId": "audit-12345",
    "timestamp": "2025-01-15T14:30:15.000Z",
    "eventType": "alert_override",
    "actor": {
        "userId": "physician-12345",
        "role": "attending_physician",
        "institution": "hospital-xyz"
    },
    "patient": {
        "patientId": "encrypted-patient-xyz",
        "encounterId": "encounter-789"
    },
    "action": {
        "alertId": "alert-67890",
        "alertType": "drug_drug_interaction",
        "severity": "high",
        "decision": "override",
        "reason": "patient-aware"
    },
    "context": {
        "ehrSystem": "Epic",
        "deviceId": "workstation-789",
        "ipAddress": "10.0.1.100"
    }
}
```

### 7.2 Required Audit Events

| Event | Trigger | Retention |
|-------|---------|-----------|
| Alert generated | CDS rule fires | 7 years |
| Alert displayed | Alert shown to user | 7 years |
| Alert acknowledged | User acknowledges | 7 years |
| Alert overridden | User overrides | 7 years |
| Rule modified | Configuration change | Permanent |
| Access granted | Permission change | 7 years |

---

## 8. Retry and Failure Handling

### 8.1 Retry Policy

```json
{
    "retryPolicy": {
        "maxRetries": 3,
        "initialDelay": 1000,
        "maxDelay": 30000,
        "backoffMultiplier": 2,
        "retryableErrors": [
            "TIMEOUT",
            "SERVICE_UNAVAILABLE",
            "RATE_LIMITED"
        ],
        "nonRetryableErrors": [
            "UNAUTHORIZED",
            "FORBIDDEN",
            "INVALID_INPUT"
        ]
    }
}
```

### 8.2 Circuit Breaker

```python
class CircuitBreaker:
    def __init__(self, failure_threshold=5, recovery_timeout=60):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failures = 0
        self.last_failure_time = None
        self.state = "CLOSED"
    
    def call(self, func, *args, **kwargs):
        if self.state == "OPEN":
            if time.time() - self.last_failure_time > self.recovery_timeout:
                self.state = "HALF_OPEN"
            else:
                raise CircuitOpenError("Circuit breaker is open")
        
        try:
            result = func(*args, **kwargs)
            self.on_success()
            return result
        except Exception as e:
            self.on_failure()
            raise e
    
    def on_success(self):
        self.failures = 0
        self.state = "CLOSED"
    
    def on_failure(self):
        self.failures += 1
        self.last_failure_time = time.time()
        if self.failures >= self.failure_threshold:
            self.state = "OPEN"
```

### 8.3 Fallback Behavior

| Scenario | Fallback Action |
|----------|-----------------|
| CDS service unavailable | Display cached rules, log for later processing |
| Network timeout | Retry with backoff, alert user after 3 failures |
| Invalid patient data | Return warning, allow order with documentation |
| Database unavailable | Use read replica, queue writes |

---

## 9. Performance Requirements

### 9.1 Latency SLAs

| Operation | P50 | P95 | P99 |
|-----------|-----|-----|-----|
| Drug interaction check | 50ms | 100ms | 200ms |
| Allergy check | 30ms | 50ms | 100ms |
| Dose validation | 20ms | 40ms | 80ms |
| Guideline lookup | 100ms | 200ms | 500ms |

### 9.2 Throughput Requirements

| Metric | Minimum | Target |
|--------|---------|--------|
| Requests per second | 100 | 1,000 |
| Concurrent connections | 1,000 | 10,000 |
| Alert delivery time | 500ms | 100ms |

---

## 10. Monitoring Protocol

### 10.1 Health Check Endpoint

```json
{
    "endpoint": "GET /wia/cds/v1/health",
    "response": {
        "status": "healthy",
        "version": "1.0.0",
        "components": {
            "database": "healthy",
            "cache": "healthy",
            "drugDatabase": "healthy",
            "guidelineEngine": "healthy"
        },
        "latency": {
            "database": 5,
            "cache": 1,
            "drugDatabase": 15
        }
    }
}
```

### 10.2 Metrics

```prometheus
# Alert metrics
cds_alerts_generated_total{severity="critical",category="drug_safety"} 523
cds_alerts_acknowledged_total{action="override"} 234
cds_alert_latency_seconds{quantile="0.99"} 0.15

# Performance metrics
cds_request_duration_seconds{endpoint="/check/interactions"} 0.05
cds_requests_total{status="200"} 15234
cds_errors_total{code="SERVICE_UNAVAILABLE"} 12
```

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Clinical Decision Support Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
