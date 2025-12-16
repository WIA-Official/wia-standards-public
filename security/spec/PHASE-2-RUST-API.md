# WIA Security Phase 2: Rust API Implementation

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2024-12-16

## Overview

Phase 2 implements multi-language SDKs for the WIA Security Standard, providing type-safe APIs for creating, validating, and processing security events.

## Table of Contents

1. [Rust SDK](#1-rust-sdk)
2. [Python SDK](#2-python-sdk)
3. [TypeScript SDK](#3-typescript-sdk)
4. [Common Patterns](#4-common-patterns)

---

## 1. Rust SDK

### 1.1 Core Types

Located in `security/api/rust/src/types.rs`:

```rust
/// Source of security event
pub struct Source {
    pub source_type: SourceType,
    pub name: String,
    pub version: Option<String>,
}

/// Security alert
pub struct Alert {
    pub source: Source,
    pub alert_id: String,
    pub title: String,
    pub category: String,
    pub status: String,
    pub priority: String,
    pub severity: f64,
    pub timestamp: DateTime<Utc>,
    // ... additional fields
}

/// Security incident
pub struct Incident {
    pub incident_id: String,
    pub title: String,
    pub status: IncidentStatus,
    pub severity: Severity,
    pub alerts: Vec<Alert>,
    // ... additional fields
}

/// Vulnerability information
pub struct Vulnerability {
    pub cve_id: String,
    pub cvss: CvssScore,
    pub affected_products: Vec<AffectedProduct>,
    // ... additional fields
}
```

### 1.2 Builder Pattern

Located in `security/api/rust/src/builder.rs`:

```rust
use wia_security::{AlertBuilder, Source, SourceType};

let alert = AlertBuilder::new()
    .source(Source::new(SourceType::Siem, "Splunk"))
    .alert_id("ALERT-001")
    .title("Suspicious Activity Detected")
    .category("malware")
    .status("new")
    .priority("high")
    .severity(7.5)
    .build()?;
```

### 1.3 Validator

Located in `security/api/rust/src/validator.rs`:

```rust
use wia_security::{validate_event, ValidationResult};

let result = validate_event(&alert);
if result.is_valid() {
    println!("Event is valid");
} else {
    for error in result.errors() {
        println!("Validation error: {}", error);
    }
}
```

### 1.4 Converter

Located in `security/api/rust/src/converter.rs`:

Supports format conversions:
- WIA ↔ STIX 2.1
- WIA ↔ ECS (Elastic Common Schema)
- WIA ↔ OCSF
- WIA ↔ CEF (Common Event Format)

```rust
use wia_security::converter::{to_stix, to_ecs, from_stix};

// Convert to STIX
let stix_bundle = to_stix(&alert)?;

// Convert to ECS
let ecs_doc = to_ecs(&alert)?;

// Import from STIX
let wia_alert = from_stix(&stix_indicator)?;
```

### 1.5 Client

Located in `security/api/rust/src/client.rs`:

```rust
use wia_security::client::{SecurityClient, ClientConfig};

let client = SecurityClient::new(ClientConfig {
    base_url: "https://api.example.com".to_string(),
    api_key: Some("your-api-key".to_string()),
    timeout: Duration::from_secs(30),
})?;

// Submit alert
let response = client.submit_alert(&alert).await?;

// Query incidents
let incidents = client.query_incidents(QueryParams {
    status: Some(IncidentStatus::Open),
    severity: Some(Severity::High),
    limit: 100,
}).await?;
```

---

## 2. Python SDK

### 2.1 Installation

```bash
pip install wia-security
```

### 2.2 Core Types

```python
from wia_security import Alert, Incident, Vulnerability, Source, SourceType

# Create source
source = Source(
    source_type=SourceType.SIEM,
    name="Splunk",
    version="9.0"
)

# Create alert
alert = Alert(
    source=source,
    alert_id="ALERT-001",
    title="Suspicious Activity",
    category="malware",
    status="new",
    priority="high",
    severity=7.5
)
```

### 2.3 Builder Pattern

```python
from wia_security import AlertBuilder

alert = (AlertBuilder()
    .source(Source(SourceType.SIEM, "Splunk"))
    .alert_id("ALERT-001")
    .title("Suspicious Activity")
    .category("malware")
    .status("new")
    .priority("high")
    .severity(7.5)
    .build())
```

### 2.4 Validation

```python
from wia_security import validate_event

result = validate_event(alert)
if result.is_valid:
    print("Valid event")
else:
    for error in result.errors:
        print(f"Error: {error}")
```

### 2.5 Conversion

```python
from wia_security.converter import to_stix, to_ecs, from_stix

# Convert to STIX 2.1
stix_bundle = to_stix(alert)

# Convert to ECS
ecs_doc = to_ecs(alert)
```

---

## 3. TypeScript SDK

### 3.1 Installation

```bash
npm install @wia/security
```

### 3.2 Core Types

```typescript
import { Alert, Incident, Source, SourceType } from '@wia/security';

const source: Source = {
  sourceType: SourceType.SIEM,
  name: 'Splunk',
  version: '9.0'
};

const alert: Alert = {
  source,
  alertId: 'ALERT-001',
  title: 'Suspicious Activity',
  category: 'malware',
  status: 'new',
  priority: 'high',
  severity: 7.5,
  timestamp: new Date().toISOString()
};
```

### 3.3 Builder Pattern

```typescript
import { AlertBuilder, Source, SourceType } from '@wia/security';

const alert = new AlertBuilder()
  .source(new Source(SourceType.SIEM, 'Splunk'))
  .alertId('ALERT-001')
  .title('Suspicious Activity')
  .category('malware')
  .status('new')
  .priority('high')
  .severity(7.5)
  .build();
```

### 3.4 Validation

```typescript
import { validateEvent, ValidationResult } from '@wia/security';

const result: ValidationResult = validateEvent(alert);
if (result.isValid) {
  console.log('Valid event');
} else {
  result.errors.forEach(error => console.error(error));
}
```

---

## 4. Common Patterns

### 4.1 Event Types

All SDKs support these event types:
- `Alert` - Security alerts from SIEM/detection systems
- `Incident` - Security incidents with related alerts
- `Vulnerability` - CVE/CVSS vulnerability data
- `ThreatIntel` - STIX 2.1 threat intelligence
- `NetworkEvent` - Network traffic events
- `EndpointEvent` - Endpoint security events
- `AuthEvent` - Authentication/authorization events

### 4.2 Severity Levels

| Level | Numeric Range | Description |
|-------|---------------|-------------|
| Critical | 9.0 - 10.0 | Immediate action required |
| High | 7.0 - 8.9 | Urgent attention needed |
| Medium | 4.0 - 6.9 | Should be addressed soon |
| Low | 0.1 - 3.9 | Informational, low risk |
| Info | 0.0 | Informational only |

### 4.3 Status Values

**Alert Status:**
- `new` - Newly created
- `in_progress` - Being investigated
- `resolved` - Issue resolved
- `closed` - No action needed
- `false_positive` - Determined to be benign

**Incident Status:**
- `open` - Active incident
- `investigating` - Under investigation
- `contained` - Threat contained
- `remediated` - Issue fixed
- `closed` - Incident resolved

### 4.4 MITRE ATT&CK Integration

```rust
use wia_security::types::{MitreAttack, Tactic, Technique};

let attack = MitreAttack {
    tactics: vec![Tactic {
        id: "TA0001".to_string(),
        name: "Initial Access".to_string(),
    }],
    techniques: vec![Technique {
        id: "T1566".to_string(),
        name: "Phishing".to_string(),
        subtechniques: vec!["T1566.001".to_string()],
    }],
};
```

---

## API Reference

### Rust Crate

```toml
[dependencies]
wia-security = "1.0"
```

### Python Package

```
wia-security>=1.0.0
```

### NPM Package

```json
{
  "dependencies": {
    "@wia/security": "^1.0.0"
  }
}
```

---

## Implementation Status

| Component | Rust | Python | TypeScript |
|-----------|------|--------|------------|
| Types | ✅ | ✅ | ✅ |
| Builder | ✅ | ✅ | ✅ |
| Validator | ✅ | ✅ | ✅ |
| Converter | ✅ | ✅ | ✅ |
| Client | ✅ | ✅ | ✅ |
