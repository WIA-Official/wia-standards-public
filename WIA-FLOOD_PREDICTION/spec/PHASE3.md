# WIA-FLOOD_PREDICTION Specification - PHASE 3

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Security, Compliance & Performance

This phase defines security controls, compliance frameworks, performance requirements, disaster response integration, and operational benchmarks for the WIA-FLOOD_PREDICTION system.

---

## Table of Contents

1. [Security Architecture](#security-architecture)
2. [Authentication & Authorization](#authentication--authorization)
3. [Data Security & Privacy](#data-security--privacy)
4. [Compliance & Certifications](#compliance--certifications)
5. [Performance Requirements](#performance-requirements)
6. [Disaster Response Integration](#disaster-response-integration)
7. [Service Level Agreements (SLAs)](#service-level-agreements-slas)

---

## Security Architecture

### Defense in Depth

```
┌─────────────────────────────────────────────────────┐
│  Layer 7: User Education & Awareness                │
├─────────────────────────────────────────────────────┤
│  Layer 6: Application Security (OWASP Top 10)      │
│  - Input validation    - SQL injection prevention   │
│  - XSS protection     - CSRF tokens                 │
├─────────────────────────────────────────────────────┤
│  Layer 5: API Security                              │
│  - OAuth 2.0 + OIDC   - Rate limiting               │
│  - API key rotation   - Request signing             │
├─────────────────────────────────────────────────────┤
│  Layer 4: Transport Security                        │
│  - TLS 1.3           - Certificate pinning          │
│  - HSTS              - Perfect forward secrecy      │
├─────────────────────────────────────────────────────┤
│  Layer 3: Network Security                          │
│  - VPC isolation     - Security groups              │
│  - WAF (Web App FW)  - DDoS protection              │
├─────────────────────────────────────────────────────┤
│  Layer 2: Infrastructure Security                   │
│  - Container scanning - Secrets management          │
│  - Encrypted storage  - Audit logging               │
├─────────────────────────────────────────────────────┤
│  Layer 1: Physical Security                         │
│  - Data center access - Redundant power/cooling     │
│  - Biometric auth     - 24/7 surveillance           │
└─────────────────────────────────────────────────────┘
```

### Security Principles

1. **Zero Trust**: Never trust, always verify
2. **Least Privilege**: Minimum necessary permissions
3. **Defense in Depth**: Multiple security layers
4. **Fail Secure**: Secure defaults, fail closed
5. **Separation of Duties**: No single point of control

---

## Authentication & Authorization

### Authentication Methods

#### 1. API Key (Machine-to-Machine)

**Format**: `wia_[environment]_[32-char random]`

**Examples**:
- Production: `wia_live_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6`
- Test: `wia_test_x1y2z3a4b5c6d7e8f9g0h1i2j3k4l5m6`

**Storage**: Environment variables or secrets manager (never in code)

**Rotation**: Every 90 days (automated)

**Scopes**: `read:predictions`, `write:alerts`, `admin:all`

#### 2. OAuth 2.0 + OpenID Connect (User Authentication)

**Supported Flows**:
- Authorization Code Flow (web apps)
- PKCE (Proof Key for Code Exchange) for mobile/SPA
- Client Credentials (service accounts)

**Token Lifetime**:
- Access Token: 1 hour
- Refresh Token: 30 days (rotating)
- ID Token: 1 hour

**Example Authorization Request**:
```http
GET /oauth/authorize?
  response_type=code&
  client_id=wia_client_abc123&
  redirect_uri=https://myapp.com/callback&
  scope=openid profile email read:predictions&
  state=xyz123&
  code_challenge=E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM&
  code_challenge_method=S256
```

**Token Exchange**:
```bash
curl -X POST https://auth.wia-flood.org/oauth/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=authorization_code" \
  -d "code=AUTH_CODE_HERE" \
  -d "redirect_uri=https://myapp.com/callback" \
  -d "client_id=wia_client_abc123" \
  -d "code_verifier=VERIFIER_STRING"
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "def502003eaf...",
  "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

#### 3. JWT (JSON Web Token) Structure

**Header**:
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "2024-key-01"
}
```

**Payload**:
```json
{
  "sub": "user_12345",
  "name": "Jane Emergency Manager",
  "email": "jane@emergencyops.gov",
  "iss": "https://auth.wia-flood.org",
  "aud": "https://api.wia-flood.org",
  "iat": 1704988800,
  "exp": 1704992400,
  "scope": "read:predictions write:alerts",
  "role": "emergency_manager",
  "org_id": "fema_dc_region"
}
```

**Signature**: RSA-SHA256 with 2048-bit keys

### Authorization (RBAC - Role-Based Access Control)

#### Roles

| Role | Permissions | Use Case |
|------|-------------|----------|
| **Public** | Read public predictions | General public, news media |
| **Registered User** | Read all predictions, subscribe to alerts | Homeowners, businesses |
| **Emergency Manager** | Read predictions, create custom alerts, export data | FEMA, local EOCs |
| **Hydrologist** | Read predictions, view raw data, run custom models | USGS, universities |
| **Admin** | Full access, user management, system config | WIA staff |
| **Data Provider** | Upload external data (gauges, weather) | Partner agencies |

#### Permission Matrix

| Resource | Public | User | Emergency Manager | Hydrologist | Admin |
|----------|--------|------|-------------------|-------------|-------|
| `GET /predictions` | ✓ | ✓ | ✓ | ✓ | ✓ |
| `GET /gauges` | ✓ | ✓ | ✓ | ✓ | ✓ |
| `GET /alerts` | ✓ | ✓ | ✓ | ✓ | ✓ |
| `POST /alerts/subscribe` | ✗ | ✓ | ✓ | ✓ | ✓ |
| `POST /alerts/custom` | ✗ | ✗ | ✓ | ✗ | ✓ |
| `GET /satellite/images` | ✗ | ✓ | ✓ | ✓ | ✓ |
| `GET /models/raw-output` | ✗ | ✗ | ✗ | ✓ | ✓ |
| `POST /data/upload` | ✗ | ✗ | ✗ | ✗ | ✓ |
| `PUT /config/*` | ✗ | ✗ | ✗ | ✗ | ✓ |

---

## Data Security & Privacy

### Encryption

#### At Rest

- **Database**: AES-256-GCM encryption
- **Object Storage** (S3/MinIO): Server-side encryption (SSE-KMS)
- **Backups**: Encrypted with AWS KMS or HashiCorp Vault
- **Logs**: Encrypted in CloudWatch Logs or ELK

#### In Transit

- **TLS 1.3** for all HTTPS connections
- **Perfect Forward Secrecy** (PFS) with ECDHE key exchange
- **Certificate Authority**: Let's Encrypt or DigiCert
- **Cipher Suites** (preferred):
  - `TLS_AES_128_GCM_SHA256`
  - `TLS_AES_256_GCM_SHA384`
  - `TLS_CHACHA20_POLY1305_SHA256`

**TLS Configuration** (nginx):
```nginx
ssl_protocols TLSv1.3;
ssl_prefer_server_ciphers on;
ssl_ciphers 'TLS_AES_128_GCM_SHA256:TLS_AES_256_GCM_SHA384:TLS_CHACHA20_POLY1305_SHA256';
ssl_ecdh_curve secp384r1;
ssl_session_timeout 10m;
ssl_session_cache shared:SSL:10m;
ssl_stapling on;
ssl_stapling_verify on;
add_header Strict-Transport-Security "max-age=31536000; includeSubDomains; preload" always;
```

### Data Retention

| Data Type | Retention Period | Rationale |
|-----------|------------------|-----------|
| Predictions | 5 years | Historical analysis, model validation |
| Satellite Imagery | 2 years | Storage cost, available from Copernicus |
| River Gauge Data | 10 years | Long-term hydrological trends |
| User Activity Logs | 1 year | Security audits, compliance |
| API Access Logs | 90 days | Debugging, rate limit enforcement |
| Alert History | Indefinite | Disaster records, legal requirements |

### Data Anonymization

**Personally Identifiable Information (PII)**:
- User emails: Hashed with bcrypt before storage
- Phone numbers: Tokenized, only last 4 digits visible
- Location subscriptions: Aggregated for analytics (no individual tracking)

**Anonymization Techniques**:
- **k-anonymity**: Aggregate data to groups of ≥10 users
- **Differential privacy**: Add statistical noise to public datasets
- **Pseudonymization**: Replace user IDs with random tokens

### Secrets Management

**HashiCorp Vault** for secrets:
- API keys for external services (Copernicus, NOAA, USGS)
- Database credentials
- TLS certificates
- Encryption keys

**Access Control**:
```hcl
# Vault policy for prediction service
path "secret/data/flood-prediction/*" {
  capabilities = ["read", "list"]
}

path "database/creds/flood-prediction" {
  capabilities = ["read"]
}
```

---

## Compliance & Certifications

### 1. GDPR (General Data Protection Regulation)

**Applicability**: EU users subscribing to alerts

**Requirements**:
- ✓ Explicit consent for data collection
- ✓ Right to access (data export)
- ✓ Right to erasure ("right to be forgotten")
- ✓ Data portability (JSON export)
- ✓ Breach notification (within 72 hours)
- ✓ Privacy by Design & Default
- ✓ Data Protection Officer (DPO) appointed

**Implementation**:
```javascript
// User consent tracking
{
  "user_id": "user_12345",
  "consents": [
    {
      "type": "alert_subscription",
      "granted": true,
      "timestamp": "2026-01-11T10:00:00Z",
      "ip_address": "203.0.113.45",
      "version": "v1.2"
    }
  ]
}
```

### 2. HIPAA (Health Insurance Portability and Accountability Act)

**Not Directly Applicable**: Flood prediction is not PHI (Protected Health Information)

**Indirect Relevance**: Hospitals in flood zones may use data for evacuation planning

### 3. FedRAMP (Federal Risk and Authorization Management Program)

**Target**: Moderate Impact Level

**Requirements**:
- ✓ NIST SP 800-53 security controls (325 controls)
- ✓ Continuous monitoring (ConMon)
- ✓ Annual audit by 3PAO (Third-Party Assessment Organization)
- ✓ ATO (Authority to Operate) from federal sponsor

**Timeline**:
- Phase 1 (Q1 2026): FedRAMP Ready
- Phase 2 (Q3 2026): FedRAMP Authorized (Moderate)

### 4. SOC 2 Type II (Service Organization Control)

**Trust Service Criteria**:
- ✓ **Security**: Access controls, encryption, vulnerability mgmt
- ✓ **Availability**: 99.9% uptime SLA
- ✓ **Processing Integrity**: Accurate predictions, data validation
- ✓ **Confidentiality**: NDA, access restrictions
- ✓ **Privacy**: GDPR compliance, data anonymization

**Audit Period**: 12 months (annual)

**Auditor**: Big 4 accounting firm (Deloitte, PwC, EY, KPMG)

### 5. ISO 27001 (Information Security Management)

**Certification Status**: Planned Q4 2026

**Scope**: All systems, data, personnel

**Key Controls**:
- A.8.1.3: Acceptable use of assets
- A.9.2.1: User registration and de-registration
- A.10.1.1: Cryptographic controls
- A.12.6.1: Management of technical vulnerabilities
- A.14.2.1: Secure development policy

### 6. FEMA National Incident Management System (NIMS) Compliance

**Requirements**:
- ✓ CAP (Common Alerting Protocol) v1.2 format
- ✓ Integration with IPAWS (Integrated Public Alert & Warning System)
- ✓ Interoperability with emergency management systems
- ✓ Standardized terminology (FEMA IS-700)

---

## Performance Requirements

### Prediction Accuracy

| Metric | Target | Current | Measurement Method |
|--------|--------|---------|-------------------|
| **Overall Accuracy** | ≥85% | 87% | True positives + true negatives / total |
| **Precision (Flood)** | ≥80% | 82% | True positives / (true positives + false positives) |
| **Recall (Flood)** | ≥85% | 84% | True positives / (true positives + false negatives) |
| **F1 Score** | ≥0.82 | 0.83 | Harmonic mean of precision and recall |
| **False Alarm Ratio (FAR)** | ≤20% | 18% | False positives / (true positives + false positives) |
| **Probability of Detection (POD)** | ≥80% | 84% | Same as recall |
| **Critical Success Index (CSI)** | ≥0.70 | 0.73 | True positives / (true positives + false positives + false negatives) |

**Validation Dataset**: 500 historical flood events (2020-2025) from NOAA Storm Events Database

### Lead Time Accuracy

| Lead Time | Accuracy Target | Accuracy Achieved | Use Case |
|-----------|----------------|-------------------|----------|
| **0-24 hours** | 95% | 96% | Emergency evacuation |
| **1-3 days** | 90% | 92% | Pre-positioning resources |
| **4-7 days** | 85% | 87% | Public awareness campaigns |
| **8-14 days** | 75% | 78% | Long-term planning |

### Spatial Resolution & Coverage

- **Prediction Grid**: 10m x 10m (Sentinel-2 resolution)
- **Minimum Detectable Flood Area**: 0.01 km² (100m × 100m)
- **Vertical Accuracy** (flood depth): ±0.3m RMSE
- **Global Coverage**: 60°N to 60°S (all populated areas)
- **High-Resolution Zones** (5m grid):
  - Major US cities (100+ cities)
  - European capitals
  - High-risk river basins (Ganges, Mekong, Mississippi)

### API Performance

| Metric | Target | Current | Measurement |
|--------|--------|---------|-------------|
| **Response Time (p50)** | <200ms | 145ms | All GET requests |
| **Response Time (p95)** | <500ms | 380ms | Complex queries |
| **Response Time (p99)** | <1000ms | 850ms | Batch predictions |
| **Throughput** | 10,000 req/sec | 12,500 req/sec | Load testing |
| **Concurrent WebSocket Connections** | 100,000 | 85,000 | Peak capacity |
| **Prediction Generation Time** | <45 min | 38 min | Full cycle |

### System Availability

| Component | Target Uptime | Measured Uptime | MTTR (Mean Time To Repair) |
|-----------|---------------|-----------------|----------------------------|
| **API Gateway** | 99.95% | 99.97% | <5 minutes |
| **Prediction Engine** | 99.9% | 99.92% | <15 minutes |
| **Database (PostgreSQL)** | 99.99% | 99.995% | <2 minutes (auto-failover) |
| **Object Storage (S3)** | 99.99% | 99.999% | N/A (AWS SLA) |
| **WebSocket Server** | 99.9% | 99.88% | <10 minutes |
| **Overall System** | 99.9% | 99.91% | <10 minutes |

**Downtime Budget** (99.9% uptime):
- Per year: 8.76 hours
- Per month: 43.8 minutes
- Per week: 10.1 minutes

### Data Freshness

| Data Source | Update Frequency | Acceptable Latency | Current Latency |
|-------------|------------------|-------------------|-----------------|
| **Sentinel-1 Imagery** | 6 days | <24 hours | 3-12 hours |
| **Sentinel-2 Imagery** | 5 days | <24 hours | 2-18 hours |
| **NOAA GFS Forecast** | 6 hours | <2 hours | 45 minutes |
| **USGS River Gauges** | 15 minutes | <30 minutes | 15-20 minutes |
| **SMAP Soil Moisture** | 2-3 days | <48 hours | 12-36 hours |
| **Predictions (Output)** | 6 hours | <1 hour | 45 minutes |

### Scalability

**Current Capacity**:
- 10 million predictions/day
- 500,000 active alert subscriptions
- 5 TB satellite imagery processed/day

**Peak Capacity** (during major disaster):
- 50 million predictions/day (5x normal)
- 2 million active subscriptions (4x normal)
- 20 TB imagery processed/day (4x normal)

**Auto-scaling Triggers**:
- CPU > 70% for 5 minutes → Add 2 pods
- Request queue > 1000 → Add API instances
- WebSocket connections > 80% capacity → Add WS servers

---

## Disaster Response Integration

### 1. FEMA Integration

#### IPAWS (Integrated Public Alert & Warning System)

**Connection Type**: Direct HTTPS POST to IPAWS gateway

**Alert Flow**:
```
WIA Prediction System → CAP XML Generation → IPAWS Gateway → EAS/WEA/NOAA Weather Radio
```

**CAP Message Example**:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<alert xmlns="urn:oasis:names:tc:emergency:cap:1.2">
  <identifier>WIA-FF-DC-20260118-001</identifier>
  <sender>flood@wia-flood.org</sender>
  <sent>2026-01-18T14:30:00-05:00</sent>
  <status>Actual</status>
  <msgType>Alert</msgType>
  <scope>Public</scope>
  <code>IPAWSv1.0</code>
  <info>
    <language>en-US</language>
    <category>Met</category>
    <event>Flash Flood Warning</event>
    <responseType>Evacuate</responseType>
    <urgency>Immediate</urgency>
    <severity>Severe</severity>
    <certainty>Likely</certainty>
    <effective>2026-01-18T15:00:00-05:00</effective>
    <expires>2026-01-19T03:00:00-05:00</expires>
    <senderName>WIA Flood Prediction System</senderName>
    <headline>Flash Flood Warning for Northwest DC until 3:00 AM EST</headline>
    <description>AI prediction indicates 87% probability of severe flooding in the next 6 hours. Expected depth: 2.4 meters. Affected area: 3.7 km².</description>
    <instruction>Move to higher ground immediately. Do not drive through flooded areas. Monitor local news and emergency alerts.</instruction>
    <web>https://wia-flood.org/alerts/WIA-FF-DC-20260118-001</web>
    <contact>emergency@wia-flood.org</contact>
    <parameter>
      <valueName>prediction_id</valueName>
      <value>pred_2026011114_38.9072_-77.0369</value>
    </parameter>
    <parameter>
      <valueName>flood_probability</valueName>
      <value>0.87</value>
    </parameter>
    <parameter>
      <valueName>max_depth_meters</valueName>
      <value>2.4</value>
    </parameter>
    <area>
      <areaDesc>Northwest Washington DC, Montgomery County MD</areaDesc>
      <polygon>38.88,-77.12 38.88,-77.01 38.96,-77.01 38.96,-77.12 38.88,-77.12</polygon>
      <geocode>
        <valueName>FIPS6</valueName>
        <value>011001</value>
      </geocode>
    </area>
  </info>
</alert>
```

**Authentication**: Mutual TLS (mTLS) with FEMA-issued certificates

#### FEMA Data Exchange

**Push to FEMA**:
- Flood predictions (JSON, every 6 hours)
- Alert summaries (CAP XML, real-time)

**Pull from FEMA**:
- Disaster declarations (to prioritize regions)
- Evacuation zones (to refine risk maps)

### 2. National Weather Service (NWS) Integration

**NWS Flash Flood Warnings** (ingest):
- API: `https://api.weather.gov/alerts/active`
- Purpose: Cross-validate WIA predictions
- Action: If NWS issues warning but WIA didn't, trigger review

**WIA → NWS Data Sharing**:
- Share satellite-derived flood extent
- Provide ML model confidence scores
- Format: AWIPS (Advanced Weather Interactive Processing System)

### 3. Local Emergency Operations Centers (EOCs)

**Data Feeds**:
- WebSocket: Real-time predictions
- GeoJSON exports: For GIS import (QGIS, ArcGIS)
- CSV reports: Daily summaries

**API Integration**:
```python
# Example: Houston EOC pulls predictions every 30 minutes
import requests

response = requests.get(
    'https://api.wia-flood.org/v1/predictions',
    params={
        'lat': 29.7604,
        'lng': -95.3698,
        'radius_km': 100,
        'min_probability': 0.3
    },
    headers={'Authorization': 'Bearer houston_eoc_api_key'}
)
predictions = response.json()['data']['predictions']

# Import into EOC dispatch system
for pred in predictions:
    dispatch_system.create_incident(
        location=pred['location'],
        severity=pred['risk_level'],
        eta=pred['peak_time']
    )
```

### 4. Mobile Alert Systems

**SMS Alerts** (via Twilio):
```python
from twilio.rest import Client

client = Client(account_sid, auth_token)
message = client.messages.create(
    body="FLOOD WARNING: High risk (87%) in your area by 6pm today. Move to higher ground. Details: https://wia-flood.org/a/xyz",
    from_='+12025551000',
    to=user.phone_number
)
```

**Push Notifications** (via Firebase Cloud Messaging):
```json
{
  "notification": {
    "title": "Flood Warning",
    "body": "High flood risk detected near your location. Tap for details.",
    "icon": "flood_warning_icon",
    "sound": "emergency_alert.mp3"
  },
  "data": {
    "prediction_id": "pred_2026011114_38.9072_-77.0369",
    "risk_level": "high",
    "probability": 0.87,
    "deeplink": "wiaflood://prediction/pred_2026011114_38.9072_-77.0369"
  },
  "priority": "high"
}
```

### 5. Siren Systems

**Integration**: Direct API to municipal siren controllers

**Trigger Criteria**:
- Risk level: Extreme
- Probability: ≥90%
- Population at risk: ≥10,000
- Lead time: <2 hours

**Activation**:
```json
POST https://sirens.cityofhouston.gov/api/activate
{
  "zone": "flood_zone_3",
  "pattern": "tornado_siren",  // Most recognizable
  "duration": 180,  // 3 minutes
  "repeat": 3,
  "message": "Flash flood warning. Seek higher ground immediately."
}
```

---

## Service Level Agreements (SLAs)

### Uptime SLA

**Commitment**: 99.9% monthly uptime

**Downtime Credits**:
| Uptime | Credit |
|--------|--------|
| ≥99.9% | 0% |
| 99.0% - 99.9% | 10% |
| 95.0% - 99.0% | 25% |
| <95.0% | 50% |

**Exclusions** (not counted as downtime):
- Scheduled maintenance (announced 7 days prior, max 4 hours/month)
- User-caused issues (invalid API requests)
- Force majeure (natural disasters, war, pandemics)

### Prediction Accuracy SLA

**Commitment**: ≥85% overall accuracy (F1 score ≥0.82)

**Measurement**: Quarterly validation against observed flood events

**Remediation**: If accuracy drops below 82% for 2 consecutive quarters, customers can terminate without penalty

### Support SLA

| Tier | Response Time | Resolution Time | Channels |
|------|---------------|-----------------|----------|
| **Emergency** (system down) | 15 minutes | 2 hours | Phone, 24/7 |
| **High** (major feature broken) | 1 hour | 8 hours | Email, chat |
| **Medium** (minor issue) | 4 hours | 48 hours | Email |
| **Low** (question, enhancement) | 24 hours | Best effort | Email, forum |

**Support Hours**:
- Critical issues: 24/7/365
- Non-critical: Monday-Friday, 8am-8pm EST

---

## Next Steps (PHASE 4)

- Deployment architecture (Kubernetes, CI/CD)
- Monitoring & alerting (Prometheus, Grafana)
- ML model serving (TensorFlow Serving, MLflow)
- Operational runbooks

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
