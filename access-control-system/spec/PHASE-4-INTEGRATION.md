# WIA Access Control System - Phase 4: Integration Specification
## Version 1.0

### Document Information
- **Standard:** WIA-ACS (World Industry Association - Access Control System)
- **Phase:** 4 (Integration)
- **Version:** 1.0
- **Status:** Approved
- **Date:** 2025-12-26
- **Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Introduction

Phase 4 defines integration patterns and adapters for connecting WIA-ACS with existing enterprise systems, including LDAP/Active Directory, physical access control systems (PACS), biometric devices, smart cards, RFID/NFC readers, and cloud identity providers.

---

## 2. Integration Architecture

### 2.1 Adapter Pattern

WIA-ACS uses an adapter pattern enabling integration without core platform modifications.

```
┌─────────────────────────────────┐
│       WIA-ACS Core Platform      │
└──────────────┬──────────────────┘
               │
    ┌──────────▼────────────┐
    │  Integration Adapters  │
    └─┬────────┬────────┬───┘
      │        │        │
 ┌────▼───┐ ┌─▼────┐ ┌─▼──────┐
 │  LDAP  │ │ PACS │ │  Bio   │
 │Adapter │ │Adapter│ │Adapter │
 └────────┘ └──────┘ └────────┘
```

### 2.2 Adapter Interface

All adapters MUST implement:

```typescript
interface IAdapter {
  // Lifecycle
  initialize(config: AdapterConfig): Promise<void>;
  healthCheck(): Promise<HealthStatus>;
  shutdown(): Promise<void>;

  // Data sync
  syncUsers(options: SyncOptions): Promise<SyncResult>;
  syncCredentials(options: SyncOptions): Promise<SyncResult>;

  // Events
  onEvent(event: ExternalEvent): Promise<WiaEvent>;
  subscribeToEvents(callback: EventCallback): void;

  // Capabilities
  getCapabilities(): AdapterCapabilities;
}
```

---

## 3. LDAP / Active Directory Integration

### 3.1 Connection Configuration

```json
{
  "adapter_type": "ldap",
  "connection": {
    "url": "ldaps://ldap.example.com:636",
    "bind_dn": "cn=wia-acs-service,ou=Service Accounts,dc=example,dc=com",
    "bind_password": "${LDAP_BIND_PASSWORD}",
    "tls": {
      "enabled": true,
      "verify_cert": true,
      "min_tls_version": "1.2"
    }
  }
}
```

### 3.2 Schema Mapping

```json
{
  "schema_mapping": {
    "user": {
      "base_dn": "ou=Users,dc=example,dc=com",
      "object_class": "inetOrgPerson",
      "filter": "(&(objectClass=inetOrgPerson)(!(userAccountControl:1.2.840.113556.1.4.803:=2)))",
      "attributes": {
        "user_id": "employeeNumber",
        "given_name": "givenName",
        "family_name": "sn",
        "email": "mail",
        "department": "department"
      }
    }
  }
}
```

### 3.3 Synchronization

**Incremental Sync (every 4 hours):**
```
1. Query LDAP for changes since last sync
   (&(modifyTimestamp>=20251226120000Z))
2. Map LDAP attributes to WIA-ACS schema
3. Resolve group memberships to roles
4. Create/update users in WIA-ACS
5. Handle deletions (mark as terminated)
```

**Full Sync (daily):**
```
1. Retrieve all users from LDAP
2. Compare with WIA-ACS database
3. Identify orphaned accounts
4. Reconcile discrepancies
5. Generate sync report
```

---

## 4. PACS Integration

### 4.1 Supported Vendors

| Vendor | Protocol | Integration Method | Capabilities |
|--------|----------|-------------------|--------------|
| HID VertX | REST API, OSDP | Direct API | Bidirectional sync, real-time events |
| Lenel OnGuard | OpenAccess, SQL | Database integration | User sync, event monitoring |
| Software House CCure | SDK, REST API | SDK wrapper | Full control, credential provisioning |
| Genetec Synergis | SDK, WebSDK | Web services | Real-time monitoring, video integration |

### 4.2 HID VertX Integration Example

**Credential Provisioning:**

```
1. WIA-ACS creates credential
   POST /v1/credentials { user_id, credential_type, encoding }

2. Adapter converts to HID format
   POST /api/v1/credentials
   Host: vertx.example.com
   {
     "CardNumber": 45678,
     "FacilityCode": 123,
     "Doors": [101, 102]
   }

3. Event monitoring (HID webhook)
   POST /webhook/hid-vertx
   {
     "EventType": "AccessGranted",
     "DoorID": 101,
     "CardNumber": 45678
   }

4. Adapter translates to WIA-ACS event
   {
     "event_type": "access_granted",
     "credential_id": "cred-xyz"
   }
```

---

## 5. Biometric Integration

### 5.1 Supported Modalities

- Fingerprint (ISO 19794-2)
- Facial recognition (ISO 19794-5)
- Iris scanning (ISO 19794-6)
- Palm vein recognition

### 5.2 Enrollment Workflow

```
1. Capture biometric samples (3-5 samples)
2. Generate template (ISO standard format)
3. Calculate quality score
4. Encrypt template (AES-256)
5. Store in WIA-ACS
   POST /v1/biometrics
   {
     "user_id": "usr-001",
     "biometric_type": "fingerprint",
     "template_encrypted": "...",
     "quality_score": 85
   }
```

### 5.3 Authentication Workflow

```
1. Capture authentication sample
2. Generate template
3. Send to WIA-ACS for matching
   POST /v1/biometrics/authenticate
   {
     "device_id": "bio-reader-001",
     "template": "..."
   }
4. Perform 1:N matching
5. Return best match if above threshold
   {
     "match": true,
     "user_id": "usr-001",
     "confidence": 98.7
   }
```

### 5.4 Performance Requirements

- **1:1 matching:** < 100ms
- **1:10,000 matching:** < 2 seconds
- **False Accept Rate (FAR):** < 0.001%
- **False Reject Rate (FRR):** < 0.1%

---

## 6. Smart Card Integration

### 6.1 PIV/CAC Support

**Card Structure:**
```
PIV Card:
├── Contactless interface (13.56 MHz)
├── Contact interface (ISO 7816)
├── Cryptographic processor
└── Data objects:
    ├── Cardholder Unique Identifier (CHUID)
    ├── PIV Authentication Certificate (9A)
    ├── Digital Signature Certificate (9C)
    ├── Card Authentication Key (9E)
    └── Fingerprint templates (optional)
```

**Authentication Flow:**
```
1. Card presented to reader
2. Reader requests PIV Auth Certificate
   APDU: 00 CB 3F FF 05 5C 03 5F C1 05
3. Card returns certificate
4. Reader generates challenge
5. Card signs challenge with private key
6. Reader verifies signature
7. Extract user ID from certificate DN
8. Authenticate with WIA-ACS
```

### 6.2 RFID/NFC Integration

| Technology | Frequency | Read Range | Use Case |
|------------|-----------|------------|----------|
| LF Proximity | 125 kHz | 10 cm | Legacy access cards |
| HF RFID/NFC | 13.56 MHz | 10-30 cm | Smart cards, mobile NFC |
| UHF RFID | 860-960 MHz | 1-10 meters | Vehicle access, asset tracking |

---

## 7. Cloud Identity Provider Integration

### 7.1 Azure AD (SCIM Provisioning)

**Configuration:**
```
1. Create Enterprise Application in Azure AD
2. Enable SCIM provisioning
3. Set tenant URL: https://acs.example.com/scim/v2
4. Set secret token: SCIM_BEARER_TOKEN
5. Configure attribute mapping
```

**Attribute Mapping:**
```
Azure AD              →  WIA-ACS
────────────────────────────────
id                    →  external_id
displayName           →  identity.name
userPrincipalName     →  identity.email
department            →  employment.department
jobTitle              →  employment.title
```

**User Provisioning:**
```
POST /scim/v2/Users
{
  "schemas": ["urn:ietf:params:scim:schemas:core:2.0:User"],
  "userName": "john.doe@example.com",
  "name": {
    "givenName": "John",
    "familyName": "Doe"
  },
  "emails": [{
    "value": "john.doe@example.com",
    "primary": true
  }],
  "active": true
}
```

**Deprovisioning:**
```
PATCH /scim/v2/Users/{id}
{
  "Operations": [{
    "op": "replace",
    "path": "active",
    "value": false
  }]
}
```

---

## 8. Event Correlation

### 8.1 Unified Audit Trail

Correlate events from all integrated systems:

```
Time    System          Event
─────────────────────────────────────────────
10:00   Azure AD        User created
10:01   WIA-ACS         User provisioned via SCIM
10:02   WIA-ACS         Credential issued
10:03   HID VertX       Card programmed
14:30   HID VertX       Card presented at Door 101
14:30   WIA-ACS         Authentication successful
14:30   HID VertX       Door 101 unlocked
17:00   Azure AD        User disabled
17:01   WIA-ACS         User suspended
17:01   WIA-ACS         Credential revoked
17:02   HID VertX       Card deactivated
```

### 8.2 Correlation Query

```
GET /v1/audit/events?user_id=usr-001&start_date=2025-12-26

Returns chronological events from all systems with correlation IDs.
```

---

## 9. Integration Testing

### 9.1 Test Scenarios

**User Provisioning Flow:**
```
1. Create user in LDAP
2. Wait for sync (< 5 min)
3. Verify user in WIA-ACS
4. Check role mappings
5. Validate audit logs
```

**Access Grant Flow:**
```
1. Issue credential
2. Provision to PACS
3. Present at reader
4. Verify authentication
5. Check authorization
6. Confirm door unlock
7. Validate event chain
```

**Access Revocation Flow:**
```
1. Disable user in LDAP
2. Wait for sync
3. Verify credentials revoked
4. Attempt access (should fail)
5. Confirm PACS updated
6. Audit trail complete
```

---

## 10. Compliance Checklist

Phase 4 Integration compliance requires:
- [ ] At least one directory integration (LDAP/AD)
- [ ] At least one PACS integration
- [ ] Biometric device support
- [ ] Smart card/RFID support
- [ ] Cloud IdP integration (SCIM)
- [ ] Bidirectional sync working
- [ ] Event correlation functional
- [ ] Integration test suite passing
- [ ] Adapter documentation complete

---

**Document Control**
- Author: WIA Technical Committee
- Approved: 2025-12-26
- Next Review: 2026-12-26
- License: CC BY 4.0

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
