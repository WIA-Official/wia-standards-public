# WIA-SOC-003 Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 4 defines integration patterns for e-government systems, including cross-border interoperability, third-party service integration, smart city connectivity, and legacy system migration strategies.

## 2. Cross-Border Interoperability

### 2.1 International Standards Alignment

**Supported Frameworks**:
```json
{
  "frameworks": {
    "eIDAS": {
      "region": "European Union",
      "version": "2.0",
      "support": "full",
      "trust_services": ["eSignature", "eSeal", "eTimestamp", "eDelivery"],
      "nodes": ["eIDAS-Node-KR-001"]
    },
    "APEC_CBPR": {
      "region": "Asia-Pacific",
      "version": "2023",
      "support": "full",
      "certification": "APEC-CBPR-2024-KR-001"
    },
    "UN_E_Gov": {
      "standards": ["EGOV-2.0", "Open-Data-Charter"],
      "compliance": "Level 4 (Very High)"
    },
    "ISO_IEC": {
      "standards": ["ISO/IEC 27001", "ISO/IEC 27701", "ISO 37001"],
      "certifications": ["ISO-27001-2024", "ISO-27701-2024"]
    }
  }
}
```

### 2.2 Data Exchange Protocol

**Cross-Border Request Flow**:
```yaml
cross_border_request:
  1_initiation:
    - citizen_submits_request
    - system_identifies_foreign_jurisdiction
    - checks_bilateral_agreement

  2_authorization:
    - citizen_consent_obtained
    - purpose_limitation_verified
    - data_minimization_applied

  3_request_submission:
    - encrypted_request_sent
    - through_government_gateway
    - to_foreign_counterpart

  4_processing:
    - foreign_system_authenticates
    - verifies_legal_basis
    - processes_request

  5_response:
    - encrypted_response_returned
    - data_integrity_verified
    - citizen_notified

  6_audit:
    - full_transaction_logged
    - compliance_verified
    - retention_policy_applied
```

**Technical Implementation**:
```xml
<!-- SOAP-based Government Gateway Message -->
<soapenv:Envelope xmlns:soapenv="http://schemas.xmlsoap.org/soap/envelope/">
  <soapenv:Header>
    <wsse:Security>
      <wsse:BinarySecurityToken>X509_CERTIFICATE</wsse:BinarySecurityToken>
      <ds:Signature>DIGITAL_SIGNATURE</ds:Signature>
    </wsse:Security>
    <egov:MessageHeader>
      <egov:MessageId>MSG-2025-KR-JP-12345</egov:MessageId>
      <egov:SourceCountry>KR</egov:SourceCountry>
      <egov:DestinationCountry>JP</egov:DestinationCountry>
      <egov:LegalBasis>Agreement-KR-JP-2024-Art-7.2</egov:LegalBasis>
      <egov:Purpose>background_check_employment</egov:Purpose>
      <egov:ConsentId>CONSENT-2025-5678</egov:ConsentId>
    </egov:MessageHeader>
  </soapenv:Header>
  <soapenv:Body>
    <egov:CitizenVerificationRequest>
      <egov:EncryptedData>AES256_ENCRYPTED_CITIZEN_INFO</egov:EncryptedData>
      <egov:DataHash>SHA256_HASH</egov:DataHash>
    </egov:CitizenVerificationRequest>
  </soapenv:Body>
</soapenv:Envelope>
```

### 2.3 Mutual Recognition Agreements

**Partner Countries**:
```json
{
  "agreements": [
    {
      "partner": "JP",
      "agreementId": "KR-JP-2024",
      "effectiveDate": "2024-01-01",
      "services": [
        "background_checks",
        "education_verification",
        "professional_licensing"
      ],
      "dataTypes": ["identity", "criminal_records", "qualifications"],
      "reciprocal": true
    },
    {
      "partner": "SG",
      "agreementId": "KR-SG-2023",
      "effectiveDate": "2023-06-01",
      "services": ["business_registration", "tax_records"],
      "dataTypes": ["corporate_identity", "financial_records"],
      "reciprocal": true
    }
  ]
}
```

## 3. Third-Party Service Integration

### 3.1 API Gateway Architecture

```
┌─────────────┐
│   Citizens  │
└──────┬──────┘
       │
       ↓
┌─────────────────────┐
│   API Gateway       │
│  - Authentication   │
│  - Rate Limiting    │
│  - Load Balancing   │
└──────┬──────────────┘
       │
       ├─→ ┌──────────────┐
       │   │ Tax Service  │
       │   └──────────────┘
       │
       ├─→ ┌──────────────┐
       │   │Health Service│
       │   └──────────────┘
       │
       └─→ ┌──────────────┐
           │ 3rd Party    │
           │ Services     │
           └──────────────┘
```

### 3.2 Partner Onboarding

**Registration Process**:
```json
{
  "partner_registration": {
    "step_1_application": {
      "required_info": [
        "organization_name",
        "business_registration",
        "technical_contact",
        "service_description",
        "data_processing_agreement"
      ],
      "review_time": "14 days"
    },
    "step_2_technical_review": {
      "requirements": [
        "security_audit",
        "api_compatibility_test",
        "load_testing",
        "disaster_recovery_plan"
      ],
      "review_time": "30 days"
    },
    "step_3_legal_review": {
      "requirements": [
        "privacy_policy",
        "terms_of_service",
        "data_processing_agreement",
        "liability_insurance"
      ],
      "review_time": "21 days"
    },
    "step_4_approval": {
      "grant_api_credentials": true,
      "assign_rate_limits": true,
      "provide_sandbox_access": true,
      "onboarding_support": "30 days"
    }
  }
}
```

### 3.3 Partner API Specification

**Webhook Integration**:
```json
{
  "webhook": {
    "url": "https://partner.example.com/webhooks/egov",
    "method": "POST",
    "headers": {
      "X-Gov-Signature": "HMAC-SHA256 signature",
      "X-Gov-Timestamp": "Unix timestamp",
      "Content-Type": "application/json"
    },
    "payload": {
      "event": "service_completed",
      "requestId": "REQ-2025-001234",
      "citizenId": "hashed-citizen-id",
      "result": {
        "status": "approved",
        "documents": ["DOC-2025-5678"],
        "notificationUrl": "https://api.egov.kr/notifications/ABC123"
      },
      "timestamp": "2025-12-26T14:30:00Z"
    },
    "security": {
      "signature_verification": "required",
      "timestamp_validation": "within 5 minutes",
      "ip_whitelist": ["203.0.113.0/24"]
    }
  }
}
```

## 4. Smart City Integration

### 4.1 IoT Device Integration

**Supported Protocols**:
```json
{
  "protocols": {
    "MQTT": {
      "broker": "mqtts://iot.egov.kr:8883",
      "auth": "X.509 client certificates",
      "topics": {
        "traffic": "smartcity/traffic/{region}/{sensor-id}",
        "environment": "smartcity/environment/{region}/{sensor-id}",
        "public_safety": "smartcity/safety/{region}/{sensor-id}"
      },
      "qos": 1,
      "retain": false
    },
    "CoAP": {
      "endpoint": "coaps://iot.egov.kr:5684",
      "auth": "DTLS with PSK",
      "resources": {
        "sensors": "/sensors/{type}/{id}",
        "actuators": "/actuators/{type}/{id}"
      }
    },
    "HTTP_REST": {
      "endpoint": "https://api.iot.egov.kr/v1",
      "auth": "Bearer token",
      "rate_limit": "1000 req/min per device"
    }
  }
}
```

### 4.2 Smart City Data Exchange

**Data Formats**:
```json
{
  "traffic_data": {
    "@context": "https://wiastandards.com/smartcity/traffic/v1",
    "@type": "TrafficSensor",
    "sensorId": "TRAFFIC-SEL-001",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "address": "Gangnam-daero, Seoul"
    },
    "measurements": [
      {
        "timestamp": "2025-12-26T14:30:00Z",
        "vehicleCount": 142,
        "averageSpeed": 45.5,
        "congestionLevel": "moderate"
      }
    ],
    "metadata": {
      "manufacturer": "SmartSensor Inc",
      "model": "TS-2024",
      "firmwareVersion": "2.1.4",
      "lastMaintenance": "2025-12-01"
    }
  }
}
```

### 4.3 Citizen-Facing Smart Services

**Integration Example** (Parking):
```yaml
smart_parking:
  citizen_app:
    - find_available_parking
    - reserve_parking_spot
    - navigate_to_spot
    - pay_via_gov_wallet

  backend_integration:
    - realtime_parking_availability (via IoT sensors)
    - reservation_system (via API)
    - payment_processing (via payment gateway)
    - navigation (via maps API)

  data_flow:
    1. IoT sensors → Smart City Platform → Availability API
    2. Citizen App → Reserve Parking → Reservation System
    3. Citizen App → Payment → Government Payment Gateway
    4. Confirmation → Citizen App + Email/SMS
```

## 5. Legacy System Integration

### 5.1 Modernization Strategy

**Strangler Fig Pattern**:
```
┌─────────────────────────────────────┐
│     Legacy Mainframe System         │
│  (COBOL, DB2, Batch Processing)     │
└───────────┬─────────────────────────┘
            │
            ↓
┌───────────────────────────────────────┐
│    Integration Layer (Middleware)     │
│  - API Gateway                        │
│  - Message Queue (RabbitMQ/Kafka)     │
│  - ETL for Data Sync                  │
└───────────┬───────────────────────────┘
            │
            ↓
┌───────────────────────────────────────┐
│    Modern Microservices               │
│  - RESTful APIs                       │
│  - Cloud Native                       │
│  - Real-time Processing               │
└───────────────────────────────────────┘
```

**Implementation Phases**:
```json
{
  "modernization": {
    "phase_1_assessment": {
      "duration": "3 months",
      "activities": [
        "inventory_legacy_systems",
        "identify_dependencies",
        "assess_technical_debt",
        "prioritize_migration"
      ]
    },
    "phase_2_integration_layer": {
      "duration": "6 months",
      "activities": [
        "deploy_api_gateway",
        "implement_data_sync",
        "create_facade_apis",
        "test_integration"
      ]
    },
    "phase_3_gradual_migration": {
      "duration": "18-24 months",
      "activities": [
        "migrate_service_by_service",
        "maintain_data_consistency",
        "parallel_running",
        "incremental_cutover"
      ]
    },
    "phase_4_decommission": {
      "duration": "6 months",
      "activities": [
        "final_data_migration",
        "archive_legacy_data",
        "decommission_old_systems",
        "knowledge_transfer"
      ]
    }
  }
}
```

### 5.2 Data Migration

**ETL Pipeline**:
```python
# Example: Mainframe to Cloud Migration
class LegacyDataMigration:
    def extract(self, legacy_db):
        """Extract from COBOL/DB2"""
        query = """
        SELECT CITIZEN_ID, NAME, DOB, ADDRESS
        FROM LEGACY.CITIZENS
        WHERE UPDATED_DATE > ?
        """
        return legacy_db.execute(query, last_sync_date)

    def transform(self, legacy_data):
        """Transform to modern format"""
        return {
            "@context": "https://wiastandards.com/soc-003/v1",
            "@type": "CitizenIdentity",
            "citizenId": f"urn:uuid:{generate_uuid(row.CITIZEN_ID)}",
            "givenName": row.NAME.split()[0],
            "familyName": row.NAME.split()[1],
            "dateOfBirth": parse_cobol_date(row.DOB),
            "residency": parse_address(row.ADDRESS)
        }

    def load(self, transformed_data, modern_db):
        """Load into cloud database"""
        modern_db.upsert(
            table="citizens",
            data=transformed_data,
            on_conflict="citizen_id"
        )

    def validate(self, legacy_count, modern_count):
        """Verify data integrity"""
        assert legacy_count == modern_count
        # Run checksum validation
        # Verify referential integrity
```

## 6. Mobile App Integration

### 6.1 Native App SDKs

**iOS SDK (Swift)**:
```swift
import WIAEGovernment

let client = WIAEGovClient(
    countryCode: "KR",
    apiKey: "your-api-key",
    environment: .production
)

// Authenticate
client.authenticate(
    identityNumber: "123456-7890123",
    biometricData: faceIDData
) { result in
    switch result {
    case .success(let token):
        print("Authenticated: \(token.citizenId)")
    case .failure(let error):
        print("Error: \(error)")
    }
}

// Submit request
let request = ServiceRequest(
    serviceType: .taxFiling,
    priority: .normal,
    documents: [documentId]
)

client.submitRequest(request) { result in
    // Handle result
}
```

**Android SDK (Kotlin)**:
```kotlin
import com.wia.egov.WIAEGovClient

val client = WIAEGovClient.Builder()
    .countryCode("KR")
    .apiKey("your-api-key")
    .environment(Environment.PRODUCTION)
    .build()

// Authenticate
client.authenticate(
    identityNumber = "123456-7890123",
    biometricData = fingerprintData
).observe(this) { result ->
    when (result) {
        is Success -> println("Authenticated: ${result.token.citizenId}")
        is Failure -> println("Error: ${result.error}")
    }
}
```

### 6.2 Push Notifications

**Implementation**:
```json
{
  "push_notification": {
    "platforms": {
      "ios": {
        "provider": "APNs",
        "certificate": "production.p12",
        "topic": "com.wia.egov.kr"
      },
      "android": {
        "provider": "FCM",
        "server_key": "firebase-server-key",
        "sender_id": "firebase-sender-id"
      }
    },
    "message_format": {
      "title": "Service Request Update",
      "body": "Your tax filing request has been approved",
      "data": {
        "requestId": "REQ-2025-001234",
        "type": "service_update",
        "action": "open_request"
      },
      "priority": "high",
      "sound": "default"
    }
  }
}
```

## 7. Analytics and Business Intelligence

### 7.1 Data Warehouse Integration

**Architecture**:
```
┌──────────────────────────────────────┐
│   Operational Databases              │
│  (PostgreSQL, MongoDB)               │
└───────────┬──────────────────────────┘
            │
            ↓
┌───────────────────────────────────────┐
│   ETL Pipeline (Apache Airflow)       │
│  - Extract: Daily batch jobs          │
│  - Transform: Anonymize, Aggregate    │
│  - Load: Data warehouse               │
└───────────┬───────────────────────────┘
            │
            ↓
┌───────────────────────────────────────┐
│   Data Warehouse (Snowflake/BigQuery) │
│  - Historical data                    │
│  - Analytics tables                   │
│  - ML feature store                   │
└───────────┬───────────────────────────┘
            │
            ├─→ ┌────────────────────┐
            │   │  BI Tools          │
            │   │  (Tableau, Looker) │
            │   └────────────────────┘
            │
            └─→ ┌────────────────────┐
                │  ML Models         │
                │  (TensorFlow)      │
                └────────────────────┘
```

### 7.2 Public Analytics API

**Endpoint**: `GET /analytics/public`

**Example Response**:
```json
{
  "period": "2025-12",
  "metrics": {
    "total_services": 142,
    "total_requests": 892567,
    "completion_rate": 94.7,
    "average_satisfaction": 4.8,
    "digital_adoption_rate": 87.3,
    "cost_savings": {
      "amount": 4500000000,
      "currency": "KRW",
      "vs_traditional": "45% reduction"
    },
    "environmental_impact": {
      "paper_saved_tons": 125,
      "co2_reduction_kg": 15000
    }
  },
  "top_services": [
    {"service": "Tax Filing", "requests": 142567},
    {"service": "Healthcare", "requests": 98234}
  ]
}
```

## 8. Disaster Recovery and Business Continuity

### 8.1 Multi-Region Architecture

```yaml
architecture:
  primary_region: ap-northeast-2 (Seoul)
  secondary_region: ap-northeast-1 (Tokyo)
  tertiary_region: us-west-2 (Oregon)

  data_replication:
    - synchronous: primary → secondary (< 10ms latency)
    - asynchronous: primary → tertiary (< 1min latency)

  failover:
    - automatic: primary → secondary (RTO: 5 minutes)
    - manual: primary → tertiary (RTO: 30 minutes)

  testing:
    - disaster_recovery_drill: monthly
    - full_failover_test: quarterly
```

### 8.2 Backup Strategy

```json
{
  "backup": {
    "database": {
      "frequency": "continuous",
      "retention": "30 days (hourly), 1 year (daily), 7 years (monthly)",
      "encryption": "AES-256",
      "offsite": "3 geographically distributed locations"
    },
    "documents": {
      "frequency": "real-time replication",
      "retention": "permanent (legal requirement)",
      "archival": "glacier storage after 2 years"
    },
    "audit_logs": {
      "frequency": "real-time to blockchain",
      "retention": "10 years (immutable)",
      "verification": "merkle proof available"
    }
  }
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
