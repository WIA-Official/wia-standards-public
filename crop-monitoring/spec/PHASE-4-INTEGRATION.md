# WIA Crop Monitoring Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

---

## Table of Contents

1. [Overview](#overview)
2. [Weather Service Integration](#weather-service-integration)
3. [AI/ML Model Integration](#aiml-model-integration)
4. [Marketplace Integration](#marketplace-integration)
5. [Insurance Platform Integration](#insurance-platform-integration)
6. [Government & Compliance Systems](#government--compliance-systems)
7. [Farm Management Software](#farm-management-software)
8. [Blockchain & Traceability](#blockchain--traceability)
9. [Integration Examples](#integration-examples)
10. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Crop Monitoring Integration Standard defines standardized interfaces for connecting crop monitoring systems with external services including weather APIs, AI models, marketplaces, insurance platforms, government systems, and farm management software.

**Integration Categories**:
- **Weather Services**: Real-time forecasts and climate data
- **AI/ML Models**: Disease detection, yield prediction
- **Marketplaces**: Crop trading and price discovery
- **Insurance**: Crop insurance and claims automation
- **Government**: Subsidy applications and compliance reporting
- **Farm Management**: ERP and precision agriculture platforms
- **Blockchain**: Supply chain traceability and certification

### 1.2 Integration Architecture

```
┌────────────────────────────────────────┐
│   WIA Crop Monitoring Platform         │
│   - Data Format (Phase 1)              │
│   - API Interface (Phase 2)            │
│   - Protocol (Phase 3)                 │
└──────────┬─────────────────────────────┘
           │
    ┌──────┴──────────┬──────────┬───────────┐
    ▼                 ▼          ▼           ▼
┌─────────┐    ┌──────────┐ ┌────────┐ ┌─────────┐
│ Weather │    │ AI Model │ │ Market │ │Insurance│
│ Service │    │ Service  │ │ API    │ │ API     │
└─────────┘    └──────────┘ └────────┘ └─────────┘
```

### 1.3 Integration Principles

1. **Loose Coupling**: Services are independently deployable
2. **Event-Driven**: Real-time notifications via webhooks
3. **Resilient**: Graceful degradation if external service fails
4. **Versioned**: API versioning for backward compatibility
5. **Monitored**: Health checks and SLA tracking

---

## Weather Service Integration

### 2.1 Weather API Interface

**Supported Providers**:
- OpenWeatherMap
- Weather.com (IBM)
- NOAA Weather API
- AccuWeather
- Custom weather stations

**Integration Flow:**
```
Crop Monitoring → Weather API → Forecast Data → Crop Impact Analysis
```

### 2.2 Weather Data Schema

**Request:**
```http
GET /integrations/weather/forecast
Authorization: Bearer {api_key}

{
  "location": {"latitude": 37.5665, "longitude": 126.9780},
  "days": 7,
  "includeHourly": true,
  "cropType": "rice"
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "location": {"latitude": 37.5665, "longitude": 126.9780},
    "provider": "OpenWeatherMap",
    "forecast": [
      {
        "date": "2025-06-16",
        "temperature": {"min": 18, "max": 28, "avg": 23, "unit": "°C"},
        "humidity": {"min": 60, "max": 85, "avg": 72, "unit": "%"},
        "precipitation": {"amount": 5.2, "probability": 40, "unit": "mm"},
        "windSpeed": {"avg": 12, "max": 18, "unit": "km/h"},
        "solarRadiation": {"value": 18.5, "unit": "MJ/m²"},
        "cropImpact": {
          "diseaseRisk": {
            "overall": "medium",
            "blight": 0.45,
            "rust": 0.30
          },
          "wateringNeeded": false,
          "alerts": [
            "High humidity may increase fungal disease risk",
            "Monitor for late blight in next 48 hours"
          ]
        }
      }
    ]
  }
}
```

### 2.3 Weather Alert Webhook

**Webhook Registration:**
```http
POST /integrations/weather/webhooks
Authorization: Bearer {api_key}

{
  "webhookUrl": "https://your-farm.com/weather-alerts",
  "events": ["extreme_weather", "disease_risk", "frost_warning"],
  "location": {"latitude": 37.5665, "longitude": 126.9780}
}
```

**Webhook Payload:**
```json
{
  "event": "disease_risk",
  "severity": "high",
  "timestamp": "2025-06-15T10:30:00Z",
  "location": {"latitude": 37.5665, "longitude": 126.9780},
  "alert": {
    "type": "Late Blight Risk",
    "probability": 0.75,
    "recommendedActions": [
      "Apply preventive fungicide within 24 hours",
      "Increase field monitoring frequency",
      "Harvest early if crop is near maturity"
    ]
  },
  "weather": {
    "temperature": 22,
    "humidity": 88,
    "rainfall": 15.5
  }
}
```

---

## AI/ML Model Integration

### 3.1 Disease Detection Integration

**AI Model Endpoint:**
```http
POST /integrations/ai/detect-disease
Authorization: Bearer {api_key}
Content-Type: multipart/form-data

{
  "image": [binary image data],
  "cropType": "tomato",
  "growthStage": "BBCH-70",
  "location": {"latitude": 37.5665, "longitude": 126.9780}
}
```

**AI Response:**
```json
{
  "status": "success",
  "data": {
    "modelId": "WIA-DiseaseDetector-v2.1",
    "processingTime": 1.23,
    "detections": [
      {
        "disease": "Late Blight (Phytophthora infestans)",
        "confidence": 0.87,
        "severity": "medium",
        "affectedArea": 15.5,
        "boundingBoxes": [
          {"x": 100, "y": 150, "width": 200, "height": 180}
        ],
        "recommendations": [
          {
            "action": "Chemical Treatment",
            "product": "Copper-based fungicide",
            "timing": "Within 24 hours",
            "dosage": "2 kg/ha"
          },
          {
            "action": "Cultural Practice",
            "description": "Remove and destroy infected leaves",
            "timing": "Immediately"
          }
        ],
        "relatedDiseases": ["Early Blight", "Septoria Leaf Spot"]
      }
    ],
    "healthScore": 65,
    "overallRisk": "medium"
  }
}
```

### 3.2 Yield Prediction Integration

**Yield Forecasting API:**
```http
POST /integrations/ai/predict-yield
Authorization: Bearer {api_key}

{
  "cropId": "CROP-2025-001",
  "historicalData": {
    "growthMeasurements": [...],
    "weatherHistory": [...],
    "soilData": {...}
  },
  "currentConditions": {
    "growthStage": "BBCH-70",
    "ndvi": 0.75,
    "leafAreaIndex": 4.2
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "yieldPrediction": {
      "value": 5500,
      "unit": "kg/ha",
      "confidence": 0.82,
      "range": {"min": 4800, "max": 6200}
    },
    "harvestDate": {
      "estimated": "2025-09-01",
      "confidence": 0.78,
      "range": {"earliest": "2025-08-28", "latest": "2025-09-05"}
    },
    "factors": {
      "weather": {"impact": 0.15, "description": "Favorable rainfall"},
      "soilHealth": {"impact": 0.10, "description": "Good nutrient levels"},
      "diseaseRisk": {"impact": -0.05, "description": "Low disease pressure"},
      "growthRate": {"impact": 0.20, "description": "Above average"}
    },
    "recommendations": [
      "Continue current irrigation schedule",
      "Monitor for maturity indicators starting Aug 20",
      "Plan harvest logistics 2 weeks in advance"
    ]
  }
}
```

### 3.3 Custom AI Model Integration

**Model Registration:**
```http
POST /integrations/ai/register-model
Authorization: Bearer {api_key}

{
  "modelName": "Custom Tomato Disease Detector",
  "modelType": "disease-detection",
  "endpoint": "https://your-ml-service.com/predict",
  "authentication": {
    "type": "api-key",
    "key": "your_ml_api_key"
  },
  "inputFormat": {
    "imageFormat": "JPEG",
    "resolution": "1024x1024",
    "metadata": ["cropType", "growthStage"]
  },
  "outputFormat": {
    "diseases": "array",
    "confidence": "float",
    "boundingBoxes": "array"
  }
}
```

---

## Marketplace Integration

### 4.1 Price Discovery API

**Get Current Prices:**
```http
GET /integrations/marketplace/prices
Authorization: Bearer {api_key}

{
  "cropType": "rice",
  "grade": "premium",
  "region": "KR",
  "quantity": 1000
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "cropType": "rice",
    "grade": "premium",
    "region": "KR",
    "currency": "USD",
    "prices": [
      {
        "marketplace": "Global Grain Exchange",
        "pricePerKg": 2.50,
        "minimumQuantity": 500,
        "trend": "stable",
        "lastUpdated": "2025-06-15T10:00:00Z"
      },
      {
        "marketplace": "Korea Agricultural Cooperative",
        "pricePerKg": 2.48,
        "minimumQuantity": 100,
        "trend": "rising",
        "lastUpdated": "2025-06-15T09:30:00Z"
      }
    ],
    "recommendations": {
      "bestPrice": 2.50,
      "bestMarketplace": "Global Grain Exchange",
      "optimalSellTime": "Current prices are favorable"
    }
  }
}
```

### 4.2 Crop Listing API

**List Crop for Sale:**
```http
POST /integrations/marketplace/listings
Authorization: Bearer {api_key}

{
  "cropId": "CROP-2025-001",
  "cropType": "rice",
  "variety": "Koshihikari",
  "quantity": 5000,
  "unit": "kg",
  "pricePerKg": 2.50,
  "currency": "USD",
  "harvestDate": "2025-09-01",
  "certifications": ["organic", "gap"],
  "deliveryOptions": ["farm-pickup", "delivery"],
  "location": {"latitude": 37.5665, "longitude": 126.9780}
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "listingId": "LISTING-2025-001",
    "status": "active",
    "visibility": "public",
    "expiresAt": "2025-09-15T00:00:00Z",
    "estimatedInterest": "high",
    "suggestedPrice": 2.55,
    "marketplaceUrl": "https://marketplace.wiastandards.com/listings/LISTING-2025-001"
  }
}
```

---

## Insurance Platform Integration

### 5.1 Crop Insurance Enrollment

**Enroll Crop:**
```http
POST /integrations/insurance/enroll
Authorization: Bearer {api_key}

{
  "farmId": "FARM-KR-12345",
  "cropId": "CROP-2025-001",
  "cropType": "rice",
  "areaHectares": 2.5,
  "estimatedYield": 5500,
  "coverageType": "multi-peril",
  "coverageLevel": 0.75,
  "insuredValue": 13750
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "policyId": "POL-2025-001",
    "premium": 550,
    "coverageAmount": 13750,
    "deductible": 1375,
    "effectiveDate": "2025-06-01",
    "expirationDate": "2025-09-30",
    "coveredPerils": [
      "drought",
      "flood",
      "hail",
      "frost",
      "disease",
      "pest"
    ]
  }
}
```

### 5.2 Automated Claims Processing

**Submit Claim:**
```http
POST /integrations/insurance/claims
Authorization: Bearer {api_key}

{
  "policyId": "POL-2025-001",
  "cropId": "CROP-2025-001",
  "incidentDate": "2025-07-15",
  "perilType": "disease",
  "description": "Late blight outbreak affecting 30% of crop",
  "evidence": {
    "diseaseDetectionResults": {...},
    "photos": ["https://..."],
    "weatherData": {...},
    "yieldLossEstimate": 1650
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "claimId": "CLAIM-2025-001",
    "status": "under_review",
    "estimatedPayout": 4125,
    "reviewTimeline": "5-7 business days",
    "requiredActions": [
      "Field inspection scheduled for 2025-07-18",
      "Provide harvest documentation by 2025-09-05"
    ]
  }
}
```

---

## Government & Compliance Systems

### 6.1 Subsidy Application Integration

**Submit Subsidy Application:**
```http
POST /integrations/government/subsidies/apply
Authorization: Bearer {api_key}

{
  "farmId": "FARM-KR-12345",
  "programId": "AGRI-SUBSIDY-2025",
  "cropType": "rice",
  "areaHectares": 2.5,
  "certifications": ["organic"],
  "complianceData": {
    "pesticideUsage": false,
    "waterConservation": true,
    "soilHealthPractices": ["crop-rotation", "cover-crops"]
  }
}
```

### 6.2 Compliance Reporting

**Generate Compliance Report:**
```http
GET /integrations/government/compliance/report
Authorization: Bearer {api_key}

{
  "farmId": "FARM-KR-12345",
  "reportType": "annual-environmental",
  "year": 2025
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "reportId": "COMP-2025-001",
    "farmId": "FARM-KR-12345",
    "year": 2025,
    "metrics": {
      "waterUsage": {"value": 5000, "unit": "m³", "compliance": "within-limits"},
      "pesticideUsage": {"value": 0, "unit": "kg", "compliance": "compliant"},
      "carbonFootprint": {"value": 1200, "unit": "kg CO2e", "compliance": "excellent"},
      "biodiversityIndex": {"value": 0.75, "compliance": "good"}
    },
    "certifications": ["organic", "gap", "sustainable"],
    "violations": []
  }
}
```

---

## Farm Management Software

### 7.1 ERP Integration

**Sync Crop Data to ERP:**
```http
POST /integrations/erp/sync
Authorization: Bearer {api_key}

{
  "erpSystem": "SAP Agriculture",
  "credentials": {...},
  "syncType": "incremental",
  "dataTypes": ["crops", "measurements", "yield-predictions"]
}
```

### 7.2 Precision Agriculture Platforms

**Integration with John Deere Operations Center:**
```http
POST /integrations/precision-ag/john-deere
Authorization: Bearer {api_key}

{
  "action": "import-field-data",
  "organizationId": "ORG-12345",
  "fields": ["FIELD-A-01", "FIELD-B-02"]
}
```

---

## Blockchain & Traceability

### 8.1 Supply Chain Traceability

**Record to Blockchain:**
```http
POST /integrations/blockchain/record
Authorization: Bearer {api_key}

{
  "cropId": "CROP-2025-001",
  "batchId": "BATCH-2025-001",
  "blockchainNetwork": "ethereum",
  "dataHash": "0xabc123...",
  "certifications": ["organic", "gap"],
  "traceabilityData": {
    "seedSource": "Certified Organic Seed Co.",
    "plantingDate": "2025-04-15",
    "harvestDate": "2025-09-01",
    "qualityGrade": "A+",
    "handlingHistory": [...]
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "transactionHash": "0xdef456...",
    "blockNumber": 12345678,
    "qrCode": "https://verify.wiastandards.com/BATCH-2025-001",
    "verificationUrl": "https://etherscan.io/tx/0xdef456..."
  }
}
```

---

## Integration Examples

### 9.1 Complete Weather + AI Integration

```python
import requests

API_KEY = "wia_api_key_1234567890abcdef"
BASE_URL = "https://api.crop-monitoring.wiastandards.com/v1"

# Get weather forecast
weather_response = requests.get(
    f"{BASE_URL}/integrations/weather/forecast",
    headers={"Authorization": f"Bearer {API_KEY}"},
    json={
        "location": {"latitude": 37.5665, "longitude": 126.9780},
        "days": 7,
        "cropType": "rice"
    }
)

weather_data = weather_response.json()

# Check for disease risk
if weather_data['data']['forecast'][0]['cropImpact']['diseaseRisk']['overall'] == 'high':
    # Trigger AI disease detection
    with open('field_image.jpg', 'rb') as image_file:
        ai_response = requests.post(
            f"{BASE_URL}/integrations/ai/detect-disease",
            headers={"Authorization": f"Bearer {API_KEY}"},
            files={'image': image_file},
            data={'cropType': 'rice', 'growthStage': 'BBCH-70'}
        )

    ai_result = ai_response.json()

    if ai_result['data']['detections']:
        # Disease detected, alert farmer and suggest treatment
        print(f"Disease detected: {ai_result['data']['detections'][0]['disease']}")
        print(f"Recommendations: {ai_result['data']['detections'][0]['recommendations']}")
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial integration specification |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: integrations@wiastandards.com
**Documentation**: https://docs.crop-monitoring.wiastandards.com/integrations


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
