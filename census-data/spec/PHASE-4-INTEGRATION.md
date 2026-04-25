# WIA-SOC-016 PHASE 4: Integration Specification

## Census Data Standard - System Integration

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies integration patterns for census systems with external systems, administrative data sources, and international platforms.

---

## 2. Administrative Data Integration

### 2.1 Population Registers

**Integration Method:** Batch ETL, Real-time API
**Frequency:** Daily updates
**Data Elements:** Names, addresses, vital events

**Matching:**
- Personal identification numbers
- Probabilistic matching (when no IDs)
- Manual review for conflicts

### 2.2 Tax Systems

**Data Sources:** Income, employment, property
**Privacy:** Aggregated only, no individual linkage
**Validation:** Cross-check employment statistics

### 2.3 Health Systems

**Integration:** De-identified linkage
**Purpose:** Health statistics, disability rates
**Governance:** Ethics approval required

### 2.4 Education Systems

**Data:** Enrollment, graduation, credentials
**Linkage:** Student IDs, birthdates
**Quality:** Validate education statistics

---

## 3. Geographic Information Systems (GIS)

### 3.1 Boundary Integration

**Formats:** Shapefile, GeoJSON, GeoPackage
**Standards:** OGC (Open Geospatial Consortium)
**Topology:** Validated, no gaps/overlaps

### 3.2 Geocoding Services

**Providers:** Google Maps API, OSM Nominatim, proprietary
**Batch Processing:** Large-scale address geocoding
**Quality Control:** Manual review of unmatched addresses

### 3.3 Spatial Analytics

**Tools Integration:**
- QGIS
- ArcGIS
- PostGIS
- GeoPandas

---

## 4. Statistical Software Integration

### 4.1 R Integration

```r
# Census data package
library(census.wia)

# Connect to API
census <- connect_census(api_key = "YOUR_KEY")

# Query data
pop_data <- census %>%
  get_population(geo = "USA-CA", year = 2025) %>%
  filter(age_group == "25-34")
```

### 4.2 Python Integration

```python
from census_wia import CensusAPI

# Initialize
api = CensusAPI(api_key="YOUR_KEY")

# Query
pop_data = api.population(
    geo_code="USA-CA",
    year=2025,
    age_group="25-34"
)
```

### 4.3 SAS, Stata, SPSS

**Data Formats:** Import via CSV, Parquet
**Labels:** Metadata included
**Documentation:** Codebooks provided

---

## 5. Cloud Platform Integration

### 5.1 AWS Integration

**Services:**
- S3: Data storage
- RDS/Redshift: Databases
- Lambda: Processing functions
- API Gateway: API management

### 5.2 Google Cloud Integration

**Services:**
- Cloud Storage: Data lake
- BigQuery: Analytics
- Cloud Functions: Serverless
- Cloud Endpoints: API management

### 5.3 Azure Integration

**Services:**
- Blob Storage: Object storage
- SQL Database: Relational data
- Azure Functions: Compute
- API Management: API gateway

---

## 6. Business Intelligence (BI) Tools

### 6.1 Tableau Integration

**Connector:** Web Data Connector
**Live Query:** Direct API connection
**Extracts:** Scheduled refresh

### 6.2 Power BI Integration

**Connection:** REST API, OData
**Refresh:** Automatic scheduling
**Visuals:** Custom visuals available

### 6.3 Looker Integration

**LookML:** Data modeling
**API:** RESTful connection
**Embedding:** Dashboard embedding

---

## 7. International Data Exchange

### 7.1 IPUMS Integration

**Format:** DDI metadata + fixed-width ASCII
**Harmonization:** Common coding schemes
**Documentation:** Comprehensive variable descriptions

### 7.2 Eurostat Integration

**Protocol:** SDMX-ML
**Validation:** Eurostat validation rules
**Submission:** Secure FTP

### 7.3 UN Statistics Division

**Format:** UN-recommended classifications
**Reporting:** Standard tables
**Frequency:** Decennial + annual updates

---

## 8. Visualization Platforms

### 8.1 Interactive Maps

**Libraries:**
- Leaflet.js
- Mapbox GL JS
- D3.js
- Plotly

**Data Service:** Vector tiles for performance

### 8.2 Dashboards

**Frameworks:**
- Dash (Python)
- Shiny (R)
- Observable
- Streamlit

---

## 9. Machine Learning Platforms

### 9.1 Model Training

**Platforms:**
- TensorFlow
- PyTorch
- Scikit-learn
- H2O.ai

**Data Access:** Privacy-preserving APIs
**Compliance:** Differential privacy in training

### 9.2 MLOps Integration

**Tools:**
- MLflow
- Kubeflow
- SageMaker
- Vertex AI

---

## 10. Microdata Access Systems

### 10.1 Secure Research Data Centers

**Access Control:** Multi-factor authentication
**Network:** Air-gapped or VPN
**Output Review:** All results screened
**Audit:** Complete access logging

### 10.2 Remote Execution Systems

**Model:** Submit analysis code, receive results
**Review:** Automated + manual disclosure check
**Languages:** R, Python, SAS, Stata

### 10.3 Synthetic Data Platforms

**Generation:** Statistical synthesis
**Validation:** Utility metrics
**Distribution:** Public download

---

## 11. API Gateway Integration

### 11.1 Kong Gateway

```yaml
services:
  - name: census-api
    url: https://backend.census.wia.org
    routes:
      - name: population-route
        paths:
          - /population
    plugins:
      - name: rate-limiting
        config:
          minute: 100
      - name: key-auth
```

### 11.2 AWS API Gateway

**Features:**
- Lambda integration
- Cognito authentication
- Usage plans
- Throttling

---

## 12. ETL Pipeline Integration

### 12.1 Apache Airflow

```python
from airflow import DAG
from airflow.operators.python import PythonOperator

dag = DAG(
    'census_etl',
    schedule_interval='@daily'
)

extract = PythonOperator(
    task_id='extract_census_data',
    python_callable=extract_census,
    dag=dag
)

transform = PythonOperator(
    task_id='transform_census_data',
    python_callable=transform_census,
    dag=dag
)

load = PythonOperator(
    task_id='load_census_data',
    python_callable=load_census,
    dag=dag
)

extract >> transform >> load
```

### 12.2 dbt (Data Build Tool)

**Models:** SQL-based transformations
**Testing:** Data quality tests
**Documentation:** Auto-generated docs

---

## 13. Monitoring and Observability

### 13.1 Prometheus + Grafana

**Metrics:**
- API request rate
- Response times
- Error rates
- Data freshness

### 13.2 ELK Stack

**Components:**
- Elasticsearch: Log storage
- Logstash: Log processing
- Kibana: Visualization

---

## 14. Identity and Access Management

### 14.1 OAuth 2.0 Integration

**Flows:**
- Authorization code
- Client credentials
- Implicit (deprecated)

**Providers:**
- Auth0
- Okta
- Azure AD
- Keycloak

### 14.2 SAML Integration

**Use Case:** Enterprise SSO
**Attributes:** User roles, permissions
**Encryption:** Assertion encryption

---

## 15. Continuous Integration/Deployment

### 15.1 GitHub Actions

```yaml
name: Census API CI/CD

on: [push]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run tests
        run: pytest tests/
      - name: Deploy
        run: ./deploy.sh
```

### 15.2 Jenkins Integration

**Pipelines:** Declarative, scripted
**Deployment:** Blue-green, canary
**Testing:** Unit, integration, E2E

---

**Document Version:** 1.0
© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for census-data is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/census-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/census-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/census-data/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


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
