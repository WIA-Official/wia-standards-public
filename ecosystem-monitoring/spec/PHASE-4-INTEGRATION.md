# WIA Ecosystem Monitoring Standard - Phase 4: Integration Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**License:** CC BY 4.0

## 1. Introduction

Phase 4 specifies integration frameworks connecting WIA-compliant systems with external platforms.

## 2. GIS Platform Integration

### 2.1 OGC Web Services

Required implementations:
- **WFS (Web Feature Service)**: Vector data access
- **WMS (Web Map Service)**: Map visualization
- **WCS (Web Coverage Service)**: Raster data access
- **CSW (Catalog Service)**: Metadata discovery

### 2.2 GeoJSON Support

All spatial endpoints MUST support GeoJSON format:
```
GET /api/v1/observations?format=geojson
```

GeoJSON structure:
```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [longitude, latitude]
      },
      "properties": { /* WIA observation data */ }
    }
  ]
}
```

### 2.3 ArcGIS Integration

Supported interfaces:
- ArcGIS REST Feature Services
- Geoprocessing tools for import/export
- ArcGIS Dashboards real-time connections
- ArcGIS Field Maps mobile data collection

### 2.4 QGIS Integration

Provided components:
- QGIS plugins for WIA data access
- Processing algorithms
- Database connection profiles
- Style templates

## 3. Conservation Database Integration

### 3.1 Darwin Core Mapping

WIA to Darwin Core field mappings:

| WIA Field | Darwin Core Term |
|-----------|------------------|
| observation_id | occurrenceID |
| timestamp | eventDate |
| location.latitude | decimalLatitude |
| location.longitude | decimalLongitude |
| taxon.scientific_name | scientificName |
| abundance | individualCount |
| detection_method | samplingProtocol |

### 3.2 GBIF Integration

Export to GBIF Darwin Core Archive format:
- Meta.xml descriptor
- Occurrence.txt core file
- EML.xml metadata
- Resource relationships

### 3.3 iNaturalist Integration

Bidirectional integration:
- Import research-grade observations via API
- Export WIA observations to iNaturalist
- Leverage AI identification and expert review
- Synchronize taxonomic updates

### 3.4 eBird Integration

Converter for eBird checklists to WIA format:
```javascript
function ebirdToWIA(checklist) {
  return {
    wia_version: "1.0",
    schema_type: "species-observation",
    observation_id: `eBird-${checklist.subId}`,
    // ... mapping specification
  };
}
```

## 4. Cloud Platform Integration

### 4.1 Google Earth Engine

Integration pattern:
```javascript
var wiaData = ee.FeatureCollection('projects/wia/observations');
var imagery = ee.ImageCollection('LANDSAT/LC08/C02/T1_L2')
  .filterBounds(wiaData);
  
// Extract remote sensing values at observation points
var enriched = imagery.map(function(image) {
  return image.reduceRegions({
    collection: wiaData,
    reducer: ee.Reducer.mean(),
    scale: 30
  });
});
```

### 4.2 AWS Integration

Services mapping:
- **S3**: Long-term data storage
- **RDS**: PostgreSQL/PostGIS databases
- **IoT Core**: MQTT sensor ingestion
- **Lambda**: Serverless data processing
- **SageMaker**: ML model development

### 4.3 Azure Integration

Services mapping:
- **Blob Storage**: Data archival
- **PostgreSQL**: Managed databases
- **IoT Hub**: Device connectivity
- **Functions**: Serverless computing
- **ML Studio**: Machine learning

## 5. Statistical Computing Integration

### 5.1 R Package Specification

Required functions:
```r
# Connect to WIA API
wia <- connect_wia(api_key = "KEY")

# Query observations
obs <- get_observations(wia, taxon = "species", bbox = c(...))

# Convert to spatial object
obs_sf <- wia_to_sf(obs)

# Validate data
validate_wia(data)

# Export to WIA format
export_wia(data, "output.json")
```

### 5.2 Python Package Specification

Required classes and methods:
```python
from wia import Client

# Initialize client
client = Client(api_key='KEY')

# Query data
observations = client.get_observations(
    taxon='species',
    bbox=(...),
    start_date='2025-01-01'
)

# Convert to DataFrame
df = client.to_dataframe(observations)

# Convert to GeoDataFrame
gdf = client.to_geodataframe(observations)

# Validate
client.validate(data)

# Export
client.export(data, 'output.json')
```

## 6. Decision Support Integration

### 6.1 Marxan Integration

Workflow:
1. Export species occurrence density from WIA
2. Generate planning units
3. Define conservation features
4. Set targets based on WIA population estimates
5. Run Marxan optimization
6. Update plans with new WIA data

### 6.2 InVEST Integration

Model calibration:
- Water quality models ← WIA water monitoring
- Habitat quality ← WIA species observations
- Carbon storage ← WIA biomass measurements
- Scenario analysis ← WIA land use data

### 6.3 SMART Integration

Integration points:
- Import ranger patrols to WIA
- Combine patrol data with sensors
- Analyze effort alongside automated monitoring
- Export WIA data for threat assessment

## 7. Alert and Response Systems

### 7.1 Alert Rule Configuration

```json
{
  "alert_rule_id": "string",
  "name": "string",
  "triggers": [
    {
      "parameter": "string",
      "threshold": "number",
      "operator": "enum",
      "duration": "string"
    }
  ],
  "actions": [
    {
      "type": "email|sms|webhook",
      "recipients": ["string"],
      "template": "string",
      "url": "string"
    }
  ]
}
```

### 7.2 Emergency Management Integration

Connection points:
- Public health surveillance ← disease monitoring
- Fire management ← vegetation indices
- Water utilities ← quality monitoring
- Wildlife management ← movement tracking

## 8. Data Publication

### 8.1 DOI Assignment

Integration with:
- DataCite for DOI minting
- EZID for identifier management
- Citation metadata in multiple formats
- Usage tracking via DOI resolution

### 8.2 Repository Integration

Automated deposition to:
- **DataONE**: Environmental data network
- **Dryad**: General research data
- **Zenodo**: Open research repository
- **GBIF**: Biodiversity data
- **OBIS**: Ocean biodiversity

### 8.3 Publication Linking

Bidirectional links:
- Datasets cite publications
- Publications cite datasets
- ORCID author disambiguation
- Crossref integration

## 9. Interoperability Testing

### 9.1 Test Suite

Required integration tests:
1. GeoJSON export/import
2. Darwin Core conversion accuracy
3. API response time under load
4. WebSocket connection stability
5. Database write/read performance
6. Cloud platform deployment

### 9.2 Certification Requirements

Integration certification requires:
- Successful test suite completion
- Documentation of integrations
- Working examples
- Performance benchmarks
- Support contact information

## 10. Future Extensibility

Design for evolution:
- Plugin architecture for new platforms
- Versioned integration specifications
- Deprecation policies (12-month minimum)
- Community contribution process


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
