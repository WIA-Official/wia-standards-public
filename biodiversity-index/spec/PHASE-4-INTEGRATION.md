# WIA Biodiversity Index Standard - Phase 4: Integration
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity, Preserve All Life

## Overview

Phase 4 ensures WIA-standardized biodiversity data flows seamlessly into decision-making systems. This specification covers integration with GIS platforms, conservation planning tools, policy reporting frameworks, and environmental management systems.

## GIS Platform Integration

### ArcGIS Integration

**WIA ArcGIS Toolbox:**
- **Installation:** ArcGIS Pro 2.9+ or ArcMap 10.8+
- **Location:** `https://github.com/WIA-Official/arcgis-biodiversity-tools`

**Tools Provided:**
1. **Biodiversity Hotspot Identifier** - Calculate and map diversity hotspots
2. **Species Distribution Mapper** - Generate species distribution maps
3. **Temporal Trend Analyzer** - Visualize diversity trends over time
4. **Protected Area Assessment** - Compare diversity inside/outside protected areas
5. **Habitat Suitability Modeler** - Model species habitat preferences

**Example Usage (Python Toolbox):**
```python
import arcpy
from wia_biodiversity import BiodiversityAPI

# Calculate biodiversity hotspots
result = arcpy.BiodiversityHotspots_wia(
    region="Amazon_Basin",
    index_type="Shannon",
    cell_size_km=5,
    threshold_percentile=90,
    output_fc="C:/GIS/hotspots.shp"
)
```

### QGIS Plugin

**WIA Biodiversity Browser:**
- **Installation:** QGIS 3.16+ via Plugin Manager
- **Repository:** https://plugins.qgis.org/plugins/wia-biodiversity

**Features:**
- Query WIA API directly from QGIS interface
- Load occurrences as vector layers
- Calculate diversity indices for selected polygons
- Export to standard formats (Shapefile, GeoPackage, GeoJSON)

**Python API:**
```python
from qgis.core import QgsVectorLayer, QgsProject
from wia_qgis import WIABiodiversity

# Load occurrences from WIA API
wia = WIABiodiversity(api_key='your_key')
layer = wia.load_occurrences(
    species='Panthera tigris',
    year=2025,
    layer_name='Tiger Occurrences 2025'
)

QgsProject.instance().addMapLayer(layer)
```

## Conservation Planning Integration

### Marxan Integration

**WIA-to-Marxan Converter:**
```python
from wia_biodiversity import BiodiversityAPI, MarxanConverter

client = BiodiversityAPI(api_key='your_key')
converter = MarxanConverter()

# Define planning region
region = client.spatial.define_region(
    polygon=[[lon, lat], ...],
    planning_unit_size_km2=100
)

# Generate Marxan input files
converter.generate_marxan_files(
    region=region,
    output_dir='marxan_inputs/',
    species_target=0.3,  # 30% representation
    cost_metric='area'
)

# Creates: spec.dat, puvspr.dat, pu.dat
```

**Output Files:**
- `spec.dat` - Species list with conservation targets
- `puvspr.dat` - Planning units vs species matrix
- `pu.dat` - Planning unit costs and status
- `bound.dat` - Boundary length between units

### Zonation Integration

**Spatial Conservation Prioritization:**
```python
from wia_biodiversity import ZonationConverter

converter = ZonationConverter()

# Generate rasterized species distributions
converter.create_species_layers(
    species_list=['Panthera tigris', 'Elephas maximus', ...],
    resolution_m=1000,
    output_dir='zonation_inputs/',
    format='geotiff'
)

# Generate settings file
converter.create_settings_file(
    species_weights='iucn_based',  # Weight by IUCN status
    analysis_type='core_area',
    output_file='zonation_settings.dat'
)
```

## Policy Reporting Integration

### CBD (Convention on Biological Diversity)

**Automated Aichi Target Reporting:**
```python
from wia_biodiversity import ReportingFramework

reporter = ReportingFramework(api_key='your_key')

# Generate Aichi Target 12 report
target_12 = reporter.cbd.aichi_target_12(
    country='BR',
    period=('2020-01-01', '2025-12-31')
)

# Export as official PDF
reporter.export(
    target_12,
    format='pdf',
    template='cbd_official'
)
```

**Indicators Generated:**
- Red List Index (RLI)
- Threatened species trends by IUCN category
- Extinction prevention actions
- Species recovery programs
- Protected area coverage

### IPBES (Intergovernmental Science-Policy Platform)

**Nature's Contributions to People (NCP):**
```python
# Map biodiversity to ecosystem services
ncpncp_report = reporter.ipbes.ncp_assessment(
    region='Southeast_Asia',
    ncp_categories=[
        'food_feed',
        'materials_companionship',
        'pollination_seed_dispersal'
    ]
)
```

**Assessment Components:**
- Drivers of change analysis
- Biodiversity trend indicators
- Ecosystem service mapping
- Scenario modeling integration
- Indigenous knowledge synthesis

### National Reporting

**Automated Report Generation:**
```python
# National biodiversity report
national_report = reporter.generate_national_report(
    country='KE',
    year=2025,
    sections=[
        'state_and_trends',
        'threats_and_pressures',
        'conservation_actions',
        'policy_integration'
    ],
    language='en'
)
```

## Environmental Impact Assessment (EIA)

### Baseline Assessment

```python
from wia_biodiversity import EIAModule

eia = EIAModule(api_key='your_key')

# Define project area and impact zones
project_site = {
    "type": "Polygon",
    "coordinates": [[project boundary]]
}

impact_zones = {
    "direct_impact": project_site,
    "indirect_500m": eia.spatial.buffer(project_site, 500),
    "regional_5km": eia.spatial.buffer(project_site, 5000)
}

# Baseline assessment
baseline = eia.baseline_assessment(
    zones=impact_zones,
    time_period=('2020-01-01', '2025-12-31'),
    taxa=['birds', 'mammals', 'amphibians', 'plants']
)
```

**Baseline Report Includes:**
- Species richness by zone
- Shannon diversity with confidence intervals
- Protected/threatened species list
- Habitat types and condition
- Seasonal variation
- Critical habitat identification

### Impact Prediction

```python
# Predict impacts
impact_prediction = eia.predict_impacts(
    baseline=baseline,
    habitat_loss_percent={'tropical_forest': 35, 'wetland': 12},
    noise_disturbance_radius_m=800,
    lighting_disturbance=True
)

# Mitigation recommendations
mitigation = eia.recommend_mitigation(
    impacts=impact_prediction,
    objectives=['no_net_loss', 'threatened_species_protection']
)
```

## Alert and Monitoring Systems

### Real-Time Endangered Species Alerts

```python
from wia_biodiversity import AlertSystem

alerts = AlertSystem(api_key='your_key')

# Subscribe to critically endangered detections
alerts.subscribe(
    event_type='species_detection',
    filters={
        'iucn_status': ['CR', 'EN'],
        'regions': ['Southeast_Asia'],
        'detection_methods': ['camera_trap', 'edna']
    },
    notification_channels=['email', 'webhook']
)

# Webhook handler
@app.route('/biodiversity-alerts', methods=['POST'])
def handle_alert():
    data = request.json
    if data['species']['iucn_status'] == 'CR':
        patrol_system.dispatch_patrol(
            location=data['location'],
            priority='high'
        )
```

### Biodiversity Threshold Monitoring

```python
# Set up threshold monitoring
alerts.threshold_monitor(
    metric='shannon_diversity',
    region='Yellowstone_National_Park',
    calculation_frequency='monthly',
    threshold_value=3.2,
    threshold_direction='below',
    consecutive_violations=2
)
```

## Mobile Field App Integration

### Supported Platforms

| App | Platform | Integration Method | Use Case |
|-----|----------|-------------------|----------|
| iNaturalist | iOS/Android | API sync | Citizen science |
| eBird | iOS/Android | Data export + conversion | Bird monitoring |
| Survey123 (Esri) | iOS/Android | Feature service sync | Structured surveys |
| ODK Collect | Android | XForm submission | Offline surveys |
| WIA Field App | iOS/Android | Native WIA integration | Full compliance |

### Data Flow

1. Field app records observation
2. Real-time validation against WIA schema
3. Photo/audio evidence attached
4. GPS coordinates verified
5. Sync to WIA database (online) or queue (offline)
6. Automated quality checks
7. Expert review if flagged
8. Data available via API

## Dashboard Integration

### Protected Area Dashboard

**Components:**
- Species richness heat map
- Temporal trend charts (diversity indices)
- Threatened species tracker
- Habitat condition overlay (satellite + ground truth)
- Management effectiveness metrics

**Tech Stack:**
- Frontend: React + Mapbox GL JS
- Backend: WIA API + PostgreSQL/PostGIS
- Updates: Real-time via WebSocket

### National Biodiversity Dashboard

**Key Visualizations:**
- CBD/Kunming target progress indicators
- Red List Index trend line
- Ecosystem coverage in protected areas
- Data gap maps (regions/taxa undersampled)
- Conservation funding vs. outcomes correlation

## Data Export Formats

| Format | Use Case | Endpoint |
|--------|----------|----------|
| Darwin Core Archive | GBIF submission | `/export/dwca` |
| GeoJSON | GIS mapping | `/export/geojson` |
| CSV | Statistical analysis | `/export/csv` |
| Shapefile | Desktop GIS | `/export/shapefile` |
| KML | Google Earth | `/export/kml` |
| Parquet | Big data analytics | `/export/parquet` |

## Compliance Requirements

### Platinum Certification
- Phase 4 integrations operational (GIS + planning + policy)
- Contributions to WIA standard development
- Demonstrated conservation impact using WIA data
- Mentorship of other organizations

## Performance Considerations

**Optimization:**
- Spatial indexes (R-tree, QuadTree) for fast queries
- Materialized views for common aggregations
- CDN caching for static map tiles
- Async processing for large exports
- Database partitioning by region/date

**Scalability:**
- Horizontal scaling via load balancer
- Database read replicas
- Message queues for background jobs
- Microservices architecture for independent scaling

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity · Preserve All Life


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
