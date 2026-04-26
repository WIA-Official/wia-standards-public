# WIA-ENE-062: Glacier Preservation
## Phase 4 - Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

---

## Overview

This document defines integration patterns and interoperability standards for connecting glacier preservation systems with satellite networks, climate models, water resource management systems, and other WIA standards.

## System Architecture

### High-Level Integration Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA-ENE-062 Core Platform                     │
├─────────────────────────────────────────────────────────────────┤
│  Glacier Registry │ Measurement DB │ Analytics │ Alert System  │
└──────────┬──────────────────┬──────────────┬────────────────────┘
           │                  │              │
    ┌──────┴──────┐   ┌──────┴──────┐   ┌──┴─────────┐
    │  Satellite  │   │   Climate   │   │   Water    │
    │   Systems   │   │   Models    │   │  Resource  │
    └─────────────┘   └─────────────┘   └────────────┘
```

## Satellite System Integration

### 1. Earth Observation Satellites

#### Supported Satellite Systems

**Optical Imaging:**

- **Landsat 8/9** - 30m resolution, 16-day revisit
- **Sentinel-2 A/B** - 10m resolution, 5-day revisit
- **MODIS** - 250m resolution, daily coverage
- **Planet Labs** - 3m resolution, daily coverage

**Radar (SAR):**

- **Sentinel-1 A/B** - C-band, 10m resolution, 6-12 day revisit
- **RADARSAT-2** - Multi-polarization, variable resolution
- **ALOS-2 PALSAR-2** - L-band, 3-10m resolution

**Altimetry:**

- **ICESat-2** - Laser altimetry, cm-level precision
- **CryoSat-2** - Radar altimetry, polar focus

**Gravity:**

- **GRACE-FO** - Gravity anomalies, monthly resolution

#### Integration Protocol

**1. Automated Data Retrieval**

```yaml
integration:
  type: pull
  service: USGS_EarthExplorer, ESA_Copernicus_Hub
  authentication: OAuth2

  search_criteria:
    area_of_interest: glacier_boundaries
    cloud_cover_max: 20
    snow_cover_min: 50
    products:
      - Landsat_8_L2
      - Sentinel-2_L2A
      - Sentinel-1_GRD

  schedule:
    frequency: daily
    time: 02:00_UTC

  processing:
    - download_scene
    - extract_glacier_area
    - calculate_indices (NDSI, NDVI, albedo)
    - update_glacier_record
    - trigger_alerts_if_anomaly
```

**2. API Integration Example (Sentinel Hub)**

```javascript
const axios = require('axios');

async function fetchSentinelData(glacierId, date) {
  const glacier = await getGlacierBoundary(glacierId);

  const evalscript = `
    //VERSION=3
    function setup() {
      return {
        input: ["B02", "B03", "B04", "B11"],
        output: { bands: 4 }
      };
    }
    function evaluatePixel(sample) {
      // Calculate NDSI for snow/ice detection
      let ndsi = (sample.B03 - sample.B11) / (sample.B03 + sample.B11);
      return [sample.B04, sample.B03, sample.B02, ndsi];
    }
  `;

  const response = await axios.post(
    'https://services.sentinel-hub.com/api/v1/process',
    {
      input: {
        bounds: {
          bbox: glacier.boundingBox,
          properties: { crs: 'EPSG:4326' }
        },
        data: [{
          type: 'sentinel-2-l2a',
          dataFilter: {
            timeRange: {
              from: date,
              to: date
            },
            maxCloudCoverage: 20
          }
        }]
      },
      evalscript: evalscript,
      output: {
        width: 512,
        height: 512,
        responses: [{
          identifier: 'default',
          format: { type: 'image/tiff' }
        }]
      }
    },
    {
      headers: {
        'Authorization': `Bearer ${SENTINEL_HUB_TOKEN}`,
        'Content-Type': 'application/json'
      }
    }
  );

  return processImagery(response.data);
}
```

**3. Satellite Tasking Integration**

For commercial satellites with tasking capability:

```json
{
  "taskingRequest": {
    "satellite": "Planet_SkySat",
    "priority": "high",
    "target": {
      "glacierId": "GLR-001-HIM",
      "boundingBox": {
        "north": 31.0,
        "south": 30.8,
        "east": 79.2,
        "west": 79.0
      }
    },
    "requirements": {
      "resolution": "0.5m",
      "cloudCover": "< 10%",
      "sunElevation": "> 30°",
      "viewAngle": "< 15°"
    },
    "acquisitionWindow": {
      "start": "2025-12-26T06:00:00Z",
      "end": "2025-12-26T18:00:00Z"
    },
    "deliveryFormat": "GeoTIFF",
    "processingLevel": "Analytic_SR"
  }
}
```

### 2. Ground Station Integration

For direct satellite downlinks:

```yaml
ground_station:
  provider: AWS_Ground_Station
  locations:
    - Ohio
    - Bahrain
    - Sydney

  satellite_passes:
    - satellite: NOAA-20
      frequency: 7812_MHz
      antenna: S-band
      duration: 12_minutes
      data_rate: 15_Mbps

  processing:
    realtime: true
    pipeline:
      - demodulate
      - decode_CCSDS
      - extract_VIIRS_data
      - calibrate
      - georeference
      - publish_to_API
```

## Climate Model Integration

### 1. Global Climate Models (GCMs)

#### CMIP6 Integration

```python
import xarray as xr
import intake

# Connect to CMIP6 data catalog
cat = intake.open_esm_datastore(
    'https://storage.googleapis.com/cmip6/pangeo-cmip6.json'
)

# Query for glacier-relevant data
query = cat.search(
    experiment_id=['historical', 'ssp245', 'ssp585'],
    variable_id=['tas', 'pr', 'rsds'],  # temp, precip, radiation
    table_id='Amon',
    source_id=['CESM2', 'MPI-ESM1-2-HR', 'UKESM1-0-LL']
)

# Load data for glacier region
datasets = query.to_dataset_dict(
    zarr_kwargs={'consolidated': True}
)

# Extract glacier location data
glacier_coords = get_glacier_coordinates('GLR-001-HIM')
climate_data = datasets['tas'].sel(
    lat=glacier_coords['lat'],
    lon=glacier_coords['lon'],
    method='nearest'
)

# Calculate temperature projections
temp_projection = climate_data.mean(dim='time').values
```

#### Regional Climate Model (RCM) Integration

```yaml
rcm_integration:
  model: WRF-ARW
  domain: Himalayan_region
  resolution: 3km

  boundary_conditions:
    source: ERA5_reanalysis
    update_frequency: 6_hours

  glacier_physics:
    - snow_albedo_feedback
    - debris_cover_effect
    - topographic_shading
    - katabatic_winds

  output:
    variables: [T2, PREC, SWDOWN, LWDOWN, U10, V10]
    frequency: hourly
    format: NetCDF
    destination: s3://wia-glacier/rcm-output/
```

### 2. Glacier Mass Balance Models

#### Integration with OGGM (Open Global Glacier Model)

```python
from oggm import cfg, workflow, tasks, graphics

# Initialize OGGM
cfg.initialize()

# Define glacier from WIA registry
glacier_id = 'GLR-001-HIM'
glacier_data = wia_api.get_glacier(glacier_id)

# Convert to OGGM format
rgi_id = convert_wia_to_rgi(glacier_id)

# Set up working directory
cfg.PATHS['working_dir'] = '/path/to/oggm_workdir'

# Download and process glacier data
gdirs = workflow.init_glacier_directories([rgi_id])

# Run preprocessing tasks
workflow.gis_prepro_tasks(gdirs)
workflow.climate_tasks(gdirs)

# Run mass balance model
for gdir in gdirs:
    tasks.apparent_mb_from_any_mb(gdir)
    tasks.run_random_climate(gdir, nyears=100, seed=42)

# Extract results and push to WIA API
for gdir in gdirs:
    results = {
        'glacierId': glacier_id,
        'model': 'OGGM',
        'projection': {
            'year2050': gdir.read_pickle('model_run')['volume'][50],
            'year2100': gdir.read_pickle('model_run')['volume'][100]
        }
    }
    wia_api.submit_projection(results)
```

### 3. Climate Data Sync

Periodic synchronization with climate data sources:

```json
{
  "syncSchedule": {
    "frequency": "daily",
    "time": "03:00 UTC"
  },
  "dataSources": [
    {
      "name": "ERA5",
      "type": "reanalysis",
      "parameters": ["temperature", "precipitation", "radiation"],
      "resolution": "0.25°",
      "latency": "5 days"
    },
    {
      "name": "MERRA-2",
      "type": "reanalysis",
      "parameters": ["temperature", "wind", "humidity"],
      "resolution": "0.5° x 0.625°",
      "latency": "2 weeks"
    },
    {
      "name": "CMIP6_ensemble",
      "type": "projection",
      "scenarios": ["SSP2-4.5", "SSP5-8.5"],
      "models": 15,
      "period": "2025-2100"
    }
  ],
  "processing": {
    "spatialInterpolation": "bilinear",
    "temporalAggregation": "daily_mean",
    "storage": "TimescaleDB"
  }
}
```

## Water Resource Management Integration

### 1. Hydrological Models

#### Integration with SWAT (Soil & Water Assessment Tool)

```yaml
swat_integration:
  model_setup:
    watershed: Bhagirathi_Basin
    area: 7,950_km²
    elevation_range: 1,000-7,138_m

  glacier_input:
    source: WIA-ENE-062_API
    parameters:
      - glacier_area
      - ice_volume
      - melt_rate
      - runoff_coefficient
    update_frequency: monthly

  calibration:
    observed_streamflow: gauge_station_data
    period: 2015-2024
    metrics: [NSE, PBIAS, R²]

  scenarios:
    - current_glacier_extent
    - glacier_retreat_SSP2-4.5
    - glacier_retreat_SSP5-8.5

  output:
    streamflow: daily
    water_availability: monthly
    drought_indicators: seasonal
    destination: WIA-ENE-062_waterSupply_endpoint
```

### 2. Water Supply Forecasting

**Seasonal Water Supply Forecast API:**

```javascript
const forecastWaterSupply = async (glacierId, season) => {
  // Get glacier current state
  const glacier = await wiaApi.getGlacier(glacierId);

  // Get climate forecast
  const climateForecast = await getSeasonalForecast(
    glacier.location,
    season
  );

  // Calculate snowmelt and ice melt contributions
  const snowmelt = calculateSnowmelt(
    climateForecast.temperature,
    climateForecast.precipitation,
    glacier.snowpack
  );

  const icemelt = calculateIcemelt(
    climateForecast.temperature,
    climateForecast.radiation,
    glacier.albedo
  );

  // Total water supply
  const totalRunoff = snowmelt + icemelt;

  // Publish forecast
  return await wiaApi.submitWaterSupplyForecast({
    glacierId: glacierId,
    season: season,
    forecast: {
      totalRunoff: totalRunoff,
      snowmeltContribution: snowmelt,
      icemeltContribution: icemelt,
      confidence: 0.78,
      scenarioSSP: 'SSP2-4.5'
    }
  });
};
```

### 3. Reservoir Management Integration

```json
{
  "reservoirId": "RES-BHA-001",
  "name": "Tehri Dam",
  "glacierDependency": {
    "primaryGlaciers": ["GLR-001-HIM", "GLR-002-HIM"],
    "contributionPercent": 35.2,
    "seasonalVariation": {
      "summer": 58.3,
      "winter": 12.1
    }
  },
  "integration": {
    "dataFeed": "WIA-ENE-062_API",
    "updateFrequency": "daily",
    "forecastHorizon": "6_months",
    "triggers": [
      {
        "condition": "glacierMeltRate > 3x_normal",
        "action": "increase_reservoir_buffer",
        "magnitude": "15%"
      },
      {
        "condition": "glacierMassLoss > 10%",
        "action": "revise_annual_water_plan",
        "notification": "water_authority"
      }
    ]
  }
}
```

## Integration with Other WIA Standards

### 1. WIA-ENE-001 (Energy Measurement)

Integration for hydropower impact assessment:

```yaml
energy_integration:
  standard: WIA-ENE-001
  use_case: hydropower_generation_forecast

  data_exchange:
    from_glacier_preservation:
      - water_availability_forecast
      - seasonal_runoff_pattern
      - long_term_flow_reduction

    to_energy_measurement:
      - hydropower_generation_capacity
      - seasonal_energy_availability
      - grid_stability_impact

  api_mapping:
    glacier_water_supply: /api/v1/glaciers/{id}/water-supply
    energy_forecast: /api/v1/energy/hydropower/{plant_id}/forecast
```

### 2. WIA-CLIMATE (Climate Data Standard)

```json
{
  "integration": "WIA-CLIMATE",
  "dataSharing": {
    "glacierAsClimateIndicator": {
      "metrics": [
        "mass_balance_trend",
        "albedo_change",
        "equilibrium_line_altitude_shift"
      ],
      "frequency": "annual",
      "contribution": "early_warning_signal"
    },
    "climateDataForGlacier": {
      "metrics": [
        "regional_temperature_trend",
        "precipitation_pattern",
        "atmospheric_circulation"
      ],
      "frequency": "daily",
      "contribution": "glacier_model_forcing"
    }
  }
}
```

### 3. WIA-DISASTER (Disaster Management)

Integration for GLOF early warning:

```yaml
disaster_integration:
  standard: WIA-DISASTER
  scenario: glacier_lake_outburst_flood

  risk_assessment:
    source: WIA-ENE-062_glacier_lake_monitoring
    parameters:
      - lake_volume
      - dam_stability
      - melt_rate
      - precipitation_forecast

  early_warning:
    trigger: GLOF_risk_score > 0.7
    notification: WIA-DISASTER_alert_system
    affected_area: downstream_communities
    evacuation_time: 12_hours

  coordination:
    emergency_services: automatic_notification
    population: multi_channel_alert
    infrastructure: protective_action_triggers
```

## Data Export and Interoperability

### 1. Standard Format Support

**Climate and Forecast (CF) Conventions:**

```python
import xarray as xr
import numpy as np

def export_glacier_data_to_cf(glacier_id, start_date, end_date):
    # Fetch data from WIA API
    data = wia_api.get_measurements(glacier_id, start_date, end_date)

    # Create CF-compliant NetCDF
    ds = xr.Dataset(
        {
            'glacier_mass': (['time'], data['mass']),
            'melt_rate': (['time'], data['melt_rate']),
            'temperature': (['time'], data['temperature'])
        },
        coords={
            'time': pd.date_range(start_date, end_date, freq='D'),
            'lat': glacier['latitude'],
            'lon': glacier['longitude']
        }
    )

    # Add CF metadata
    ds['glacier_mass'].attrs = {
        'standard_name': 'land_ice_mass',
        'units': 'Gt',
        'long_name': 'Total glacier mass'
    }

    ds.attrs = {
        'Conventions': 'CF-1.8',
        'title': f'Glacier measurements for {glacier_id}',
        'institution': 'WIA - World Certification Industry Association',
        'source': 'WIA-ENE-062 Glacier Preservation Standard',
        'references': 'https://wia.org/standards/ENE-062'
    }

    # Export
    ds.to_netcdf(f'{glacier_id}_CF.nc')
```

### 2. THREDDS Data Server Integration

```xml
<catalog xmlns="http://www.unidata.ucar.edu/namespaces/thredds/InvCatalog/v1.0">
  <service name="all" base="" serviceType="compound">
    <service name="odap" serviceType="OpenDAP" base="/thredds/dodsC/" />
    <service name="http" serviceType="HTTPServer" base="/thredds/fileServer/" />
    <service name="wcs" serviceType="WCS" base="/thredds/wcs/" />
    <service name="wms" serviceType="WMS" base="/thredds/wms/" />
    <service name="ncss" serviceType="NetcdfSubset" base="/thredds/ncss/" />
  </service>

  <dataset name="WIA Glacier Preservation Data">
    <metadata inherited="true">
      <serviceName>all</serviceName>
      <authority>wia.org</authority>
      <dataType>Grid</dataType>
    </metadata>

    <datasetScan name="Glacier Mass Balance" path="glacier/mass_balance"
                 location="/data/glacier/mass_balance/">
      <filter>
        <include wildcard="*.nc"/>
      </filter>
    </datasetScan>
  </dataset>
</catalog>
```

## Third-Party System Integration

### 1. GIS Platform Integration

**ArcGIS Integration:**

```javascript
// ArcGIS Feature Service
const glacierFeatureLayer = new FeatureLayer({
  url: "https://services.wia.org/arcgis/rest/services/Glaciers/FeatureServer/0",
  outFields: ["glacierId", "name", "mass", "meltRate"],
  popupTemplate: {
    title: "{name}",
    content: `
      <b>Glacier ID:</b> {glacierId}<br>
      <b>Current Mass:</b> {mass} Gt<br>
      <b>Melt Rate:</b> {meltRate} Gt/year<br>
      <a href="https://wia.org/glacier/{glacierId}" target="_blank">View Details</a>
    `
  }
});

map.add(glacierFeatureLayer);
```

### 2. IoT Platform Integration

**Azure IoT Hub:**

```csharp
using Microsoft.Azure.Devices.Client;

public class GlacierSensorDevice
{
    private DeviceClient _deviceClient;

    public async Task SendMeasurementAsync(GlacierMeasurement measurement)
    {
        var message = new Message(
            Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(measurement))
        );

        message.Properties.Add("glacierId", measurement.GlacierId);
        message.Properties.Add("sensorType", "temperature");
        message.Properties.Add("priority", "normal");

        await _deviceClient.SendEventAsync(message);
    }
}
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA


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
