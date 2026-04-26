# WIA Drought Monitoring Standard - Phase 3: Protocol Specification v1.0

## Overview

Phase 3 specifies standard protocols for processing raw satellite and sensor data into WIA-compliant drought indices. Ensures consistent calculation methods and data quality across implementations.

**Version:** 1.0.0
**Status:** Published
**Last Updated:** 2025-12-26

## Atmospheric Correction

### Standard Method: 6S Radiative Transfer Model

**Required for:** All optical satellite data (Landsat, Sentinel-2, MODIS)

**Inputs:**
- Solar zenith angle
- Sensor zenith angle
- Relative azimuth
- Water vapor (cm)
- Ozone (atm-cm)
- Aerosol optical depth
- Surface pressure (mb)

**Quality Requirements:**
- RMSE vs ground targets: < 5%
- Residual atmospheric effects: < 2% reflectance units

**Alternative Methods (if 6S unavailable):**
- DOS (Dark Object Subtraction): ±8% accuracy
- QUAC (Quick Atmospheric Correction): ±6% accuracy
- Sen2Cor (Sentinel-2 specific): ±5% accuracy

**Validation:** Must include validation against ground reference targets when not using 6S

## Cloud Masking

### Standard Algorithm: Fmask (Function of mask)

**Process:**
1. **Stage 1:** Identify potential cloud pixels using spectral tests
2. **Stage 2:** Calculate cloud probability (0-1 scale)
3. **Stage 3:** Detect cloud shadows using geometry

**Quality Requirements:**
- Cloud detection accuracy: > 95%
- Commission error (false positives): < 3%
- Omission error (missed clouds): < 5%

**Implementation:**
```python
def cloud_mask(surface_reflectance, brightness_temp, metadata):
    # Stage 1: Potential clouds
    potential = identify_potential_clouds(
        blue, green, red, nir, swir1, swir2, thermal
    )

    # Stage 2: Cloud probability
    probability = calculate_cloud_probability(
        temperature, whiteness, brightness, variability
    )

    # Stage 3: Cloud shadows
    shadows = detect_shadows(
        cloud_mask=probability > 0.5,
        solar_geometry=metadata.solar_angles,
        cloud_height=estimate_height(brightness_temp)
    )

    return {
        'cloud_mask': probability > 0.5,
        'shadow_mask': shadows,
        'probability': probability
    }
```

## NDVI Calculation

### Standard Formula

```
NDVI = (NIR - Red) / (NIR + Red)
```

**Spectral Bands:**
- Red: 0.63-0.69 μm
- NIR: 0.76-0.90 μm

**Precision Requirement:** 0.01 NDVI units
**Cross-sensor Consistency:** ±0.02 NDVI units

### NDVI Anomaly Calculation

**Requirements:**
- Historical baseline: Minimum 10 years of data
- Window matching: ±16 days from current date
- Statistical methods: Mean, standard deviation, percentile

```python
def calculate_ndvi_anomaly(current_ndvi, historical_data, date):
    same_period = filter_same_period(historical_data, date, window_days=16)

    mean = calculate_mean(same_period)
    stddev = calculate_stddev(same_period)

    anomaly = current_ndvi - mean
    zscore = anomaly / stddev
    percentile = calculate_percentile(current_ndvi, same_period)

    return {
        'ndvi': current_ndvi,
        'anomaly': anomaly,
        'zscore': zscore,
        'percentile': percentile,
        'drought_status': classify_drought(zscore)
    }
```

## Evapotranspiration Calculation

### Standard Method: FAO-56 Penman-Monteith

**Required Inputs:**
- Temperature (min, max) in °C
- Relative humidity (min, max) in %
- Wind speed at 2m in m/s
- Solar radiation in MJ/m²/day
- Latitude, elevation, date

**Formula:**
```
ET0 = (0.408 × Δ × (Rn - G) + γ × (900/(T+273)) × u2 × (es - ea)) /
      (Δ + γ × (1 + 0.34 × u2))
```

Where:
- Δ = Slope of vapor pressure curve (kPa/°C)
- Rn = Net radiation (MJ/m²/day)
- G = Soil heat flux (MJ/m²/day, ≈0 for daily)
- γ = Psychrometric constant (kPa/°C)
- T = Mean temperature (°C)
- u2 = Wind speed at 2m (m/s)
- es = Saturation vapor pressure (kPa)
- ea = Actual vapor pressure (kPa)

**Accuracy Requirement:** RMSE < 15% vs eddy covariance measurements

### Crop ET Calculation

```python
def calculate_etc(et0, crop_type, growth_stage):
    kc = get_crop_coefficient(crop_type, growth_stage)
    ks = calculate_stress_coefficient(soil_moisture)

    etc = et0 * kc * ks

    return {
        'etc_mm_day': etc,
        'et0_mm_day': et0,
        'crop_coefficient': kc,
        'stress_coefficient': ks
    }
```

## Soil Moisture Retrieval

### L-Band Radiometry (SMAP, SMOS)

**Algorithm:** Tau-Omega Model

**Process:**
1. Remove vegetation attenuation
2. Invert dielectric model
3. Convert to volumetric moisture

**Accuracy Target:** 0.04 m³/m³ (4% volumetric)
**Validation:** Against in-situ sensor networks

**Depth:** 0-5 cm (L-band penetration)

```python
def retrieve_soil_moisture(brightness_temp, land_cover, soil_texture, vegetation):
    # Remove vegetation effect
    tau = vegetation_optical_depth(vegetation.water_content)
    tsoil = remove_vegetation_effect(brightness_temp, tau)

    # Invert dielectric model
    dielectric = calculate_dielectric(tsoil, soil_texture)

    # Convert to moisture
    moisture = dielectric_to_moisture(dielectric, soil_texture)

    return {
        'soil_moisture_percent': moisture * 100,
        'depth_cm': [0, 5],
        'uncertainty_percent': calculate_uncertainty(...)
    }
```

## Quality Control

### Required Quality Checks

| Check | Method | Threshold | Action if Failed |
|-------|--------|-----------|-----------------|
| Range validation | Physical limits | Index-specific | Flag as invalid |
| Spatial consistency | Neighbor comparison | Z-score < 3.0 | Flag for review |
| Temporal consistency | Time series | Change < 2σ | Flag rapid changes |
| Cross-validation | Multiple indices | Agreement > 80% | Lower confidence |
| Ground truth | Field data | Within RMSE | Adjust algorithms |

### Uncertainty Quantification

**Components:**
- Sensor uncertainty (from specifications)
- Atmospheric correction error (±3%)
- Cloud mask error (±2%)
- Algorithm error (method-specific)
- Validation RMSE

**Combined Uncertainty:** Root sum of squares

```python
def quantify_uncertainty(drought_index, processing_chain):
    components = {
        'sensor': get_sensor_uncertainty(),
        'atmospheric': 0.03,
        'cloud_mask': 0.02,
        'algorithm': get_algorithm_uncertainty(),
        'validation': get_validation_rmse()
    }

    total = sqrt(sum(c**2 for c in components.values()))

    return {
        'uncertainty': total,
        'components': components,
        'confidence': uncertainty_to_confidence(total)
    }
```

## Data Fusion

### Multi-Sensor Fusion

**Method:** Precision-weighted averaging

```python
def fuse_drought_indices(indices, uncertainties):
    weights = [1/(u**2) for u in uncertainties]
    total_weight = sum(weights)

    fused_value = sum(i*w for i,w in zip(indices, weights)) / total_weight
    fused_uncertainty = sqrt(1 / total_weight)

    return {
        'fused_index': fused_value,
        'uncertainty': fused_uncertainty,
        'weights': [w/total_weight for w in weights]
    }
```

## Implementation Requirements

Phase 3 compliant systems MUST:

1. Use 6S or approved alternative for atmospheric correction
2. Implement Fmask or equivalent cloud detection (>95% accuracy)
3. Calculate NDVI using standard formula with 0.01 precision
4. Use FAO-56 Penman-Monteith for ET0
5. Perform all 5 quality control checks
6. Quantify and report uncertainty
7. Validate against ground truth networks

## Conformance Checklist

- [ ] Atmospheric correction: RMSE < 5%
- [ ] Cloud detection: Accuracy > 95%
- [ ] NDVI precision: 0.01 units
- [ ] ET0 accuracy: RMSE < 15%
- [ ] All 5 QC checks implemented
- [ ] Uncertainty quantified
- [ ] Ground truth validation performed
- [ ] Processing methods documented

## Supported Satellites

| Satellite | Sensor | Products | Resolution |
|-----------|--------|----------|-----------|
| MODIS Terra/Aqua | MOD13Q1 | NDVI, EVI | 250m |
| Landsat 8/9 | OLI, TIRS | NDVI, LST | 30m |
| Sentinel-2 | MSI | NDVI | 10-20m |
| SMAP | L-band | Soil Moisture | 9km |
| SMOS | MIRAS | Soil Moisture | 25km |

---

© 2025 SmileStory Inc. / WIA
弘益人間


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.
