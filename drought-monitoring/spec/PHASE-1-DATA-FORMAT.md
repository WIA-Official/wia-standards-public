# WIA Drought Monitoring Standard - Phase 1: Data Format Specification v1.0

## Overview

Phase 1 of the WIA Drought Monitoring Standard establishes standardized data formats for drought indices and monitoring parameters. This specification ensures interoperability between different drought monitoring systems worldwide.

**Version:** 1.0.0
**Status:** Published
**Last Updated:** 2025-12-26
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Core Principles

1. **Human Readable:** JSON-based formats understandable without specialized tools
2. **Machine Processable:** Strict schemas enabling automated validation
3. **Extensible:** New fields can be added without breaking existing implementations
4. **Self-Documenting:** Data structures include metadata explaining contents
5. **Efficient:** Minimal size while maintaining clarity

## Universal Header Structure

All WIA drought data objects MUST include these base fields:

```json
{
  "wia_version": "1.0",
  "data_type": "string",
  "timestamp": "ISO 8601 date-time",
  "location": {
    "type": "Point" | "Polygon",
    "coordinates": [...],
    "properties": {...}
  },
  "metadata": {
    "source_organization": "string",
    "processing_date": "ISO 8601 date-time",
    "quality_flag": "excellent" | "good" | "fair" | "poor" | "invalid",
    "confidence_score": 0.0-1.0
  }
}
```

## Palmer Drought Severity Index (PDSI) Schema

### PDSI Data Structure

```json
{
  "wia_version": "1.0",
  "data_type": "pdsi",
  "timestamp": "2025-12-26T00:00:00Z",
  "location": {
    "type": "Point",
    "coordinates": [-74.0060, 40.7128],
    "properties": {
      "region": "Northeast US",
      "elevation_m": 10
    }
  },
  "pdsi": {
    "value": -2.5,
    "classification": "moderate_drought",
    "range": [-10.0, 10.0],
    "percentile": 15.3
  },
  "parameters": {
    "precipitation_mm": 45.2,
    "temperature_c": 15.3,
    "potential_evapotranspiration_mm": 114.0,
    "soil_moisture_mm": 87.5
  },
  "metadata": {
    "source_organization": "NOAA NCEI",
    "calculation_method": "Palmer 1965",
    "processing_date": "2025-12-26T12:00:00Z",
    "quality_flag": "good",
    "confidence_score": 0.87
  }
}
```

### PDSI Classification Thresholds

| PDSI Range | Classification Code | Label | Description |
|------------|-------------------|-------|-------------|
| ≥ 4.0 | extremely_wet | Extremely Wet | Exceptionally moist conditions |
| 3.0 to 3.99 | very_wet | Very Wet | Significantly above normal |
| 2.0 to 2.99 | moderately_wet | Moderately Wet | Above normal moisture |
| 1.0 to 1.99 | slightly_wet | Slightly Wet | Mildly above normal |
| -0.99 to 0.99 | near_normal | Near Normal | Normal conditions |
| -1.99 to -1.0 | mild_drought | Mild Drought | Incipient drought |
| -2.99 to -2.0 | moderate_drought | Moderate Drought | Established drought |
| -3.99 to -3.0 | severe_drought | Severe Drought | Serious impacts |
| ≤ -4.0 | extreme_drought | Extreme Drought | Exceptional drought |

## Standardized Precipitation Index (SPI) Schema

### SPI Data Structure

```json
{
  "wia_version": "1.0",
  "data_type": "spi",
  "timestamp": "2025-12-26T00:00:00Z",
  "location": {
    "type": "Point",
    "coordinates": [-119.4179, 36.7783]
  },
  "spi": {
    "value": -1.8,
    "time_scale_months": 12,
    "classification": "moderate_drought",
    "probability": 0.036,
    "percentile": 3.6
  },
  "precipitation": {
    "period_total_mm": 285.4,
    "historical_mean_mm": 450.2,
    "percent_of_normal": 63.4
  },
  "metadata": {
    "calculation_method": "관련 분야 자료",
    "reference_period": "1991-2020",
    "quality_flag": "good"
  }
}
```

## Soil Moisture Schema

### Soil Moisture Structure

```json
{
  "wia_version": "1.0",
  "data_type": "soil_moisture",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": {
    "type": "Point",
    "coordinates": [-96.7970, 32.7767]
  },
  "soil_moisture": {
    "measurement_type": "volumetric",
    "unit": "percent",
    "layers": [
      {
        "depth_range_cm": [0, 10],
        "moisture_percent": 12.5,
        "field_capacity_percent": 35.0,
        "wilting_point_percent": 15.0,
        "stress_level": "high"
      }
    ]
  },
  "metadata": {
    "method": "TDR",
    "quality_flag": "good"
  }
}
```

## NDVI Vegetation Index Schema

### NDVI Data Structure

```json
{
  "wia_version": "1.0",
  "data_type": "ndvi",
  "timestamp": "2025-12-20T10:30:00Z",
  "location": {
    "type": "Polygon",
    "coordinates": [[...]]
  },
  "ndvi": {
    "value": 0.45,
    "range": [-1.0, 1.0],
    "anomaly": -0.15,
    "percentile": 22.5
  },
  "satellite_info": {
    "satellite": "MODIS Terra",
    "sensor": "MOD13Q1",
    "resolution_m": 250
  },
  "metadata": {
    "quality_flag": "good"
  }
}
```

## Validation Requirements

All WIA Phase 1 compliant data MUST:

1. Validate against provided JSON schemas with zero errors
2. Include all required fields (wia_version, data_type, timestamp, location, metadata)
3. Use ISO 8601 format for all timestamps (UTC timezone)
4. Include quality metadata (quality_flag, confidence_score)
5. Provide data provenance (source_organization, processing_date)

## Implementation Checklist

- [ ] Data validates against WIA JSON schemas
- [ ] All required header fields present
- [ ] Timestamps in ISO 8601 UTC format
- [ ] Location uses GeoJSON format
- [ ] Quality metadata included
- [ ] Classification codes match standard values
- [ ] Units clearly specified
- [ ] Sample dataset prepared (100+ records)

## Conformance

An implementation conforms to WIA Phase 1 if:

1. All published drought data validates against WIA JSON schemas
2. Required metadata fields are present and accurate
3. Classification codes match standard definitions
4. Sample dataset validates with 100% success rate

## References

- Palmer, W.C. (1965). Meteorological Drought. Research Paper No. 45, U.S. Weather Bureau

## License

This specification is released under Creative Commons CC0 1.0 Universal (Public Domain).

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Benefit All Humanity)


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
