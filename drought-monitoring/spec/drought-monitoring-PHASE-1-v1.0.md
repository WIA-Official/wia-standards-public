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
    "calculation_method": "선행 연구",
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
- 선행 연구. The Relationship of Drought Frequency and Duration to Time Scales
- 선행 연구. Crop Evapotranspiration - FAO Irrigation and Drainage Paper 56

## License

This specification is released under Creative Commons CC0 1.0 Universal (Public Domain).

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Benefit All Humanity)
