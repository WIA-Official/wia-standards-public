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
