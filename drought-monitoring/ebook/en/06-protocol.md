# Chapter 6: Protocols and Algorithms

## Scientific Methods for Drought Index Calculation and Data Processing

---

## 6.1 Atmospheric Correction Protocols (6S Model)

### The Need for Atmospheric Correction

Satellite sensors observe the top of the atmosphere, not the Earth's surface. The atmosphere intervenes through:

| Atmospheric Effect | Impact | Correction Approach |
|-------------------|--------|---------------------|
| Rayleigh Scattering | Adds blue haze, reduces contrast | Molecular scattering model |
| Aerosol Scattering | Haze, path radiance | Aerosol optical depth estimation |
| Absorption | Reduces signal (O₃, H₂O, CO₂) | Gaseous transmittance model |
| Adjacency Effect | Blurs boundaries | Spatial correction |

Without correction, NDVI values can vary by 20-40% due to atmospheric conditions alone, making temporal comparisons unreliable.

### 6S Radiative Transfer Model

The Second Simulation of a Satellite Signal in the Solar Spectrum (6S) provides comprehensive atmospheric correction:

```
6S Model Components:
====================

1. Atmospheric Profile
   ├── Standard atmospheres (tropical, mid-latitude, etc.)
   ├── Custom profiles (radiosondes)
   └── Satellite-derived water vapor

2. Aerosol Model
   ├── Continental
   ├── Maritime
   ├── Urban
   ├── Desert
   └── Biomass burning

3. Surface Properties
   ├── Lambertian assumption
   ├── BRDF models
   └── Elevation correction

4. Geometric Parameters
   ├── Solar zenith/azimuth
   ├── View zenith/azimuth
   └── Sun-target-sensor geometry
```

### 6S Implementation Protocol

**Step 1: Input Parameter Preparation**

```python
def prepare_6s_inputs(metadata, atmospheric_data):
    """
    Prepare input parameters for 6S atmospheric correction.

    Parameters:
    -----------
    metadata : dict
        Satellite image metadata (geometry, date, etc.)
    atmospheric_data : dict
        Atmospheric parameters (AOD, water vapor, ozone)

    Returns:
    --------
    dict : 6S input configuration
    """
    inputs = {
        'geometry': {
            'solar_zenith': metadata['sun_elevation'],
            'solar_azimuth': metadata['sun_azimuth'],
            'view_zenith': metadata['view_zenith'],
            'view_azimuth': metadata['view_azimuth']
        },
        'date': {
            'month': metadata['acquisition_date'].month,
            'day': metadata['acquisition_date'].day
        },
        'atmospheric_profile': select_atmosphere(
            metadata['latitude'],
            metadata['acquisition_date']
        ),
        'aerosol': {
            'model': determine_aerosol_model(metadata['location']),
            'optical_depth_550': atmospheric_data.get('aod_550', 0.1)
        },
        'water_vapor_gcm2': atmospheric_data.get('water_vapor', 2.0),
        'ozone_atm_cm': atmospheric_data.get('ozone', 0.3)
    }

    return inputs

def select_atmosphere(latitude, date):
    """Select appropriate atmospheric profile based on location and season."""
    month = date.month

    if abs(latitude) < 23.5:
        return 'tropical'
    elif abs(latitude) < 45:
        if (month >= 4 and month <= 9) and latitude > 0:
            return 'midlatitude_summer'
        elif (month >= 10 or month <= 3) and latitude > 0:
            return 'midlatitude_winter'
        else:
            return 'midlatitude_summer' if (month >= 10 or month <= 3) else 'midlatitude_winter'
    else:
        if (month >= 4 and month <= 9) and latitude > 0:
            return 'subarctic_summer'
        else:
            return 'subarctic_winter'
```

**Step 2: Surface Reflectance Calculation**

```python
from Py6S import SixS, AtmosProfile, AeroProfile, Geometry

def calculate_surface_reflectance(toa_radiance, wavelength, params):
    """
    Calculate surface reflectance from TOA radiance using 6S.

    Parameters:
    -----------
    toa_radiance : array
        Top-of-atmosphere radiance
    wavelength : float
        Central wavelength in micrometers
    params : dict
        6S input parameters

    Returns:
    --------
    array : Surface reflectance
    """
    # Initialize 6S
    s = SixS()

    # Set geometry
    s.geometry = Geometry.User()
    s.geometry.solar_z = params['geometry']['solar_zenith']
    s.geometry.solar_a = params['geometry']['solar_azimuth']
    s.geometry.view_z = params['geometry']['view_zenith']
    s.geometry.view_a = params['geometry']['view_azimuth']

    s.geometry.month = params['date']['month']
    s.geometry.day = params['date']['day']

    # Set atmospheric profile
    s.atmos_profile = getattr(AtmosProfile, f"Predefined{params['atmospheric_profile'].title()}")()

    # Set aerosol model
    s.aero_profile = getattr(AeroProfile, f"Predefined{params['aerosol']['model'].title()}")()
    s.aot550 = params['aerosol']['optical_depth_550']

    # Set wavelength
    s.wavelength = Wavelength(wavelength)

    # Run 6S
    s.run()

    # Extract correction coefficients
    xa = s.outputs.coef_xa
    xb = s.outputs.coef_xb
    xc = s.outputs.coef_xc

    # Apply correction: y = xa * radiance - xb
    # surface_reflectance = y / (1 + xc * y)
    y = xa * toa_radiance - xb
    surface_reflectance = y / (1 + xc * y)

    # Clamp to valid range
    surface_reflectance = np.clip(surface_reflectance, 0, 1)

    return surface_reflectance
```

### 6S Quality Control

| QC Check | Threshold | Action |
|----------|-----------|--------|
| AOD availability | AOD data exists | Use climatology if missing |
| AOD range | 0 < AOD < 3.0 | Flag extreme values |
| Reflectance range | 0 ≤ ρ ≤ 1.0 | Clamp and flag |
| Solar zenith | < 75° | Flag high zenith angles |
| Adjacency correction | Apply if heterogeneous | Optional correction |

---

## 6.2 Cloud Masking Algorithms (Fmask)

### Cloud Detection Challenges

Accurate cloud masking is critical for vegetation monitoring. Clouds and cloud shadows can cause:

- False drought signals (shadows appear as stressed vegetation)
- False recovery signals (bright clouds affect compositing)
- Data gaps if too aggressive
- Data contamination if too conservative

### Fmask Algorithm Overview

The Function of mask (Fmask) algorithm uses a multi-step approach:

```
Fmask Processing Flow:
======================

Step 1: Potential Cloud Layer (PCL)
├── Basic cloud tests (brightness, temperature)
├── Whiteness test (cloud vs bright surface)
├── HOT (Haze Optimized Transformation)
└── NIR/SWIR test (snow/cloud discrimination)

Step 2: Cloud Probability
├── Land/water discrimination
├── Temperature probability
├── Spectral variability probability
└── Combined probability

Step 3: Cloud Shadow Detection
├── Project clouds using solar geometry
├── Match shadows with clouds
├── Flood-fill potential shadows
└── Validate shadow geometry

Step 4: Snow/Ice Detection
├── NDSI calculation
├── Temperature threshold
└── NIR brightness test

Step 5: Final Classification
├── Combine all masks
├── Apply probability thresholds
├── Buffer cloud edges
└── Generate QA flags
```

### Fmask Implementation

```python
def fmask_cloud_detection(bands, metadata, params):
    """
    Implement Fmask cloud detection algorithm.

    Parameters:
    -----------
    bands : dict
        Dictionary of spectral bands (blue, green, red, nir, swir1, swir2, thermal)
    metadata : dict
        Image metadata (solar geometry, date, etc.)
    params : dict
        Fmask parameters

    Returns:
    --------
    dict : Cloud, shadow, snow, water masks
    """
    # Extract bands
    blue = bands['blue']
    green = bands['green']
    red = bands['red']
    nir = bands['nir']
    swir1 = bands['swir1']
    swir2 = bands['swir2']
    bt = bands['thermal']  # Brightness temperature

    # Basic cloud test - bright and cold
    basic_cloud = (
        (blue > params.get('blue_threshold', 0.15)) &
        (bt < params.get('temp_threshold_high', 300))
    )

    # Whiteness test (clouds are spectrally flat)
    mean_vis = (blue + green + red) / 3
    whiteness = (
        np.abs(blue - mean_vis) +
        np.abs(green - mean_vis) +
        np.abs(red - mean_vis)
    ) / mean_vis

    white_cloud = whiteness < params.get('whiteness_threshold', 0.7)

    # HOT (Haze Optimized Transformation)
    hot = blue - 0.5 * red - 0.08
    hot_cloud = hot > params.get('hot_threshold', 0.0)

    # NIR/SWIR test for cirrus
    cirrus_test = (nir / swir1) > params.get('cirrus_ratio', 0.75)

    # Combine potential cloud layer
    potential_cloud = basic_cloud & white_cloud & (hot_cloud | cirrus_test)

    # Temperature probability
    t_prob = calculate_temperature_probability(bt, metadata)

    # Spectral variability probability
    ndsi = (green - swir1) / (green + swir1)
    ndvi = (nir - red) / (nir + red)

    spectral_prob = calculate_spectral_probability(ndsi, ndvi, nir, bt)

    # Combined cloud probability
    cloud_prob = t_prob * spectral_prob

    # Apply threshold
    cloud_mask = (potential_cloud) & (cloud_prob > params.get('cloud_prob_threshold', 0.225))

    # Buffer cloud edges
    cloud_mask = binary_dilation(cloud_mask, iterations=params.get('cloud_buffer', 3))

    # Cloud shadow detection
    shadow_mask = detect_cloud_shadows(
        cloud_mask, bt, nir,
        metadata['solar_zenith'],
        metadata['solar_azimuth'],
        params
    )

    # Snow detection
    snow_mask = (
        (ndsi > 0.15) &
        (nir > 0.11) &
        (green > 0.1) &
        (bt < 283)
    )

    # Water detection
    ndwi = (green - nir) / (green + nir)
    water_mask = (ndwi > 0.0) & (nir < 0.15)

    return {
        'cloud': cloud_mask,
        'shadow': shadow_mask,
        'snow': snow_mask,
        'water': water_mask,
        'cloud_probability': cloud_prob,
        'clear_land': ~(cloud_mask | shadow_mask | snow_mask | water_mask)
    }


def detect_cloud_shadows(cloud_mask, bt, nir, solar_zenith, solar_azimuth, params):
    """
    Detect cloud shadows using cloud projection.

    Parameters:
    -----------
    cloud_mask : array
        Binary cloud mask
    bt : array
        Brightness temperature
    nir : array
        NIR reflectance
    solar_zenith : float
        Solar zenith angle in degrees
    solar_azimuth : float
        Solar azimuth angle in degrees
    params : dict
        Algorithm parameters

    Returns:
    --------
    array : Binary shadow mask
    """
    # Estimate cloud heights from temperature
    lapse_rate = 6.5  # K/km
    surface_temp = np.percentile(bt[~cloud_mask], 95)
    cloud_heights = (surface_temp - bt) / lapse_rate * 1000  # meters

    # Calculate shadow projection direction
    shadow_direction = (solar_azimuth + 180) % 360
    shadow_distance = cloud_heights * np.tan(np.radians(solar_zenith))

    # Project shadows
    potential_shadows = project_shadows(cloud_mask, shadow_direction, shadow_distance)

    # Validate shadows (dark in NIR)
    nir_threshold = np.percentile(nir[~cloud_mask], 17.5)
    dark_pixels = nir < nir_threshold

    # Match projected shadows with dark pixels
    shadow_mask = potential_shadows & dark_pixels

    # Buffer shadow edges
    shadow_mask = binary_dilation(shadow_mask, iterations=params.get('shadow_buffer', 3))

    return shadow_mask
```

### Fmask Quality Flags

```json
{
  "fmask_quality_flags": {
    "bit_0": {"name": "fill", "description": "No data fill"},
    "bit_1": {"name": "clear", "description": "Clear land/water"},
    "bit_2": {"name": "cloud", "description": "Cloud detected"},
    "bit_3": {"name": "cloud_shadow", "description": "Cloud shadow"},
    "bit_4": {"name": "snow", "description": "Snow/ice"},
    "bit_5": {"name": "water", "description": "Water body"},
    "bit_6_7": {
      "name": "cloud_confidence",
      "values": {
        "00": "Not determined",
        "01": "Low confidence",
        "10": "Medium confidence",
        "11": "High confidence"
      }
    }
  }
}
```

---

## 6.3 NDVI Calculation and Validation

### NDVI Formula and Theory

The Normalized Difference Vegetation Index exploits the contrast between red absorption and NIR reflection by healthy vegetation:

```
NDVI = (NIR - Red) / (NIR + Red)

Where:
- NIR: Near-infrared reflectance (typically 0.76-0.90 μm)
- Red: Red reflectance (typically 0.63-0.69 μm)

Value range: -1 to +1
Typical vegetation: 0.2 to 0.9
```

### Band Specifications by Sensor

| Sensor | Red Band | NIR Band | Resolution |
|--------|----------|----------|------------|
| Landsat 8/9 OLI | B4 (0.64-0.67 μm) | B5 (0.85-0.88 μm) | 30m |
| Sentinel-2 MSI | B4 (0.65-0.68 μm) | B8 (0.79-0.90 μm) | 10m |
| MODIS | B1 (0.62-0.67 μm) | B2 (0.84-0.88 μm) | 250m |
| VIIRS | I1 (0.60-0.68 μm) | I2 (0.85-0.88 μm) | 375m |

### NDVI Calculation Protocol

```python
def calculate_ndvi(red, nir, quality_mask, params):
    """
    Calculate NDVI with quality control.

    Parameters:
    -----------
    red : array
        Red band surface reflectance
    nir : array
        NIR band surface reflectance
    quality_mask : array
        Quality flags from cloud masking
    params : dict
        Processing parameters

    Returns:
    --------
    dict : NDVI array and quality information
    """
    # Initialize output
    ndvi = np.full_like(red, np.nan, dtype=np.float32)
    quality_flags = np.zeros_like(red, dtype=np.uint8)

    # Identify valid pixels
    valid = (
        (quality_mask == 0) &  # Clear
        (red > 0) &
        (nir > 0) &
        (red < 1) &
        (nir < 1)
    )

    # Calculate NDVI for valid pixels
    denominator = nir[valid] + red[valid]
    numerator = nir[valid] - red[valid]

    # Avoid division by zero
    nonzero_denom = denominator > 0.001
    ndvi[valid][nonzero_denom] = numerator[nonzero_denom] / denominator[nonzero_denom]

    # Quality checks
    # Check for saturated pixels
    saturated = (red >= params.get('saturation_threshold', 0.95)) | \
                (nir >= params.get('saturation_threshold', 0.95))
    quality_flags[saturated] |= 0x01

    # Check for negative reflectance (atmospheric overcorrection)
    negative_refl = (red < 0) | (nir < 0)
    quality_flags[negative_refl] |= 0x02

    # Check for physically unrealistic NDVI
    unrealistic = (ndvi < -0.5) | (ndvi > 1.0)
    quality_flags[unrealistic] |= 0x04
    ndvi[unrealistic] = np.nan

    # Calculate uncertainty
    uncertainty = calculate_ndvi_uncertainty(red, nir, params)

    return {
        'ndvi': ndvi,
        'quality_flags': quality_flags,
        'uncertainty': uncertainty,
        'valid_pixel_count': np.sum(valid & ~np.isnan(ndvi)),
        'total_pixel_count': np.size(ndvi)
    }


def calculate_ndvi_uncertainty(red, nir, params):
    """
    Calculate NDVI uncertainty from reflectance uncertainties.

    Using error propagation for NDVI = (NIR - Red)/(NIR + Red)
    """
    # Assume typical reflectance uncertainties
    red_uncertainty = params.get('red_uncertainty', 0.02)
    nir_uncertainty = params.get('nir_uncertainty', 0.02)

    sum_refl = nir + red
    diff_refl = nir - red

    # Partial derivatives
    dndvi_dnir = 2 * red / (sum_refl ** 2)
    dndvi_dred = -2 * nir / (sum_refl ** 2)

    # Propagated uncertainty
    ndvi_uncertainty = np.sqrt(
        (dndvi_dnir * nir_uncertainty) ** 2 +
        (dndvi_dred * red_uncertainty) ** 2
    )

    return ndvi_uncertainty
```

### NDVI Compositing Methods

| Method | Description | Best For |
|--------|-------------|----------|
| Maximum Value Composite (MVC) | Highest NDVI in period | Minimizing cloud effects |
| Median | Middle value in period | Reducing outliers |
| Mean | Average value | Temporal smoothing |
| Best Quality | Pixel with best QA | Data quality priority |
| Closest to Nadir | Minimum view angle | BRDF consistency |

```python
def create_ndvi_composite(ndvi_stack, qa_stack, method='mvc', period_days=16):
    """
    Create temporal composite from NDVI time stack.

    Parameters:
    -----------
    ndvi_stack : list of arrays
        Stack of NDVI images
    qa_stack : list of arrays
        Corresponding quality arrays
    method : str
        Compositing method (mvc, median, mean, best_quality)
    period_days : int
        Compositing period

    Returns:
    --------
    array : Composite NDVI
    """
    ndvi_array = np.stack(ndvi_stack, axis=0)
    qa_array = np.stack(qa_stack, axis=0)

    # Mask cloudy pixels
    masked_ndvi = np.ma.array(ndvi_array, mask=(qa_array > 0))

    if method == 'mvc':
        composite = np.ma.max(masked_ndvi, axis=0)
    elif method == 'median':
        composite = np.ma.median(masked_ndvi, axis=0)
    elif method == 'mean':
        composite = np.ma.mean(masked_ndvi, axis=0)
    elif method == 'best_quality':
        best_idx = np.argmin(qa_array, axis=0)
        composite = np.take_along_axis(ndvi_array, best_idx[np.newaxis, ...], axis=0)[0]

    return composite.filled(np.nan)
```

---

## 6.4 Evapotranspiration Estimation (FAO-56 Penman-Monteith)

### FAO-56 Penman-Monteith Equation

The FAO-56 Penman-Monteith equation is the standard method for calculating reference evapotranspiration:

```
            0.408 Δ(Rn - G) + γ (900/(T+273)) u₂ (es - ea)
ET₀ = ─────────────────────────────────────────────────────
                        Δ + γ(1 + 0.34 u₂)

Where:
- ET₀ = Reference evapotranspiration (mm/day)
- Rn = Net radiation (MJ/m²/day)
- G = Soil heat flux (MJ/m²/day)
- T = Mean daily air temperature (°C)
- u₂ = Wind speed at 2m height (m/s)
- es = Saturation vapor pressure (kPa)
- ea = Actual vapor pressure (kPa)
- Δ = Slope of saturation vapor pressure curve (kPa/°C)
- γ = Psychrometric constant (kPa/°C)
```

### FAO-56 Implementation

```python
import numpy as np

def calculate_et0_pm(weather_data, location, date):
    """
    Calculate reference ET using FAO-56 Penman-Monteith.

    Parameters:
    -----------
    weather_data : dict
        Daily weather observations
    location : dict
        Location parameters (lat, lon, elevation)
    date : datetime
        Calculation date

    Returns:
    --------
    dict : ET0 and intermediate calculations
    """
    # Extract inputs
    T_max = weather_data['temp_max_c']
    T_min = weather_data['temp_min_c']
    T_mean = (T_max + T_min) / 2

    RH_max = weather_data.get('rh_max', 80)
    RH_min = weather_data.get('rh_min', 40)

    u_2 = weather_data.get('wind_speed_2m', 2.0)
    Rs = weather_data.get('solar_radiation_mj', None)

    lat = location['latitude']
    elevation = location['elevation_m']

    # Calculate day of year
    doy = date.timetuple().tm_yday

    # Atmospheric pressure (kPa)
    P = 101.3 * ((293 - 0.0065 * elevation) / 293) ** 5.26

    # Psychrometric constant (kPa/°C)
    gamma = 0.665e-3 * P

    # Saturation vapor pressure (kPa)
    es_max = 0.6108 * np.exp(17.27 * T_max / (T_max + 237.3))
    es_min = 0.6108 * np.exp(17.27 * T_min / (T_min + 237.3))
    es = (es_max + es_min) / 2

    # Actual vapor pressure (kPa)
    ea = (es_min * RH_max / 100 + es_max * RH_min / 100) / 2

    # Slope of saturation vapor pressure curve (kPa/°C)
    delta = 4098 * es / (T_mean + 237.3) ** 2

    # Solar calculations
    lat_rad = np.radians(lat)
    dr = 1 + 0.033 * np.cos(2 * np.pi * doy / 365)
    solar_dec = 0.409 * np.sin(2 * np.pi * doy / 365 - 1.39)

    # Sunset hour angle
    ws = np.arccos(-np.tan(lat_rad) * np.tan(solar_dec))

    # Extraterrestrial radiation (MJ/m²/day)
    Ra = (24 * 60 / np.pi) * 0.0820 * dr * (
        ws * np.sin(lat_rad) * np.sin(solar_dec) +
        np.cos(lat_rad) * np.cos(solar_dec) * np.sin(ws)
    )

    # Clear-sky solar radiation
    Rso = (0.75 + 2e-5 * elevation) * Ra

    # Net shortwave radiation
    if Rs is None:
        # Estimate from temperature range
        Rs = 0.16 * np.sqrt(T_max - T_min) * Ra

    Rns = (1 - 0.23) * Rs  # Albedo = 0.23 for grass

    # Net longwave radiation
    sigma = 4.903e-9  # Stefan-Boltzmann constant
    Rnl = sigma * ((T_max + 273.16) ** 4 + (T_min + 273.16) ** 4) / 2 * \
          (0.34 - 0.14 * np.sqrt(ea)) * (1.35 * Rs / Rso - 0.35)

    # Net radiation
    Rn = Rns - Rnl

    # Soil heat flux (daily = 0)
    G = 0

    # Reference ET (mm/day)
    numerator = 0.408 * delta * (Rn - G) + gamma * (900 / (T_mean + 273)) * u_2 * (es - ea)
    denominator = delta + gamma * (1 + 0.34 * u_2)
    ET0 = numerator / denominator

    # Quality checks
    quality_flag = 0
    if ET0 < 0:
        quality_flag |= 0x01
        ET0 = 0
    if ET0 > 15:
        quality_flag |= 0x02

    return {
        'et0_mm_day': ET0,
        'net_radiation_mj': Rn,
        'vapor_pressure_deficit_kpa': es - ea,
        'quality_flag': quality_flag,
        'intermediate': {
            'es': es,
            'ea': ea,
            'delta': delta,
            'gamma': gamma,
            'Ra': Ra
        }
    }
```

### ET Variables for Drought Monitoring

| Variable | Calculation | Drought Application |
|----------|-------------|---------------------|
| ET₀ | FAO-56 PM | Reference evaporative demand |
| ETa | Remote sensing (METRIC, SSEBop) | Actual water use |
| ET Fraction | ETa/ET₀ | Crop stress indicator |
| EDDI | Standardized ET₀ anomaly | Flash drought detection |
| ESI | Standardized ETa/ET₀ anomaly | Agricultural stress |

---

## 6.5 PDSI Computation Algorithm

### PDSI Algorithm Overview

The Palmer Drought Severity Index uses a water balance approach:

```
PDSI Algorithm Flow:
====================

Step 1: Water Balance Accounting
├── Potential evapotranspiration (PE)
├── Potential recharge (PR)
├── Potential loss (PL)
├── Potential runoff (PRO)
└── Actual values (ET, R, L, RO)

Step 2: Coefficient Calculation
├── α = ET/PE (evaporation coefficient)
├── β = R/PR (recharge coefficient)
├── γ = RO/PRO (runoff coefficient)
├── δ = L/PL (loss coefficient)

Step 3: CAFEC Precipitation
├── P̂ = αPE + βPR + γPRO - δPL
├── d = P - P̂ (moisture departure)
├── D = d/average(|d|) (moisture anomaly index)

Step 4: PDSI Calculation
├── Z = K × d (Z-index)
├── PDSI = 0.897 × PDSI_prev + Z/3
```

### Self-Calibrated PDSI (SC-PDSI)

The self-calibrated version addresses PDSI's geographic inconsistency:

```python
def calculate_sc_pdsi(precip, pet, awc, calibration_period):
    """
    Calculate self-calibrated Palmer Drought Severity Index.

    Parameters:
    -----------
    precip : array
        Monthly precipitation (mm)
    pet : array
        Monthly potential evapotranspiration (mm)
    awc : float
        Available water capacity of soil (mm)
    calibration_period : tuple
        (start_year, end_year) for calibration

    Returns:
    --------
    dict : SC-PDSI values and components
    """
    n_months = len(precip)

    # Initialize water balance components
    soil_moisture = np.zeros(n_months)
    soil_moisture[0] = awc  # Start at field capacity

    et = np.zeros(n_months)
    recharge = np.zeros(n_months)
    runoff = np.zeros(n_months)
    loss = np.zeros(n_months)

    # Step 1: Water balance accounting
    for i in range(n_months):
        # Potential values based on current soil moisture
        pe = pet[i]
        pr = awc - soil_moisture[max(0, i-1)]  # Potential recharge
        pro = awc - pr  # Potential runoff (simplified)
        pl = soil_moisture[max(0, i-1)]  # Potential loss

        # Available water
        available = precip[i] + soil_moisture[max(0, i-1)]

        # Actual ET (limited by available water and PE)
        et[i] = min(pe, available)

        # Recharge
        if available > et[i]:
            recharge[i] = min(pr, available - et[i])
        else:
            recharge[i] = 0

        # Runoff (excess after recharge)
        if available > et[i] + recharge[i]:
            runoff[i] = min(pro, available - et[i] - recharge[i])
        else:
            runoff[i] = 0

        # Loss (soil moisture used for ET when P insufficient)
        if et[i] > precip[i]:
            loss[i] = et[i] - precip[i]
        else:
            loss[i] = 0

        # Update soil moisture
        soil_moisture[i] = soil_moisture[max(0, i-1)] + recharge[i] - loss[i]
        soil_moisture[i] = np.clip(soil_moisture[i], 0, awc)

    # Step 2: Calculate coefficients for each month
    alpha = np.zeros(12)
    beta = np.zeros(12)
    gamma = np.zeros(12)
    delta = np.zeros(12)

    for month in range(12):
        month_mask = np.arange(n_months) % 12 == month

        pe_month = pet[month_mask]
        et_month = et[month_mask]

        alpha[month] = np.mean(et_month) / np.mean(pe_month) if np.mean(pe_month) > 0 else 1

        # Similar calculations for beta, gamma, delta...

    # Step 3: Calculate CAFEC precipitation and departures
    p_hat = np.zeros(n_months)
    d = np.zeros(n_months)

    for i in range(n_months):
        month = i % 12
        p_hat[i] = alpha[month] * pet[i]  # Simplified
        d[i] = precip[i] - p_hat[i]

    # Step 4: Self-calibration of K factor
    # Calculate K for calibration period to make extreme values = ±4
    cal_mask = np.zeros(n_months, dtype=bool)
    # Set calibration mask based on calibration_period...

    k = calculate_self_calibrated_k(d[cal_mask])

    # Calculate Z-index
    z = k * d

    # Step 5: PDSI calculation with Palmer's recursive formula
    pdsi = np.zeros(n_months)
    for i in range(1, n_months):
        pdsi[i] = 0.897 * pdsi[i-1] + z[i] / 3

    return {
        'pdsi': pdsi,
        'z_index': z,
        'departures': d,
        'soil_moisture': soil_moisture,
        'components': {
            'et': et,
            'recharge': recharge,
            'runoff': runoff,
            'loss': loss
        }
    }
```

---

## 6.6 SPI/SPEI Statistical Methods

### SPI Calculation Procedure

```python
from scipy.stats import gamma as gamma_dist
from scipy.stats import norm

def calculate_spi(precip_series, time_scale_months, baseline_period):
    """
    Calculate Standardized Precipitation Index.

    Parameters:
    -----------
    precip_series : array
        Monthly precipitation values
    time_scale_months : int
        Accumulation period (1-48 months)
    baseline_period : tuple
        (start_year, end_year) for fitting distribution

    Returns:
    --------
    dict : SPI values and parameters
    """
    # Step 1: Accumulate precipitation
    accumulated = accumulate_precipitation(precip_series, time_scale_months)

    # Step 2: Fit gamma distribution for each month
    gamma_params = {}
    spi = np.zeros_like(accumulated)

    for month in range(12):
        # Extract values for this month during baseline
        month_values = extract_month_values(accumulated, month, baseline_period)

        # Remove zeros for gamma fitting
        nonzero_values = month_values[month_values > 0]
        zero_prob = np.sum(month_values == 0) / len(month_values)

        if len(nonzero_values) < 30:
            # Insufficient data
            gamma_params[month] = None
            continue

        # Fit gamma distribution using maximum likelihood
        alpha, loc, beta = gamma_dist.fit(nonzero_values, floc=0)

        gamma_params[month] = {
            'alpha': alpha,
            'beta': beta,
            'zero_probability': zero_prob
        }

        # Calculate SPI for all values of this month
        month_mask = np.arange(len(accumulated)) % 12 == month

        for i in np.where(month_mask)[0]:
            if np.isnan(accumulated[i]):
                spi[i] = np.nan
            elif accumulated[i] == 0:
                # Handle zeros
                prob = zero_prob
                spi[i] = norm.ppf(prob)
            else:
                # CDF of gamma distribution
                gamma_cdf = gamma_dist.cdf(accumulated[i], alpha, loc=0, scale=beta)
                # Adjust for zero probability
                prob = zero_prob + (1 - zero_prob) * gamma_cdf
                # Transform to standard normal
                spi[i] = norm.ppf(prob)

    # Clamp extreme values
    spi = np.clip(spi, -3.5, 3.5)

    return {
        'spi': spi,
        'accumulated_precip': accumulated,
        'gamma_params': gamma_params,
        'time_scale': time_scale_months
    }


def accumulate_precipitation(precip, n_months):
    """
    Calculate n-month accumulated precipitation.
    """
    accumulated = np.zeros_like(precip)
    for i in range(len(precip)):
        start = max(0, i - n_months + 1)
        if i - start + 1 == n_months:
            accumulated[i] = np.sum(precip[start:i+1])
        else:
            accumulated[i] = np.nan  # Insufficient data
    return accumulated
```

### SPEI Calculation

SPEI extends SPI by incorporating evapotranspiration:

```python
def calculate_spei(precip_series, pet_series, time_scale_months, baseline_period):
    """
    Calculate Standardized Precipitation Evapotranspiration Index.

    Parameters:
    -----------
    precip_series : array
        Monthly precipitation
    pet_series : array
        Monthly potential evapotranspiration
    time_scale_months : int
        Accumulation period
    baseline_period : tuple
        Baseline for distribution fitting

    Returns:
    --------
    dict : SPEI values
    """
    # Calculate water balance
    water_balance = precip_series - pet_series

    # Accumulate water balance
    accumulated = accumulate_values(water_balance, time_scale_months)

    # Fit log-logistic distribution
    from scipy.stats import fisk  # Log-logistic

    spei = np.zeros_like(accumulated)

    for month in range(12):
        month_values = extract_month_values(accumulated, month, baseline_period)

        # Fit log-logistic distribution
        c, loc, scale = fisk.fit(month_values - np.min(month_values) + 1)

        month_mask = np.arange(len(accumulated)) % 12 == month

        for i in np.where(month_mask)[0]:
            if np.isnan(accumulated[i]):
                spei[i] = np.nan
            else:
                adjusted_value = accumulated[i] - np.min(month_values) + 1
                prob = fisk.cdf(adjusted_value, c, loc=loc, scale=scale)
                spei[i] = norm.ppf(prob)

    spei = np.clip(spei, -3.5, 3.5)

    return {
        'spei': spei,
        'water_balance': accumulated,
        'time_scale': time_scale_months
    }
```

---

## 6.7 Drought Classification and Severity Mapping

### Multi-Index Classification

```python
def classify_drought_status(indices, weights=None):
    """
    Combine multiple indices into drought classification.

    Parameters:
    -----------
    indices : dict
        Dictionary of index names to values
    weights : dict
        Optional weights for each index

    Returns:
    --------
    dict : Classification result
    """
    if weights is None:
        weights = {
            'pdsi': 0.25,
            'spi_3': 0.20,
            'soil_moisture_pct': 0.25,
            'ndvi_anomaly': 0.15,
            'esi': 0.15
        }

    # Normalize indices to common scale (-4 to +4)
    normalized = {}

    if 'pdsi' in indices:
        normalized['pdsi'] = indices['pdsi']  # Already on PDSI scale

    if 'spi_3' in indices:
        normalized['spi_3'] = indices['spi_3'] * 2  # SPI to PDSI-like scale

    if 'soil_moisture_pct' in indices:
        # Convert percentile to z-score-like
        from scipy.stats import norm
        normalized['soil_moisture_pct'] = norm.ppf(indices['soil_moisture_pct'] / 100) * 2

    if 'ndvi_anomaly' in indices:
        # Scale NDVI anomaly
        normalized['ndvi_anomaly'] = indices['ndvi_anomaly'] * 20

    # Calculate weighted composite
    composite = 0
    total_weight = 0

    for idx, value in normalized.items():
        if idx in weights and not np.isnan(value):
            composite += value * weights[idx]
            total_weight += weights[idx]

    if total_weight > 0:
        composite /= total_weight

    # Classify
    if composite >= 0.5:
        category = 'W'
        severity = 'wet'
    elif composite >= -1.0:
        category = 'N'
        severity = 'normal'
    elif composite >= -2.0:
        category = 'D0'
        severity = 'abnormally_dry'
    elif composite >= -3.0:
        category = 'D1'
        severity = 'moderate_drought'
    elif composite >= -4.0:
        category = 'D2'
        severity = 'severe_drought'
    elif composite >= -5.0:
        category = 'D3'
        severity = 'extreme_drought'
    else:
        category = 'D4'
        severity = 'exceptional_drought'

    return {
        'composite_value': composite,
        'category': category,
        'severity': severity,
        'contributing_indices': list(normalized.keys()),
        'confidence': min(total_weight / sum(weights.values()), 1.0)
    }
```

---

## 6.8 Quality Control and Validation Protocols

### Multi-Stage QC Framework

| Stage | Checks | Action on Failure |
|-------|--------|-------------------|
| Level 0 | Format, completeness | Reject |
| Level 1 | Physical limits | Flag |
| Level 2 | Spatial consistency | Flag/investigate |
| Level 3 | Temporal consistency | Flag/interpolate |
| Level 4 | Cross-validation | Document |

### QC Implementation

```python
def quality_control_drought_data(data, qc_config):
    """
    Apply multi-stage quality control to drought data.

    Parameters:
    -----------
    data : dict
        Drought data record
    qc_config : dict
        QC configuration parameters

    Returns:
    --------
    dict : QC results and flagged data
    """
    qc_results = {
        'passed': True,
        'flags': [],
        'level': 4  # Highest level passed
    }

    # Level 0: Format validation
    if not validate_format(data):
        qc_results['passed'] = False
        qc_results['level'] = 0
        qc_results['flags'].append('INVALID_FORMAT')
        return qc_results

    # Level 1: Physical limits
    value = data.get('value')
    index_type = data.get('index_type', 'pdsi')

    limits = qc_config.get('limits', {}).get(index_type, (-10, 10))

    if value < limits[0] or value > limits[1]:
        qc_results['flags'].append('OUT_OF_RANGE')
        qc_results['level'] = min(qc_results['level'], 1)

    # Level 2: Spatial consistency
    neighbors = data.get('neighbor_values', [])
    if len(neighbors) >= 3:
        neighbor_mean = np.mean(neighbors)
        neighbor_std = np.std(neighbors)

        if abs(value - neighbor_mean) > 3 * neighbor_std:
            qc_results['flags'].append('SPATIAL_OUTLIER')
            qc_results['level'] = min(qc_results['level'], 2)

    # Level 3: Temporal consistency
    previous_value = data.get('previous_value')
    if previous_value is not None:
        max_change = qc_config.get('max_monthly_change', {}).get(index_type, 2.0)

        if abs(value - previous_value) > max_change:
            qc_results['flags'].append('TEMPORAL_JUMP')
            qc_results['level'] = min(qc_results['level'], 3)

    # Level 4: Cross-validation with independent data
    independent_value = data.get('independent_validation')
    if independent_value is not None:
        correlation_threshold = qc_config.get('cross_validation_threshold', 0.7)
        # Check agreement with independent source...

    qc_results['passed'] = len(qc_results['flags']) == 0

    return qc_results
```

---

## 6.9 Review Questions and Key Takeaways

### Review Questions

1. **Atmospheric Correction**: Explain why atmospheric correction is essential for temporal NDVI analysis. What would happen if you compared uncorrected NDVI values from different dates?

2. **Cloud Masking Trade-offs**: The Fmask algorithm has tunable probability thresholds. Describe the trade-off between using a low threshold (0.1) versus a high threshold (0.5). When might each be appropriate?

3. **NDVI Compositing**: A 16-day NDVI composite uses maximum value compositing (MVC). What are the advantages and potential biases of this approach?

4. **ET₀ Estimation**: The FAO-56 Penman-Monteith equation requires numerous inputs. How would you estimate ET₀ if only temperature data were available?

5. **PDSI Limitations**: The original PDSI has been criticized for spatial inconsistency. Explain how the self-calibrated PDSI (SC-PDSI) addresses this limitation.

6. **SPI Time Scales**: A location shows SPI-1 = +0.5 (near normal) but SPI-12 = -2.0 (extremely dry). Interpret this pattern in terms of drought conditions.

7. **Multi-Index Classification**: Design a weighting scheme for drought classification in an irrigated agricultural region. How would the weights differ from a rainfed region?

8. **Quality Control**: A soil moisture sensor reports a value 3 standard deviations from neighboring sensors. Describe the QC investigation process you would follow.

### Key Takeaways

1. **Atmospheric Correction is Fundamental**: The 6S model removes atmospheric effects that can cause 20-40% variation in surface reflectance, enabling valid temporal comparisons.

2. **Cloud Masking is Multi-Step**: Fmask combines spectral tests, temperature probability, and shadow projection to achieve robust cloud detection.

3. **NDVI Has Known Limitations**: Sensor differences, atmospheric effects, and saturation in dense vegetation require careful calibration and quality control.

4. **FAO-56 is the ET₀ Standard**: The Penman-Monteith equation provides physically-based reference ET, enabling drought assessment through evaporative demand.

5. **PDSI Requires Water Balance Modeling**: The Palmer index integrates precipitation, temperature, and soil properties through a two-layer bucket model.

6. **SPI/SPEI Are Statistically Robust**: Fitting precipitation or water balance to probability distributions enables standardized comparison across locations and times.

7. **Multi-Index Integration Improves Assessment**: Combining meteorological, agricultural, and hydrological indices provides more complete drought characterization.

8. **Quality Control is Multi-Level**: Progressive QC from format validation through cross-validation ensures data quality while documenting uncertainty.

9. **Algorithms Have Parameters**: Each algorithm includes tunable parameters that should be adjusted based on local conditions and application requirements.

10. **Validation is Essential**: All algorithms should be validated against independent data to ensure accuracy and document limitations.

---

## Chapter Summary

This chapter has detailed the scientific protocols and algorithms underlying drought monitoring—from satellite data processing through index calculation to quality control. These methods transform raw observations into actionable drought intelligence.

Atmospheric correction using the 6S model removes atmospheric interference, enabling valid multi-temporal analysis. Cloud masking with Fmask identifies contaminated pixels that would otherwise corrupt drought assessments. Together, these preprocessing steps prepare satellite data for vegetation analysis.

NDVI calculation follows established formulas but requires careful attention to sensor differences, saturation effects, and quality flagging. Compositing methods aggregate multi-temporal observations while minimizing cloud effects.

Evapotranspiration estimation using FAO-56 Penman-Monteith provides the reference against which actual water use is compared. The ET fraction (ETa/ET₀) indicates crop water stress independent of evaporative demand.

PDSI and SPI/SPEI calculations implement standardized drought indices through water balance modeling and statistical transformation. The self-calibrated PDSI addresses spatial inconsistency, while multi-scale SPI captures drought at different time horizons.

Quality control ensures data reliability through multi-stage validation from format checking through cross-validation with independent observations. Quality flags communicate data limitations to users.

These protocols provide the scientific foundation for drought monitoring systems. Consistent implementation across institutions enables interoperable data exchange and valid cross-boundary drought assessment.

---

**Next Chapter: [Chapter 7: System Integration](07-system-integration.md)**
