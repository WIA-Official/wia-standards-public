# Chapter 6: Protocol Specification

## Learning Objectives

After completing this chapter, you will be able to:

1. Implement quality assurance and quality control (QA/QC) procedures
2. Follow calibration protocols for environmental sensors
3. Apply standardized field sampling methods
4. Document data management and validation workflows
5. Conduct proficiency testing for laboratories and observers

---

## 6.1 Quality Assurance Overview

### 6.1.1 QA vs. QC

```
Quality Assurance vs. Quality Control:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  QUALITY ASSURANCE (QA) - Planned, systematic activities            │
│  ├─ Standard operating procedures (SOPs)                           │
│  ├─ Training and certification programs                            │
│  ├─ Equipment calibration schedules                                │
│  ├─ Data management plans                                          │
│  └─ Audit and review processes                                     │
│                                                                     │
│  QUALITY CONTROL (QC) - Specific techniques to detect problems     │
│  ├─ Field blanks and replicates                                   │
│  ├─ Laboratory duplicates and spikes                               │
│  ├─ Automated data validation                                      │
│  ├─ Outlier detection                                              │
│  └─ Cross-parameter consistency checks                             │
│                                                                     │
│  GOAL: Generate defensible, publication-quality data               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.1.2 QA/QC Requirements by Data Type

| Data Type | Field QC | Lab QC | Automated Validation |
|-----------|----------|--------|---------------------|
| Species Observations | 10% field replicates | Expert verification | Taxonomic, spatial, temporal |
| Water Quality | 5% field blanks, 10% replicates | Duplicates, spikes, blanks | Range, rate-of-change |
| Air Quality | Co-location tests | Calibration verification | Flatline, spike detection |
| Sensor Data | Monthly calibration | Annual factory calibration | Multi-parameter consistency |

---

## 6.2 Field Quality Control

### 6.2.1 Field QC Requirements

**Field Blanks (5% of samples):**
- Purpose: Detect contamination during collection, transport, or processing
- Procedure: Clean water/solution exposed to field conditions
- Acceptance: Analyte concentrations < detection limit

**Equipment Blanks:**
- Purpose: Verify cleaning procedures
- Procedure: Clean water run through sampling equipment
- Frequency: Before/after each sampling event
- Acceptance: < detection limit

**Field Replicates (10% of samples):**
- Purpose: Assess sampling precision and natural variability
- Procedure: Collect 2-3 samples from same location/time
- Acceptance: Relative Percent Difference (RPD) < 20%

```
RPD = |Sample1 - Sample2| / ((Sample1 + Sample2) / 2) × 100%
```

**Positive Controls:**
- Purpose: Verify detection methods work
- Procedure: Known sample with target organism/analyte
- Examples: Reference specimen for species ID, standard solution for chemistry
- Acceptance: Correct identification or recovery 80-120%

### 6.2.2 Observer Calibration

**Annual Calibration Events:**
```typescript
interface ObserverCalibration {
  setup: {
    location: "Area with known species composition";
    participants: "All observers who will collect data";
    duration: "Half-day minimum";
  };

  activities: {
    independent_surveys: "Each observer surveys same area independently";
    comparison: "Compare observations across observers";
    expert_verification: "Taxonomic expert confirms identifications";
    discussion: "Review discrepancies and correct errors";
  };

  metrics: {
    detection_probability: "% of true species detected by each observer";
    false_positive_rate: "% of identifications that were incorrect";
    consistency: "Agreement with expert (kappa statistic > 0.7)";
  };

  followup: {
    training: "Additional training for observers with low agreement";
    adjustment: "Detection probabilities used to adjust analyses";
    documentation: "Inter-observer variability documented in metadata";
  };
}
```

---

## 6.3 Laboratory Quality Control

### 6.3.1 Laboratory QC Requirements

**Method Blanks (Each analytical batch):**
- Purpose: Detect laboratory contamination
- Procedure: Clean matrix through entire analytical process
- Acceptance: < detection limit

**Calibration Verification (Every 10 samples):**
- Purpose: Ensure calibration curve remains valid
- Procedure: Analyze mid-point calibration standard
- Acceptance: Recovery 90-110%

**Spike Recovery (10% of samples):**
- Purpose: Assess matrix interference and recovery efficiency
- Procedure: Add known amount of analyte to sample before analysis
- Calculation: `Recovery = (Spiked Sample - Unspiked Sample) / Spike Amount × 100%`
- Acceptance: 75-125%

**Duplicate Analysis (10% of samples):**
- Purpose: Assess analytical precision
- Procedure: Analyze same sample twice
- Acceptance: RPD < 20%

**Certified Reference Materials (Each batch):**
- Purpose: Verify method accuracy
- Procedure: Analyze certified standard with known concentration
- Acceptance: Within ±2 standard deviations of certified value

**Blind QC Samples (5% of samples):**
- Purpose: Unbiased assessment of laboratory performance
- Procedure: Submit known samples as if they were unknowns
- Acceptance: Within acceptance criteria

---

## 6.4 Sensor Calibration Protocols

### 6.4.1 Calibration Frequency

Minimum calibration intervals:

| Sensor Type | Interval | Method | Standard |
|-------------|----------|--------|----------|
| Temperature | 6 months | Ice point check | 0°C ± 0.1°C |
| pH | 2 weeks | Two-point buffer | pH 4.0 and 7.0 (or 7.0 and 10.0) |
| Dissolved Oxygen | 1 month | Water-saturated air | 100% saturation |
| Conductivity | 3 months | Standard solution | 1000 μS/cm standard |
| Turbidity | 1 month | Formazin standards | 0, 20, 100, 800 NTU |
| Nutrients (continuous) | Each run | Standard curve | 5-point curve |
| Air quality (PM) | 6 months | Co-location with reference | EPA reference method |
| Flow meters | 1 year | Known volume | Volumetric standard |

### 6.4.2 Calibration Documentation

Required metadata for each calibration:

```json
{
  "calibration_id": "CAL-2025-042-001",
  "sensor_id": "WATER-TEMP-042",
  "calibration_date": "2025-06-15T09:00:00Z",
  "technician": "Jane Smith",
  "technician_id": "orcid:0000-0002-1825-0097",

  "method": "Ice point check",
  "standards_used": [
    {
      "description": "Ice bath (distilled water + ice)",
      "value": 0.0,
      "unit": "celsius",
      "lot_number": "N/A",
      "expiry_date": null
    }
  ],

  "pre_calibration_readings": [
    {"timestamp": "2025-06-15T09:05:00Z", "value": 0.3, "unit": "celsius"},
    {"timestamp": "2025-06-15T09:06:00Z", "value": 0.2, "unit": "celsius"},
    {"timestamp": "2025-06-15T09:07:00Z", "value": 0.3, "unit": "celsius"}
  ],
  "pre_calibration_mean": 0.27,
  "pre_calibration_std": 0.058,

  "adjustment_applied": true,
  "adjustment_offset": -0.27,

  "post_calibration_readings": [
    {"timestamp": "2025-06-15T09:15:00Z", "value": 0.0, "unit": "celsius"},
    {"timestamp": "2025-06-15T09:16:00Z", "value": 0.1, "unit": "celsius"},
    {"timestamp": "2025-06-15T09:17:00Z", "value": 0.0, "unit": "celsius"}
  ],
  "post_calibration_mean": 0.03,
  "post_calibration_std": 0.058,

  "drift_detected": 0.27,
  "pass_fail": "pass",
  "pass_criteria": "< 0.5°C drift",

  "next_calibration_due": "2025-12-15",
  "certificate_url": "https://example.org/certs/CAL-2025-042-001.pdf",
  "notes": "Sensor performing within specifications"
}
```

### 6.4.3 Calibration Failure Response

If sensor fails calibration:

```
Calibration Failure Response:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  1. FLAG DATA                                                       │
│     └─ Mark all data since last successful calibration as          │
│        questionable or invalid                                     │
│                                                                     │
│  2. INVESTIGATE CAUSE                                               │
│     ├─ Sensor fouling (algae, sediment)                            │
│     ├─ Sensor degradation (age, exposure)                          │
│     ├─ Environmental damage (flooding, wildlife)                   │
│     └─ Incorrect standard (expired, contaminated)                  │
│                                                                     │
│  3. CORRECTIVE ACTION                                               │
│     ├─ Clean sensor and recalibrate                                │
│     ├─ Replace sensor if cleaning doesn't work                     │
│     ├─ Verify standards are fresh and properly prepared            │
│     └─ Document all actions taken                                  │
│                                                                     │
│  4. VERIFY FIX                                                      │
│     └─ Repeat calibration, must pass before returning to service   │
│                                                                     │
│  5. DATA DISPOSITION                                                │
│     ├─ If drift linear and predictable: Apply correction to data   │
│     ├─ If drift non-linear or large: Flag data as invalid          │
│     └─ Document data quality implications in metadata              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6.5 Automated Data Validation

### 6.5.1 Validation Rules

**Range Checks:**
```typescript
interface RangeChecks {
  latitude: { min: -90, max: 90 };
  longitude: { min: -180, max: 180 };
  temperature_c: { min: -100, max: 100 };
  ph: { min: 0, max: 14 };
  dissolved_oxygen_mgl: { min: 0, max: 50 };
  relative_humidity: { min: 0, max: 100 };
}
```

**Rate-of-Change Detection:**
```typescript
function detectRateOfChange(readings) {
  for (let i = 1; i < readings.length; i++) {
    const dt = readings[i].timestamp - readings[i-1].timestamp; // seconds
    const dv = readings[i].value - readings[i-1].value;
    const rate = Math.abs(dv / (dt / 3600)); // change per hour

    if (rate > MAX_CHANGE_RATE) {
      readings[i].qc_flag = 'questionable';
      readings[i].qc_notes = `Rate of change ${rate.toFixed(2)}/hr exceeds maximum ${MAX_CHANGE_RATE}/hr`;
    }
  }
}

// Example thresholds
const MAX_CHANGE_RATES = {
  air_temperature: 10, // °C/hr
  water_temperature: 5, // °C/hr
  water_level: 50, // cm/hr (excluding floods)
};
```

**Flatline Detection:**
```typescript
function detectFlatline(readings, minVariance = 0.001) {
  const window = 24; // Check 24 consecutive readings
  for (let i = 0; i < readings.length - window; i++) {
    const segment = readings.slice(i, i + window);
    const values = segment.map(r => r.value);
    const variance = calculateVariance(values);

    if (variance < minVariance) {
      segment.forEach(r => {
        r.qc_flag = 'questionable';
        r.qc_notes = 'Flatline detected - sensor may be stuck';
      });
    }
  }
}
```

**Spike Detection:**
```typescript
function detectSpikes(readings, threshold = 4.0) {
  // Use median absolute deviation (MAD) for robustness
  const values = readings.map(r => r.value);
  const median = calculateMedian(values);
  const mad = calculateMAD(values, median);

  readings.forEach(r => {
    const deviation = Math.abs(r.value - median) / mad;
    if (deviation > threshold) {
      r.qc_flag = 'questionable';
      r.qc_notes = `Value deviates ${deviation.toFixed(1)} MADs from median`;
    }
  });
}
```

**Cross-Parameter Consistency:**
```typescript
function checkWaterOxygenTemperature(sample) {
  // Dissolved oxygen should decrease with increasing temperature
  const temp = sample.temperature_c;
  const do_mgl = sample.dissolved_oxygen_mgl;

  // Calculate theoretical saturation (simplified Benson & Krause)
  const do_sat = 14.652 - 0.41022 * temp + 0.007991 * temp**2 - 0.000077774 * temp**3;

  // Flag if measured DO > 110% saturation (unlikely without supersaturation source)
  if (do_mgl > 1.1 * do_sat) {
    sample.quality_flags.push('DO exceeds expected saturation for temperature');
  }

  // Flag if measured DO < 50% saturation at moderate temps (possible sensor issue)
  if (temp < 25 && do_mgl < 0.5 * do_sat) {
    sample.quality_flags.push('DO unusually low for temperature');
  }
}
```

---

## 6.6 Field Sampling Protocols

### 6.6.1 Point Count Protocol (Birds)

**Standard Procedure:**

1. **Site Selection**
   - Fixed points selected via stratified random sampling
   - Minimum 250m between points (avoid double-counting)
   - Representative of habitat types in study area

2. **Timing**
   - Breeding season (varies by latitude and species)
   - Dawn to 4 hours after sunrise (peak activity)
   - Avoid rain, high wind (>20 km/h)

3. **Procedure**
   - Observer approaches quietly, waits 1 minute to settle birds
   - Record all birds detected for 5-10 minutes
   - Note distance bands: 0-50m, 50-100m, >100m
   - Record detection method: song, call, visual

4. **Effort Documentation**
   - Start time, end time, duration
   - Weather conditions (cloud cover, wind, temperature)
   - Observer ID
   - Background noise level

5. **Repeat Visits**
   - Minimum 3 visits per season
   - Spaced 7-14 days apart
   - Allows detection probability estimation

**WIA Data Format:**
```json
{
  "schema_type": "species-observation",
  "detection_method": "visual_survey",
  "protocol": "Point count",
  "protocol_details": {
    "count_duration_minutes": 10,
    "radius_meters": 50,
    "visit_number": 2,
    "total_visits": 3
  },
  "effort": {
    "start_time": "2025-06-15T06:15:00-07:00",
    "end_time": "2025-06-15T06:25:00-07:00",
    "weather": "Partly cloudy",
    "temperature_c": 12,
    "wind_speed_ms": 2.5,
    "background_noise": "Low"
  }
}
```

### 6.6.2 Water Sampling Protocol

**Standard Procedure:**

1. **Preparation**
   - Wear clean nitrile gloves
   - Use pre-cleaned bottles (acid-washed for metals, autoclaved for organics)
   - Label bottles with waterproof marker before going to field

2. **Approach**
   - Approach sampling location from downstream
   - Avoid disturbing sediment upstream of sample location

3. **In Situ Parameters**
   - Measure temperature, pH, DO, conductivity in situ
   - Calibrate meters at start of day
   - Triple rinse probe before measurement
   - Record after reading stabilizes (±2%)

4. **Sample Collection**
   - Rinse bottle 3× with ambient water
   - Collect mid-stream, mid-depth
   - Avoid surface (oil films) and bottom (sediment)
   - For DO: Fill completely, no headspace
   - For metals: Filter immediately (0.45 μm), acidify (HNO₃ to pH <2)
   - For nutrients: Filter and freeze

5. **Documentation**
   - Site ID, GPS coordinates
   - Date and time
   - Depth of collection
   - Visual observations (color, odor, clarity)
   - Chain of custody form

6. **Transport and Storage**
   - Place on ice immediately
   - Transport to lab within 6 hours (or per parameter holding time)
   - Maintain 4°C during transport

### 6.6.3 Camera Trap Deployment

**Standard Procedure:**

1. **Site Selection**
   - Game trails, water sources, or systematic grid
   - Avoid direct sun to prevent overheating and glare

2. **Camera Setup**
   - Mount 30-50 cm above ground (ground-dwelling mammals)
   - Aim parallel to trail (not perpendicular)
   - Clear vegetation 3m in front (avoid false triggers)
   - Test trigger before leaving

3. **Configuration**
   - Photo mode: Burst of 3 photos
   - Delay: 1-5 minutes (balance detections vs. storage)
   - Sensitivity: Medium-high (adjust for site)
   - Date/time stamp: ON
   - Location stamp: ON if camera supports

4. **Metadata Documentation**
   ```json
   {
     "deployment_id": "CAM-042-2025-06",
     "camera_id": "RECONYX-PC900-001",
     "location": {"latitude": 47.68, "longitude": -121.74},
     "deployment_date": "2025-06-15T10:00:00Z",
     "retrieval_date": "2025-08-15T14:00:00Z",
     "setup": {
       "height_cm": 40,
       "direction_degrees": 270,
       "habitat": "Mixed conifer forest",
       "bait": false,
       "lure": false
     },
     "settings": {
       "trigger_sensitivity": "high",
       "burst_photos": 3,
       "delay_seconds": 60
     }
   }
   ```

5. **Maintenance**
   - Check batteries and memory every 2-4 weeks
   - Clean lens and sensor
   - Trim vegetation if growing into detection zone
   - Download images, clear card if needed

---

## 6.7 Data Management Protocols

### 6.7.1 Data Entry

**Best Practices:**

- **Double Data Entry** (for critical datasets)
  - Two people independently enter same data
  - Compare entries and reconcile differences
  - Reduces data entry error rate from ~1-3% to <0.1%

- **Real-Time Validation**
  - Validation rules applied during data entry
  - Immediate feedback to data enterer
  - Prevents propagation of errors

- **Digital Data Capture**
  - Mobile apps with offline capability
  - Pre-populated lists (species, sites) prevent typos
  - GPS integration for automatic coordinates
  - Photos linked to observations

- **Version Control**
  - Every edit logged with timestamp and user ID
  - Ability to revert to previous version
  - Separation of raw data (never edited) from processed data

### 6.7.2 Metadata Documentation

Minimum required metadata (EML/ISO 19115):

```
Required Metadata Elements:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  DISCOVERY METADATA                                                 │
│  ├─ Title (descriptive, unique)                                    │
│  ├─ Abstract (200+ words, purpose, scope, methods)                 │
│  ├─ Keywords (taxonomic, geographic, thematic)                     │
│  ├─ Authors (name, ORCID, affiliation, role)                       │
│  ├─ Contact (email, phone, current address)                        │
│  └─ License (CC BY, CC0, or custom)                                │
│                                                                     │
│  COVERAGE METADATA                                                  │
│  ├─ Temporal: Start date, end date (or ongoing)                    │
│  ├─ Geographic: Bounding box, detailed site descriptions           │
│  └─ Taxonomic: List of taxa, rank level                            │
│                                                                     │
│  METHODS METADATA                                                   │
│  ├─ Sampling design (random, stratified, systematic)               │
│  ├─ Field protocols (SOPs, training)                               │
│  ├─ Laboratory methods (EPA numbers, citations)                    │
│  ├─ Quality assurance procedures                                   │
│  ├─ Instrumentation (make, model, calibration)                     │
│  └─ Known limitations and biases                                   │
│                                                                     │
│  ACCESS METADATA                                                    │
│  ├─ Distribution (URL, repository, DOI)                            │
│  ├─ Restrictions (embargoes, sensitive species)                    │
│  ├─ Citation format                                                │
│  └─ Related resources (papers, datasets)                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6.8 Review Questions

### Question 1
A water sample has a spike recovery of 65% (acceptable range: 75-125%). What are possible causes and what corrective actions would you take?

### Question 2
Design a calibration schedule for a remote weather station with temperature, humidity, and precipitation sensors. What frequency and methods would you use?

### Question 3
Two observers conduct independent bird surveys at the same location. Observer A detects 15 species, Observer B detects 12 species, with 10 species in common. Calculate inter-observer agreement and interpret the result.

### Question 4
A dissolved oxygen sensor shows flatline behavior (constant 8.5 mg/L) for 48 hours despite changing temperature. Diagnose the likely problem and propose a solution.

### Question 5
You're designing QA/QC for a citizen science butterfly monitoring program with 100 volunteers. What are the most important quality control measures given the variability in observer expertise?

---

## 6.9 Key Takeaways

| Protocol Area | Key Requirements |
|---------------|-----------------|
| **Field QC** | 5% blanks, 10% replicates, observer calibration |
| **Lab QC** | Duplicates, spikes, blanks, certified references |
| **Calibration** | Sensor-specific intervals, documented procedures |
| **Validation** | Automated range, rate, flatline, spike detection |
| **Sampling** | Standardized protocols for each data type |
| **Documentation** | Comprehensive metadata, version control |

### Critical Statistics
- **10%**: Minimum field replicates
- **75-125%**: Acceptable spike recovery range
- **< 20%**: Maximum acceptable RPD for duplicates
- **6 months**: Typical temperature sensor calibration interval
- **3-5 visits**: Minimum for detection probability estimation

### Next Chapter Preview

Chapter 7 explores Phase 4 (System Integration), showing how to connect WIA-compliant systems to GIS platforms, conservation databases, cloud services, and decision support tools.

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
