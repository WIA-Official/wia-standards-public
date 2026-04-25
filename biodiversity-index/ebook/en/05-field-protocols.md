# Chapter 5: Field Protocols

## Phase 3: Standardized Survey Methods and Certification

### Ensuring Consistent, High-Quality Biodiversity Data Collection

---

## Overview

Phase 3 of the WIA Biodiversity Index Standard establishes standardized field protocols for biodiversity data collection, ensuring consistent, high-quality observations across regions and time periods. This chapter covers survey design, taxon-specific methods, quality assurance procedures, and professional certification requirements.

---

## Survey Design Principles

### Sampling Strategy Framework

Effective biodiversity surveys require careful planning before fieldwork begins:

**Key Considerations:**
1. **Objectives**: What questions are we trying to answer?
2. **Target taxa**: Which species groups are priorities?
3. **Spatial scope**: What area needs to be covered?
4. **Temporal scope**: What time period and seasons?
5. **Resources**: Budget, personnel, equipment available
6. **Constraints**: Access, permits, safety considerations

### Sampling Unit Specifications

| Habitat Type | Minimum Unit Size | Recommended | Notes |
|--------------|-------------------|-------------|-------|
| Terrestrial forest | 100m × 100m | 1 ha | Tree diversity plots |
| Grassland/savanna | 100m × 100m | 1 ha | Transects for mammals |
| Wetland (marsh) | 50m × 50m | 0.25 ha | Point counts for birds |
| Aquatic (river) | 100m reach | 500m reach | eDNA sampling points |
| Aquatic (lake) | 50m × 50m | Multiple depths | Stratified by depth |
| Marine (coral reef) | 50m transect | 6 × 50m transects | Belt transects |
| Urban | Variable | Census blocks | Stratified by land use |

### Stratification Approaches

**By Habitat Type:**
- Divide study area into habitat categories
- Allocate sampling effort proportional to habitat extent
- Ensure all habitat types represented

**By Elevation:**
- Create elevation bands (e.g., 500m intervals)
- Sample across elevational gradient
- Critical for mountain biodiversity

**By Management Zone:**
- Core areas vs. buffer zones
- Protected vs. unprotected land
- Restoration vs. reference sites

### Replication Requirements

| Objective | Minimum Replicates | Recommended | Statistical Basis |
|-----------|-------------------|-------------|-------------------|
| Single-site baseline | 3 plots | 5-10 plots | Capture within-site variation |
| Site comparison | 5 per site | 10 per site | Detect effect size d=0.8 |
| Temporal monitoring | 3 time points | Quarterly 3+ years | Trend detection with power 0.8 |
| Impact assessment | 5 control + 5 impact | 10 each (BACI) | Before-After-Control-Impact design |
| Species accumulation | 20+ samples | Until asymptote | Rarefaction curve plateau |

### Random Site Selection

```python
import geopandas as gpd
from shapely.geometry import Point
import numpy as np

def generate_random_points(study_area, n_points, min_distance=100):
    """
    Generate random sampling points within study area polygon.

    Args:
        study_area: GeoDataFrame with study area polygon
        n_points: Number of random points to generate
        min_distance: Minimum distance (m) between points

    Returns:
        GeoDataFrame with random point locations
    """
    bounds = study_area.total_bounds
    points = []

    while len(points) < n_points:
        # Generate random coordinate
        x = np.random.uniform(bounds[0], bounds[2])
        y = np.random.uniform(bounds[1], bounds[3])
        point = Point(x, y)

        # Check if within study area
        if study_area.contains(point).any():
            # Check minimum distance from existing points
            if all(point.distance(p) >= min_distance for p in points):
                points.append(point)

    return gpd.GeoDataFrame(
        {'geometry': points},
        crs=study_area.crs
    )
```

---

## Taxon-Specific Protocols

### Birds: Point Count Protocol

**Purpose:** Standardized method for estimating bird species richness and relative abundance.

**Equipment Required:**
| Item | Specification | Purpose |
|------|---------------|---------|
| Binoculars | 8× or 10× magnification | Visual identification |
| GPS unit | ±5m accuracy | Location recording |
| Field guide/app | Regional species coverage | Species confirmation |
| Weather meter | Temperature, wind, humidity | Environmental data |
| Audio recorder | Optional | Document songs/calls |
| Clipboard/tablet | Waterproof | Data recording |
| Watch | Second hand/stopwatch | Timing |

**Survey Timing:**
- **Primary window**: 30 minutes after sunrise to 4 hours post-sunrise
- **Avoid**: Rain, wind >20 km/h, dense fog, extreme temperatures
- **Season**: Breeding season for residents; migration periods for migrants
- **Frequency**: 2-3 visits per season recommended

**Point Count Procedure:**

1. **Navigate to point** (GPS ±5m accuracy)
2. **Wait 2 minutes** for birds to settle after arrival
3. **Record metadata**: Date, time, observer ID, weather conditions
4. **Begin 10-minute count**:
   - Record ALL birds detected within 50m radius
   - Note detection method (visual, song, call, fly-over)
   - Estimate distance band (0-25m, 25-50m, >50m fly-over)
   - Record individual count or estimate for flocks
   - Note behavior (foraging, singing, nesting, flying)
5. **Complete environmental data**: Wind, cloud cover, noise level

**Data Recording Format:**

| Field | Example | Required |
|-------|---------|----------|
| Point_ID | PT-001 | Yes |
| Date | 2025-04-15 | Yes |
| Start_Time | 06:45 | Yes |
| Observer_ID | OBS-042 | Yes |
| Species_Code | AMRO | Yes |
| Count | 2 | Yes |
| Distance_Band | 0-25m | Yes |
| Detection_Method | Song | Yes |
| Time_Interval | 0-3min | Recommended |
| Behavior | Singing | Optional |
| Sex | Male | Optional |
| Age | Adult | Optional |

**Observer Requirements:**
- Identify 90%+ of regional species by sight and sound
- WIA Level 1 certification minimum
- Pass annual species identification quiz
- Document continuing education

### Mammals: Camera Trap Protocol

**Purpose:** Non-invasive monitoring of medium to large mammal populations.

**Equipment Specifications:**

| Component | Requirement | Notes |
|-----------|-------------|-------|
| Camera | ≥5MP resolution | Higher for species ID |
| Trigger speed | <0.5 second | Fast for moving animals |
| Detection range | 15-20m | Adjusted for target species |
| Flash type | Infrared (no-glow preferred) | Minimize disturbance |
| Battery | Lithium (cold weather) | 6+ months runtime |
| Memory | 32GB+ SD card | Class 10 for video |
| Security | Cable lock + lock box | Theft prevention |
| Housing | IP66+ weather resistance | All weather operation |

**Camera Deployment:**

**Site Selection:**
- Game trails and wildlife corridors
- Water sources and mineral licks
- Forest edges and natural funnels
- Known den/resting sites
- Avoid high human traffic areas

**Installation:**
1. **Height**: 40-50cm for medium mammals; 20-30cm for small mammals
2. **Angle**: Perpendicular to expected movement path
3. **Tilt**: Slight downward angle to capture body
4. **Distance**: 3-5m from expected animal path
5. **Orientation**: North-facing preferred (avoid sun glare)
6. **Vegetation**: Clear within 3m to prevent false triggers

**Camera Settings:**

| Setting | Recommended Value | Notes |
|---------|-------------------|-------|
| Sensitivity | High/Medium | Adjust for site |
| Images per trigger | 3 photos | Capture sequence |
| Delay between triggers | 1 minute | Balance coverage/storage |
| Video length | 15-30 seconds | If video enabled |
| Time stamp | Enabled | Essential for analysis |
| Temperature stamp | Enabled | Environmental data |
| Moon phase | Enabled if available | Activity patterns |

**Operational Requirements:**
- Minimum deployment: 30 days
- Recommended deployment: 60-90 days
- Check interval: 14 days (battery, SD card, vegetation)
- Trap-nights calculation: (cameras) × (operational days)

**Data Processing:**

```python
def process_camera_trap_image(image_path, model):
    """
    Process camera trap image for species identification.

    Args:
        image_path: Path to camera trap image
        model: Trained species classification model

    Returns:
        Dictionary with detection results
    """
    import exifread
    from PIL import Image

    # Extract metadata
    with open(image_path, 'rb') as f:
        tags = exifread.process_file(f)

    metadata = {
        'datetime': str(tags.get('EXIF DateTimeOriginal', '')),
        'camera_id': str(tags.get('Image Model', '')),
        'temperature': extract_temperature(tags)
    }

    # Run species detection
    image = Image.open(image_path)
    predictions = model.predict(image)

    return {
        'image_path': image_path,
        'metadata': metadata,
        'detections': [
            {
                'species': pred['species'],
                'confidence': pred['confidence'],
                'bbox': pred['bounding_box'],
                'count': pred['individual_count']
            }
            for pred in predictions
            if pred['confidence'] > 0.85
        ]
    }
```

### Vegetation: Quadrat Sampling Protocol

**Purpose:** Systematic assessment of plant species composition and structure.

**Nested Quadrat Design:**

```
┌─────────────────────────────────────────┐
│                                         │
│    10m × 10m (Trees >5cm DBH)          │
│                                         │
│    ┌─────────────────────┐             │
│    │                     │             │
│    │   5m × 5m (Saplings)│             │
│    │                     │             │
│    │   ┌─────────┐       │             │
│    │   │ 1m × 1m │       │             │
│    │   │ (Herbs) │       │             │
│    │   │┌───────┐│       │             │
│    │   ││0.25m² ││       │             │
│    │   ││Mosses ││       │             │
│    │   │└───────┘│       │             │
│    │   └─────────┘       │             │
│    └─────────────────────┘             │
│                                         │
└─────────────────────────────────────────┘
```

| Quadrat Size | Target Vegetation | Measurements |
|--------------|-------------------|--------------|
| 10m × 10m | Trees (>5cm DBH) | Species, DBH, height, crown width |
| 5m × 5m (nested) | Saplings (1-5cm DBH) | Species, height, count |
| 1m × 1m (nested) | Herbs, grasses | Species, % cover, height |
| 0.25m × 0.25m (nested) | Mosses, lichens | Species, % cover |

**Measurement Protocols:**

**DBH (Diameter at Breast Height):**
- Measured at 1.3m height using DBH tape
- Precision: ±0.1cm
- For leaning trees: measure on uphill side
- For multi-stemmed: measure each stem
- For buttressed trees: measure above buttress

**Height Measurement:**
- Clinometer or laser rangefinder
- Measure to top of canopy
- Precision: ±0.5m
- Record instrument and method used

**Cover Estimation:**
- Visual estimation in 5% increments
- Use cover class scales for consistency:

| Cover Class | Range | Midpoint |
|-------------|-------|----------|
| 0 | 0% | 0 |
| 1 | <1% | 0.5 |
| 2 | 1-5% | 3 |
| 3 | 5-25% | 15 |
| 4 | 25-50% | 37.5 |
| 5 | 50-75% | 62.5 |
| 6 | 75-95% | 85 |
| 7 | 95-100% | 97.5 |

### eDNA: Aquatic Sampling Protocol

**Purpose:** Detect species presence through environmental DNA in water samples.

**Equipment (Sterilized):**

| Item | Specification | Purpose |
|------|---------------|---------|
| Collection bottles | 1L Nalgene, sterile | Water collection |
| Filtration unit | 0.45μm cellulose nitrate | DNA capture |
| Pump | Peristaltic or vacuum | Water filtration |
| Cooler | With ice packs | Sample preservation |
| Gloves | Nitrile, powder-free | Contamination prevention |
| Forceps | Sterile, disposable | Filter handling |
| Storage tubes | 2mL with lysis buffer | DNA preservation |
| Bleach | 10% solution | Equipment decontamination |

**Sampling Procedure:**

1. **Preparation**:
   - Don new nitrile gloves
   - Rinse collection bottle 3× with site water
   - Set up filtration equipment on clean surface

2. **Sample Collection**:
   - Collect at 0.5m below surface
   - Avoid disturbing sediment
   - Fill to 1L mark
   - Record GPS coordinates immediately
   - Collect 3 replicates spaced 2-5m apart
   - Include 1 negative control (sterile water)

3. **Environmental Data**:
   - Water temperature (±0.1°C)
   - pH (±0.1 units)
   - Turbidity (NTU)
   - Conductivity (µS/cm)
   - Dissolved oxygen (mg/L)
   - Flow velocity (if applicable)

4. **Filtration (within 24 hours)**:
   - Pre-wet filter with 10mL sterile water
   - Filter entire 1L under vacuum
   - Record actual volume if filter clogs
   - Remove filter with sterile forceps
   - Place in 2mL tube with lysis buffer
   - Store at -20°C until extraction

5. **Chain of Custody**:
   - Label: Site_Date_Rep#_Initials
   - Complete metadata form
   - Ship overnight on dry ice to certified lab
   - Maintain temperature log

**Quality Control:**

| QC Check | Requirement | Action if Failed |
|----------|-------------|------------------|
| Field negative | No target DNA | Investigate contamination |
| Extraction blank | No amplification | Re-extract batch |
| PCR negative | No amplification | Proceed normally |
| Positive control | Expected species detected | Re-run PCR |
| Read depth | >10,000 reads/sample | Re-sequence |

---

## Quality Assurance Procedures

### Pre-Survey QA

**Equipment Calibration:**

| Equipment | Calibration Method | Frequency |
|-----------|-------------------|-----------|
| GPS unit | Benchmark test at known location | Before each survey |
| DBH tape | Verify against certified ruler | Annually |
| Thermometer | NIST-traceable standard | Annually |
| pH meter | Buffer solutions (4.0, 7.0, 10.0) | Before each use |
| Camera trap | Trigger speed test | Before deployment |
| Audio recorder | Reference tone test | Before each survey |

**Observer Preparation:**
- Species identification quiz: ≥90% accuracy required
- Protocol walkthrough and demonstration
- Equipment check and functionality test
- Safety briefing and emergency procedures

**Site Reconnaissance:**
- Access verification (permits, permissions)
- Hazard assessment (terrain, wildlife, weather)
- Communication plan (cell coverage, check-in times)
- Evacuation routes identified

### During-Survey QC

**Duplicate Sampling:**
- 10% of points surveyed by independent observer
- Compare species lists and counts
- Calculate inter-observer reliability

**Photo Documentation:**
- All uncertain identifications photographed
- Include scale bar for size reference
- Document habitat at each sampling point
- GPS-tagged images when possible

**Real-Time Validation:**
- Mobile app with range checks
- Taxonomy verification against regional lists
- Alert for unusual records requiring verification
- Environmental data bounds checking

### Post-Survey QA

**Data Validation:**
```python
def validate_survey_data(records):
    """
    Validate biodiversity survey records.

    Args:
        records: List of survey records

    Returns:
        Dictionary with validation results
    """
    errors = []
    warnings = []

    for record in records:
        # Coordinate validation
        if not (-90 <= record['latitude'] <= 90):
            errors.append(f"Invalid latitude: {record['record_id']}")

        # Date validation
        if record['date'] > datetime.now():
            errors.append(f"Future date: {record['record_id']}")

        # Taxonomic validation
        if not validate_species_name(record['species']):
            warnings.append(f"Unrecognized species: {record['species']}")

        # Range check
        if not within_known_range(record['species'], record['latitude'], record['longitude']):
            warnings.append(f"Outside known range: {record['record_id']}")

    return {
        'total_records': len(records),
        'errors': errors,
        'warnings': warnings,
        'pass_rate': (len(records) - len(errors)) / len(records)
    }
```

---

## Certification Program

### Level 1: Field Technician

**Requirements:**
- 16-hour online course completion
- Field practical exam (supervised)
- 90%+ on species identification quiz

**Competencies:**
- Execute standard protocols accurately
- Operate survey equipment correctly
- Record data completely and legibly
- Follow safety procedures

**Validity:** 2 years

### Level 2: Survey Lead

**Requirements:**
- Level 1 certification + 40 documented field days
- Advanced course completion (24 hours)
- Demonstrated leadership in field

**Competencies:**
- Design survey sampling schemes
- Manage field teams
- Implement QA/QC procedures
- Validate and correct field data

**Validity:** 3 years

### Level 3: Expert Practitioner

**Requirements:**
- Level 2 certification + 200 documented field days
- Peer-reviewed publication OR WIA protocol contribution
- Training delivery experience

**Competencies:**
- Develop new protocols
- Train and certify others
- Peer review survey designs
- Expert identification capabilities

**Validity:** 5 years (with continuing education)

### Recertification Requirements

| Requirement | Level 1 | Level 2 | Level 3 |
|-------------|---------|---------|---------|
| Annual field days | 10 | 20 | 20 |
| Continuing education | 4 hrs/yr | 8 hrs/yr | 12 hrs/yr |
| Recertification exam | Yes | Yes | Waived |
| Major QA violations | None | None | None |

---

## Safety Protocols

### Field Safety Essentials

**General Requirements:**
- Two-person minimum for remote sites
- Communication device (satellite for no-cell areas)
- First aid certification (≥1 team member)
- Emergency action plan documented
- Check-in schedule with base contact

**Personal Protective Equipment:**

| Environment | Required PPE |
|-------------|--------------|
| Forest | Boots, long pants, tick protection |
| Wetland | Waders/waterproof boots, life jacket near deep water |
| Tropical | Sun protection, insect repellent, hydration |
| Alpine | Layered clothing, emergency shelter, ice axe if needed |
| Marine | PFD, exposure suit, buddy system |

**Wildlife Encounter Protocols:**
- Species-specific training for dangerous wildlife
- Bear spray/deterrents where appropriate
- Avoid surprising animals
- Maintain safe distances
- Know retreat routes

### Ethical Considerations

**Minimize Disturbance:**
- Avoid breeding/nesting areas during sensitive periods
- Limit wildlife handling to essential identification
- Prefer non-invasive methods (eDNA, camera traps, audio)
- Restore vegetation trampled during surveys

**Respect Local Rights:**
- Obtain required permits before fieldwork
- Follow Nagoya Protocol for genetic resources
- Respect indigenous land rights and protocols
- Share data and findings with local communities
- Acknowledge traditional knowledge contributions

---

## Key Takeaways

1. **Survey design** requires clear objectives, appropriate stratification, and adequate replication
2. **Taxon-specific protocols** ensure standardized, comparable data across studies
3. **Quality assurance** spans pre-survey preparation through post-survey validation
4. **Certification program** provides structured pathway for professional development
5. **Safety and ethics** are non-negotiable priorities in all field activities

## Review Questions

1. What is the recommended minimum survey unit size for terrestrial forest plots?
2. Describe the key steps in a bird point count protocol.
3. How long should camera traps be deployed for reliable mammal detection?
4. What QC checks are required for eDNA sampling?
5. What are the requirements for WIA Level 2 Survey Lead certification?

---

**Next Chapter Preview:** Chapter 6 explores Phase 4: Integration Systems, covering GIS platforms, conservation planning tools, and policy reporting frameworks.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
