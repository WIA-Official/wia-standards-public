# WIA Biodiversity Index Standard - Phase 3: Protocol
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity, Preserve All Life

## Overview

Phase 3 establishes standardized field protocols for biodiversity data collection, ensuring consistent, high-quality observations across regions and time periods. This specification covers survey design, taxon-specific methods, quality assurance procedures, and professional certification.

## Survey Design Principles

### Sampling Strategy
- **Survey Unit:** Minimum 100m × 100m (terrestrial), 50m × 50m (aquatic)
- **Stratification:** By habitat type, elevation, or management zone
- **Random Placement:** GIS-generated random coordinates within strata
- **Accessibility:** If random point inaccessible, shift ≤50m and document

### Replication Requirements
| Objective | Minimum Replicates | Recommended | Purpose |
|-----------|-------------------|-------------|---------|
| Single-site baseline | 3 plots | 5-10 | Within-site variation |
| Site comparison | 5 per site | 10 per site | Detect effect sizes (d=0.8) |
| Temporal monitoring | 3 time points | Quarterly 3+ years | Trend detection |
| Impact assessment | 5 control + 5 impact | 10 each (BACI) | Before-After-Control-Impact |

## Taxon-Specific Protocols

### Birds: Point Count Protocol

**Equipment:**
- Binoculars (8x or 10x magnification)
- GPS unit (±5m accuracy)
- Field guide or eBird app
- Weather meter

**Procedure:**
1. Conduct surveys 30min after sunrise to 4hr post-sunrise
2. Avoid rain, high wind (>20 km/h), dense fog
3. Locate point via GPS (±5m)
4. Wait 2 minutes for birds to settle
5. Record ALL birds within 50m radius for 10 minutes
6. Note distance band (0-25m, 25-50m), detection method (visual/song/call)
7. Record behavior (perched/flying/foraging/territorial)

**Data Format:**
```
Species | Count | Distance | Method | Time | Behavior
AMRO   | 2     | 0-25m    | Visual | 2min | Foraging
BAWW   | 1     | 25-50m   | Song   | 5min | Territorial
```

**Observer Requirements:**
- Identify 90% of regional species by sight/sound
- WIA Level 1 certification minimum

### Mammals: Camera Trap Protocol

**Equipment:**
- Motion-triggered camera (≥5MP, <0.5s trigger, IR flash)
- SD card (32GB minimum)
- Security cable/lock box
- Spare batteries (lithium for cold weather)

**Deployment:**
- Height: 40-50cm (medium mammals), 20-30cm (small mammals)
- Angle: Perpendicular to trail, slight downward tilt
- Settings: High sensitivity, 3 images per trigger, 1-min quiet period
- Operation: Minimum 30 days, recommended 60+ days
- Check interval: 14 days (battery/SD, vegetation clearing)

**Data Processing:**
- Tag: species, count, sex, age class, behavior
- Calculate trap-nights: (cameras) × (operational days)
- Upload to WIA database with deployment metadata

### Vegetation: Quadrat Sampling

**Nested Quadrat Design:**
| Size | Target | Measurements |
|------|--------|--------------|
| 10m × 10m | Trees (>5cm DBH) | Species, DBH, height, crown width |
| 5m × 5m (nested) | Saplings (1-5cm DBH) | Species, height, count |
| 1m × 1m (nested) | Herbs, grasses | Species, % cover, height |
| 0.25m × 0.25m (nested) | Mosses, lichens | Species, % cover |

**Measurements:**
- DBH: Diameter at breast height (1.3m) using DBH tape (±0.1cm)
- Height: Clinometer or laser rangefinder
- Cover: Visual estimation in 5% increments

### eDNA: Water Sampling Protocol

**Equipment (sterilized):**
- 1L Nalgene bottles (3 replicates + 1 control)
- Filtration unit (0.45μm cellulose nitrate filters)
- Peristaltic or vacuum pump
- Cooler with ice packs
- Nitrile gloves (new pair per sample)

**Sampling:**
1. New gloves for each sample
2. Rinse bottle 3× with site water
3. Collect at 0.5m below surface (avoid sediment disturbance)
4. Fill to 1L mark
5. Record GPS, temp, pH, turbidity, time
6. 3 replicates spaced 2-5m apart
7. Negative control (sterile water)

**Filtration (within 24hr):**
1. Pre-wet filter with 10mL sterile water
2. Filter 1L under vacuum (record actual volume if clogged)
3. Remove filter with sterile forceps
4. Place in 2mL tube with lysis buffer
5. Store at -20°C until extraction

**Chain of Custody:**
- Label: Site_Date_Rep#_Initials
- Metadata form completed
- Ship overnight on dry ice to certified lab

## Quality Assurance Procedures

### Pre-Survey QA
- Equipment calibration (GPS accuracy test, thermometer calibration)
- Observer training (species ID quiz ≥90%, protocol walkthrough)
- Site reconnaissance (access verification, hazard assessment)
- Permit verification (research permits current, landowner permission)

### During-Survey QC
- Duplicate samples (10% of points by independent observer)
- Photo vouchers (all uncertain IDs with scale bar)
- Real-time validation (app with range checks, taxonomy verification)
- Environmental documentation (temp, humidity, weather at each point)

### Post-Survey QA
- Data validation (automated checks: coordinates, dates, outliers)
- Taxonomic verification (expert review of uncertain IDs via photos)
- Completeness check (all required metadata populated)
- Archiving (raw data, processed data, metadata in redundant locations)

## Equipment Standards

| Equipment | Specification | Calibration |
|-----------|--------------|-------------|
| GPS Unit | ±5m horizontal, WAAS/EGNOS | Daily benchmark test |
| DBH Tape | Steel/fiberglass, 0.1cm precision | Annual vs. certified ruler |
| Thermometer | Digital, ±0.5°C, -20 to 50°C | Annual NIST-traceable calibration |
| pH Meter | Digital, 0.01 pH resolution, ATC | Daily buffer solution calibration |
| Camera Trap | ≥5MP, <0.5s trigger, IR flash | Trigger speed test pre-deployment |

## Certification Program

### Level 1: Field Technician
- **Requirements:** 16-hour online course + field practical exam
- **Competencies:** Protocol execution, equipment operation, data recording
- **Valid:** 2 years

### Level 2: Survey Lead
- **Requirements:** Level 1 + 40 field days + advanced course
- **Competencies:** Survey design, team management, QA/QC, data validation
- **Valid:** 3 years

### Level 3: Expert Practitioner
- **Requirements:** Level 2 + 200 field days + publication/contribution
- **Competencies:** Protocol development, training delivery, peer review
- **Valid:** 5 years (with continuing education)

### Recertification
- Minimum 20 field days/year using WIA protocols
- 8 hours continuing education every 2 years
- Passing score (80%) on recertification exam
- No major QA violations

## Safety Protocols

**Field Safety:**
- Two-person minimum for remote sites
- Satellite communication (no cell coverage areas)
- First aid certification (≥1 team member)
- Wildlife encounter protocols (dangerous species training)
- Weather monitoring, evacuation triggers defined

**Ethical Considerations:**
- Avoid breeding/nesting areas during sensitive periods
- Limit wildlife handling to essential ID only
- Non-invasive methods preferred (eDNA, cameras)
- Follow ABS requirements for genetic resources
- Respect indigenous rights, traditional knowledge protocols

## Compliance Requirements

### Gold Certification
- 50%+ field staff WIA Level 1 certified
- Phase 3 protocols documented and implemented
- QA/QC procedures verified through audit
- Peer review of methods by Level 3 practitioner

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity · Preserve All Life
