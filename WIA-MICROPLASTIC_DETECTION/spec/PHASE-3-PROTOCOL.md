# WIA-MICROPLASTIC_DETECTION
## PHASE 3: Protocol Specification v1.0

**Status**: FULL Implementation
**Philosophy**: 弘益人間 (Benefit All Humanity)
**Last Updated**: 2026-01-12

---

## 1. Overview

This specification defines standardized protocols for microplastic sampling, sample preparation, analysis workflows, quality control procedures, and data validation. These protocols ensure consistency, reproducibility, and comparability of microplastic detection results across different laboratories and monitoring programs.

### 1.1 Protocol Objectives

- **Standardization**: Consistent methods across all operators
- **Reproducibility**: Reliable results independent of location
- **Quality Assurance**: Built-in validation and verification
- **Contamination Control**: Minimize false positives
- **Traceability**: Complete chain of custody
- **Efficiency**: Optimized workflows for throughput

### 1.2 Scope

This phase covers:
- Sampling protocols for different environmental matrices
- Sample preparation and processing workflows
- Analytical procedures for particle detection and identification
- Quality control and quality assurance (QC/QA)
- Data validation and verification protocols
- Safety and contamination prevention

---

## 2. Sampling Protocols

### 2.1 Marine Surface Water Sampling

#### 2.1.1 Manta Trawl Method

**Equipment Required:**
- Manta trawl net (mesh size: 330 μm or 100 μm)
- Flow meter
- GPS device
- Waterproof data sheets
- Sample collection bottles (glass, pre-cleaned)

**Procedure:**

1. **Pre-sampling Preparation**
   ```
   - Clean all equipment with filtered water
   - Record location, date, time, weather conditions
   - Attach flow meter to net opening
   - Reset flow meter to zero
   ```

2. **Sample Collection**
   ```
   - Deploy manta trawl from stern of vessel
   - Maintain speed: 2-3 knots
   - Tow duration: 30 minutes
   - Monitor net depth (surface, half-submerged)
   - Record GPS coordinates (start and end)
   ```

3. **Sample Retrieval**
   ```
   - Retrieve trawl slowly
   - Record final flow meter reading
   - Transfer sample from cod end to bottle
   - Rinse cod end with filtered water
   - Volume sampled = Flow meter reading × Net opening area
   ```

4. **Sample Preservation**
   ```
   - Transfer to pre-cleaned glass bottle
   - Add 70% ethanol (optional, for biological preservation)
   - Label with sample ID, date, location
   - Store at 4°C or room temperature (with ethanol)
   ```

**Quality Control:**
- Blank sample: Deploy empty net for contamination check
- Replicate samples: 10% of samples should be duplicated
- Photodocumentation of sampling location

#### 2.1.2 Bulk Water Sampling

**Equipment:**
- Niskin bottle or water sampler (5-10 L capacity)
- Pump with filtration system
- Filter holders (47 mm or 90 mm diameter)
- Membrane filters (0.7 μm - 100 μm pore sizes)

**Procedure:**

1. **Sample Collection**
   ```
   - Collect water at specified depth
   - Use glass or stainless steel containers
   - Avoid plastic containers
   - Volume: 100-1000 L depending on particle concentration
   ```

2. **Filtration**
   ```
   - Filter water through cascade of filters
   - First filter: 100 μm (large particles)
   - Second filter: 20 μm (medium particles)
   - Third filter: 5 μm (small particles)
   - Record volume filtered
   ```

3. **Filter Handling**
   ```
   - Transfer filters to petri dishes (glass or aluminum)
   - Dry at room temperature or 40°C
   - Store in sealed containers
   - Label with sample ID and filter size
   ```

### 2.2 Sediment Sampling

#### 2.2.1 Marine Sediment

**Equipment:**
- Van Veen grab or box corer
- Stainless steel scoops
- Aluminum foil
- Glass or metal containers

**Procedure:**

1. **Core/Grab Collection**
   ```
   - Deploy grab/corer to seafloor
   - Collect surface sediment (0-5 cm depth)
   - Record penetration depth
   - Note sediment type and color
   ```

2. **Sub-sampling**
   ```
   - Use stainless steel tools only
   - Collect 500 g - 1 kg per sample
   - Wrap in aluminum foil
   - Place in glass jar or metal container
   ```

3. **Preservation**
   ```
   - Freeze at -20°C
   - Or air-dry at room temperature
   - Do not use plastic bags or containers
   ```

#### 2.2.2 Soil Sampling

**Equipment:**
- Stainless steel auger or corer
- Metal spatulas
- Aluminum foil
- Glass containers

**Procedure:**
```
1. Remove surface litter layer
2. Collect soil from 0-10 cm depth
3. Homogenize sample
4. Store 500 g in glass container
5. Dry or freeze as per protocol
```

### 2.3 Freshwater Sampling

**Methods:**
- Similar to marine surface water sampling
- Adjust mesh size based on water turbidity
- Consider seasonal variations (ice cover, algal blooms)

**Specific Considerations:**
```
- River/stream: Sample from bridge or boat
- Lakes: Multiple depth profiles
- Wastewater: Use grab samples, not nets
```

### 2.4 Air Sampling

**Equipment:**
- High-volume air sampler
- Glass fiber filters (0.7 μm)
- Flow meter

**Procedure:**
```
1. Set up sampler at 1.5 m height
2. Run for 24-48 hours
3. Flow rate: 20-30 m³/hour
4. Calculate air volume sampled
5. Store filters in petri dishes
```

---

## 3. Sample Preparation Protocols

### 3.1 Density Separation

**Purpose**: Separate low-density microplastics from high-density sediment/organic matter

**Equipment:**
- Separatory funnels or flotation columns
- Density solutions (NaCl, NaI, ZnCl₂)
- Vacuum filtration system

**Procedure:**

1. **Preparation of Density Solution**
   ```
   - NaCl solution: 1.2 g/cm³ (inexpensive, for PE, PP)
   - NaI solution: 1.6 g/cm³ (moderate cost, for most plastics)
   - ZnCl₂ solution: 1.7 g/cm³ (expensive, for all plastics including PVC)
   ```

2. **Separation Process**
   ```
   a. Add dried sediment/sample to separatory funnel
   b. Add density solution (1:5 sample:solution ratio)
   c. Stir vigorously for 30 seconds
   d. Let settle for 24 hours
   e. Plastics float to surface
   f. Drain bottom sediment layer
   g. Filter supernatant through membrane filter
   h. Rinse particles with distilled water
   ```

3. **Multiple Extractions**
   ```
   - Repeat density separation 2-3 times
   - Combine all supernatant fractions
   - Calculate recovery efficiency
   ```

**Quality Control:**
- Spike samples with known plastic particles
- Target recovery rate: >90%

### 3.2 Organic Matter Removal

**Purpose**: Remove natural organic matter (algae, plankton, detritus) that interferes with analysis

#### 3.2.1 Hydrogen Peroxide Digestion

**Reagents:**
- 30% H₂O₂ (hydrogen peroxide)
- 0.05 M Fe(II) catalyst (Fenton's reagent, optional)

**Procedure:**
```
1. Place sample in glass beaker
2. Add H₂O₂ (50 mL per gram of organic matter)
3. Cover with watch glass
4. Heat to 50°C for 24-48 hours
5. Add more H₂O₂ as needed (reaction stops when bubbling ceases)
6. Cool and dilute with distilled water
7. Filter through membrane filter (5 μm)
8. Rinse thoroughly
```

**Safety:**
- Perform in fume hood
- Wear appropriate PPE
- H₂O₂ is corrosive and oxidizing

#### 3.2.2 Enzymatic Digestion

**Reagents:**
- Proteinase K (for proteins)
- Cellulase (for cellulose)
- Lipase (for lipids)

**Procedure:**
```
1. Add enzyme solution to sample
2. Incubate at optimal temperature (37-55°C)
3. Duration: 12-24 hours
4. Deactivate enzyme by heating to 90°C
5. Filter and rinse
```

**Advantages:**
- Gentler than chemical digestion
- Less risk of polymer degradation
- Effective for biological samples

### 3.3 Filtration and Concentration

**Procedure:**
```
1. Pre-wet filter with ethanol, then distilled water
2. Apply vacuum (< 10 psi)
3. Filter sample through membrane
4. Rinse filter edges with distilled water
5. Dry filter in covered petri dish
6. Store in desiccator until analysis
```

**Filter Selection:**
- Pore size: 5-10 μm for microscopy
- Material: Cellulose nitrate, PTFE, aluminum oxide
- Diameter: 47 mm (standard), 90 mm (large volumes)

---

## 4. Analytical Protocols

### 4.1 Visual Identification

**Equipment:**
- Stereomicroscope (10-50× magnification)
- Transmitted and reflected light
- Camera for documentation

**Procedure:**

1. **Visual Screening**
   ```
   - Place filter under microscope
   - Scan entire filter systematically
   - Identify potential plastic particles
   ```

2. **Visual Criteria for Plastics**
   ```
   - No cellular or organic structures visible
   - Homogeneous color throughout particle
   - Uniform thickness
   - Clear or colored (not brown/green biological colors)
   - Fibers: equal thickness, no tapering
   - Fragments: sharp edges, no cellular structure
   ```

3. **Documentation**
   ```
   - Photograph each particle
   - Record position on filter
   - Measure size (length, width)
   - Note color and shape
   - Assign preliminary polymer type (if possible)
   ```

**Quality Control:**
- Inter-analyst comparison: 10% of samples analyzed by second analyst
- Agreement should be >80%

### 4.2 Raman Spectroscopy

**Equipment:**
- Raman microscope
- Laser: 532 nm, 633 nm, or 785 nm
- Spectral range: 100-4000 cm⁻¹

**Procedure:**

1. **Sample Preparation**
   ```
   - Place filter on microscope stage
   - Locate particle under optical microscope
   - Focus laser on particle center
   ```

2. **Spectrum Acquisition**
   ```
   - Laser power: 1-10 mW (adjust to avoid burning)
   - Exposure time: 5-30 seconds
   - Accumulations: 2-5
   - Spectral resolution: 1-4 cm⁻¹
   ```

3. **Data Processing**
   ```
   - Baseline correction (polynomial fitting)
   - Cosmic ray removal
   - Normalization
   - Smoothing (Savitzky-Golay filter)
   ```

4. **Polymer Identification**
   ```
   - Compare spectrum to reference library
   - Hit Quality Index (HQI) > 0.70 for positive ID
   - Check for characteristic peaks:
     * PE: 1128, 1295, 1440, 2850, 2883 cm⁻¹
     * PP: 809, 841, 973, 1168, 1458 cm⁻¹
     * PS: 621, 795, 1001, 1031, 1602 cm⁻¹
     * PET: 632, 858, 1097, 1290, 1616 cm⁻¹
     * PVC: 634, 693, 1429, 1604 cm⁻¹
   ```

**Quality Control:**
- Daily calibration with silicon standard (520.5 cm⁻¹)
- Reference polymer standards every 50 samples
- Blind duplicates: 5% of samples

### 4.3 FTIR Spectroscopy

**Equipment:**
- FTIR microscope (micro-FTIR)
- ATR accessory (optional)
- Spectral range: 4000-600 cm⁻¹

**Procedure:**

1. **Sample Preparation**
   ```
   - Place filter on FTIR stage
   - Locate particle
   - Choose measurement mode (transmission or ATR)
   ```

2. **Spectrum Acquisition**
   ```
   - Resolution: 4 cm⁻¹
   - Scans: 32-64
   - Aperture size: Match particle size
   - Background: Collect every 15 minutes
   ```

3. **Data Processing**
   ```
   - ATR correction (if applicable)
   - Baseline correction
   - Normalization
   ```

4. **Polymer Identification**
   ```
   - Compare to reference library
   - Correlation coefficient > 0.70
   - Check for characteristic peaks:
     * PE: 2916, 2850, 1463, 720 cm⁻¹
     * PP: 2950, 2917, 1456, 1376, 973 cm⁻¹
     * PS: 3026, 2923, 1601, 1493, 698 cm⁻¹
     * PET: 1713, 1241, 1097, 723 cm⁻¹
     * PVC: 2912, 1425, 1255, 690, 615 cm⁻¹
   ```

**Quality Control:**
- Polystyrene film reference every session
- Verify library matches manually
- Repeat measurements: 10% of particles

### 4.4 Fluorescence Staining

**Purpose**: Rapid screening, distinguish from natural particles

**Stains:**
- Nile Red: General plastic stain
- BODIPY: Lipophilic dye
- Calcofluor White: Cellulose (negative control)

**Procedure:**

1. **Nile Red Staining**
   ```
   a. Prepare Nile Red solution (1 μg/mL in ethanol)
   b. Add 1 mL to sample on filter
   c. Incubate 30 minutes in dark
   d. Rinse with filtered water
   e. Observe under fluorescence microscope
      - Excitation: 460-550 nm
      - Emission: 590-650 nm (orange-red fluorescence)
   ```

2. **Image Analysis**
   ```
   - Capture fluorescence images
   - Apply threshold to segment particles
   - Count particles automatically
   - Measure size distribution
   ```

**Limitations:**
- Not polymer-specific (all plastics fluoresce)
- False positives from natural organic matter
- Requires confirmation with Raman or FTIR

---

## 5. Quality Control Protocols

### 5.1 Contamination Prevention

**Laboratory Requirements:**
```
- Clean room or laminar flow hood
- HEPA-filtered air
- Cotton lab coats (no synthetic fibers)
- Nitrile gloves
- Glass or metal equipment only (no plastic)
- Dedicated workspace for microplastics
```

**Procedural Controls:**
```
1. Wet cleaning surfaces before work
2. Cover all samples when not in use
3. Use filtered water for all rinsing
4. Minimize air exposure during transfers
5. No plastic pens, clipboards, or clothing
```

### 5.2 Blank Controls

#### 5.2.1 Field Blanks

**Procedure:**
```
- Expose clean filter to field conditions
- Do not filter any sample
- Process identically to samples
- Analyze for contamination
```

**Acceptance Criteria:**
```
- < 5 particles per blank
- Blank contamination < 10% of sample count
```

#### 5.2.2 Procedural Blanks

**Procedure:**
```
- Process blank filter through all steps
- Density separation with no sediment
- Digestion with no sample
- Analysis on microscope
```

**Acceptance Criteria:**
```
- < 3 particles per procedural blank
- If exceeded, re-clean and repeat
```

### 5.3 Positive Controls

**Procedure:**
```
1. Spike sediment with known plastic particles
   - Type: PE, PP, PS beads
   - Size: 50, 100, 200 μm
   - Quantity: 100 particles
2. Process through entire protocol
3. Calculate recovery rate
```

**Acceptance Criteria:**
```
- Recovery rate: 90-110%
- Polymer identification accuracy: >95%
```

### 5.4 Quality Assurance Samples

**Types:**
```
- Duplicates: 10% of samples
- Split samples: Same sample, different analysts
- Reference materials: Certified plastic particles
```

**Metrics:**
```
- Relative percent difference (RPD) < 20%
- Coefficient of variation < 15%
```

---

## 6. Data Validation Protocol

### 6.1 Particle Validation

**Criteria:**
```
1. Visual identification confidence
2. Spectroscopic confirmation (Raman or FTIR)
3. Match score > 0.70
4. Absence of biological markers
5. Peer review by second analyst (for critical samples)
```

### 6.2 Data Completeness Check

**Required Fields:**
```
✓ Sample ID
✓ Collection date and location
✓ Environmental parameters
✓ Processing date and method
✓ Analysis date and analyst
✓ Particle count and characteristics
✓ QC results (blanks, controls)
```

### 6.3 Data Quality Flags

**Flag System:**
```
- A: High quality (spectroscopically confirmed)
- B: Good quality (visual + partial spectrum)
- C: Acceptable (visual only, high confidence)
- D: Questionable (uncertain identification)
- E: Invalid (failed QC)
```

---

## 7. Safety Protocols

### 7.1 Chemical Safety

**Hazardous Chemicals:**
```
- Hydrogen peroxide (30%): Corrosive, oxidizer
- Sodium iodide: Irritant
- Zinc chloride: Corrosive, toxic
- Acids (HCl, HNO₃): Corrosive
```

**Safety Measures:**
```
- Fume hood for all digestions
- Nitrile gloves, safety glasses, lab coat
- Spill kit available
- Emergency shower and eyewash accessible
```

### 7.2 Biological Safety

**Samples from Wastewater/Sewage:**
```
- Treat as biohazard
- Autoclave before disposal
- Separate workspace from clean samples
```

### 7.3 Laser Safety

**Raman Spectroscopy:**
```
- Class 3B or 4 lasers
- Laser safety goggles required
- Training certification
- Warning signs posted
```

---

## 8. Standard Operating Procedure (SOP) Template

### 8.1 SOP Structure

```
1. Title and SOP Number
2. Purpose and Scope
3. Responsibilities
4. Materials and Equipment
5. Procedure (step-by-step)
6. Quality Control
7. Data Recording
8. Safety Precautions
9. References
10. Revision History
```

### 8.2 SOP Management

```
- Review annually
- Update after method changes
- Version control
- Training records for all users
```

---

## 9. Proficiency Testing

### 9.1 Inter-laboratory Comparison

**Procedure:**
```
1. Prepare homogeneous sample
2. Distribute to participating labs
3. Each lab analyzes independently
4. Compare results statistically
5. Identify outliers and systematic biases
```

**Metrics:**
```
- Z-score: (Lab result - Mean) / SD
- Acceptable: |Z| < 2
```

### 9.2 Method Validation

**Parameters to Validate:**
```
- Precision (repeatability and reproducibility)
- Accuracy (recovery rate)
- Detection limit
- Quantification limit
- Linearity
- Specificity
```

---

## 10. Protocol Decision Tree

```
Sample Type?
│
├─ Water → Volume? → <100 L: Bulk filtration
│                   → >100 L: Trawl net
│
├─ Sediment → Organic matter high? → Yes: Digestion + Density separation
│                                   → No: Density separation only
│
├─ Soil → Similar to sediment
│
└─ Air → High-volume sampler + filtration

Analysis Method?
│
├─ Screening → Visual + Fluorescence staining
│
└─ Confirmation → Raman or FTIR spectroscopy
```

---

**Document Version**: 1.0
**Last Updated**: 2026-01-12
**Status**: FULL Implementation
**Philosophy**: 弘益人間 - Rigorous science for a cleaner planet

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
