# WIA-AGRI-019: Lab-Grown Food
## Phase 3: Protocol Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Introduction

This specification defines standardized protocols for cellular agriculture and lab-grown food production. It provides detailed workflows, best practices, and quality standards for cultivating animal cells for human consumption.

### 1.1 Scope

This protocol covers:
- Cell isolation and line establishment
- Bioreactor cultivation procedures
- Differentiation and maturation protocols
- Quality control and safety testing
- Harvest and processing methods
- Environmental and safety requirements

### 1.2 Regulatory Context

This protocol aligns with:
- FDA Novel Food Guidelines
- EFSA Cell-Based Food Regulations
- ISO 20387 Biobanking Standards
- GMP Cell Culture Manufacturing
- HACCP Food Safety Principles

---

## 2. Cell Source Management

### 2.1 Animal Tissue Collection

**2.1.1 Biopsy Procedures**

Pre-requisites:
- Veterinary supervision required
- Animal welfare compliance (EU Directive 2010/63/EU)
- Sterile surgical environment
- Anesthesia protocols approved

Procedure:
```
1. Animal Selection
   - Age: 6-24 months optimal
   - Health: Complete veterinary examination
   - Documentation: Individual animal ID, health records

2. Biopsy Collection
   - Location: Gluteus medius or semitendinosus muscle
   - Size: 5-10 grams tissue
   - Method: Percutaneous needle biopsy (14-16 gauge)
   - Anesthesia: Local + sedation

3. Immediate Processing
   - Transport: Sterile container with transport medium
   - Temperature: 4°C
   - Time: Process within 4 hours of collection
   - Medium: DMEM + 10% FBS + 1% Pen/Strep
```

**2.1.2 Post-Biopsy Care**

- Monitor animal for 24 hours
- Administer antibiotics if needed
- Document any complications
- Animal can return to normal activity within 48 hours

### 2.2 Cell Isolation

**2.2.1 Tissue Dissociation**

Materials:
- Sterile dissection tools (scissors, forceps, scalpels)
- Collagenase Type II (0.2% w/v in DMEM)
- 70 μm cell strainers
- Centrifuge tubes (50 mL)
- Cell culture dishes (100 mm)

Procedure:
```
1. Tissue Preparation (Biosafety Cabinet Class II)
   - Remove tissue from transport medium
   - Wash 3× with PBS (Ca²⁺/Mg²⁺ free)
   - Trim connective tissue and fat
   - Mince into 1-2 mm³ pieces using sterile scalpels

2. Enzymatic Digestion
   - Add minced tissue to collagenase solution (10 mL per gram tissue)
   - Incubate at 37°C with gentle agitation
   - Duration: 90-120 minutes
   - Mix every 30 minutes

3. Cell Collection
   - Add equal volume DMEM + 10% FBS (stops enzyme activity)
   - Filter through 70 μm strainer
   - Centrifuge 300 × g for 5 minutes
   - Discard supernatant
   - Resuspend pellet in growth medium

4. Initial Plating
   - Count cells using hemocytometer
   - Seed at 5×10⁴ cells/cm²
   - Incubate 37°C, 5% CO₂
   - Change medium every 2-3 days
```

**2.2.2 Quality Assessment**

Day 1 Post-Isolation:
- Cell attachment: >60% cells adhered
- Viability: >85% (trypan blue exclusion)
- Contamination check: Microscopy for bacteria/fungi

Day 7 Post-Isolation:
- Confluence: 40-60%
- Morphology: Spindle-shaped satellite cells
- Growth: Evidence of colony formation

### 2.3 Cell Line Establishment

**2.3.1 Primary Culture Expansion**

Passage 0 → Passage 5:
```
Goal: Establish stable growth characteristics

1. Monitor Growth
   - Daily microscopy
   - Confluence assessment
   - Morphology documentation (photos)

2. Passage Protocol
   - Passage at 80% confluence
   - Trypsin-EDTA (0.25%, 3-5 minutes)
   - Neutralize with medium + serum
   - Split ratio: 1:3 to 1:4
   - Record passage number, date, density

3. Characterization (Passage 5)
   - Cell morphology
   - Growth kinetics (doubling time)
   - Cell markers (immunofluorescence)
   - Mycoplasma testing (PCR)
```

**2.3.2 Banking Protocol**

Master Cell Bank (MCB):
```
Passage: P5-P8
Vials: Minimum 20 vials
Cells per vial: 5×10⁶ - 1×10⁷
Freezing medium: 90% FBS + 10% DMSO

Procedure:
1. Collect cells at 80% confluence
2. Count and assess viability (>90% required)
3. Centrifuge and resuspend in freezing medium
4. Aliquot into cryovials (1 mL each)
5. Freeze using controlled rate freezer (-1°C/min)
6. Transfer to liquid nitrogen storage
7. Test 1 vial for viability after 24 hours
```

Working Cell Bank (WCB):
- Derived from MCB (expand 2-3 passages)
- Minimum 50 vials
- Same freezing protocol as MCB
- Use for production batches

**2.3.3 Cell Line Validation**

Required tests:
1. Identity: STR profiling or DNA fingerprinting
2. Purity: Species verification (PCR)
3. Sterility: 14-day bacterial/fungal culture
4. Mycoplasma: PCR and culture methods
5. Endotoxin: LAL assay (<0.5 EU/mL)
6. Viral screening: PCR panel for common viruses
7. Karyotype: G-banding chromosome analysis
8. Functionality: Differentiation capacity test

---

## 3. Bioreactor Cultivation

### 3.1 Small-Scale Culture (Flask/T-Flask)

**3.1.1 Seed Culture Preparation**

Purpose: Generate sufficient cells for bioreactor inoculation

Protocol:
```
1. Thaw WCB vial
   - Rapid thaw in 37°C water bath
   - Dilute 1:10 in pre-warmed medium
   - Centrifuge to remove DMSO
   - Resuspend and plate

2. Expansion
   - Start: 1×10⁵ cells/cm² in T-175 flasks
   - Medium: Proliferation medium (see Section 3.3)
   - Change medium every 2-3 days
   - Passage at 80% confluence

3. Scale-Up
   - Day 0-3: T-175 flasks (6 flasks)
   - Day 3-6: Cell factories (10-layer)
   - Day 6-9: Roller bottles
   - Target: 5×10⁸ total cells for 5L bioreactor

4. Quality Check Before Inoculation
   - Viability >95%
   - Mycoplasma negative
   - Morphology normal
   - Passage number <P15
```

### 3.2 Bioreactor Setup

**3.2.1 Equipment Specifications**

Stirred-Tank Bioreactor:
- Working volume: 5-100 L
- Material: Stainless steel (316L) or single-use bag
- Impeller: Marine or pitched blade (3-blade)
- Sparger: Microsparger for oxygen delivery
- Probes: pH, DO, temperature
- Sampling port: Aseptic sampling valve

**3.2.2 Pre-Culture Preparation**

```
1. Cleaning (for reusable systems)
   - CIP (Clean-in-Place): 1M NaOH, 80°C, 30 min
   - Rinse with WFI (Water for Injection)
   - Confirm conductivity <1.3 μS/cm

2. Sterilization
   - Autoclave: 121°C, 30 min, 15 psi
   - OR Steam-in-Place (SIP): 121°C, 60 min
   - Cool to room temperature

3. Probe Calibration
   - pH: 2-point calibration (pH 4.0 and 7.0)
   - DO: 0% (nitrogen) and 100% (air saturation)
   - Temperature: Verify against reference thermometer

4. Medium Preparation
   - Add 70% final volume of proliferation medium
   - Equilibrate to 37°C
   - Adjust pH to 7.2 ± 0.1
   - Aerate to 30% DO saturation
```

**3.2.3 Inoculation**

```
Target: 5×10⁵ cells/mL initial density

1. Transfer seed culture aseptically
2. Mix gently (30 RPM for 5 minutes)
3. Take sample for cell count and viability
4. Begin monitoring and control
5. Record start time and conditions
```

### 3.3 Culture Media Formulations

**3.3.1 Proliferation Medium**

Base: DMEM/F12 (1:1)

Supplements:
| Component | Concentration | Purpose |
|-----------|---------------|---------|
| FBS | 10% v/v | Growth support |
| Glucose | 4.5 g/L | Energy source |
| L-Glutamine | 4 mM | Amino acid |
| Sodium Pyruvate | 1 mM | Metabolism |
| HEPES | 10 mM | pH buffering |
| Pen/Strep | 100 U/mL | Antibiotics |
| FGF-2 | 5-10 ng/mL | Proliferation |
| EGF | 10 ng/mL | Growth factor |
| IGF-1 | 10 ng/mL | Growth factor |

pH: 7.2 ± 0.1
Osmolality: 290-310 mOsm/kg

**3.3.2 Differentiation Medium**

Base: DMEM (low glucose, 1 g/L)

Supplements:
| Component | Concentration | Purpose |
|-----------|---------------|---------|
| Horse Serum | 2% v/v | Differentiation |
| Insulin | 10 μg/mL | Metabolism |
| Transferrin | 5.5 μg/mL | Iron transport |
| Selenium | 5 ng/mL | Antioxidant |
| Dexamethasone | 0.4 μM | Differentiation |

pH: 7.4 ± 0.1

**3.3.3 Serum-Free Alternative**

For regulatory/cost considerations:

Base: Chemically defined medium

Supplements:
- Recombinant albumin (5 g/L)
- Soy hydrolysate (2 g/L)
- Lipid concentrate (1× solution)
- Recombinant growth factors
- Trace elements

### 3.4 Process Control

**3.4.1 Critical Process Parameters (CPPs)**

| Parameter | Set Point | Range | Action Limit |
|-----------|-----------|-------|--------------|
| pH | 7.20 | 7.15-7.25 | <7.10 or >7.30 |
| Temperature | 37.0°C | 36.8-37.2°C | <36.5°C or >37.5°C |
| DO | 30% | 25-35% | <20% or >40% |
| Agitation | 80 RPM | 60-100 RPM | <50 or >120 |
| Glucose | 3.0 g/L | 2.0-4.5 g/L | <1.5 g/L |
| Glutamine | 2.0 mM | 1.5-4.0 mM | <1.0 mM |
| Lactate | <15 mM | 0-15 mM | >20 mM |
| Ammonia | <2 mM | 0-2 mM | >3 mM |

**3.4.2 Feeding Strategy**

Batch Mode:
- No feeding during culture
- Monitor nutrient depletion
- Harvest at target density

Fed-Batch Mode:
```
Glucose Feeding:
- Trigger: <2.0 g/L
- Feed: 50% glucose solution
- Target: 4.0 g/L
- Method: Bolus or continuous

Glutamine Feeding:
- Trigger: <1.5 mM
- Feed: 200 mM stock solution
- Target: 3.0 mM
- Frequency: Every 2-3 days
```

Perfusion Mode:
- Continuous medium exchange
- Rate: 0.5-2 reactor volumes/day
- Cell retention: Spin filter or acoustic settler
- Advantage: Longer culture duration

**3.4.3 Monitoring Schedule**

Real-time (automated):
- pH, temperature, DO, agitation
- Data logging: Every 1 minute

Daily (manual):
- Cell count (hemocytometer or Vi-CELL)
- Viability (trypan blue)
- Glucose, lactate (biochemistry analyzer)
- Glutamine, ammonia (enzymatic assay)
- Microscopy (morphology check)

Weekly:
- Mycoplasma testing (PCR)
- Sterility test (culture 3 days)
- Cell markers (flow cytometry)

### 3.5 Proliferation Phase

**3.5.1 Growth Kinetics**

Target exponential growth:
- Doubling time: 20-30 hours
- Specific growth rate (μ): 0.023-0.035 h⁻¹
- Duration: 7-10 days
- Final density: 1×10⁷ cells/mL

Growth curve monitoring:
```
Day 0: 5×10⁵ cells/mL (inoculation)
Day 2: 1.5×10⁶ cells/mL
Day 4: 4×10⁶ cells/mL
Day 6: 8×10⁶ cells/mL
Day 8: 1×10⁷ cells/mL (harvest or differentiation)
```

**3.5.2 Troubleshooting**

Slow Growth:
- Check: Nutrient levels, pH, temperature
- Action: Adjust feeding, verify probe calibration

High Lactate Production:
- Check: Glucose concentration, DO level
- Action: Reduce glucose feed, increase aeration

Cell Aggregation:
- Check: Agitation speed, cell density
- Action: Increase agitation gently, passage earlier

### 3.6 Differentiation Phase

**3.6.1 Differentiation Induction**

Timing: When cells reach 1×10⁷ cells/mL

Protocol:
```
1. Medium Exchange
   - Centrifuge or settle cells
   - Remove 90% proliferation medium
   - Add differentiation medium (Section 3.3.2)
   - Resuspend gently

2. Parameter Adjustment
   - Temperature: Maintain 37°C
   - pH: Adjust to 7.4
   - DO: Reduce to 20-25%
   - Agitation: Reduce to 40-60 RPM

3. Duration
   - 7-14 days
   - Medium change every 2-3 days
   - Monitor differentiation markers
```

**3.6.2 Differentiation Markers**

Muscle differentiation:
- Myogenin expression (qPCR, Western blot)
- MyoD upregulation
- Myosin heavy chain (MHC) production
- Fusion index: % multinucleated cells

Expected timeline:
- Day 0-3: Myogenin induction
- Day 4-7: Cell fusion begins
- Day 8-14: Myotube formation, MHC expression

---

## 4. Scaffold Integration

### 4.1 Scaffold Types

**4.1.1 Plant-Based Scaffolds**

Soy protein scaffold:
- Composition: Soy protein isolate
- Structure: Porous network (100-500 μm pores)
- Preparation: Freeze-drying, crosslinking
- Sterilization: 70% ethanol, gamma irradiation

Decellularized plant tissue:
- Source: Spinach leaves, apple slices
- Decellularization: SDS + Triton X-100
- Advantage: Natural vascular network
- Preparation time: 48-72 hours

**4.1.2 Hydrogel Scaffolds**

Alginate-based:
```
Formulation:
- Sodium alginate: 2% w/v
- CaCl₂: 100 mM (crosslinker)
- Cell density: 2×10⁶ cells/mL

Gelation:
- Mix cells with alginate solution
- Drop into CaCl₂ bath
- Crosslink 10 minutes
- Wash with culture medium
```

Collagen hydrogel:
- Type I collagen (rat tail, 3-5 mg/mL)
- Neutralize with NaOH to pH 7.4
- Polymerize at 37°C for 30 minutes

### 4.2 Cell Seeding

**4.2.1 Static Seeding**

```
1. Scaffold Preparation
   - Cut to desired size (e.g., 1 cm³ cubes)
   - Sterilize and equilibrate in medium
   - Pre-wet in serum-containing medium

2. Cell Seeding
   - Density: 2×10⁶ cells/cm³ scaffold
   - Method: Pipette cell suspension onto scaffold
   - Incubation: 2 hours static for attachment
   - Then: Transfer to culture dish with medium

3. Culture Conditions
   - Medium: Differentiation medium
   - Change: Every 2 days
   - Duration: 14-21 days
```

**4.2.2 Dynamic Seeding (Bioreactor)**

Advantages: Better cell distribution, nutrient perfusion

```
1. Use spinner flask or rotating wall vessel
2. Add scaffold and cell suspension
3. Rotate at 20-40 RPM
4. Allow cells to infiltrate and attach
5. Increase rotation gradually over 7 days
```

### 4.3 Tissue Maturation

**4.3.1 Mechanical Stimulation**

Rationale: Improves muscle fiber alignment and protein synthesis

Equipment:
- Stretch bioreactor
- Strain: 5-10%
- Frequency: 0.5-1 Hz
- Duration: 1-4 hours/day
- Total: 7-14 days

**4.3.2 Electrical Stimulation**

Parameters:
- Voltage: 2-5 V/cm
- Frequency: 1 Hz
- Pulse width: 2-5 ms
- Duration: 30-60 minutes/day

Effects:
- Enhanced myotube maturation
- Increased contractile protein expression
- Improved tissue structure

**4.3.3 Co-Culture Systems**

Muscle + Fat:
- Improves flavor and texture
- Ratio: 80% muscle, 20% adipocytes
- Differentiate separately, combine at maturation

Muscle + Endothelial:
- Creates vascular-like structures
- Ratio: 90% muscle, 10% endothelial
- Supports nutrient diffusion

---

## 5. Harvest and Processing

### 5.1 Harvest Criteria

Ready for harvest when:
- Cell density: 1-2×10⁷ cells/mL (suspension)
- Viability: >85%
- Differentiation markers: Positive
- Tissue thickness: 2-5 mm (scaffold)
- Culture duration: 21-28 days total

### 5.2 Harvest Procedure

**5.2.1 Suspension Culture**

```
1. Final sampling for QC
2. Stop agitation and aeration
3. Allow cells to settle (or centrifuge)
4. Remove spent medium
5. Wash with PBS (2×)
6. Collect cell pellet
7. Weigh wet mass
8. Process immediately or freeze
```

**5.2.2 Scaffold Culture**

```
1. Remove from culture vessel
2. Rinse gently with PBS
3. Remove scaffold (if non-edible)
4. Collect tissue
5. Weigh and measure dimensions
6. Process for further use
```

### 5.3 Processing Methods

**5.3.1 Homogenization**

For ground meat products:
```
1. Mechanical grinding
2. Add food-grade binders (if needed)
3. Form into patties or sausages
4. Package under aseptic conditions
```

**5.3.2 Whole-Cut Products**

For steak-like products:
```
1. Keep tissue intact on scaffold
2. Trim to desired shape
3. Package with modified atmosphere
4. Refrigerate at 4°C
5. Shelf life: 7-14 days
```

### 5.4 Post-Harvest Quality Control

Required tests before release:
1. Microbiological safety (Section 6.2)
2. Nutritional analysis (Section 6.3)
3. Allergen testing
4. Heavy metals
5. Sensory evaluation

---

## 6. Quality Control

### 6.1 In-Process Controls

Daily monitoring:
- Cell count and viability
- pH, temperature, DO
- Glucose, lactate, glutamine
- Visual inspection for contamination

Weekly monitoring:
- Mycoplasma (PCR)
- Endotoxin (LAL assay)
- Cell morphology (microscopy)

### 6.2 Microbiological Testing

**6.2.1 Sterility Test**

Method: Direct inoculation
- Sample: 1 mL culture or 1 g tissue
- Media: TSB (bacteria) and SDA (fungi)
- Incubation: 14 days at 30-35°C
- Result: No growth = pass

**6.2.2 Pathogen Testing**

| Pathogen | Method | Limit |
|----------|--------|-------|
| E. coli O157:H7 | PCR | Not detected |
| Salmonella | ELISA | Not detected |
| Listeria monocytogenes | PCR | Not detected |
| Campylobacter | Culture | Not detected |

**6.2.3 Total Viable Count**

- Method: Pour plate
- Media: Plate Count Agar
- Incubation: 35°C, 48 hours
- Limit: <10⁴ CFU/g

### 6.3 Nutritional Analysis

Required analyses:
```
Macronutrients:
- Protein (Kjeldahl method)
- Fat (Soxhlet extraction)
- Carbohydrates (calculation)
- Moisture (oven drying)
- Ash (incineration)

Micronutrients:
- Iron (ICP-MS)
- Vitamin B12 (HPLC)
- Zinc, selenium (ICP-MS)

Amino Acid Profile:
- HPLC analysis
- Essential amino acids quantified
```

### 6.4 Safety Testing

**6.4.1 Heavy Metals**

| Metal | Method | Limit (mg/kg) |
|-------|--------|---------------|
| Lead (Pb) | ICP-MS | 0.1 |
| Cadmium (Cd) | ICP-MS | 0.05 |
| Mercury (Hg) | ICP-MS | 0.03 |
| Arsenic (As) | ICP-MS | 0.1 |

**6.4.2 Allergen Testing**

Test for:
- Soy (if scaffolds used)
- Milk proteins (from serum)
- Common allergens per region

**6.4.3 Residues**

- Antibiotics (ELISA)
- Growth hormones (if used)
- Processing aids

### 6.5 Sensory Evaluation

**6.5.1 Trained Panel**

Evaluate:
- Color: Compare to reference meat
- Texture: Firmness, juiciness, chewiness
- Odor: Fresh, off-odors
- Taste: Cooked product
- Overall acceptability

Scale: 1-9 (1=dislike extremely, 9=like extremely)

**6.5.2 Instrumental Analysis**

- Color: Colorimeter (L*, a*, b*)
- Texture: Texture analyzer (firmness, springiness)
- Water holding capacity
- Cooking loss percentage

---

## 7. Documentation and Traceability

### 7.1 Batch Records

Required documentation:
- Cell line ID and passage number
- Culture start/end dates
- Operator names
- Medium lot numbers
- All process parameters (continuous log)
- Deviations and corrective actions
- QC test results
- Final disposition (approved/rejected)

### 7.2 Chain of Custody

Track:
1. Animal ID → Biopsy → Cell line
2. Cell line → Batch ID → Final product
3. All materials and reagents (lot numbers)
4. Processing steps with timestamps
5. Distribution records

---

## 8. Safety and Environmental

### 8.1 Biosafety

- BSL-1 practices minimum
- BSL-2 if using human cells
- Proper PPE (lab coat, gloves, eye protection)
- Biosafety cabinets for cell handling
- Autoclave all waste

### 8.2 Waste Management

- Liquid waste: Inactivate with bleach before drain
- Solid waste: Autoclave before disposal
- Sharps: Designated containers
- Chemical waste: Follow local regulations

### 8.3 Environmental Controls

- Room temperature: 18-25°C
- Humidity: 30-60%
- Air changes: 15-20 per hour
- HEPA filtration for cleanrooms
- Positive pressure for cleanrooms

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
