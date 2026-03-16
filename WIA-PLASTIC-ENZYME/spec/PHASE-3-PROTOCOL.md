# WIA-PLASTIC-ENZYME Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 of the WIA-PLASTIC-ENZYME standard defines the operational protocols that ensure consistent, safe, and efficient enzymatic plastic degradation. These protocols cover the entire process from waste collection to monomer certification.

### 1.1 Scope

This specification covers:

- Pre-treatment protocols
- Enzymatic reaction protocols
- Monomer recovery protocols
- Quality control protocols
- Safety and environmental protocols

### 1.2 Process Flow

```
Collection → Sorting → Pre-treatment → Reaction → Recovery → QC → Certification
```

---

## 2. Pre-treatment Protocol

### 2.1 Cleaning and Decontamination

**Objective:** Remove labels, adhesives, and contaminants that could inhibit enzyme activity.

| Step | Process | Parameters | Duration |
|------|---------|------------|----------|
| 1 | Hot water wash | 80°C | 10 min |
| 2 | Alkaline treatment | 1% NaOH | 5 min |
| 3 | Rinse | Deionized water | 5 min |
| 4 | Drying | 60°C hot air | Until <0.5% moisture |

**Quality Checkpoints:**
- [ ] Visual inspection for remaining labels
- [ ] pH verification (6.5-7.5)
- [ ] Moisture content measurement (<0.5%)
- [ ] Contamination assessment

### 2.2 Size Reduction

**Objective:** Increase surface area for better enzyme access.

| Target Size | Method | Surface Area Increase | Rate Improvement |
|-------------|--------|----------------------|------------------|
| <10mm flakes | Primary grinding | Baseline | 1.0x |
| <2mm particles | Secondary milling | 5x | 2.5x |
| <0.5mm powder | Micronization | 20x | 4x |

**Equipment Requirements:**
- Shredder: Min. 50 kW capacity
- Granulator: Adjustable screen size 1-10mm
- Mill: Cryogenic or ambient, depending on plastic type

### 2.3 Crystallinity Optimization

**Objective:** Reduce crystallinity to increase amorphous regions accessible to enzymes.

**Protocol:**
1. Heat plastic above Tg (>75°C for PET)
2. Maintain for 5-10 minutes
3. Rapid cooling to ambient temperature
4. Verify crystallinity reduction via DSC or FTIR

**Target:** Reduce crystallinity from 30-40% to 15-25%

---

## 3. Enzymatic Reaction Protocol

### 3.1 Reactor Preparation

**Buffer System:**
- 50mM Tris-HCl or phosphate buffer
- Target pH: 8.0 ± 0.2
- Ionic strength: 100-200 mM

**Pre-heat Checklist:**
- [ ] Buffer prepared and pH verified
- [ ] Reactor cleaned and sterilized
- [ ] Temperature controller calibrated
- [ ] Agitation system functional

### 3.2 Standard Operating Conditions

| Parameter | Value | Tolerance |
|-----------|-------|-----------|
| Temperature | 50-55°C | ±2°C |
| pH | 8.0 | ±0.3 |
| Substrate loading | 10-15% (w/v) | - |
| Agitation | 100-200 RPM | - |
| PETase concentration | 2-3 mg/g substrate | - |
| MHETase concentration | 1-1.5 mg/g substrate | - |

### 3.3 Enzyme Addition Protocol

1. **Pre-mixing:** Dissolve enzymes in buffer at room temperature
2. **Gradual addition:** Add enzyme solution over 30 minutes with mixing
3. **Homogenization:** Continue mixing for 15 minutes
4. **Reaction start:** Begin temperature and time logging

### 3.4 Reaction Monitoring

**Monitoring Schedule:**

| Time (h) | Actions |
|----------|---------|
| 0 | Record initial conditions, start logging |
| 6 | First sample, check pH |
| 12 | Sample, TPA/MHET analysis |
| 24 | Sample, assess progress |
| 36 | Sample, pH adjustment if needed |
| 48 | End-point determination |

**Analytical Methods:**
- HPLC-UV for TPA, MHET, BHET quantification
- Gravimetric analysis for residual plastic
- pH and temperature continuous logging

### 3.5 Completion Criteria

| Criterion | Target |
|-----------|--------|
| Degradation | ≥95% |
| TPA concentration plateau | 4+ hours stable |
| Residual plastic | <5% of input |

---

## 4. Monomer Recovery Protocol

### 4.1 Solid-Liquid Separation

1. **Enzyme inactivation:** Heat to 80°C for 15 minutes
2. **Coarse filtration:** 200 mesh filter to remove undegraded material
3. **Centrifugation:** 10,000g for 20 minutes
4. **Clarification:** 0.45µm membrane filtration

### 4.2 TPA Crystallization

| Step | Process | Parameters |
|------|---------|------------|
| 1 | Acidification | Add HCl to pH 2.5 |
| 2 | Cooling | Reduce to 4°C over 2 hours |
| 3 | Crystallization | Hold at 4°C for 4 hours |
| 4 | Filtration | Vacuum filter crystals |
| 5 | Washing | Cold DI water, 2x volume |
| 6 | Drying | Vacuum dry at 60°C |

**Expected Yield:** 85-95% of theoretical

### 4.3 Ethylene Glycol Recovery

| Step | Process | Parameters |
|------|---------|------------|
| 1 | Neutralization | Adjust to pH 7 |
| 2 | Concentration | Evaporate to 50% volume |
| 3 | Distillation | Vacuum, 80°C |
| 4 | Dehydration | Molecular sieves |

**Expected Purity:** ≥98.5%

---

## 5. Quality Control Protocol

### 5.1 TPA Quality Specifications

| Parameter | Method | Specification |
|-----------|--------|---------------|
| Purity | HPLC | ≥99.0% |
| Color (Hazen) | Spectrophotometry | ≤20 |
| Ash content | Gravimetric | ≤0.01% |
| Metal ions (each) | ICP-MS | ≤1 ppm |
| Moisture | Karl Fischer | ≤0.1% |
| MHET impurity | HPLC | ≤500 ppm |
| BHET impurity | HPLC | ≤200 ppm |

### 5.2 EG Quality Specifications

| Parameter | Method | Specification |
|-----------|--------|---------------|
| Purity | GC-FID | ≥99.5% |
| Water content | Karl Fischer | ≤0.1% |
| Acidity | Titration | ≤0.01 mg KOH/g |
| Color (APHA) | Spectrophotometry | ≤10 |
| DEG content | GC | ≤0.1% |

### 5.3 Food Contact Certification Requirements

For food-contact grade monomers, additional testing:

| Test | Requirement |
|------|-------------|
| Heavy metals | Meet EC 1935/2004 |
| Microbial count | <100 CFU/g |
| Endotoxin | <0.5 EU/mL |
| Residual enzyme activity | Undetectable |
| Volatile organics | Meet FDA 21 CFR 177.1630 |

---

## 6. Safety Protocol

### 6.1 Personal Protective Equipment

| Operation | Required PPE |
|-----------|-------------|
| Chemical handling | Lab coat, safety glasses, chemical gloves |
| Powder handling | Respirator (N95+), safety glasses |
| High temperature | Heat-resistant gloves, face shield |
| Sampling | Disposable gloves, lab coat |

### 6.2 Chemical Hazards

| Chemical | Hazard | Mitigation |
|----------|--------|------------|
| NaOH (1%) | Corrosive | Neutralization station, eyewash |
| HCl (concentrated) | Corrosive, fumes | Fume hood, neutralization |
| Enzymes | Potential allergen | Avoid aerosol generation |

### 6.3 Emergency Procedures

- **Enzyme spill:** Inactivate with 70% ethanol, clean with detergent
- **Acid/base spill:** Neutralize, absorb, dispose as hazardous waste
- **High temperature exposure:** Cool with water, seek medical attention

---

## 7. Environmental Protocol

### 7.1 Wastewater Management

| Stream | Treatment | Discharge Limit |
|--------|-----------|-----------------|
| Wash water | Neutralization, settling | pH 6-9, TSS <50 mg/L |
| Reaction liquor | Enzyme inactivation | No active enzymes |
| Acid/base waste | Neutralization | pH 6-9 |

### 7.2 Solid Waste Management

| Waste | Classification | Disposal |
|-------|---------------|----------|
| Undegraded plastic residue | Non-hazardous | Energy recovery or landfill |
| Filter media | Non-hazardous | Industrial waste |
| Contaminated PPE | Non-hazardous | Incineration |

### 7.3 Energy Efficiency

**Target:** <50 kWh per ton of plastic processed

**Energy optimization:**
- Heat recovery from reactor cooling
- Optimized agitation speed
- Insulated reactors and piping
- Off-peak electricity usage

---

## 8. Documentation Requirements

### 8.1 Batch Records

Each batch must include:

- [ ] Batch number and date
- [ ] Input material characterization
- [ ] Enzyme lot numbers and concentrations
- [ ] Time-temperature-pH log
- [ ] Sampling records and results
- [ ] Output yields and quality results
- [ ] Deviation reports (if any)
- [ ] Operator signatures

### 8.2 Retention Period

| Document Type | Retention |
|---------------|-----------|
| Batch records | 5 years |
| Quality certificates | 10 years |
| Training records | Duration of employment + 3 years |
| Equipment logs | 3 years |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association
