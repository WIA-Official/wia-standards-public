# WIA-SPACE-009: Space Radiation Protection Standard
## Version 1.0

**Status:** Active
**Published:** 2025-01-26
**Category:** Space Safety / Radiation Protection
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Introduction

### 1.1 Purpose

WIA-SPACE-009 establishes comprehensive standards and protocols for protecting astronauts and spacecraft from space radiation during all phases of space missions, including:

- Low Earth Orbit (LEO) operations
- Lunar missions and surface operations
- Mars expeditions and deep space exploration
- Extravehicular activities (EVA)
- Emergency radiation events

### 1.2 Scope

This standard covers:

- Radiation sources (GCR, SPE, trapped radiation)
- Health effects and risk assessment
- Shielding technologies (passive and active)
- Radiation monitoring systems
- Operational protocols and procedures
- Medical countermeasures
- Mission planning considerations

### 1.3 Applicable Documents

- ICRP Publications on radiation protection
- NASA STD-3001 (Space Flight Human System Standard)
- ISO 15390 (Space environment - Galactic cosmic rays)
- NCRP Reports on space radiation

---

## 2. Radiation Sources

### 2.1 Galactic Cosmic Rays (GCR)

**Characteristics:**
- Composition: ~85% protons, ~14% helium, ~1% HZE particles
- Energy range: Hundreds of MeV to hundreds of GeV
- Continuous exposure throughout mission
- Modulated by solar cycle (30-50% variation)

**Risk Level:** High for long-duration missions (>6 months)

**Shielding Difficulty:** Very High - requires meters of shielding for complete protection

### 2.2 Solar Particle Events (SPE)

**Characteristics:**
- Composition: >90% protons
- Energy range: Tens to hundreds of MeV
- Duration: Hours to days
- Frequency: Correlated with solar cycle

**Risk Level:** Very High during events (acute radiation syndrome possible)

**Warning Time:** 10 minutes to several hours

### 2.3 Trapped Radiation (Van Allen Belts)

**Inner Belt:**
- Altitude: 1,000-6,000 km
- Primary particles: High-energy protons (10-hundreds of MeV)
- Stability: Very stable

**Outer Belt:**
- Altitude: 13,000-60,000 km
- Primary particles: Electrons (0.1-10 MeV)
- Stability: Highly variable

**South Atlantic Anomaly (SAA):**
- Location: ~30°S, 60°W
- Contributes 70-80% of ISS astronaut dose
- Transit frequency: 5-6 times/day on ISS orbit

---

## 3. Dose Limits and Risk Criteria

### 3.1 NASA Dose Limits

**Career Limits (3% Risk of Exposure-Induced Death):**
- Age 25 Female: 600 mSv
- Age 25 Male: 800 mSv
- Age 35 Female: 900 mSv
- Age 35 Male: 1,000 mSv
- Age 45 Female: 1,100 mSv
- Age 45 Male: 1,200 mSv

**Short-term Limits:**
- 30 days: 250 mSv
- Annual: 500 mSv

**Organ-specific Limits:**
- Eye lens (career): 2,000 mSv (cataract prevention)
- Skin (30 days): 6,000 mSv (acute skin damage prevention)

### 3.2 ALARA Principle

All radiation exposure shall be kept As Low As Reasonably Achievable through:
- Time: Minimize exposure duration
- Distance: Maximize distance from radiation sources
- Shielding: Use appropriate shielding materials

---

## 4. Shielding Requirements

### 4.1 Passive Shielding

**Habitat Shielding:**
- Minimum: 10 g/cm² aluminum equivalent
- Recommended: 20 g/cm² for sleeping quarters
- Storm shelter: 40 g/cm² polyethylene or water equivalent

**Materials Priority:**
1. Hydrogen-rich materials (polyethylene, water)
2. Multi-layer structures
3. Low-Z materials to minimize secondary radiation

**Material Specifications:**

| Material | Density (g/cm³) | Hydrogen Content | Application |
|----------|-----------------|------------------|-------------|
| Polyethylene | 0.92 | 14% | Dedicated shielding layers |
| Water | 1.0 | 11% | Multi-purpose (life support + shielding) |
| Aluminum | 2.7 | 0% | Structural (baseline) |
| Boron composite | 1.8-2.5 | Varies | Neutron capture |

### 4.2 Storm Shelter Design

**Requirements:**
- Shielding: 20-40 g/cm² effective thickness
- Capacity: All crew members
- Duration: Minimum 72 hours self-sufficient
- Location: Central area with maximum shielding
- Access: Reachable within 15 minutes from any location

**Life Support:**
- Oxygen supply
- CO₂ removal
- Temperature control
- Food and water
- Waste management
- Communications

### 4.3 EVA Suit Enhancement

**Current EMU Shielding:** ~0.1-0.2 g/cm²

**Enhancement Targets:**
- Torso: 0.5 g/cm² (critical organ protection)
- Helmet: Enhanced polycarbonate + hydrogen gel
- Flexible joints: Polyethylene fiber weave

---

## 5. Monitoring Systems

### 5.1 Personal Dosimeters

**Passive Dosimeters:**
- TLD (Thermoluminescent Dosimeter): Minimum 2 per crew member
- OSL (Optically Stimulated Luminescence): Backup dosimeter
- Read-out frequency: Weekly minimum, post-mission

**Active Dosimeters:**
- Real-time dose and dose rate display
- Alarming capability (threshold: 2x background)
- Data logging (minimum 1-hour resolution)
- Wireless data transmission to health monitoring system

**Required Measurements:**
- Absorbed dose (Gy)
- Dose equivalent (Sv)
- Dose rate (mSv/day)
- LET spectrum (for quality factor determination)

### 5.2 Area Monitoring

**Fixed Monitors:**
- Location: All habitable modules
- Measurement: Continuous dose rate
- Alert: Automatic when dose rate exceeds threshold
- Data: Archived for mission duration

**Environmental Monitors:**
- External radiation environment
- Particle flux and energy spectrum
- Real-time SPE detection

### 5.3 Data Management

**Real-time Requirements:**
- Data transmission: Every 24 hours minimum
- Emergency alerts: Immediate transmission
- Ground analysis: Daily dose assessment
- Crew notification: Within 1 hour of analysis

**Database:**
- Individual cumulative dose tracking
- Historical mission data
- Comparison with models
- Long-term health correlation

---

## 6. Operational Protocols

### 6.1 Mission Planning

**Launch Window Selection:**
- Solar cycle consideration
- SPE probability assessment
- GCR flux prediction

**Route Optimization:**
- Van Allen belt transit: Fastest trajectory through thinnest regions
- Earth shadow utilization when possible
- SPE avoidance maneuvers (contingency)

**Mission Duration:**
- Minimize transit time (faster propulsion)
- Balance surface stay time with radiation risk
- Dose budget allocation

### 6.2 EVA Procedures

**Pre-EVA:**
1. Solar activity forecast review (48 hours)
2. Current radiation environment assessment
3. Go/No-Go decision based on:
   - No M-class or larger flares in past 24 hours
   - No active region with high flare probability
   - Background dose rate within normal range
   - SAA passage timing (ISS)

**During EVA:**
1. Continuous personal dosimeter monitoring
2. Ground-based environment monitoring
3. Time limit enforcement
4. Emergency abort criteria:
   - SPE warning received
   - Dose rate >5x background
   - Cumulative dose approaching daily limit

**Post-EVA:**
1. Dosimeter reading and recording
2. Crew health assessment
3. Cumulative dose update

**EVA Limits:**
- Single EVA: 8 hours maximum
- Weekly: 16 hours maximum
- Mission total: Based on dose budget

### 6.3 SPE Response Protocol

**Alert Levels:**

| Level | Condition | Action |
|-------|-----------|--------|
| Green | Normal | Continue operations |
| Yellow | M-class flare or dose rate 2x background | Increase monitoring, review EVA |
| Orange | X-class flare or SPE predicted | Abort EVA, prepare shelter |
| Red | SPE in progress, dose rate >10x | Immediate shelter evacuation |

**Shelter Procedures:**
1. Alert reception
2. Cease non-essential activities
3. Shelter entry (target: <15 minutes)
4. Hatch closure and verification
5. Continuous monitoring (internal/external dosimeters)
6. Ground communication
7. Duration: Until dose rate returns to <2x background

**Shelter Activities:**
- Minimize movement (energy conservation)
- Health monitoring
- Maintain communication
- Psychological support

### 6.4 Acute Radiation Syndrome (ARS) Response

**Diagnosis:**
- Symptom onset time (nausea, vomiting)
- Dose estimation (dosimeters)
- Lymphocyte count (blood test if available)

**Treatment:**
1. Antiemetics (Ondansetron)
2. Hydration (oral or IV)
3. G-CSF (if dose >2 Gy)
4. Antibiotics (prophylactic if dose >1 Gy)
5. Supportive care
6. Ground medical consultation
7. Consider early return if severe (>4 Gy)

---

## 7. Medical Countermeasures

### 7.1 Radioprotectors

**Definition:** Agents administered before exposure to prevent or reduce cellular damage

**Candidates:**
- Amifostine (WR-2721): FDA-approved, but significant side effects
- Antioxidants: Vitamin C, E, Selenium, N-acetylcysteine
- Administration: Prior to high-risk activities (EVA during elevated risk)

### 7.2 Radiation Mitigators

**Definition:** Agents administered after exposure to limit damage progression

**Primary:**
- G-CSF (Granulocyte Colony-Stimulating Factor): Bone marrow recovery
- EPO (Erythropoietin): Red blood cell production
- TPO (Thrombopoietin): Platelet production

**Administration Timeline:** Within 24-48 hours of significant exposure (>1 Gy)

### 7.3 Pharmaceutical Inventory

**Required Medications:**
- Antiemetics: Ondansetron, Metoclopramide
- Growth factors: G-CSF, EPO
- Antibiotics: Broad-spectrum
- Analgesics: Pain management
- IV fluids and electrolytes
- Antioxidants

**Storage:**
- Radiation-shielded packaging
- Temperature-controlled
- Expiration date monitoring (minimum 3-year shelf life for Mars missions)

---

## 8. Post-Mission Health Monitoring

### 8.1 Immediate Post-Flight

**Within 7 days:**
- Complete physical examination
- Blood tests (complete blood count, biomarkers)
- Chromosome aberration analysis
- Cognitive function testing
- Ophthalmological exam

**Dose Verification:**
- All dosimeter read-outs
- Final dose assessment
- Career dose update

### 8.2 Long-term Follow-up

**Annual Examinations:**
- Comprehensive health assessment
- Cancer screening (enhanced frequency)
- Cardiovascular function evaluation
- Cognitive testing
- Cataract screening

**Lifetime Database:**
- Individual health records
- Radiation exposure history
- Correlation analysis
- Epidemiological studies

---

## 9. Quality Assurance

### 9.1 Equipment Calibration

**Dosimeters:**
- Calibration frequency: Annually minimum
- Calibration source: NIST-traceable standards
- Energy response: Verified across spectrum
- Documentation: Calibration certificates maintained

**Shielding Materials:**
- Composition verification
- Density measurements
- Quality control sampling

### 9.2 Procedure Verification

**Training:**
- All crew members: Radiation safety training
- Refresher: Annual
- Simulation exercises: SPE response, ARS treatment

**Audits:**
- Pre-mission: Verify all equipment and procedures
- In-mission: Periodic ground review
- Post-mission: Lessons learned analysis

---

## 10. Future Development

### 10.1 Technology Roadmap

**Near-term (2025-2030):**
- Advanced passive shielding materials (nanotechnology)
- Wearable real-time dosimeters
- Improved SPE prediction models

**Mid-term (2030-2040):**
- Active electromagnetic shielding demonstrations
- Personalized radioprotective pharmaceuticals
- ISRU shielding (lunar regolith, Martian soil)

**Long-term (2040+):**
- Compact active shielding for deep space vehicles
- Gene therapy for radiation resistance
- Regenerative medicine for radiation damage

### 10.2 Research Priorities

1. Long-term low-dose and low-dose-rate effects
2. HZE particle biological mechanisms
3. Individual radiosensitivity and personalized limits
4. CNS effects and cognitive countermeasures
5. Combined stressors (radiation + microgravity + isolation)

---

## 11. Compliance and Certification

### 11.1 Compliance Requirements

All space missions shall demonstrate compliance with WIA-SPACE-009 through:
- Pre-mission radiation protection plan
- Documented shielding analysis
- Dosimetry system verification
- Crew training records
- Medical preparedness assessment

### 11.2 Certification Process

1. Submit mission radiation profile
2. Review by WIA Radiation Safety Board
3. Pre-launch inspection
4. In-mission monitoring
5. Post-mission evaluation
6. Certification issuance or recommendations

---

## Appendix A: Acronyms and Definitions

**ARS:** Acute Radiation Syndrome
**CME:** Coronal Mass Ejection
**DSB:** Double-Strand Break
**EVA:** Extravehicular Activity
**GCR:** Galactic Cosmic Rays
**G-CSF:** Granulocyte Colony-Stimulating Factor
**HZE:** High-Z and Energy (heavy ions)
**ISS:** International Space Station
**ISRU:** In-Situ Resource Utilization
**LET:** Linear Energy Transfer
**LEO:** Low Earth Orbit
**OSL:** Optically Stimulated Luminescence
**REID:** Risk of Exposure-Induced Death
**SAA:** South Atlantic Anomaly
**SPE:** Solar Particle Event
**TLD:** Thermoluminescent Dosimeter

---

## Document Control

**Prepared by:** WIA Radiation Protection Working Group
**Approved by:** WIA Standards Committee
**Version:** 1.0
**Effective Date:** 2025-01-26
**Next Review:** 2027-01-26

---

**© 2025 SmileStory Inc. / World Certification Industry Association (WIA)**

弘益人間 (Hongik Ingan) · Benefit All Humanity
