# WIA-ENE-050: Direct Air Capture Standard
## PHASE 3: PROTOCOLS

**Version:** 1.0
**Status:** Draft
**Date:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Overview

This document defines the operational protocols for WIA-ENE-050 compliant Direct Air Capture facilities. It covers capture operations, storage procedures, safety protocols, and verification workflows.

### 1.1 Protocol Categories

1. **Capture Protocols:** DAC operation procedures
2. **Storage Protocols:** CO2 sequestration procedures
3. **Safety Protocols:** Operational safety requirements
4. **Verification Protocols:** MRV and certification workflows
5. **Emergency Protocols:** Incident response procedures

---

## 2. Capture Operation Protocols

### 2.1 Pre-Operation Protocol

**Protocol ID:** DAC-CAPTURE-PRE-001
**Version:** 1.0

#### 2.1.1 Pre-Operation Checklist

**Daily Pre-Operation:**
- [ ] Verify sorbent condition (saturation < 90%)
- [ ] Check energy supply availability
- [ ] Validate ambient conditions within operational range
- [ ] Test airflow systems (target: design specification)
- [ ] Verify data logging systems operational
- [ ] Check CO2 purity sensors calibrated
- [ ] Confirm storage facility availability
- [ ] Review weather forecast for next 24 hours

**Weekly Pre-Operation:**
- [ ] Inspect sorbent for physical degradation
- [ ] Check pressure vessel integrity
- [ ] Verify safety system functionality
- [ ] Calibrate temperature sensors
- [ ] Test emergency shutdown systems
- [ ] Review maintenance logs
- [ ] Update operational procedures if needed

**Monthly Pre-Operation:**
- [ ] Comprehensive sensor calibration
- [ ] Sorbent performance analysis
- [ ] Energy efficiency audit
- [ ] Third-party equipment inspection
- [ ] Safety drill execution
- [ ] Regulatory compliance check

#### 2.1.2 Environmental Conditions

Acceptable operating ranges:

| Parameter | Minimum | Optimal | Maximum |
|-----------|---------|---------|---------|
| Ambient CO2 | 350 ppm | 400-420 ppm | 500 ppm |
| Temperature | -10°C | 15-25°C | 40°C |
| Humidity | 20% | 40-70% | 95% |
| Wind Speed | 0 m/s | 2-5 m/s | 15 m/s |
| Pressure | 900 hPa | 1000-1020 hPa | 1100 hPa |

**Out-of-Range Actions:**
- **Minor deviation (5-10%):** Log warning, continue operation
- **Moderate deviation (10-20%):** Reduce capture rate, monitor closely
- **Major deviation (>20%):** Initiate controlled shutdown

### 2.2 Adsorption Cycle Protocol

**Protocol ID:** DAC-CAPTURE-ADS-001
**Version:** 1.0

#### 2.2.1 Solid Sorbent DAC

**Duration:** 8-12 hours
**Objective:** Capture CO2 from ambient air onto sorbent material

**Procedure:**

1. **Initialization (0-15 min)**
   - Open air intake valves
   - Start fans to achieve target airflow (e.g., 5000 m³/min)
   - Monitor pressure drop across contactor (<500 Pa)
   - Begin data logging (1-minute intervals)

2. **Adsorption Phase (15 min - 12 hours)**
   - Maintain constant airflow through contactor
   - Monitor sorbent saturation via mass balance
   - Track CO2 concentration differential (inlet vs. outlet)
   - Record energy consumption
   - Log ambient conditions every 5 minutes

3. **Saturation Monitoring**
   - Calculate real-time capture efficiency:
     ```
     Efficiency = (CO2_in - CO2_out) / CO2_in
     ```
   - Target saturation: 80-90% of maximum capacity
   - Trigger regeneration when saturation reaches 85%

4. **End of Adsorption**
   - Gradually reduce airflow (ramp down over 10 min)
   - Close intake valves
   - Isolate contactor module
   - Record total CO2 captured

**Data Recording:**
```json
{
  "cycleId": "ads-20251225-001",
  "startTime": "2025-12-25T00:00:00Z",
  "endTime": "2025-12-25T10:00:00Z",
  "duration": 10.0,
  "airflowRate": 5000,
  "co2Captured": 0.5,
  "energyConsumed": 1.2,
  "efficiency": 0.92,
  "sorbentSaturation": 0.87
}
```

#### 2.2.2 Liquid Solvent DAC

**Duration:** 4-8 hours
**Objective:** Absorb CO2 into liquid solvent

**Procedure:**

1. **Contactor Operation**
   - Circulate solvent through packed column
   - Draw air through column (counter-current)
   - Maintain solvent temperature (15-30°C)
   - Monitor CO2 loading in solvent

2. **Rich Solvent Collection**
   - Pump CO2-rich solvent to regeneration unit
   - Measure solvent flow rate and CO2 concentration
   - Replenish with lean solvent
   - Continuous operation (no cycle interruption)

### 2.3 Regeneration Cycle Protocol

**Protocol ID:** DAC-CAPTURE-REG-001
**Version:** 1.0

#### 2.3.1 Solid Sorbent Regeneration

**Duration:** 2-4 hours
**Objective:** Release CO2 from sorbent using heat

**Procedure:**

1. **Heat-Up Phase (0-30 min)**
   - Close system to ambient air
   - Begin heating (electric or steam)
   - Ramp temperature: 25°C → 100°C (or design temp)
   - Rate: 2-5°C per minute
   - Monitor pressure buildup

2. **CO2 Release Phase (30 min - 3 hours)**
   - Maintain regeneration temperature (80-120°C)
   - Collect released CO2 stream
   - Monitor CO2 purity (target >95%)
   - Measure CO2 flow rate
   - Record total CO2 released

3. **Cool-Down Phase (3-4 hours)**
   - Stop heating
   - Optional active cooling (water or air)
   - Ramp down: 100°C → 40°C
   - Rate: 1-3°C per minute
   - Purge with dry air if needed

4. **Ready for Next Cycle**
   - Verify sorbent temperature <40°C
   - Check sorbent regeneration efficiency:
     ```
     Regen_Efficiency = CO2_released / CO2_captured
     ```
   - Target: >95% regeneration
   - Log cycle data

**Energy Optimization:**
- Use waste heat recovery where possible
- Schedule regeneration during low electricity prices
- Optimize for renewable energy availability

#### 2.3.2 Liquid Solvent Regeneration

**Duration:** Continuous
**Objective:** Strip CO2 from solvent using calcination

**Procedure:**

1. **Calcination**
   - Heat rich solvent to 900°C (for carbonate solvents)
   - Or lower temps for amine solvents (100-140°C)
   - Decompose carbonate → lime + CO2
   - Collect pure CO2 stream

2. **Solvent Regeneration**
   - Rehydrate lime with water
   - Produce fresh solvent
   - Recirculate to contactor
   - Continuous process flow

### 2.4 CO2 Collection Protocol

**Protocol ID:** DAC-CAPTURE-COL-001
**Version:** 1.0

#### 2.4.1 CO2 Stream Processing

**Objective:** Produce high-purity CO2 for storage

**Procedure:**

1. **Purity Verification**
   - Measure CO2 concentration (target >95%)
   - Detect contaminants:
     - Water vapor (<1000 ppm)
     - Nitrogen (<3%)
     - Oxygen (<1000 ppm)
   - If purity <95%, reject stream or reprocess

2. **Compression**
   - Compress CO2 to transport pressure
   - For pipeline: 85-150 bar
   - For storage: >73 bar (supercritical)
   - Monitor compression energy

3. **Dehydration**
   - Remove water vapor (freeze-out or desiccant)
   - Target: <50 ppm water
   - Prevents corrosion in pipelines

4. **Metering**
   - Accurate mass flow measurement (±2% uncertainty)
   - Continuous recording
   - Tamper-proof metering for verification

5. **Temporary Storage**
   - Buffer tank for batch transport
   - Or continuous pipeline injection
   - Record tank inventory

---

## 3. Storage Protocols

### 3.1 Geological Storage Protocol

**Protocol ID:** DAC-STORAGE-GEO-001
**Version:** 1.0

#### 3.1.1 Site Selection Criteria

**Geological Requirements:**
- Depth: 800-3000 meters
- Porosity: >10% for saline aquifers
- Permeability: 10-1000 millidarcies
- Caprock: Low permeability seal (<0.001 mD)
- Fault assessment: No active faults within 5 km

**Capacity Requirements:**
- Minimum 1 million tons CO2 storage capacity
- Injection rate capability: 50-500 tons/day

#### 3.1.2 Injection Protocol

**Procedure:**

1. **Pre-Injection**
   - Well integrity testing (pressure test)
   - Baseline monitoring (seismic, groundwater)
   - Inject tracer compounds for monitoring
   - Establish injection parameters

2. **Injection Operation**
   - Inject at controlled rate
   - Monitor bottom-hole pressure (must stay below fracture pressure)
   - Continuous flow rate measurement
   - Real-time data to MRV system

3. **Post-Injection**
   - Well shut-in and monitoring
   - Pressure falloff analysis
   - Verify no leakage to surface

**Monitoring Requirements:**
- Pressure: Continuous
- Seismic: Annual microseismic surveys
- Groundwater: Quarterly sampling
- Surface CO2: Continuous atmospheric monitoring
- Satellite InSAR: Annual surface deformation

#### 3.1.3 Long-Term Monitoring

**Duration:** Minimum 30 years post-injection

**Activities:**
- Pressure monitoring (first 10 years: monthly, after: annually)
- 3D seismic surveys (every 5 years)
- Groundwater sampling (annually)
- Well integrity inspections (every 2 years)
- Atmospheric CO2 monitoring (continuous for 10 years, then quarterly)

### 3.2 Mineralization Protocol

**Protocol ID:** DAC-STORAGE-MIN-001
**Version:** 1.0 (Based on CarbFix methodology)

#### 3.2.1 Site Requirements

**Geological Requirements:**
- Basaltic rock formations
- Reactive minerals (Ca, Mg, Fe)
- Sufficient porosity and permeability
- Depth: >500 meters (ideally >700m)

#### 3.2.2 Injection Protocol

**Procedure:**

1. **CO2 Dissolution**
   - Dissolve CO2 in water (CO2:water ratio ~1:25 by mass)
   - Creates carbonated water (pH ~3-4)
   - Lowers risk of buoyancy-driven leakage

2. **Injection**
   - Inject carbonated water into basalt formation
   - Typical depth: 700-2000 meters
   - Injection rate: 50-200 tons CO2/day
   - Monitor pH, conductivity, flow rate

3. **Mineralization Reaction**
   - CO2 reacts with Ca, Mg in basalt
   - Forms stable carbonate minerals (calcite, magnesite)
   - Timeline: >90% mineralized within 2 years
   - Permanent storage (>10,000 years)

**Chemical Reactions:**
```
CO2 + H2O → H2CO3 (carbonic acid)
H2CO3 + CaSiO3 → CaCO3 + SiO2 + H2O
H2CO3 + MgSiO3 → MgCO3 + SiO2 + H2O
```

#### 3.2.3 Monitoring

**During Injection:**
- Water chemistry (pH, conductivity, major ions)
- Injection pressure and temperature
- CO2 dissolution efficiency

**Post-Injection:**
- Core sampling (verify mineralization)
- Geochemical modeling
- Tracer tests
- Reduced monitoring after 5 years (mineralization confirmed)

### 3.3 Utilization Protocol

**Protocol ID:** DAC-STORAGE-UTI-001
**Version:** 1.0

#### 3.3.1 Acceptable Utilization Pathways

**Permanent Utilization (>100 years):**
- Concrete curing (carbonate mineralization)
- Building materials (CO2-derived aggregates)
- Polyurethane production (if stable)

**Long-Term Utilization (10-100 years):**
- Enhanced weathering
- Biochar with CO2 injection

**Short-Term Utilization (NOT acceptable for credits):**
- Food and beverage carbonation
- Enhanced oil recovery (unless with permanent storage)
- Synthetic fuels (re-emitted when burned)

#### 3.3.2 Verification Requirements

For utilization to qualify as permanent carbon removal:

1. **Life Cycle Assessment (LCA)**
   - Document all emissions from production
   - Net CO2 removal must be positive
   - Include transportation, energy use

2. **Permanence Proof**
   - Chemical analysis showing CO2 locked in product
   - Product lifetime estimation (>100 years)
   - End-of-life scenarios assessed

3. **Third-Party Verification**
   - Independent auditor confirms LCA
   - Product testing for CO2 content
   - Annual recertification

---

## 4. Safety Protocols

### 4.1 General Safety Protocol

**Protocol ID:** DAC-SAFETY-GEN-001
**Version:** 1.0

#### 4.1.1 Hazard Identification

**CO2 Hazards:**
- Asphyxiation risk in confined spaces (CO2 >5% by volume)
- Dry ice formation (frostbite risk)
- High pressure (>10 bar) - rupture risk

**Chemical Hazards:**
- Amine solvents (corrosive, toxic)
- Caustic solutions (KOH, NaOH)
- Regeneration heat (burn risk)

**Mechanical Hazards:**
- Rotating equipment (fans, compressors)
- High-voltage electrical systems
- Pressure vessels

#### 4.1.2 Personal Protective Equipment (PPE)

**Required PPE:**
- Safety glasses (always in operational areas)
- Gloves (chemical-resistant for solvent handling)
- Hard hat (in industrial zones)
- Safety shoes (steel-toed)
- High-visibility vest

**Additional PPE for Specific Tasks:**
- Respirator (for confined space entry or high CO2 areas)
- Face shield (for chemical handling)
- Insulated gloves (for high-temp equipment)
- Fall protection (for elevated work)

#### 4.1.3 Emergency Equipment

**Required On-Site:**
- CO2 detectors (alarm at >0.5% CO2)
- Oxygen monitors
- Emergency eyewash and showers
- Fire extinguishers (CO2, dry chemical)
- First aid kits
- Emergency communication system
- Spill containment materials

### 4.2 Emergency Shutdown Protocol

**Protocol ID:** DAC-SAFETY-ESD-001
**Version:** 1.0

#### 4.2.1 Emergency Shutdown Triggers

**Automatic Shutdown:**
- CO2 leak detected (>1000 ppm in operational areas)
- Pressure exceeds safe limits (>110% design pressure)
- Temperature exceeds limits
- Fire detection
- Power failure (unless backup available)
- Seismic event (>5.0 magnitude)

**Manual Shutdown:**
- Operator judgment (unsafe conditions)
- Equipment malfunction
- Personnel injury
- External emergency (evacuation order)

#### 4.2.2 Shutdown Procedure

**Immediate Actions (0-5 minutes):**
1. Press emergency stop button
2. Close all CO2 valves
3. Stop fans and compressors
4. Vent system to atmosphere (safe location)
5. Cut power to non-essential systems
6. Activate alarms and notify personnel

**Follow-Up Actions (5-30 minutes):**
1. Evacuate personnel if necessary
2. Assess situation (leak, fire, etc.)
3. Contact emergency services if needed
4. Secure area
5. Begin incident investigation
6. Document all actions

**Recovery (hours to days):**
1. Root cause analysis
2. Repair or replace faulty equipment
3. Safety review and approval to restart
4. Restart sequence (reverse of shutdown)

### 4.3 CO2 Leak Response Protocol

**Protocol ID:** DAC-SAFETY-LEAK-001
**Version:** 1.0

#### 4.3.1 Leak Detection

**Detection Methods:**
- Fixed CO2 sensors (continuous monitoring)
- Portable CO2 detectors (regular surveys)
- Visual inspection (frost formation, noise)
- Acoustic leak detection

**Alarm Levels:**
- **Low Alarm (500 ppm):** Investigate, log incident
- **High Alarm (5000 ppm / 0.5%):** Evacuate area, ventilate
- **Critical (10,000 ppm / 1%):** Emergency shutdown, full evacuation

#### 4.3.2 Response Procedure

**Small Leak (<1 kg/min):**
1. Identify leak location
2. Increase ventilation
3. Monitor CO2 levels
4. Isolate leaking section
5. Repair during next maintenance window

**Medium Leak (1-10 kg/min):**
1. Emergency notification
2. Evacuate immediate area
3. Shut down leaking section
4. Ventilate aggressively
5. Repair immediately

**Large Leak (>10 kg/min):**
1. Activate emergency shutdown
2. Full facility evacuation
3. Call emergency services
4. Approach only with SCBA (self-contained breathing apparatus)
5. Stop leak at source (close valves remotely if possible)
6. Notify regulatory authorities

---

## 5. Verification Protocols

### 5.1 Monitoring, Reporting, and Verification (MRV)

**Protocol ID:** DAC-MRV-001
**Version:** 1.0

#### 5.1.1 Monitoring Protocol

**Continuous Monitoring (Real-Time):**
- CO2 capture rate
- Energy consumption
- Ambient conditions
- Equipment status

**Daily Monitoring:**
- Sorbent condition
- System performance metrics
- Safety checks
- Data backup verification

**Monthly Monitoring:**
- Aggregated capture totals
- Energy efficiency analysis
- Sorbent degradation trends
- Equipment maintenance

**Annual Monitoring:**
- Third-party verification
- Comprehensive system audit
- Regulatory compliance review
- Long-term performance trends

#### 5.1.2 Reporting Protocol

**Internal Reporting (Daily):**
- Operations dashboard update
- Incident logs (if any)
- Production summary

**Public Reporting (Monthly):**
- Total CO2 captured (tons)
- Energy consumption (MWh)
- Operational uptime (%)
- Carbon intensity (MWh/ton CO2)
- Via WIA-ENE-050 compliant API

**Regulatory Reporting (Annually):**
- Comprehensive emissions/removals report
- Environmental impact assessment
- Safety incidents
- Compliance certification

**Verification Reporting (Annually):**
- Third-party audit report
- Carbon credit issuance request
- Detailed methodology documentation

#### 5.1.3 Verification Protocol

**Verification Steps:**

1. **Document Review**
   - Facility design and operational procedures
   - Data collection and QA/QC procedures
   - Historical performance data
   - Calibration records

2. **On-Site Inspection**
   - Equipment walk-through
   - Measurement device calibration check
   - Interview facility operators
   - Witness operational procedures

3. **Data Analysis**
   - Verify calculation methods
   - Cross-check data sources
   - Statistical analysis for anomalies
   - Uncertainty assessment

4. **Verification Statement**
   - Verified CO2 removal amount
   - Confidence level (e.g., 95%)
   - Uncertainty range (±X%)
   - Conformance with standards
   - Recommendations for improvement

5. **Certification Issuance**
   - Generate verifiable credential
   - Register with carbon credit registry
   - Publish public verification report

---

## 6. Data Management Protocols

### 6.1 Data Collection Protocol

**Protocol ID:** DAC-DATA-COL-001
**Version:** 1.0

**Requirements:**
- Automated data logging (no manual entry for primary data)
- Timestamp all measurements (UTC)
- Record at appropriate frequency (1-60 minute intervals)
- Immediate backup (redundant storage)
- Data validation (real-time range checks)

### 6.2 Data Quality Protocol

**Protocol ID:** DAC-DATA-QC-001
**Version:** 1.0

**Quality Control Checks:**

1. **Range Checks:** Values within expected physical limits
2. **Rate of Change:** Detect sensor malfunctions
3. **Cross-Validation:** Compare multiple sensors
4. **Mass Balance:** Energy and material balances close
5. **Calibration:** Regular sensor calibration (monthly to annually)

**Data Flags:**
- **Good Data:** Passed all QC checks
- **Suspect Data:** Failed one QC check, review needed
- **Bad Data:** Failed multiple checks, do not use
- **Missing Data:** Sensor malfunction or communication error

### 6.3 Data Security Protocol

**Protocol ID:** DAC-DATA-SEC-001
**Version:** 1.0

**Security Measures:**
- Encryption at rest (AES-256)
- Encryption in transit (TLS 1.3)
- Access control (role-based permissions)
- Audit logging (all data access logged)
- Blockchain anchoring (for verified data)
- Regular security audits

---

## 7. Maintenance Protocols

### 7.1 Preventive Maintenance Protocol

**Protocol ID:** DAC-MAINT-PREV-001
**Version:** 1.0

**Daily Maintenance:**
- Visual inspection of equipment
- Check for leaks (CO2, water, oil)
- Monitor vibration and noise

**Weekly Maintenance:**
- Filter replacements (air intake)
- Lubrication of moving parts
- Sensor accuracy checks

**Monthly Maintenance:**
- Detailed equipment inspection
- Calibration of key sensors
- Performance testing

**Quarterly Maintenance:**
- Sorbent sample analysis
- Heat exchanger cleaning
- Valve testing and repair

**Annual Maintenance:**
- Major overhaul (shutdown required)
- Sorbent replacement (if needed)
- Pressure vessel inspection
- Electrical system audit

### 7.2 Corrective Maintenance Protocol

**Protocol ID:** DAC-MAINT-CORR-001
**Version:** 1.0

**Procedure:**
1. **Fault Detection:** Automated alarms or operator report
2. **Assessment:** Diagnose root cause
3. **Isolation:** Shut down affected section
4. **Repair:** Fix or replace faulty component
5. **Testing:** Verify proper operation
6. **Documentation:** Log incident and corrective actions
7. **Restart:** Return to operation

**Priority Levels:**
- **Critical (P1):** Immediate safety risk → Stop operations
- **High (P2):** Significant performance impact → Repair within 24h
- **Medium (P3):** Minor impact → Repair within 1 week
- **Low (P4):** No impact → Repair at next maintenance window

---

## 8. Protocol Compliance

### 8.1 Compliance Checklist

Facilities must demonstrate compliance with:
- [ ] All capture operation protocols
- [ ] Storage protocols (appropriate to method)
- [ ] Safety protocols (general and specific)
- [ ] Verification protocols (MRV)
- [ ] Data management protocols
- [ ] Maintenance protocols

### 8.2 Audit Requirements

**Internal Audits:** Quarterly
**External Audits:** Annually (third-party)
**Regulatory Audits:** As required by jurisdiction

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / World Internet Alliance (WIA)


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
