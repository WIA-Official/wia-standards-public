# WIA-ENE-052: Seawater Desalination - PHASE 3 PROTOCOL STANDARDS

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Overview

### 1.1 Purpose

This document defines standardized protocols for water quality testing, facility certification, operational procedures, and compliance verification in seawater desalination systems. These protocols ensure consistent, reliable, and safe operation across all facilities following the WIA-ENE-052 standard.

### 1.2 Scope

Protocol standards cover:
- Water quality testing procedures
- Facility certification and auditing
- Operational protocols (startup, shutdown, emergency)
- Performance verification
- Compliance monitoring
- Incident reporting and management
- Continuous improvement processes

---

## 2. Water Quality Testing Protocol

### 2.1 Sampling Protocol

#### 2.1.1 Sampling Frequency

| Sample Location | Parameter Type | Frequency | Method |
|----------------|----------------|-----------|--------|
| Feed Water | Physical/Chemical | Every 4 hours | Grab sample |
| Feed Water | Microbiological | Daily | Composite (24h) |
| Permeate (Product) | Physical/Chemical | Every 2 hours | Continuous inline |
| Permeate (Product) | Microbiological | Every 8 hours | Grab sample |
| Concentrate (Brine) | Physical/Chemical | Every 8 hours | Grab sample |
| Distribution Network | All parameters | Weekly | Grab sample |

#### 2.1.2 Sampling Procedure

**Step 1: Preparation**
1. Calibrate all sampling equipment
2. Prepare clean, sterile sample containers
3. Label containers with:
   - Sample ID (unique identifier)
   - Date and time
   - Location
   - Sample type
   - Analyst name
4. Wear appropriate PPE (gloves, safety glasses)

**Step 2: Sample Collection**
1. **Grab Samples:**
   - Flush sampling point for 2-3 minutes
   - Rinse container 3 times with sample water
   - Fill container completely, leaving minimal headspace
   - Cap immediately to prevent contamination

2. **Composite Samples (24-hour):**
   - Use automatic sampler or manual time-proportional collection
   - Collect equal volumes at regular intervals
   - Store in refrigerated conditions (4°C)
   - Mix thoroughly before analysis

3. **Continuous Inline Monitoring:**
   - Verify sensor calibration
   - Record readings at specified intervals
   - Validate with grab samples weekly

**Step 3: Sample Handling**
1. Transport samples in cooler with ice packs (4°C)
2. Deliver to laboratory within 6 hours for microbiological analysis
3. Preserve samples as required (e.g., acidification for metals)
4. Document chain of custody

#### 2.1.3 Sample Preservation

| Parameter | Container | Preservation | Holding Time |
|-----------|-----------|--------------|--------------|
| pH | Plastic/Glass | None, analyze immediately | 15 minutes |
| TDS, Conductivity | Plastic | Cool to 4°C | 28 days |
| Chloride, Sulfate | Plastic | Cool to 4°C | 28 days |
| Microbiological | Sterile plastic | Cool to 4°C, no freezing | 8 hours |
| Metals | Acid-washed plastic | HNO₃ to pH < 2 | 6 months |
| Alkalinity | Plastic/Glass | Cool to 4°C | 14 days |

### 2.2 Analytical Methods

#### 2.2.1 Standard Test Methods

| Parameter | Method | Standard | Reporting Limit |
|-----------|--------|----------|-----------------|
| pH | Electrometric | ASTM D1293 | 0.1 pH units |
| TDS | Gravimetric | ASTM D5907 | 10 mg/L |
| Conductivity | Electrometric | ASTM D1125 | 1 µS/cm |
| Turbidity | Nephelometric | EPA 180.1 | 0.1 NTU |
| Chloride | Argentometric | ASTM D512 | 1 mg/L |
| Sodium | ICP-MS | EPA 200.8 | 0.5 mg/L |
| Calcium | ICP-MS | EPA 200.8 | 0.5 mg/L |
| Magnesium | ICP-MS | EPA 200.8 | 0.1 mg/L |
| Sulfate | Ion Chromatography | EPA 300.0 | 1 mg/L |
| Total Coliforms | Membrane Filtration | EPA 1103.1 | 1 CFU/100mL |
| E. coli | Membrane Filtration | EPA 1103.1 | 1 CFU/100mL |
| HPC | Pour Plate | APHA 9215B | 1 CFU/mL |

#### 2.2.2 Quality Control Requirements

**For each analytical batch:**
- Method blank: 1 per batch (< 10 samples)
- Laboratory Control Sample (LCS): 1 per batch
- Duplicate sample: 1 per 10 samples (minimum 1 per batch)
- Matrix Spike (MS): 1 per 10 samples for complex matrices
- Reference material: Monthly verification

**Acceptance Criteria:**
- Method blank: < MDL (Method Detection Limit)
- LCS recovery: 85-115%
- Duplicate RPD (Relative Percent Difference): < 20%
- Matrix Spike recovery: 75-125%

### 2.3 Water Quality Acceptance Criteria

#### 2.3.1 Product Water Standards (Permeate)

**WHO Drinking Water Guidelines Compliance:**

| Parameter | WHO Guideline | WIA-ENE-052 Target | Action Limit |
|-----------|---------------|---------------------|--------------|
| TDS | < 1,000 mg/L | < 300 mg/L | 500 mg/L |
| pH | 6.5 - 8.5 | 7.0 - 8.0 | < 6.5 or > 8.5 |
| Turbidity | < 5 NTU | < 0.5 NTU | 1 NTU |
| Chloride | < 250 mg/L | < 100 mg/L | 200 mg/L |
| Sodium | < 200 mg/L | < 75 mg/L | 150 mg/L |
| Total Coliforms | 0 CFU/100mL | 0 CFU/100mL | > 0 |
| E. coli | 0 CFU/100mL | 0 CFU/100mL | > 0 |

**Additional Parameters:**
- Hardness (as CaCO₃): 60-120 mg/L (remineralization target)
- Alkalinity (as CaCO₃): 50-100 mg/L (remineralization target)
- Free chlorine residual: 0.2-0.5 mg/L (disinfection)

#### 2.3.2 Test Protocol Workflow

```
┌─────────────────────────────────────────────────────────┐
│          WATER QUALITY TEST PROTOCOL WORKFLOW            │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  1. Sample Collection                                   │
│     ↓                                                    │
│  2. Chain of Custody Documentation                      │
│     ↓                                                    │
│  3. Sample Login & Tracking                             │
│     ↓                                                    │
│  4. Sample Preparation & Preservation                   │
│     ↓                                                    │
│  5. Analytical Testing (per standard methods)           │
│     ↓                                                    │
│  6. Quality Control Review                              │
│     ↓                                                    │
│  7. Data Validation                                     │
│     ↓                                                    │
│  8. Compliance Assessment                               │
│     ↓                                                    │
│  9. Certificate Generation (if compliant)               │
│     ↓                                                    │
│  10. Reporting & Archival                               │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

---

## 3. Facility Certification Protocol

### 3.1 Initial Certification Process

#### 3.1.1 Application Phase

1. **Submit Application:**
   - Facility information form
   - Design specifications
   - Equipment list with certifications
   - Process flow diagrams
   - Operating procedures manual
   - Quality control plan
   - Environmental impact assessment

2. **Document Review:**
   - Technical evaluation team assigned
   - Completeness check (5 business days)
   - Technical review (15 business days)
   - Request for additional information if needed

#### 3.1.2 Pre-Certification Audit

**Audit Scope:**
- Facility design compliance with WIA-ENE-052
- Equipment installation and commissioning
- Process control systems
- Laboratory capabilities
- Quality management system
- Personnel training and qualifications
- Safety procedures

**Audit Duration:**
- Small facilities (< 10,000 m³/day): 2 days
- Medium facilities (10,000-50,000 m³/day): 3 days
- Large facilities (> 50,000 m³/day): 5 days

**Audit Team:**
- Lead Auditor (certified WIA-ENE-052 auditor)
- Technical Specialist (desalination technology)
- Quality Assurance Specialist
- Environmental Specialist (if required)

#### 3.1.3 Performance Testing

**Test Duration:** Minimum 30 continuous days of operation

**Test Requirements:**
1. **Production Capacity:**
   - Achieve ≥ 95% of design capacity
   - Maintain stable operation

2. **Water Quality:**
   - Daily testing per Section 2.3
   - 100% compliance with standards
   - No microbiological detections

3. **Energy Efficiency:**
   - SEC ≤ design specification
   - Energy recovery system performance verification

4. **Recovery Rate:**
   - Achieve ≥ design recovery rate
   - Stable concentrate quality

5. **Availability:**
   - ≥ 95% uptime during test period
   - Emergency shutdown/startup procedures verified

#### 3.1.4 Certification Decision

**Certification Granted if:**
- All document review requirements met
- Audit findings satisfactorily closed
- Performance testing successful
- Quality management system operational

**Certificate Validity:** 3 years

**Certificate Includes:**
- Facility identification
- Certification scope (technology, capacity)
- Effective date and expiry date
- Certificate number (unique identifier)
- Digital signature and QR code for verification

### 3.2 Surveillance Audits

**Frequency:** Annual

**Scope:**
- Operational data review
- Water quality records
- Maintenance logs
- Personnel training records
- Any modifications or changes
- Complaint and incident reports

**Duration:** 1-2 days (based on facility size)

### 3.3 Recertification

**Timing:** 6 months before certificate expiry

**Requirements:**
- Updated facility information
- Performance data for previous 3 years
- Recertification audit (similar to initial audit, reduced scope)
- Addressing any non-conformances from surveillance audits

---

## 4. Operational Protocols

### 4.1 Standard Operating Procedures (SOPs)

#### 4.1.1 Normal Startup Procedure

**Prerequisites:**
- All pre-startup checks completed
- Feed water quality verified
- Chemical dosing systems primed
- Control systems operational

**Startup Sequence:**
1. Start feed water intake pumps (low flow)
2. Verify pre-treatment system operation
3. Start cartridge filtration
4. Gradually increase flow to operating setpoint
5. Start high-pressure pumps with soft start
6. Ramp pressure gradually to operating setpoint (over 15-30 min)
7. Monitor permeate quality continuously
8. Divert permeate to drain until quality meets specifications
9. Open permeate to product water tank
10. Verify energy recovery system operation
11. Record all parameters in startup log

**Acceptance Criteria:**
- Permeate TDS < 300 mg/L
- Feed pressure within ±5% of setpoint
- No alarms or abnormal vibrations
- Recovery rate within ±5% of design

#### 4.1.2 Normal Shutdown Procedure

**Shutdown Sequence:**
1. Record final operating parameters
2. Gradually reduce feed pressure over 10 minutes
3. Stop high-pressure pumps
4. Continue feed water flow for 5 minutes (membrane flush)
5. Stop feed water pumps
6. Close isolation valves
7. If shutdown > 3 days, perform preservation flush
8. Record all parameters in shutdown log

**Long-Term Shutdown (> 7 days):**
- Perform membrane preservation
- Use sodium bisulfite solution (1,000-2,000 ppm) or formaldehyde (1%)
- Circulate preservation solution for 30 minutes
- Seal system and maintain positive pressure
- Flush every 7 days for shutdowns > 30 days

#### 4.1.3 Emergency Shutdown Procedure

**Automatic Shutdown Triggers:**
- Feed pressure > 110% of setpoint
- Permeate conductivity > 500 µS/cm
- Membrane differential pressure > critical value
- Cartridge filter differential pressure > critical value
- Fire or gas detection
- Power failure

**Emergency Actions:**
1. Automatic isolation of high-pressure system
2. High-pressure pump emergency stop
3. Permeate diversion to drain
4. Alarm activation
5. Operator notification

**Recovery Procedure:**
1. Identify and resolve root cause
2. Inspect system for damage
3. Perform quality check flush
4. Resume normal startup procedure
5. Document incident and corrective actions

### 4.2 Membrane Cleaning Protocol

#### 4.2.1 Cleaning Triggers

**Performance-Based Triggers:**
- Normalized permeate flow decrease > 10%
- Normalized salt passage increase > 10%
- Differential pressure increase > 15%
- Feed-concentrate pressure drop increase > 15%

**Time-Based Trigger:**
- Minimum: Every 6 months (preventive)
- Typical: Every 3-4 months under normal conditions

#### 4.2.2 Cleaning Procedure

**Step 1: Preparation**
1. Prepare cleaning solution (per manufacturer recommendation)
2. Heat to specified temperature (typically 30-35°C)
3. Check cleaning tank level and circulation pump
4. Verify all valves and connections

**Step 2: Low pH Cleaning (Inorganic Fouling)**
- Solution: Citric acid 2% (pH 3-4)
- Temperature: 30°C
- Flow: 40-50% of design flow
- Recirculation: 30-60 minutes
- Soak: 1-2 hours
- Rinse thoroughly with feed water

**Step 3: High pH Cleaning (Organic/Biological Fouling)**
- Solution: NaOH 0.1% + EDTA 0.5% + SDS 0.03% (pH 11-12)
- Temperature: 30°C
- Flow: 40-50% of design flow
- Recirculation: 30-60 minutes
- Soak: 1-2 hours
- Rinse thoroughly with feed water

**Step 4: Verification**
1. Resume normal operation
2. Monitor performance parameters
3. Compare with baseline data
4. Document cleaning effectiveness
5. Update maintenance records

**Cleaning Effectiveness Criteria:**
- Permeate flow recovery: > 90% of baseline
- Salt rejection: Return to within 5% of baseline
- Pressure drop: Return to within 10% of baseline

---

## 5. Performance Verification Protocol

### 5.1 Daily Performance Checks

**Operator Checklist:**
- [ ] Record feed flow, permeate flow, concentrate flow
- [ ] Record feed pressure, permeate pressure, concentrate pressure
- [ ] Record feed TDS, permeate TDS (online)
- [ ] Record energy consumption
- [ ] Visual inspection for leaks, unusual noises, vibrations
- [ ] Review alarms and events
- [ ] Check chemical dosing systems
- [ ] Verify backup power system status

### 5.2 Weekly Performance Analysis

**Process Engineer Review:**
- Calculate normalized permeate flow
- Calculate normalized salt passage
- Calculate specific energy consumption (SEC)
- Calculate recovery rate
- Trend analysis of key parameters
- Identify any performance degradation
- Schedule maintenance if needed

**Normalization Formulas:**

```
Normalized Permeate Flow =
    Actual Permeate Flow × (TCF × 1.03^(T-25))

Where:
- TCF = Temperature Correction Factor
- T = Feed water temperature (°C)

Normalized Salt Passage (%) =
    (Permeate TDS / Feed TDS) × 100 × (1.03^(25-T))
```

### 5.3 Monthly Performance Report

**Required Content:**
1. Production summary (total volume, average daily)
2. Water quality summary (compliance %, violations if any)
3. Energy consumption and SEC
4. Availability and downtime analysis
5. Maintenance activities performed
6. Membrane performance trends
7. Chemical consumption
8. Brine discharge volume and quality
9. Any incidents or non-conformances
10. Continuous improvement initiatives

---

## 6. Compliance Monitoring Protocol

### 6.1 Self-Monitoring Requirements

**Daily:**
- Permeate flow rate
- Permeate conductivity/TDS (continuous online)
- Free chlorine residual (if applicable)
- Energy consumption

**Weekly:**
- Comprehensive water quality analysis (per Section 2.3)
- Membrane performance parameters
- Chemical inventory and consumption

**Monthly:**
- Extended water quality parameters
- Trace contaminants (if applicable)
- Brine discharge monitoring
- Energy audit

**Quarterly:**
- Third-party laboratory verification samples
- Membrane autopsy (if performance issues)
- Environmental monitoring (marine ecosystem)

### 6.2 Regulatory Reporting

**Monthly Reports to Regulatory Authority:**
- Production volume
- Water quality summary
- Compliance status
- Any violations or exceedances
- Corrective actions taken

**Annual Reports:**
- Comprehensive performance review
- Environmental impact assessment
- Energy efficiency improvements
- Future plans and upgrades

### 6.3 Non-Conformance Management

#### 6.3.1 Non-Conformance Categories

**Minor Non-Conformance:**
- Single isolated water quality parameter exceedance (not microbiological)
- Documentation errors
- Missed sampling (with valid reason)

**Major Non-Conformance:**
- Repeated parameter exceedances
- Microbiological detection in product water
- Significant equipment failure affecting water safety
- Failure to implement corrective actions

**Critical Non-Conformance:**
- Distribution of unsafe water
- Deliberate falsification of data
- Major environmental incident

#### 6.3.2 Corrective Action Protocol

1. **Immediate Actions:**
   - Isolate affected water (do not distribute)
   - Identify root cause
   - Implement containment measures
   - Notify regulatory authority (if required)

2. **Investigation:**
   - Root cause analysis (5 Whys, Fishbone diagram)
   - Review procedures and controls
   - Identify systemic issues

3. **Corrective Actions:**
   - Address immediate cause
   - Implement preventive measures
   - Update procedures if needed
   - Train personnel

4. **Verification:**
   - Verify effectiveness of corrective actions
   - Resume normal operation
   - Enhanced monitoring period (minimum 1 week)

5. **Documentation:**
   - Non-conformance report
   - Investigation findings
   - Corrective action plan
   - Verification results
   - Lessons learned

---

## 7. Incident Reporting Protocol

### 7.1 Reportable Incidents

**Immediate Reporting Required (within 2 hours):**
- Water quality violation affecting public health
- Major equipment failure causing complete shutdown
- Environmental spill or discharge violation
- Personnel injury
- Fire or explosion

**24-Hour Reporting Required:**
- Persistent water quality parameter exceedance
- Significant production reduction (> 50%)
- Security breach or vandalism

**7-Day Reporting Required:**
- Minor equipment malfunctions
- Performance degradation trends
- Supplier quality issues

### 7.2 Incident Report Format

**Report Contents:**
1. Incident identification
   - Date, time, location
   - Type and severity
2. Description
   - What happened
   - Duration and extent
3. Impact assessment
   - Production impact
   - Water quality impact
   - Environmental impact
   - Public health risk
4. Immediate response
   - Actions taken
   - Personnel involved
5. Root cause analysis
6. Corrective and preventive actions
7. Follow-up and verification

---

## 8. Continuous Improvement Protocol

### 8.1 Performance Benchmarking

**Internal Benchmarking:**
- Compare current performance to historical baseline
- Track improvement trends
- Set annual improvement targets

**External Benchmarking:**
- Compare with industry averages (IDA reports)
- Participate in WIA-ENE-052 facility network
- Share best practices (anonymized data)

**Key Performance Indicators (KPIs):**
- Specific Energy Consumption (kWh/m³)
- Water Recovery Rate (%)
- Product Water Quality (TDS, etc.)
- Availability (%)
- Maintenance Cost per m³
- Carbon Footprint (kg CO₂/m³)

### 8.2 Innovation and Technology Upgrades

**Annual Technology Review:**
- Evaluate new membrane technologies
- Assess energy recovery improvements
- Consider renewable energy integration
- Explore automation and AI optimization

**Pilot Testing Protocol:**
- Define objectives and success criteria
- Implement parallel testing (small scale)
- Monitor performance for minimum 3 months
- Economic analysis (ROI, payback period)
- Scale-up plan if successful

---

## 9. Audit Protocol

### 9.1 Internal Audits

**Frequency:** Semi-annual (every 6 months)

**Scope:**
- Compliance with WIA-ENE-052 standard
- SOPs adherence
- Record keeping and documentation
- Personnel training and competency
- Equipment calibration and maintenance

**Audit Team:**
- Internal auditor (trained in WIA-ENE-052)
- Process specialist
- QA/QC specialist

**Deliverables:**
- Audit report
- Findings and observations
- Corrective action plan
- Follow-up schedule

### 9.2 External Audits

**Types:**
- Certification audits (initial, surveillance, recertification)
- Regulatory compliance audits
- Customer audits (for industrial clients)

**Preparation:**
- Document review and readiness check
- Mock audit (optional but recommended)
- Ensure all records are current and accessible
- Brief all personnel

---

## 10. Verifiable Credentials (VC) Protocol

### 10.1 Digital Certificate Issuance

**Certificate Types:**
- Facility Certification
- Water Quality Certification (batch-specific)
- Performance Compliance Certificate (annual)

**Blockchain Integration:**
- Each certificate issued as Verifiable Credential
- Anchored to blockchain for immutability
- Publicly verifiable via QR code or URL

### 10.2 Certificate Verification Process

**Public Verification:**
1. Scan QR code on water quality certificate
2. System retrieves VC from blockchain
3. Verify digital signature
4. Display certificate details and validity status
5. Option to download detailed test report

**API Verification:**
```
GET /api/v1/verify/certificate/{certificateId}
Response:
{
  "valid": true,
  "certificateType": "WaterQualityCertificate",
  "facilityId": "DESAL-ROE-UAE-001",
  "issuedDate": "2025-12-25",
  "expiryDate": "2026-01-25",
  "issuer": "WIA Water Quality Institute",
  "blockchainAnchor": "0x1234...abcd",
  "status": "ACTIVE"
}
```

---

**Document Control:**
- **Version:** 1.0
- **Author:** WIA Standards Committee
- **Approved By:** WIA Technical Board
- **Next Review:** 2026-12-25

**License:** CC BY-SA 4.0
**Copyright:** © 2025 SmileStory Inc. / WIA

弘益人間 (홍익인간) - Benefit All Humanity
