# WIA-BIO-019: Bio-Banking Specification v1.0

> **Standard ID:** WIA-BIO-019
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sample Types and Classification](#2-sample-types-and-classification)
3. [Collection Protocols](#3-collection-protocols)
4. [Storage Systems](#4-storage-systems)
5. [Quality Management](#5-quality-management)
6. [Sample Integrity Metrics](#6-sample-integrity-metrics)
7. [Chain of Custody](#7-chain-of-custody)
8. [Consent and Governance](#8-consent-and-governance)
9. [LIMS Integration](#9-lims-integration)
10. [Safety and Compliance](#10-safety-and-compliance)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for biological sample banking, ensuring quality, traceability, and regulatory compliance for biospecimens used in research, diagnostics, and therapeutic development.

### 1.2 Scope

The standard covers:
- Sample collection, processing, and storage
- Quality management systems (ISO 20387:2018)
- Sample integrity assessment
- Chain of custody tracking
- Consent management and ethics
- LIMS (Laboratory Information Management System) integration

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Bio-banking serves humanity by preserving biological materials that enable medical breakthroughs, advance scientific understanding, and improve healthcare outcomes for all people.

### 1.4 Terminology

- **Biospecimen**: Any biological material collected from humans or animals
- **Aliquot**: A portion of a sample separated for testing or storage
- **LIMS**: Laboratory Information Management System
- **Chain of Custody**: Documentation of sample handling from collection to disposal
- **Pre-analytical Variables**: Factors affecting sample quality before analysis
- **Cryopreservation**: Preservation at ultra-low temperatures (-80°C to -196°C)

---

## 2. Sample Types and Classification

### 2.1 Blood Products

#### 2.1.1 Whole Blood
```
Collection: EDTA or citrate tubes
Volume: 2-10 mL typical
Storage: -80°C
Duration: Up to 10 years
Quality Metric: Hemolysis index < 50
```

#### 2.1.2 Plasma
```
Processing: Centrifuge at 1,500g × 10 min within 2 hours
Storage: -80°C in cryovials
Aliquoting: 0.5-1.0 mL portions
Quality: Total protein 60-80 g/L
```

#### 2.1.3 Serum
```
Clotting: 30-60 minutes at room temperature
Centrifuge: 1,500g × 10 min
Storage: -80°C
Stability: >20 years for most analytes
```

#### 2.1.4 Buffy Coat (White Blood Cells)
```
Source: Interface layer after centrifugation
Storage: -80°C or liquid nitrogen
DNA Extraction: Suitable for genomic studies
Cell Count: 1-5 × 10⁶ cells/mL
```

### 2.2 Nucleic Acids

#### 2.2.1 DNA
```
Extraction: Column-based or organic methods
Concentration: 50-200 ng/μL optimal
Storage: -20°C to -80°C
Quality: A260/A280 ratio 1.8-2.0
Integrity: No fragmentation on gel
Stability: 50+ years at -20°C
```

#### 2.2.2 RNA
```
Extraction: TRIzol or column-based
Storage: -80°C mandatory
Quality: RIN score ≥ 7.0 for research
Stability: 5 years at -80°C
Special: RNase-free techniques essential
```

### 2.3 Tissue Samples

#### 2.3.1 Fresh Frozen Tissue
```
Collection: Surgical specimens
Snap Freezing: Liquid nitrogen within 30 min
Storage: -80°C or liquid nitrogen vapor phase
Size: 5-10 mm³ pieces
Quality: Cell viability >80% post-thaw
```

#### 2.3.2 FFPE (Formalin-Fixed Paraffin-Embedded)
```
Fixation: 10% neutral buffered formalin, 6-24 hours
Processing: Dehydration, clearing, infiltration
Storage: Room temperature, dark, dry
Duration: Decades (DNA/RNA quality degrades)
Applications: Histology, IHC, some molecular tests
```

### 2.4 Living Cells

#### 2.4.1 Peripheral Blood Mononuclear Cells (PBMCs)
```
Isolation: Density gradient centrifugation (Ficoll)
Freezing: 10% DMSO in FBS or albumin
Cooling: 1°C/min to -80°C
Storage: Liquid nitrogen (-196°C)
Viability: >90% post-thaw when properly frozen
```

#### 2.4.2 Cell Lines
```
Passage: Regular subculture
Freezing: Early passage preferred
Cryoprotectant: 10% DMSO or glycerol
Storage: Liquid nitrogen
Testing: Mycoplasma, sterility, authentication
```

### 2.5 Other Biospecimens

#### 2.5.1 Urine
```
Collection: Clean catch, midstream
Additive: Optional protease inhibitors
Storage: -80°C for metabolomics
Volume: 10-50 mL aliquots
Stability: 5 years for most metabolites
```

#### 2.5.2 Saliva
```
Collection: Passive drool or swab
Volume: 2-5 mL
DNA Yield: 1-10 μg per sample
Storage: -20°C to -80°C
Applications: DNA extraction, microbiome
```

#### 2.5.3 Stool
```
Collection: Sterile container
Stabilizers: DNA/RNA Shield or equivalent
Storage: -80°C
Applications: Microbiome, metabolomics
Volume: 200-500 mg aliquots
```

---

## 3. Collection Protocols

### 3.1 Pre-Collection Requirements

#### 3.1.1 Informed Consent
```
Required Elements:
- Purpose of collection and storage
- Duration of storage
- Intended uses (research, diagnostics, etc.)
- Right to withdraw
- Privacy protection measures
- Contact information

Documentation: Electronic or paper, properly archived
Audit Trail: All consent actions logged
```

#### 3.1.2 Donor Information
```
Minimum Data:
- Unique donor ID (de-identified)
- Age, sex, ethnicity
- Collection date and time
- Fasting status (if applicable)
- Medications
- Relevant medical history

Privacy: GDPR/HIPAA compliant storage
```

### 3.2 Collection Procedures

#### 3.2.1 Blood Collection
```
Procedure:
1. Verify donor identity and consent
2. Select appropriate tubes for intended tests
3. Follow order of draw (sterile → coagulation → serum → heparin → EDTA)
4. Label tubes immediately with donor ID, date, time
5. Mix anticoagulant tubes by gentle inversion (8-10×)
6. Record collection time and conditions

Time to Processing:
- Plasma/Serum: Within 2 hours
- Whole blood: Within 4 hours
- PBMCs: Within 8 hours
```

#### 3.2.2 Tissue Collection
```
Procedure:
1. Collect during surgery or biopsy
2. Place in sterile container
3. Keep on ice if processing delayed
4. Snap freeze in liquid nitrogen within 30 minutes
   OR fix in formalin within 1 hour
5. Document anatomical site, pathology
6. Photograph if needed for research

Size: 5-10 mm³ pieces for freezing
Fixation Ratio: 1:10 tissue:formalin volume
```

### 3.3 Labeling Standards

```
Barcode Format: Code128 or DataMatrix
Label Information:
- Sample ID (globally unique)
- Collection date (YYYY-MM-DD)
- Sample type
- Storage condition
- Biohazard status

Label Durability:
- Cryogenic-resistant (-196°C)
- Solvent-resistant
- Permanent adhesive
```

---

## 4. Storage Systems

### 4.1 Liquid Nitrogen Storage

#### 4.1.1 Specifications
```
Temperature: -196°C (liquid phase)
           or -150°C to -190°C (vapor phase)
Capacity: 100 - 50,000 samples per tank
Advantages:
- Ultra-low temperature
- Indefinite storage for cells
- No electrical dependency (if properly maintained)

Disadvantages:
- High cost
- LN₂ replenishment required
- Cross-contamination risk (liquid phase)
- Safety hazards

Recommended For:
- Living cells (PBMCs, stem cells)
- Long-term tissue storage
- Irreplaceable samples
```

#### 4.1.2 Safety Measures
```
Requirements:
- Oxygen monitors in storage rooms
- Proper ventilation
- PPE: Cryogenic gloves, face shields
- Alarms for low LN₂ levels
- Emergency fill procedures
- Staff training on cryogenic hazards
```

### 4.2 Ultra-Low Temperature Freezers (-80°C)

#### 4.2.1 Specifications
```
Temperature: -80°C ± 5°C
Capacity: 300 - 500 2-inch boxes
Power: 6-15 kW depending on size
Backup: LN₂ or CO₂ backup systems
Monitoring: 24/7 temperature logging

Advantages:
- No LN₂ refilling
- Easier sample access
- Lower operational cost than LN₂

Disadvantages:
- Power dependency
- Degradation faster than LN₂
- 10-20 year lifespan

Recommended For:
- DNA, plasma, serum
- RNA (short to medium term)
- Tissue (up to 10 years)
```

#### 4.2.2 Maintenance
```
Frost Removal: Annual defrost or frost-free models
Filter Cleaning: Monthly
Temperature Calibration: Quarterly
Compressor Service: Annual
Backup Testing: Monthly

Alarm Systems:
- High/Low temperature
- Power failure
- Door ajar
- Remote monitoring via network
```

### 4.3 Standard Freezers (-20°C)

```
Applications:
- DNA (long-term)
- Some enzymes and proteins
- Short-term storage

Limitations:
- Not suitable for RNA
- Protein degradation over time
- Frost-free models cause freeze-thaw cycles

Best Practice: Manual defrost models preferred
```

### 4.4 Refrigerated Storage (4°C)

```
Applications:
- Short-term storage (<72 hours)
- Some fixed samples
- Transport containers

Duration: Days to weeks maximum
Monitoring: Continuous temperature logging
```

### 4.5 Ambient Storage

```
Applications:
- FFPE blocks
- Dried blood spots
- Some fixed specimens

Conditions:
- Temperature: 15-25°C
- Humidity: <60% RH
- Dark storage
- Dust-free environment
```

---

## 5. Quality Management

### 5.1 ISO 20387:2018 Compliance

#### 5.1.1 Requirements
```
Key Elements:
1. Quality management system
2. Structural and governance requirements
3. Resource management (personnel, facilities, equipment)
4. Pre-examination, examination, post-examination processes
5. Quality assurance
6. Ethical principles and sample management

Documentation:
- Standard Operating Procedures (SOPs)
- Quality manuals
- Training records
- Audit reports
```

#### 5.1.2 Internal Audits
```
Frequency: Annual minimum
Scope: All biobanking processes
Checklist:
- Sample collection procedures
- Storage temperature logs
- Equipment calibration
- Staff training
- Consent documentation
- Inventory accuracy

Corrective Actions: Documented and tracked to closure
```

### 5.2 Standard Operating Procedures (SOPs)

```
Required SOPs:
1. Sample collection for each specimen type
2. Processing and aliquoting
3. Cryopreservation
4. Storage and retrieval
5. Equipment maintenance
6. Quality control
7. Data management
8. Consent procedures
9. Sample destruction
10. Emergency response (freezer failure, power outage)

SOP Format:
- Version control
- Approval signatures
- Review cycle (annual)
- Change history
```

### 5.3 Quality Control Samples

```
Types:
- Blank samples (negative controls)
- Pooled samples (positive controls)
- Duplicate samples (reproducibility)
- Spiked samples (recovery testing)

Frequency:
- Every batch (collection or processing)
- Monthly for stored samples
- After equipment maintenance

Metrics:
- Coefficient of variation (CV) < 15%
- Recovery 85-115%
- Contamination: None detected
```

### 5.4 Equipment Calibration and Maintenance

```
Freezers and LN₂ Tanks:
- Temperature verification: Daily (automated)
- Calibration: Quarterly (certified thermometer)
- Maintenance: Annual service

Centrifuges:
- Speed verification: Quarterly
- Timer check: Quarterly
- Service: Annual

Pipettes:
- Gravimetric calibration: Annual
- Accuracy check: Monthly

Water Baths:
- Temperature: Daily
- Calibration: Quarterly
```

---

## 6. Sample Integrity Metrics

### 6.1 Sample Integrity Score (SIS)

#### 6.1.1 Formula
```
SIS = (Vt / V0) × (1 - ΔT/Tmax) × (1 - Δt/tmax)
```

Where:
- `SIS` = Sample Integrity Score (0-1)
- `Vt` = Current viability or quality metric
- `V0` = Initial viability or quality metric
- `ΔT` = Temperature deviation from optimal (°C)
- `Tmax` = Maximum acceptable temperature deviation (°C)
- `Δt` = Time since collection (days)
- `tmax` = Maximum recommended storage duration (days)

#### 6.1.2 Interpretation
```
SIS > 0.9: Excellent quality
SIS 0.8-0.9: Good quality
SIS 0.7-0.8: Acceptable quality
SIS 0.6-0.7: Marginal quality
SIS < 0.6: Poor quality (consider discarding)
```

#### 6.1.3 Example Calculation
```
Given:
- DNA sample
- Initial quality (V0): A260/A280 = 1.85
- Current quality (Vt): A260/A280 = 1.82
- Temperature deviation (ΔT): 3°C (from -20°C to -17°C during event)
- Max acceptable deviation (Tmax): 10°C
- Storage time (Δt): 1,825 days (5 years)
- Max storage time (tmax): 18,250 days (50 years)

Calculation:
SIS = (1.82/1.85) × (1 - 3/10) × (1 - 1825/18250)
SIS = 0.984 × 0.7 × 0.9
SIS = 0.62

Interpretation: Marginal quality
```

### 6.2 Storage Quality Index (SQI)

#### 6.2.1 Arrhenius Equation Application
```
SQI = exp(-k × t × exp(Ea/RT))
```

Where:
- `SQI` = Storage Quality Index (1 at t=0, decreases over time)
- `k` = Degradation rate constant (sample-specific)
- `t` = Storage time (years)
- `Ea` = Activation energy (kJ/mol)
- `R` = Gas constant (8.314 J/mol·K)
- `T` = Storage temperature (Kelvin)

#### 6.2.2 Sample-Specific Parameters

| Sample Type | k (1/year) | Ea (kJ/mol) | Half-life at -80°C |
|-------------|------------|-------------|---------------------|
| DNA | 0.001 | 80 | >100 years |
| RNA | 0.05 | 60 | 14 years |
| Proteins | 0.02 | 70 | 35 years |
| Cells (frozen) | 0.005 | 90 | 140 years |

### 6.3 Quality Metrics by Sample Type

#### 6.3.1 DNA Quality
```
Metrics:
- A260/A280 ratio: 1.8-2.0 (purity)
- A260/A230 ratio: 2.0-2.2 (organic contamination)
- Concentration: Spectrophotometry or fluorometry
- Integrity: Gel electrophoresis (no smearing)
- Fragment size: >10 kb for high-quality genomic DNA

Advanced:
- qPCR amplification of long fragments
- DNA Integrity Number (DIN) on bioanalyzer
```

#### 6.3.2 RNA Quality
```
Metrics:
- RIN (RNA Integrity Number): 1-10 scale
  - RIN ≥ 8: Excellent
  - RIN 7-8: Good
  - RIN 5-7: Acceptable for some applications
  - RIN < 5: Degraded

- 28S/18S ratio: ~2.0 (gel electrophoresis)
- A260/A280: 1.9-2.1

Applications:
- RNA-seq: RIN ≥ 7
- qRT-PCR: RIN ≥ 6 acceptable
- Microarray: RIN ≥ 7
```

#### 6.3.3 Protein Quality
```
Metrics:
- Total protein concentration (BCA or Bradford assay)
- Electrophoresis: No degradation bands
- Western blot: Specific bands intact
- Functional assays: Activity preserved

Stability Factors:
- Protease inhibitors added
- Storage at -80°C
- Avoid freeze-thaw cycles (max 3×)
```

#### 6.3.4 Cell Viability
```
Metrics:
- Trypan blue exclusion: >80% viable
- Flow cytometry (7-AAD or PI staining): >85% viable
- Metabolic assays (MTT, ATP): >80% of control

Post-Thaw Quality:
- Recovery: >70% of pre-freeze cell count
- Viability: >80%
- Functionality: Retained (proliferation, differentiation)

Cryoprotectant Removal:
- Gradual dilution (10× volume)
- Centrifuge at 300g × 5 min
- Resuspend in fresh medium
```

---

## 7. Chain of Custody

### 7.1 Custody Tracking System

#### 7.1.1 Hash Chain Implementation
```
COC = H(S0) → H(S1) → H(S2) → ... → H(Sn)
```

Where each state Sn includes:
```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "action": "collection",
  "operator": "TECH-001",
  "location": "CLINIC-A",
  "sample_id": "SAM-123456",
  "condition": {
    "temperature": 22,
    "volume": 10.0,
    "tube_type": "EDTA"
  },
  "previous_hash": "a4b2c1d3e4f5..."
}
```

Hash: SHA-256(JSON.stringify(state))

#### 7.1.2 Custody Events
```
Required Tracking Points:
1. Collection (donor → collector)
2. Transport (clinic → biobank)
3. Receipt (biobank receives)
4. Processing (aliquoting, centrifugation)
5. Storage (placement in freezer)
6. Retrieval (removal for testing)
7. Return (back to storage or disposal)
8. Transfer (to another institution)
9. Destruction (final disposal)

Each Event Records:
- Timestamp (ISO 8601)
- Operator ID
- Location
- Temperature (if applicable)
- Sample condition
- Digital signature or hash
```

### 7.2 Barcode and RFID Tracking

```
Barcode System:
- 1D: Code128 for simple tracking
- 2D: DataMatrix for high-density information
- Scanner integration with LIMS

RFID Tags (Advanced):
- Passive UHF tags for freezer boxes
- Active tags with temperature sensors
- Real-time location tracking
- Bulk reading capability

Benefits:
- Reduced manual errors
- Faster inventory
- Automated chain of custody
```

### 7.3 Temperature Monitoring

```
Continuous Monitoring:
- Data loggers with 1-minute intervals
- Cloud-based monitoring systems
- Alarm thresholds:
  - -80°C freezers: Alert if > -75°C
  - LN₂ tanks: Alert if > -150°C
  - Refrigerators: Alert if > 8°C

Excursion Management:
1. Alarm triggered
2. Automatic notification (email, SMS, phone)
3. Staff response logged
4. Sample integrity assessment
5. Decision: Keep, flag, or discard
6. Documentation of incident and resolution

Temperature Mapping:
- Identify cold/warm spots in freezers
- Quarterly mapping for critical equipment
- Place most valuable samples in stable zones
```

---

## 8. Consent and Governance

### 8.1 Informed Consent Framework

#### 8.1.1 Consent Types
```
Broad Consent:
- Allow future unspecified research
- Common in biobanks
- Requires ethics approval for each project
- Donor can withdraw at any time

Specific Consent:
- Limited to stated research purpose
- More restrictive
- Donors must re-consent for new uses

Tiered Consent:
- Donor chooses level of participation
- Example tiers:
  1. This study only
  2. Related research
  3. Any health research
  4. Any research (including commercial)
```

#### 8.1.2 Consent Elements
```
Required Information:
- Purpose of biobank
- Sample types collected
- Storage duration
- Potential uses (research, diagnostics, commercial)
- Risks and benefits
- Privacy protections
- Right to withdraw
- Contact information
- Sharing with other researchers
- Commercialization potential
- Return of results (or not)

Documentation:
- Signed consent form
- Electronic capture with audit trail
- Version control (consent form updates)
- Language accessibility
```

### 8.2 Ethical Governance

#### 8.2.1 Ethics Review
```
Required Approvals:
- Institutional Review Board (IRB) or Ethics Committee
- Annual renewals
- Amendment approval for major changes

Considerations:
- Vulnerable populations
- Minors (parental consent)
- Genetic information sensitivity
- Cultural and religious factors
- International transfers
```

#### 8.2.2 Access Governance
```
Access Committee:
- Review sample requests
- Evaluate scientific merit
- Ensure consent compatibility
- Approve Material Transfer Agreements (MTAs)

Decision Criteria:
- Scientific quality of proposal
- Feasibility
- Ethical approval of requesting project
- Alignment with donor consent
- Sample availability
```

### 8.3 Privacy and Data Protection

#### 8.3.1 De-identification
```
Levels:
1. Anonymized: No link to donor identity (irreversible)
2. Coded: Linked via code, key held separately
3. Identifiable: Direct identifiers present

Best Practice: Coded samples with strict key access control

Coding System:
- Random alphanumeric (SAM-XXXXXX)
- No embedded information (date, sequence)
- Separate database for donor-sample linkage
```

#### 8.3.2 GDPR Compliance (EU)
```
Requirements:
- Lawful basis for processing (consent or public interest)
- Data minimization
- Storage limitation
- Security measures
- Data subject rights:
  - Access
  - Rectification
  - Erasure ("right to be forgotten")
  - Restriction of processing
  - Data portability

Special Category Data:
- Genetic data
- Health data
Requires explicit consent or substantial public interest
```

#### 8.3.3 HIPAA Compliance (USA)
```
Protected Health Information (PHI):
- Name, address, dates, phone, email, SSN, etc.
- Must be removed or coded for research use

Safe Harbor Method:
- Remove 18 identifiers
- No actual knowledge residual info can identify

Limited Data Set:
- Can retain dates, geographic region
- Requires Data Use Agreement
```

---

## 9. LIMS Integration

### 9.1 Laboratory Information Management System

#### 9.1.1 Core Functions
```
Sample Tracking:
- Unique ID assignment
- Barcode generation
- Location tracking (freezer, rack, box, position)
- Aliquot genealogy

Inventory Management:
- Real-time availability
- Low stock alerts
- Expiration tracking
- Quality flags

Data Management:
- Donor demographics (de-identified)
- Collection metadata
- Processing parameters
- Quality metrics
- Chain of custody

Workflow Management:
- Request processing
- Sample retrieval
- Shipment tracking
- Report generation
```

#### 9.9.2 API Specifications

##### Register Sample
```typescript
POST /api/v1/samples

Request:
{
  "sample_id": "SAM-123456",
  "type": "blood_plasma",
  "donor_id": "D-789012",
  "collection_date": "2025-12-26T08:30:00Z",
  "volume": 10.5,
  "volume_unit": "mL",
  "storage_condition": "minus_80",
  "location": {
    "freezer": "F-01",
    "rack": "R-03",
    "box": "B-25",
    "position": "A5"
  },
  "quality": {
    "hemolysis_index": 25,
    "total_protein": 72
  },
  "consent_id": "CNS-456789"
}

Response:
{
  "success": true,
  "sample_id": "SAM-123456",
  "barcode": "data:image/png;base64,iVBOR...",
  "registered_at": "2025-12-26T10:45:30Z"
}
```

##### Query Inventory
```typescript
GET /api/v1/inventory?type=blood_plasma&available=true&min_volume=5

Response:
{
  "total_count": 2,450,
  "available_count": 2,380,
  "samples": [
    {
      "sample_id": "SAM-123456",
      "type": "blood_plasma",
      "volume": 10.5,
      "collection_date": "2025-12-26",
      "storage_duration_days": 0,
      "quality_score": 0.98,
      "location": "F-01/R-03/B-25/A5"
    },
    ...
  ]
}
```

##### Request Sample
```typescript
POST /api/v1/requests

Request:
{
  "requester_id": "RES-001",
  "project_id": "PRJ-2025-042",
  "samples": [
    {
      "sample_id": "SAM-123456",
      "volume_requested": 0.5,
      "volume_unit": "mL"
    }
  ],
  "purpose": "Biomarker analysis",
  "ethics_approval": "IRB-2025-123",
  "return_date": "2025-12-30"
}

Response:
{
  "request_id": "REQ-987654",
  "status": "pending_approval",
  "estimated_fulfillment": "2025-12-27T14:00:00Z"
}
```

### 9.2 Data Standards

```
Interoperability:
- HL7 FHIR: Healthcare data exchange
- SPREC: Sample PREanalytical Code
- MIABIS: Minimum Information About BIobank data Sharing
- BRISQ: Biospecimen Reporting for Improved Study Quality

Data Formats:
- JSON for API exchanges
- CSV/TSV for bulk data
- XML for HL7 messages
- PDF for reports and labels
```

---

## 10. Safety and Compliance

### 10.1 Biosafety

#### 10.1.1 Risk Groups
```
Risk Group 1 (Low):
- Normal healthy human samples
- Precautions: Standard laboratory practices

Risk Group 2 (Moderate):
- Samples from patients with infectious diseases
- Examples: HIV, HBV, HCV
- Precautions: BSL-2 containment

Risk Group 3 (High):
- Highly infectious pathogens
- Examples: TB, SARS-CoV-2 (variants)
- Precautions: BSL-3 containment

Risk Group 4 (Extreme):
- Life-threatening pathogens, no treatment
- Examples: Ebola, Marburg
- Precautions: BSL-4 containment
```

#### 10.1.2 Personal Protective Equipment (PPE)
```
Minimum (Risk Group 1-2):
- Lab coat
- Gloves (nitrile, latex-free)
- Safety glasses

Cryogenic Handling:
- Cryo-gloves (insulated)
- Face shield
- Closed-toe shoes
- Lab coat

High Risk (BSL-3):
- Powered Air-Purifying Respirator (PAPR)
- Double gloves
- Disposable gown
- Dedicated shoes
```

### 10.2 Regulatory Compliance

#### 10.2.1 United States
```
FDA Regulations:
- 21 CFR Part 1271: Human Cells, Tissues, and Cellular and Tissue-Based Products (HCT/Ps)
- Applicable to reproductive tissue, stem cells

CLIA:
- Clinical Laboratory Improvement Amendments
- Applies if samples used for diagnostics

OSHA:
- Bloodborne Pathogens Standard (29 CFR 1910.1030)
- Exposure control plan
- Training and vaccinations
```

#### 10.2.2 European Union
```
GDPR:
- General Data Protection Regulation
- Applies to personal data, including genetic data

EU Tissue and Cells Directives:
- 2004/23/EC: Standards of quality and safety
- 2006/17/EC: Technical requirements
- 2006/86/EC: Traceability, serious adverse events

IVD Regulation (IVDR):
- In Vitro Diagnostic Medical Devices Regulation
- Effective May 2022
```

#### 10.2.3 International Standards
```
ISO 20387:2018:
- Biobanking - General requirements
- Quality management specific to biobanks

ISO 15189:2012:
- Medical laboratories - Quality and competence
- Accreditation standard

ISBER Best Practices:
- 4th Edition (2018)
- Comprehensive biobanking guidelines
```

### 10.3 Emergency Procedures

#### 10.3.1 Freezer Failure
```
Response Protocol:
1. Alarm acknowledgment (5 min)
2. Assess situation:
   - Temperature reading
   - Freezer operational status
   - Time to failure
3. If T > -70°C:
   - Transfer samples to backup freezer
   - Use dry ice to maintain temperature during transfer
4. If beyond recovery:
   - Salvage critical samples first
   - Document temperature excursion
   - Assess sample integrity
5. Root cause analysis
6. Preventive measures

Priority Transfer Order:
1. Irreplaceable samples
2. Living cells
3. RNA
4. DNA, proteins, plasma
```

#### 10.3.2 Liquid Nitrogen Shortage
```
Response Protocol:
1. Conserve LN₂ use
2. Emergency LN₂ delivery
3. If LN₂ unavailable:
   - Transfer cells to -80°C temporarily (max 48 hours)
   - Dry ice backup
4. Post-event viability assessment
```

#### 10.3.3 Sample Spill
```
Biological Spill Response:
1. Alert others, evacuate if necessary
2. PPE: Gloves, lab coat, face protection
3. Cover spill with absorbent material
4. Disinfectant (10% bleach, 10 min contact time)
5. Clean with soap and water
6. Dispose in biohazard waste
7. Document incident

Cryogenic Spill (LN₂):
1. Evacuate area (oxygen displacement risk)
2. Ventilate
3. Allow to evaporate (do not touch)
4. Oxygen monitor before re-entry
```

---

## 11. References

### 11.1 Standards and Guidelines

1. ISO 20387:2018 - Biotechnology - Biobanking - General requirements for biobanking
2. ISO 15189:2012 - Medical laboratories - Requirements for quality and competence
3. ISBER Best Practices (4th Edition, 2018) - International Society for Biological and Environmental Repositories
4. NCI Best Practices for Biospecimen Resources (2016)
5. OECD Best Practice Guidelines for Biological Resource Centres (2007)

### 11.2 Regulatory Documents

1. 21 CFR Part 1271 - Human Cells, Tissues, and Cellular and Tissue-Based Products (FDA)
2. EU Directive 2004/23/EC - Standards of quality and safety for the donation, procurement, testing, processing, preservation, storage and distribution of human tissues and cells
3. General Data Protection Regulation (GDPR) - EU Regulation 2016/679

### 11.3 Scientific Literature

1. Vaught, J. (2016). "Biobanking comes of age: The transition to biospecimen science." Annual Review of Pharmacology and Toxicology, 56, 211-228.
2. 선행 연구. "Biobanking: The foundation of personalized medicine." Current Opinion in Oncology, 28(5), 441-448.
3. 선행 연구. "Biospecimen science and its impact on research." Biopreservation and Biobanking, 15(5), 401-403.

### 11.4 Data Standards

1. SPREC - Sample PREanalytical Code (v2.0, 2015)
2. MIABIS - Minimum Information About BIobank data Sharing (v2.0, 2016)
3. BRISQ - Biospecimen Reporting for Improved Study Quality (2011)
4. HL7 FHIR - Fast Healthcare Interoperability Resources

### 11.5 WIA Standards

- WIA-INTENT: Intent-based interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-HEALTH: Healthcare data standards
- WIA-SOCIAL: Collaborative research networks

---

## Appendix A: Sample Collection Checklist

```
Pre-Collection:
□ Verify donor consent
□ Confirm donor identity
□ Check fasting status (if required)
□ Prepare collection materials
□ Label tubes/containers with sample ID

Collection:
□ Follow aseptic technique
□ Use appropriate collection method
□ Record collection time
□ Note any deviations or issues
□ Maintain sample at proper temperature

Post-Collection:
□ Transport to lab within specified time
□ Process according to SOP
□ Aliquot as needed
□ Store at correct temperature
□ Update LIMS with sample details
□ Generate barcode labels
```

## Appendix B: Quality Metrics Summary

| Sample Type | Key Metric | Acceptable Range | Storage Temp | Max Duration |
|-------------|------------|------------------|--------------|--------------|
| DNA | A260/A280 | 1.8-2.0 | -20°C to -80°C | 50+ years |
| RNA | RIN | ≥7.0 | -80°C | 5 years |
| Plasma | Total Protein | 60-80 g/L | -80°C | 20 years |
| Cells | Viability | >80% | -196°C | Indefinite |
| Tissue | Cell Viability | >80% | -196°C | Indefinite |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-019 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
