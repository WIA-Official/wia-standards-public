# WIA-CITY-005: Zero Energy Building - PHASE 3 PROTOCOL

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This document defines the protocols, procedures, and verification methods for Zero Energy Building (ZEB) certification, performance monitoring, and ongoing compliance verification.

---

## 2. ZEB Certification Protocol

### 2.1 Certification Levels

The WIA-CITY-005 standard defines a 5-grade certification system:

#### Grade 1: Premium ZEB 🏆
- **Balance Ratio**: ≥ 120%
- **Criteria**:
  - Annual production exceeds consumption by ≥20%
  - Consistent surplus generation throughout the year
  - Net exporter to grid
  - Carbon negative status
  - ESS capacity ≥ 3 hours of average load
- **Recognition**: Highest achievement level
- **Incentives**: Maximum rebates, grants, and recognition

#### Grade 2: Excellent ZEB ⭐
- **Balance Ratio**: 100-120%
- **Criteria**:
  - Net zero energy balance achieved
  - Annual production ≥ consumption
  - Self-sufficiency ≥ 95%
  - Carbon neutral or carbon negative
  - ESS capacity ≥ 2 hours of average load
- **Recognition**: Full ZEB certification
- **Incentives**: Standard ZEB incentives

#### Grade 3: Good ZEB ✅
- **Balance Ratio**: 80-100%
- **Criteria**:
  - Near zero energy balance
  - Self-sufficiency ≥ 80%
  - Significant renewable integration
  - Low carbon footprint
  - ESS capacity ≥ 1 hour of average load
- **Recognition**: ZEB-Ready certification
- **Path**: 3-year improvement plan to Grade 2

#### Grade 4: Fair ZEB 📊
- **Balance Ratio**: 60-80%
- **Criteria**:
  - Substantial renewable energy integration
  - Self-sufficiency ≥ 60%
  - Demonstrated commitment to ZEB goals
  - Some ESS capacity
- **Recognition**: Pre-ZEB certification
- **Path**: 5-year improvement plan to Grade 2

#### Grade 5: Needs Improvement ⚠️
- **Balance Ratio**: < 60%
- **Criteria**:
  - Initial renewable integration
  - Below ZEB performance thresholds
  - Requires significant upgrades
- **Recognition**: Conditional registration
- **Path**: Comprehensive retrofit plan required

### 2.2 Certification Process Flow

```
┌─────────────────────────────────────────────────────────────┐
│                  ZEB CERTIFICATION PROCESS                  │
└─────────────────────────────────────────────────────────────┘

Phase 1: PRE-CERTIFICATION (1-2 months)
├── Application Submission
│   ├── Building information
│   ├── Energy system designs
│   ├── Expected performance data
│   └── Fees payment
├── Document Review
│   ├── Compliance check
│   ├── Energy modeling validation
│   └── Passive design verification
└── Site Inspection
    ├── Construction review
    ├── System verification
    └── Measurement equipment installation

Phase 2: MONITORING PERIOD (12 months)
├── Data Collection
│   ├── Monthly production reports
│   ├── Monthly consumption reports
│   ├── Grid interaction logs
│   └── System performance metrics
├── Quarterly Reviews
│   ├── Performance analysis
│   ├── Issue identification
│   └── Corrective actions
└── Continuous Monitoring
    ├── Real-time dashboard
    ├── Automated alerts
    └── Data validation

Phase 3: VERIFICATION (1-2 months)
├── Data Analysis
│   ├── Annual energy balance calculation
│   ├── Carbon footprint assessment
│   ├── Performance metrics validation
│   └── Statistical analysis
├── Third-Party Audit
│   ├── On-site inspection
│   ├── Equipment testing
│   ├── Data verification
│   └── Compliance check
└── Grade Assignment
    ├── Performance scoring
    ├── Grade determination
    └── Recommendations

Phase 4: CERTIFICATION (1 month)
├── Certificate Issuance
│   ├── Digital certificate
│   ├── Physical plaque
│   ├── Verifiable Credential (VC)
│   └── QR code generation
├── Public Registration
│   ├── WIA registry listing
│   ├── Public dashboard
│   └── Marketing materials
└── Ongoing Support
    ├── Annual reporting
    ├── Re-certification (every 3 years)
    └── Upgrade pathways
```

### 2.3 Application Requirements

#### 2.3.1 Initial Application

**Required Documents:**

1. **Building Information**
   - Legal address and ownership
   - Building type and use
   - Floor area and occupancy
   - Construction year
   - Building plans and specifications

2. **Energy System Documentation**
   - Solar PV system design and specifications
   - Wind turbine specifications (if applicable)
   - Geothermal system design (if applicable)
   - ESS specifications and capacity
   - HVAC system design and efficiency ratings
   - Lighting design and controls
   - Building envelope performance (U-values, R-values)

3. **Energy Modeling**
   - Annual energy production forecast
   - Annual energy consumption forecast
   - Expected balance ratio
   - Carbon emissions projection
   - Software used (e.g., EnergyPlus, PHPP)

4. **Passive Design Features**
   - Building orientation justification
   - Insulation specifications
   - Window specifications
   - Ventilation strategy
   - Thermal mass calculations
   - Daylighting analysis

5. **Monitoring Plan**
   - Metering infrastructure
   - Data collection methodology
   - Data management system
   - Reporting schedule

**Application Fee:**
- Residential (< 500 m²): $500
- Small Commercial (500-2,000 m²): $1,500
- Medium Commercial (2,000-10,000 m²): $5,000
- Large Commercial (> 10,000 m²): $10,000

### 2.4 Monitoring Requirements

#### 2.4.1 Data Collection Frequency

| Data Type | Frequency | Retention |
|-----------|-----------|-----------|
| **Production (real-time)** | 1 minute | 7 days |
| **Consumption (real-time)** | 1 minute | 7 days |
| **Production (aggregated)** | 15 minutes | 90 days |
| **Consumption (aggregated)** | 15 minutes | 90 days |
| **Daily summaries** | Daily | 2 years |
| **Monthly reports** | Monthly | 5 years |
| **Annual reports** | Annual | Permanent |

#### 2.4.2 Required Metrics

**Production Metrics:**
- Total kWh by source (solar, wind, geothermal)
- Peak production (kW)
- Capacity factor
- System efficiency
- Performance ratio (for solar)

**Consumption Metrics:**
- Total kWh by category (HVAC, lighting, equipment)
- Peak demand (kW)
- Load factor
- Energy Use Intensity (EUI)
- System efficiencies (COP, EER)

**Grid Metrics:**
- Import kWh
- Export kWh
- Net energy (production - consumption)
- Grid dependency percentage
- Revenue from exports

**Carbon Metrics:**
- Emissions from grid electricity
- Emissions from other fuels
- Offsets from renewable production
- Net carbon footprint
- Carbon intensity (g CO₂/kWh)

#### 2.4.3 Data Submission

**Monthly Submission:**
- Deadline: 15th of following month
- Format: JSON or CSV via API
- Contents: All required metrics for the month
- Validation: Automated checks for completeness and accuracy

**Quarterly Review:**
- Performance analysis report
- Comparison to baseline
- Identification of issues
- Corrective action plans

**Annual Report:**
- Comprehensive performance summary
- Grade calculation
- Carbon footprint analysis
- Cost-benefit analysis
- Recommendations for improvement

---

## 3. Performance Verification Protocol

### 3.1 Energy Balance Verification

#### 3.1.1 Calculation Methodology

**Annual Energy Balance:**

```
Balance Ratio = (Total Annual Production / Total Annual Consumption) × 100%

Where:
- Total Annual Production = Solar + Wind + Geothermal + Other Renewables (kWh)
- Total Annual Consumption = HVAC + Lighting + Equipment + Other (kWh)
```

**Self-Sufficiency:**

```
Self-Sufficiency = ((Total Consumption - Grid Import) / Total Consumption) × 100%
```

**Energy Use Intensity (EUI):**

```
EUI = Total Annual Consumption / Conditioned Floor Area (kWh/m²/year)
```

**Renewable Fraction:**

```
Renewable Fraction = Total Renewable Production / Total Consumption
```

#### 3.1.2 Verification Steps

1. **Data Collection Review**
   - Verify data completeness (365 days)
   - Check for missing data periods
   - Validate timestamp accuracy
   - Confirm meter calibration records

2. **Data Quality Assessment**
   - Statistical analysis for outliers
   - Cross-validation with utility bills
   - Weather normalization
   - Comparison to energy model

3. **On-Site Verification**
   - Physical meter readings
   - Equipment nameplate verification
   - Visual inspection of systems
   - Operational testing

4. **Third-Party Audit**
   - Independent data review
   - Calculation verification
   - Compliance check
   - Final grade recommendation

### 3.2 Carbon Footprint Verification

#### 3.2.1 Emission Factors

**Grid Electricity** (location-specific):
- Example: 490 g CO₂/kWh (US average)
- Use marginal emission factors for exported energy
- Account for transmission losses (typically 7%)

**Natural Gas:**
- 182 g CO₂/kWh (5.3 kg CO₂/therm)

**Renewable Production Credits:**
- Same as grid emission factor (avoided emissions)

#### 3.2.2 Calculation

**Total Emissions:**

```
Total Emissions (kg CO₂) =
  (Grid Import kWh × Grid Emission Factor) +
  (Natural Gas kWh × Gas Emission Factor) +
  (Other Fuels × Respective Factors)
```

**Total Offsets:**

```
Total Offsets (kg CO₂) =
  -(Renewable Production kWh × Grid Emission Factor) -
  (Grid Export kWh × Marginal Emission Factor)
```

**Net Carbon Footprint:**

```
Net Carbon (kg CO₂) = Total Emissions + Total Offsets
```

**Carbon Intensity:**

```
Carbon Intensity (g CO₂/kWh) =
  (Net Carbon × 1000) / Total Consumption
```

**Status Classification:**
- **Carbon Positive**: Net > 0 kg CO₂
- **Carbon Neutral**: Net ≈ 0 kg CO₂ (±5%)
- **Carbon Negative**: Net < 0 kg CO₂

---

## 4. Certification Document Protocol

### 4.1 Energy Performance Certificate

**Certificate Contents:**

1. **Building Identification**
   - Certificate ID: ZEB-YYYY-XXXXX
   - Building name and address
   - Building type and floor area
   - Owner/operator information

2. **Performance Metrics**
   - ZEB Grade (1-5)
   - Balance Ratio
   - Self-Sufficiency
   - Energy Use Intensity (EUI)
   - Renewable Fraction
   - Carbon Intensity

3. **Energy Systems**
   - Solar PV capacity and production
   - Wind capacity and production (if applicable)
   - Geothermal capacity and production (if applicable)
   - ESS capacity and specifications
   - Grid interaction summary

4. **Carbon Performance**
   - Annual emissions
   - Annual offsets
   - Net carbon footprint
   - Carbon status (neutral/negative)

5. **Certification Details**
   - Issue date
   - Expiration date (typically 3 years)
   - Certifying auditor
   - WIA certification authority signature

6. **QR Code**
   - Links to public verification page
   - Real-time performance dashboard
   - Verifiable Credential (VC)

### 4.2 Verifiable Credential (VC) Protocol

#### 4.2.1 VC Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.official/credentials/zeb/v1"
  ],
  "type": ["VerifiableCredential", "ZEBCertificate"],
  "issuer": {
    "id": "did:wia:certification-authority",
    "name": "WIA Certification Authority",
    "url": "https://wia.official"
  },
  "issuanceDate": "2025-12-25T00:00:00Z",
  "expirationDate": "2028-12-25T00:00:00Z",
  "credentialSubject": {
    "id": "did:wia:building:ZEB-2025-12345",
    "buildingName": "Green Tower A",
    "address": {
      "street": "123 Eco Street",
      "city": "Green City",
      "state": "GC",
      "zip": "12345",
      "country": "US"
    },
    "buildingType": "residential",
    "floorArea": 5000,
    "zebGrade": 2,
    "gradeDescription": "Excellent",
    "energyBalance": {
      "annualProduction": 283000,
      "annualConsumption": 185800,
      "balanceRatio": 1.523,
      "selfSufficiency": 97.1,
      "eui": 37.16
    },
    "carbonPerformance": {
      "netEmissions": -15306,
      "carbonIntensity": -82.4,
      "carbonNeutral": true,
      "carbonNegative": true
    },
    "renewableSources": ["solar", "wind", "geothermal"],
    "certification": {
      "standard": "WIA-CITY-005",
      "version": "1.0",
      "certificateId": "ZEB-2025-12345",
      "auditDate": "2025-11-15",
      "certifier": {
        "name": "John Smith",
        "license": "WIA-AUDITOR-1234",
        "organization": "Green Building Certifiers Inc."
      }
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T00:00:00Z",
    "verificationMethod": "did:wia:certification-authority#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z58DAdFfa9SkqZMVPxAQp...mN1eMgjeBDDx6RgEVL4w=="
  }
}
```

#### 4.2.2 VC Issuance Protocol

1. **Credential Generation**
   - Populate template with building data
   - Assign unique DID (Decentralized Identifier)
   - Include all performance metrics
   - Add certification metadata

2. **Digital Signature**
   - Use WIA certification authority key
   - Ed25519 signature algorithm
   - Timestamp of issuance
   - Proof of authenticity

3. **Blockchain Anchoring** (optional)
   - Hash of VC stored on blockchain
   - Immutable record of issuance
   - Prevents tampering
   - Public verifiability

4. **Distribution**
   - Send to building owner's digital wallet
   - Publish to WIA public registry
   - Generate QR code for physical certificate
   - API access for third-party verification

#### 4.2.3 VC Verification Protocol

**Verification Steps:**

1. **Signature Verification**
   ```javascript
   async function verifyVC(vc) {
     const { proof, ...credential } = vc;
     const publicKey = await resolvePublicKey(proof.verificationMethod);
     const isValid = await verifySignature(
       credential,
       proof.proofValue,
       publicKey
     );
     return isValid;
   }
   ```

2. **Expiration Check**
   - Verify current date < expiration date
   - Check for revocation status

3. **Issuer Verification**
   - Confirm issuer is authorized WIA authority
   - Verify issuer's DID and public key

4. **Data Integrity**
   - Hash verification if blockchain-anchored
   - Schema validation
   - Required fields present

**Verification API:**

```
GET https://verify.wia.official/v1/zeb/{certificate_id}

Response:
{
  "valid": true,
  "certificateId": "ZEB-2025-12345",
  "buildingName": "Green Tower A",
  "zebGrade": 2,
  "issuanceDate": "2025-12-25",
  "expirationDate": "2028-12-25",
  "status": "active",
  "verifiableCredential": { ... }
}
```

---

## 5. Upgrade and Re-Certification Protocol

### 5.1 Grade Upgrade Pathway

Buildings can improve their ZEB grade through performance improvements:

**Grade 5 → Grade 4:**
- Add renewable energy capacity
- Improve building envelope
- Upgrade HVAC efficiency
- Install ESS
- Target: 60% balance ratio

**Grade 4 → Grade 3:**
- Expand renewable systems
- Optimize energy management
- Add or expand ESS
- Reduce consumption through efficiency
- Target: 80% balance ratio

**Grade 3 → Grade 2:**
- Achieve net zero energy balance
- Ensure consistent surplus generation
- Enhance ESS capacity to ≥2 hours
- Verify carbon neutrality
- Target: 100% balance ratio

**Grade 2 → Grade 1:**
- Increase renewable capacity
- Maximize surplus generation
- Expand ESS to ≥3 hours
- Achieve carbon negative status
- Target: 120% balance ratio

### 5.2 Re-Certification Protocol

**Frequency:** Every 3 years

**Process:**

1. **Performance Review** (Month 1-2)
   - Analyze 3 years of performance data
   - Calculate average balance ratio
   - Assess any degradation
   - Identify improvements made

2. **On-Site Inspection** (Month 2-3)
   - System condition assessment
   - Verify upgrades and modifications
   - Check maintenance records
   - Test equipment performance

3. **Grade Re-Assessment** (Month 3)
   - Calculate new grade based on 3-year average
   - Consider improvements
   - Issue updated certificate
   - Update VC and QR code

**Re-Certification Fee:**
- 50% of initial certification fee

### 5.3 Continuous Improvement Protocol

**Annual Performance Reports:**
- Due: January 31 each year
- Contains: Previous year's performance data
- Grade trend analysis
- Recommendations for improvement

**Mid-Certification Check-In:**
- At 18 months post-certification
- Review performance trends
- Identify potential issues
- Provide optimization recommendations

---

## 6. Auditor Certification Protocol

### 6.1 Auditor Qualifications

**Education:**
- Bachelor's degree in engineering, architecture, or related field
- OR 10+ years relevant experience

**Training:**
- WIA ZEB Auditor Training Program (40 hours)
- Building energy modeling certification
- Renewable energy systems knowledge
- Data analysis and statistics

**Experience:**
- 3+ ZEB audits as apprentice
- Demonstrated understanding of building science
- Familiarity with energy codes and standards

**Examination:**
- WIA ZEB Auditor Exam (passing score: 80%)
- Practical assessment

**Continuing Education:**
- 16 hours per year
- Stay current with standard updates

### 6.2 Auditor Responsibilities

1. **Independence**: No conflicts of interest
2. **Integrity**: Honest and accurate reporting
3. **Competence**: Maintain skills and knowledge
4. **Confidentiality**: Protect client information
5. **Professional Conduct**: Adhere to WIA code of ethics

### 6.3 Audit Quality Assurance

- **Peer Review**: 10% of audits randomly reviewed
- **Complaint Mechanism**: Process for addressing concerns
- **Disciplinary Actions**: Suspension or revocation for violations
- **Continuing Oversight**: Annual performance evaluation

---

## 7. Appeals and Dispute Resolution

### 7.1 Appeals Process

**Grounds for Appeal:**
- Disagreement with grade assignment
- Procedural errors in certification process
- Disputed data or calculations
- Auditor misconduct

**Procedure:**

1. **Formal Appeal Submission** (within 30 days of decision)
   - Written statement of grounds
   - Supporting documentation
   - Appeal fee: $500

2. **Review Panel** (60 days)
   - Three independent experts
   - Review of evidence
   - Re-calculation if necessary
   - Site visit if warranted

3. **Decision** (90 days total)
   - Uphold original decision
   - Modify grade
   - Order re-audit
   - Refund fees if error found

### 7.2 Dispute Resolution

**Mediation:**
- Voluntary process
- Neutral mediator
- Non-binding recommendations

**Arbitration:**
- Binding arbitration if mediation fails
- Single arbitrator or panel
- Decision final and enforceable

---

## 8. Revocation Protocol

**Grounds for Revocation:**
- Fraudulent data submission
- Material misrepresentation
- Failure to maintain performance
- Safety violations
- Non-payment of fees

**Procedure:**

1. **Notice of Intent** (30 days to respond)
2. **Hearing** (if requested)
3. **Decision** (within 60 days)
4. **Revocation** (if warranted)
   - Remove from registry
   - Invalidate VC
   - Public notice
   - No refund of fees

**Reinstatement:**
- Correct deficiencies
- Re-apply and pay fees
- Demonstrate compliance
- New certification issued

---

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
WIA-CITY-005 v1.0 - Certification Protocol
https://wia.official/standards/CITY/005/protocol
