# WIA-AGRI-030: Food Crisis Response
## PHASE 3 - PROTOCOL SPECIFICATION

**Version:** 1.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This phase defines standardized protocols for coordinating food crisis response across multiple stakeholders, including detection, assessment, resource allocation, distribution, and recovery procedures.

### 1.1 Protocol Layers

```
┌─────────────────────────────────────┐
│  Recovery & Prevention Protocol     │  Phase 4
├─────────────────────────────────────┤
│  Distribution & Verification        │  Phase 3
├─────────────────────────────────────┤
│  Resource Mobilization              │  Phase 2
├─────────────────────────────────────┤
│  Detection & Assessment             │  Phase 1
├─────────────────────────────────────┤
│  Monitoring & Early Warning         │  Phase 0
└─────────────────────────────────────┘
```

---

## 2. Phase 0: Monitoring & Early Warning

### 2.1 Continuous Monitoring Protocol

**Objective**: Detect potential food crises before they escalate.

**Key Indicators Monitored**:
1. **Meteorological Data**
   - Rainfall patterns (deviation from historical averages)
   - Temperature extremes
   - Drought indices (SPI, PDSI)

2. **Agricultural Data**
   - Vegetation health (NDVI from satellites)
   - Crop condition reports
   - Pest/disease outbreak tracking

3. **Socioeconomic Indicators**
   - Food prices (market monitoring)
   - Employment rates
   - Migration patterns

4. **Conflict and Displacement**
   - Conflict events (ACLED data)
   - Refugee/IDP numbers
   - Access restrictions

**Monitoring Frequency**:
- Satellite data: Daily
- Market prices: Weekly
- Household surveys: Monthly
- Comprehensive assessments: Quarterly

### 2.2 Early Warning Trigger Protocol

**Trigger Conditions**:

| Severity | Conditions | Response Time |
|----------|-----------|---------------|
| **Advisory** | 1-2 indicators exceed threshold | 30 days |
| **Watch** | 3+ indicators exceed threshold OR 1 critical | 14 days |
| **Warning** | Multiple critical indicators | 7 days |
| **Emergency** | Imminent famine conditions | 24 hours |

**Action on Trigger**:
1. **Automated Alert Generation**
   ```json
   {
     "alertType": "early-warning",
     "severity": "watch",
     "triggers": ["rainfall-deficit-60%", "crop-failure-predicted"],
     "affectedRegion": "...",
     "recommendedActions": ["preposition-stocks", "activate-early-action-protocol"]
   }
   ```

2. **Stakeholder Notification**
   - Government early warning units
   - WFP, FAO, UNICEF
   - Local NGOs and Red Cross/Crescent
   - Community leaders

3. **Early Action Activation** (if funded)
   - Pre-positioning of food stocks
   - Cash-based interventions
   - Agricultural support (seeds, tools)

### 2.3 Predictive Model Protocol

**AI/ML Model Requirements**:
- **Input data**: Weather forecasts, historical crisis data, current indicators
- **Output**: Probability of crisis, estimated severity, affected population
- **Update frequency**: Weekly
- **Accuracy target**: ≥80% for 3-month forecasts
- **Validation**: Backtesting against historical events

**Model Output Format**:
```json
{
  "predictionId": "PRED-2025-001",
  "forecastDate": "2025-03-01",
  "region": "Horn of Africa",
  "probability": 0.84,
  "estimatedSeverity": 3,
  "affectedPopulation": 4200000,
  "confidence": 0.87,
  "keyDrivers": ["el-nino", "below-average-rainfall", "conflict"]
}
```

---

## 3. Phase 1: Detection & Assessment (0-24 Hours)

### 3.1 Crisis Detection Protocol

**Automated Detection**:
When ≥3 critical indicators trigger simultaneously:
1. System automatically creates draft crisis record
2. Assign preliminary severity based on indicator values
3. Generate initial affected population estimate
4. Flag for human verification within 2 hours

**Manual Reporting**:
Field teams can submit crisis reports via:
- Web portal
- Mobile app (offline-capable)
- SMS gateway (for low-connectivity areas)
- Email (parsed by NLP system)

### 3.2 Rapid Assessment Protocol (First 24 Hours)

**Objective**: Confirm crisis, assess severity, estimate needs.

**Assessment Team Deployment**:
- **Trigger**: Crisis severity ≥2
- **Team size**: 3-5 persons (food security, nutrition, logistics)
- **Deployment time**: <24 hours from detection
- **Duration**: 2-5 days

**Rapid Assessment Checklist**:
1. ☑ Verify crisis existence and type
2. ☑ Map affected geographic area
3. ☑ Estimate affected population (disaggregated by age/gender)
4. ☑ Determine IPC phase classification
5. ☑ Assess immediate needs (food, water, nutrition, health)
6. ☑ Identify access constraints
7. ☑ Estimate response timeline
8. ☑ Identify local response capacity

**Data Collection Methods**:
- Key informant interviews (community leaders, health workers)
- Market assessments
- Satellite imagery analysis
- Health facility data (malnutrition admissions)
- Secondary data review

**Reporting Timeline**:
- Preliminary findings: Within 6 hours of arrival
- Full rapid assessment report: Within 48 hours

### 3.3 Severity Classification Protocol

**IPC Analysis Protocol**:
Follow IPC 3.0 protocols for:
1. **Acute Food Insecurity** (current and projected)
2. **Acute Malnutrition** (prevalence)
3. **Mortality** (crude and under-5)

**Quality Assurance**:
- All IPC analyses reviewed by certified IPC analysts
- Consensus building among stakeholders
- Transparent documentation of evidence and assumptions

**Communication Protocol**:
```json
{
  "ipcAnalysis": {
    "region": "...",
    "populationAnalyzed": 800000,
    "current": {
      "phase3": 250000,
      "phase4": 180000,
      "phase5": 0
    },
    "projected": {
      "period": "2025-03 to 2025-05",
      "phase3": 320000,
      "phase4": 280000,
      "phase5": 15000
    },
    "confidence": "medium",
    "keyFindings": ["..."],
    "recommendations": ["..."]
  }
}
```

---

## 4. Phase 2: Resource Mobilization (24-72 Hours)

### 4.1 Needs Calculation Protocol

**Calculation Formula**:
```
Food Need (MT) = Population × Duration (days) × Daily ration (kg) / 1000
Water Need (L) = Population × Duration × Daily water (L/person)
```

**Standard Rations**:
- **General Food Distribution**: 2100 kcal/person/day
  - Cereals: 400g
  - Pulses: 60g
  - Vegetable oil: 25g
  - Salt: 5g
  - Sugar: 20g

- **Nutrition (SAM treatment)**: RUTF 200g/child/day for 6-8 weeks

**Duration**:
- Emergency response: 3-6 months
- Extended response: Up to 12 months

### 4.2 Resource Allocation Protocol

**Allocation Algorithm** (AI-optimized):

```python
def allocate_resources(crisis, available_resources):
    """
    Optimize resource allocation based on:
    - Crisis severity (weight: 40%)
    - Affected population (weight: 30%)
    - Logistical feasibility (weight: 20%)
    - Cost-effectiveness (weight: 10%)
    """
    priority_score = (
        crisis.severity * 0.4 +
        normalize(crisis.population) * 0.3 +
        logistics_score(crisis) * 0.2 +
        cost_efficiency(crisis) * 0.1
    )

    allocation = optimize(
        resources=available_resources,
        priority=priority_score,
        constraints=[budget, capacity, timeline]
    )

    return allocation
```

**Multi-source Coordination**:
1. **Global Food Reserves** (WFP, national reserves)
2. **Emergency Procurement** (local/regional purchase)
3. **In-kind Donations** (bilateral donors)
4. **Cash-based Transfers** (where markets functional)

**Approval Workflow**:
```
Crisis Registered → Needs Assessment → Resource Request
    ↓
Allocation Proposed (AI) → Human Review → Approval
    ↓
Reservation Confirmed → Logistics Initiated
```

### 4.3 Coordination Protocol

**Cluster System Activation**:
- **Food Security Cluster**: Overall coordination (led by WFP/FAO)
- **Nutrition Cluster**: Malnutrition treatment (led by UNICEF)
- **Health Cluster**: Disease outbreak response
- **Logistics Cluster**: Transport coordination (led by WFP)
- **WASH Cluster**: Water and sanitation

**Meeting Schedule**:
- Daily during first week
- 3x per week during active response
- Weekly during recovery phase

**Information Sharing Protocol**:
- All crisis data shared via WIA-AGRI-030 API
- 5W reports (Who, What, Where, When, for Whom) updated weekly
- Real-time tracking via shared dashboard

---

## 5. Phase 3: Distribution & Verification (3-30 Days)

### 5.1 Distribution Planning Protocol

**Site Selection Criteria**:
1. Accessibility for beneficiaries (<5km walking distance)
2. Security considerations
3. Infrastructure (shade, water, space for queues)
4. Proximity to affected populations
5. Existing community structures

**Distribution Modalities**:

| Modality | Use Case | Requirements |
|----------|----------|--------------|
| **General Food Distribution** | Acute emergency, markets collapsed | Food available, logistics capacity |
| **Cash/Voucher** | Markets functional | Banking/mobile money, vendor capacity |
| **School Feeding** | Chronic food insecurity | Schools operational, cooking facilities |
| **Nutrition Programs** | High malnutrition | Health facilities, trained staff |

**Beneficiary Registration Protocol**:
```json
{
  "beneficiaryId": "BEN-2025-001",
  "headOfHousehold": {
    "name": "...",
    "nationalId": "...",
    "biometrics": "fingerprint-hash"
  },
  "householdSize": 6,
  "vulnerabilities": ["elderly", "child-headed", "disabled"],
  "distributionPoint": "DP-SO-023",
  "entitlement": {
    "modality": "food",
    "ration": "standard-2100kcal",
    "frequency": "monthly"
  },
  "registeredAt": "2025-01-17T10:30:00Z"
}
```

### 5.2 Distribution Execution Protocol

**Standard Operating Procedure**:

1. **Pre-distribution**:
   - Verify stock arrival at distribution point
   - Quality inspection (expiry, condition)
   - Set up distribution area (queues, shade, water)
   - Brief distribution team

2. **During Distribution**:
   - Verify beneficiary identity (ID + biometric)
   - Measure out correct ration
   - Beneficiary signature/thumbprint
   - Record distribution in system (real-time if connectivity available)

3. **Post-distribution**:
   - Reconcile distributed vs. planned quantities
   - Report discrepancies
   - Post-distribution monitoring (PDM) survey sample
   - Update blockchain ledger

**Blockchain Verification**:
```json
{
  "blockchainEntry": {
    "transactionHash": "0x7f3a...",
    "timestamp": "2025-01-20T14:35:00Z",
    "distributionPoint": "DP-SO-023",
    "commodity": "wheat-flour",
    "quantityDistributed": 12500,
    "beneficiaries": 2480,
    "verifiedBy": ["DP-Manager", "WFP-Monitor", "Community-Leader"],
    "photos": ["ipfs://Qm..."]
  }
}
```

### 5.3 Quality Assurance Protocol

**Food Quality Standards**:
- All food meets WFP quality standards
- No commodities past expiry date
- Storage conditions monitored (temperature, humidity)
- Pest control measures in place

**Distribution Monitoring**:
- **Process Monitoring**: Observe 10% of distributions
- **Post-Distribution Monitoring**: Survey 10% of beneficiaries within 2 weeks
- **Complaint Mechanism**: Hotline, SMS, community feedback sessions

**Key PDM Questions**:
1. Did you receive the intended assistance?
2. Was the quantity correct?
3. Was the quality satisfactory?
4. Were you treated with dignity and respect?
5. Any problems or concerns?

---

## 6. Phase 4: Recovery & Prevention (30+ Days)

### 6.1 Transition to Recovery Protocol

**Criteria for Transition**:
- IPC phase reduced to ≤2 for ≥2 months
- Malnutrition rates declining
- Markets functioning
- Agricultural production resuming

**Transition Activities**:
1. **Food Assistance for Assets (FFA)**:
   - Conditional food/cash for community projects
   - Rehabilitate irrigation, roads, soil conservation

2. **Livelihood Support**:
   - Seeds and tools distribution
   - Livestock restocking
   - Vocational training

3. **Resilience Building**:
   - Climate-smart agriculture
   - Diversified livelihoods
   - Savings and credit groups

### 6.2 Prevention Protocol

**Climate Adaptation**:
- Drought-resistant crop varieties
- Water harvesting infrastructure
- Forecast-based early action

**Social Protection**:
- Expand national safety nets
- Shock-responsive social protection
- Link humanitarian aid to government systems

**Capacity Building**:
- Train local government on food security analysis
- Strengthen national early warning systems
- Build local NGO response capacity

### 6.3 Lessons Learned Protocol

**After-Action Review (AAR)**:
- Conducted within 3 months of crisis resolution
- Multi-stakeholder participation
- Focus on what worked, what didn't, why

**AAR Format**:
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "reviewDate": "2025-04-15",
  "participants": ["WFP", "FAO", "Government", "NGOs"],
  "strengths": [
    "Early warning system detected crisis 45 days in advance",
    "Blockchain tracking prevented aid diversion"
  ],
  "challenges": [
    "Road access limited distribution speed",
    "Coordination with local government delayed"
  ],
  "recommendations": [
    "Pre-position stocks closer to high-risk areas",
    "Establish year-round coordination structures"
  ],
  "dataUpdates": [
    "Updated access constraint maps",
    "Revised vulnerability profiles"
  ]
}
```

**Knowledge Management**:
- All AAR findings entered into shared database
- Used to update AI prediction models
- Inform preparedness planning for next crisis

---

## 7. Communication Protocols

### 7.1 Internal Communication

**Daily Situation Reports (SitReps)**:
- Distributed to all response partners
- Standardized template (see below)
- Submitted by 5pm local time

**SitRep Template**:
```markdown
SITUATION REPORT: [Crisis Name]
Date: [Date]
Reporting Period: [Start] to [End]

HIGHLIGHTS:
- [Key development 1]
- [Key development 2]

POPULATION AFFECTED: [Number]

RESPONSE SUMMARY:
- Food distributed: [MT]
- Beneficiaries reached: [Number]
- Distribution points active: [Number]

CHALLENGES:
- [Challenge 1]

ACTIONS PLANNED (Next 24-48h):
- [Action 1]
```

### 7.2 External Communication

**Public Information**:
- Press releases (approved by cluster lead)
- Social media updates (sensitive to dignity)
- Donor reports (monthly)

**Beneficiary Communication**:
- Distribution schedules posted at community centers
- SMS alerts to registered beneficiaries
- Community meetings for Q&A

**Complaint and Feedback Mechanism**:
- Toll-free hotline
- SMS short code
- Suggestion boxes at distribution points
- Community feedback sessions (monthly)

---

## 8. Security Protocols

### 8.1 Staff Safety

**Risk Assessment**:
- Conducted before entering new area
- Updated weekly in volatile contexts
- Covers: conflict, criminality, natural hazards, health

**Security Levels**:
| Level | Description | Restrictions |
|-------|-------------|--------------|
| **1 - Low** | Minimal risk | Normal operations |
| **2 - Moderate** | Some risk | Curfews, movement restrictions |
| **3 - Substantial** | High risk | Armed escorts, reduced presence |
| **4 - Severe** | Very high risk | Remote management only |
| **5 - Extreme** | Imminent danger | Evacuation |

### 8.2 Asset Security

**Food Stock Protection**:
- Warehouses with perimeter fencing, locks, guards
- Access control (logged entries)
- CCTV monitoring (if available)
- Regular inventory audits

**Transport Security**:
- GPS tracking on all trucks
- Communication equipment (satellite phone/radio)
- Convoy protocols in high-risk areas
- Insurance coverage

### 8.3 Data Security

**Beneficiary Data Protection**:
- Encrypted storage (AES-256)
- Access control (role-based)
- No sharing without consent (except anonymized)
- GDPR/local data protection compliance

**Blockchain Security**:
- Distributed ledger (no single point of failure)
- Cryptographic signatures for all transactions
- Immutable audit trail

---

## 9. Compliance and Accountability

### 9.1 Standards Compliance

All operations must comply with:
- **Sphere Standards**: Humanitarian Charter and Minimum Standards
- **CHS (Core Humanitarian Standard)**: 9 commitments
- **HAP (Humanitarian Accountability Partnership)**
- **WFP Program Guidance Manual**

### 9.2 Accountability to Affected Populations (AAP)

**Key Principles**:
1. **Participation**: Communities involved in needs assessment, design, monitoring
2. **Information Sharing**: Beneficiaries informed of entitlements, complaint mechanisms
3. **Feedback and Complaints**: Safe, accessible channels; responses within 7 days
4. **Do No Harm**: Conflict sensitivity, protection mainstreaming

**AAP Minimum Actions**:
☑ Information boards at all distribution points
☑ Community feedback sessions (monthly)
☑ Functional complaint mechanism
☑ Beneficiary satisfaction survey (post-distribution)

### 9.3 Fraud Prevention

**Red Flags**:
- Ghost beneficiaries (fake registrations)
- Ration diversion (aid not reaching intended recipients)
- Kickbacks (bribes for registration/distribution)
- Commodity substitution (replacing quality items with inferior)

**Controls**:
- Biometric registration (fingerprint/iris)
- Spot checks (10% of distributions)
- Third-party monitoring
- Whistleblower hotline
- Blockchain verification

---

## 10. Protocol Versioning

**Version Control**:
- Protocols reviewed annually
- Updated based on AAR findings
- Version history maintained
- All changes documented with rationale

**Change Management**:
```json
{
  "protocolVersion": "1.1",
  "previousVersion": "1.0",
  "changesDate": "2025-06-01",
  "changes": [
    {
      "section": "3.2",
      "change": "Reduced rapid assessment deployment time from 48h to 24h",
      "rationale": "Lessons from 2025 Somalia crisis showed faster deployment critical"
    }
  ],
  "approvedBy": "WIA-AGRI Technical Committee"
}
```

---

**Next Phase**: [PHASE-4-INTEGRATION.md](PHASE-4-INTEGRATION.md)

---

弘益人間 · Benefit All Humanity
© 2025 WIA Standards - MIT License
