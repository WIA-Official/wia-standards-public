# Chapter 3: WIA E-Waste Management Standard Overview

## Learning Objectives

After completing this chapter, you will be able to:

1. Describe the WIA 4-phase architecture for e-waste management
2. Understand the product lifecycle tracking framework
3. Identify compliance levels and certification requirements
4. Explain stakeholder roles and responsibilities
5. Map WIA standards to existing regulatory frameworks

---

## 3.1 Standard Architecture

### 3.1.1 The WIA 4-Phase Framework

The WIA E-Waste Management Standard follows the proven 4-phase architecture applied across all WIA standards:

```
WIA E-Waste Management Standard Architecture:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  PHASE 1: DATA FORMAT                                              │
│  ├─ Device identification schema                                   │
│  ├─ Material composition records                                   │
│  ├─ Chain of custody data model                                    │
│  └─ Compliance documentation format                                │
│                                                                     │
│  PHASE 2: API INTERFACE                                            │
│  ├─ Device registration API                                        │
│  ├─ Collection event API                                           │
│  ├─ Processing facility API                                        │
│  ├─ Material recovery reporting API                                │
│  └─ Regulatory compliance API                                      │
│                                                                     │
│  PHASE 3: PROTOCOL                                                 │
│  ├─ Chain of custody protocol                                      │
│  ├─ Material handling specifications                               │
│  ├─ Hazardous substance protocols                                  │
│  ├─ Quality certification protocol                                 │
│  └─ Audit and verification protocol                                │
│                                                                     │
│  PHASE 4: SYSTEM INTEGRATION                                       │
│  ├─ Producer responsibility systems                                │
│  ├─ Collection network integration                                 │
│  ├─ Recycling facility systems                                     │
│  ├─ Regulatory reporting systems                                   │
│  └─ Circular economy platforms                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.1.2 Core Design Principles

```typescript
// WIA E-Waste Management design principles
interface DesignPrinciples {
  traceability: {
    principle: "Every device trackable from production to material recovery";
    implementation: {
      uniqueIdentifier: "WIA Device ID (WDID) for every product";
      lifecycleEvents: "Standardized event logging at each transfer";
      materialFlow: "Weight and composition tracking at processing";
    };
  };

  interoperability: {
    principle: "Any compliant system can exchange data with any other";
    implementation: {
      commonFormats: "JSON/XML schemas for all data types";
      restfulApis: "Standard API endpoints and authentication";
      protocolAdapters: "Bridges to existing systems (EPR, ERP)";
    };
  };

  transparency: {
    principle: "Stakeholders can verify claims independently";
    implementation: {
      publicRegistry: "Facility certifications publicly accessible";
      auditTrails: "Immutable records of all transactions";
      statisticsApi: "Aggregated data for research and reporting";
    };
  };

  scalability: {
    principle: "Works for small collectors to global manufacturers";
    implementation: {
      tieredCompliance: "Basic, Standard, Premium compliance levels";
      cloudNative: "Scalable infrastructure";
      offlineCapable: "Sync-when-connected for rural operations";
    };
  };

  circularEconomy: {
    principle: "Prioritize reuse and refurbishment over recycling";
    implementation: {
      conditionGrading: "Standardized grading for reuse potential";
      partsCatalog: "Harvested parts inventory";
      refurbishmentTracking: "Second life products tracked";
    };
  };
}
```

### 3.1.3 Standard Scope

| Included | Excluded (Future Phases) |
|----------|-------------------------|
| Consumer electronics (phones, computers, TVs) | Industrial equipment |
| Household appliances (small and large) | Medical devices (specialized protocols) |
| IT equipment (servers, networking) | Nuclear/radioactive equipment |
| Batteries (all types) | Vehicles (separate automotive standard) |
| Lighting equipment | Military equipment |
| Cables and accessories | |

---

## 3.2 Product Lifecycle Framework

### 3.2.1 Lifecycle Stages

```
E-Waste Lifecycle Stages:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  STAGE 1: PRODUCTION                                                │
│  ├─ Material composition recorded                                   │
│  ├─ WIA Device ID assigned                                         │
│  ├─ Hazardous substance declaration                                │
│  └─ Recyclability assessment                                       │
│                                                                     │
│  STAGE 2: DISTRIBUTION & SALE                                      │
│  ├─ First owner registration (optional)                            │
│  ├─ Warranty information linked                                    │
│  └─ Extended producer responsibility fee recorded                  │
│                                                                     │
│  STAGE 3: USE PHASE                                                │
│  ├─ Repair events tracked                                          │
│  ├─ Ownership transfers logged                                     │
│  └─ Condition updates recorded                                     │
│                                                                     │
│  STAGE 4: END OF FIRST LIFE                                        │
│  ├─ Collection event recorded                                      │
│  ├─ Condition assessment                                           │
│  └─ Routing decision (reuse/refurbish/recycle)                     │
│                                                                     │
│  STAGE 5A: REUSE/REFURBISHMENT                                     │
│  ├─ Repair/upgrade recorded                                        │
│  ├─ Quality certification                                          │
│  └─ Return to Stage 2 (Distribution)                               │
│                                                                     │
│  STAGE 5B: RECYCLING                                               │
│  ├─ Facility transfer recorded                                     │
│  ├─ Disassembly and sorting documented                             │
│  ├─ Material recovery measured                                     │
│  └─ Hazardous treatment verified                                   │
│                                                                     │
│  STAGE 6: MATERIAL RECOVERY                                        │
│  ├─ Recovered material quantities                                  │
│  ├─ Quality certification                                          │
│  ├─ Residual waste documentation                                   │
│  └─ Closure of device lifecycle                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2.2 Lifecycle Event Model

```typescript
// Core lifecycle event structure
interface LifecycleEvent {
  // Event identification
  eventId: string;              // UUID
  eventType: EventType;
  timestamp: string;            // ISO 8601
  timezone: string;

  // Device reference
  deviceId: string;             // WIA Device ID
  deviceCategory: DeviceCategory;

  // Actor information
  actor: {
    actorId: string;            // WIA registered entity
    actorType: "producer" | "retailer" | "collector" | "processor" | "consumer";
    facilityId?: string;        // For facility-based operations
    location: GeoLocation;
  };

  // Event-specific data
  eventData: EventSpecificData;

  // Chain of custody
  previousEvent?: string;       // Link to prior event
  custodyTransfer?: {
    fromActor: string;
    toActor: string;
    transferDocument?: string;
  };

  // Verification
  verification: {
    method: "self-declared" | "witnessed" | "automated" | "audited";
    verifier?: string;
    evidence?: string[];        // Photos, documents, sensor data
  };

  // Digital signature
  signature: {
    algorithm: "ES256" | "RS256";
    value: string;
    publicKey: string;
  };
}

// Event types enumeration
type EventType =
  | "production"
  | "distribution"
  | "sale"
  | "ownership_transfer"
  | "repair"
  | "collection"
  | "assessment"
  | "refurbishment"
  | "facility_transfer"
  | "disassembly"
  | "material_recovery"
  | "hazardous_treatment"
  | "final_disposal"
  | "lifecycle_closure";
```

### 3.2.3 Material Flow Tracking

```
Material Flow Through Processing:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  INPUT                                                              │
│  └─ Collected devices (by weight, count, category)                  │
│                                                                     │
│       ▼                                                             │
│  ┌─────────────────────────────────────────┐                       │
│  │          PRE-PROCESSING                  │                       │
│  │  ├─ Sorting by category                 │                       │
│  │  ├─ Hazard identification               │                       │
│  │  ├─ Reuse assessment                    │                       │
│  │  └─ Routing decision                    │                       │
│  └─────────────────────────────────────────┘                       │
│       │                                                             │
│       ├──► Reuse Stream (functional devices)                       │
│       │                                                             │
│       ▼                                                             │
│  ┌─────────────────────────────────────────┐                       │
│  │          DISASSEMBLY                     │                       │
│  │  ├─ Manual component removal            │                       │
│  │  ├─ Hazardous component separation      │                       │
│  │  └─ Material stream creation            │                       │
│  └─────────────────────────────────────────┘                       │
│       │                                                             │
│       ├──► Parts for Reuse                                         │
│       │                                                             │
│       ▼                                                             │
│  ┌─────────────────────────────────────────┐                       │
│  │          PROCESSING                      │                       │
│  │  ├─ Shredding                           │                       │
│  │  ├─ Magnetic separation                 │                       │
│  │  ├─ Eddy current separation             │                       │
│  │  ├─ Optical sorting                     │                       │
│  │  └─ Density separation                  │                       │
│  └─────────────────────────────────────────┘                       │
│       │                                                             │
│       ▼                                                             │
│  OUTPUT STREAMS                                                     │
│  ├─ Ferrous metals (steel, iron)                                   │
│  ├─ Non-ferrous metals (copper, aluminum)                          │
│  ├─ Precious metals (gold, silver, palladium)                      │
│  ├─ Plastics (by type)                                             │
│  ├─ Glass                                                          │
│  ├─ Hazardous fractions (treated separately)                       │
│  └─ Residuals (final disposal)                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3.3 Compliance Framework

### 3.3.1 Compliance Levels

```typescript
// Compliance tier definitions
interface ComplianceLevels {
  basic: {
    name: "WIA E-Waste Basic";
    targetAudience: "Small collectors, informal-to-formal transition";
    requirements: {
      registration: "Entity registered in WIA system";
      deviceTracking: "Batch-level tracking (weight-based)";
      reporting: "Monthly volume reports";
      hazardous: "Basic hazardous identification";
      verification: "Self-declaration with spot audits";
    };
    benefits: [
      "Participation in formal system",
      "Basic compliance documentation",
      "Access to collection network"
    ];
  };

  standard: {
    name: "WIA E-Waste Standard";
    targetAudience: "Commercial collectors, mid-size processors";
    requirements: {
      registration: "Full facility audit";
      deviceTracking: "Device-level tracking for high-value items";
      reporting: "Weekly reports, real-time for transfers";
      hazardous: "Full hazardous substance protocols";
      materialRecovery: "Material balance reporting";
      verification: "Annual third-party audit";
    };
    benefits: [
      "Producer contract eligibility",
      "Regulatory pre-approval in partner jurisdictions",
      "Material quality certification"
    ];
  };

  premium: {
    name: "WIA E-Waste Premium";
    targetAudience: "Large processors, integrated recyclers, exporters";
    requirements: {
      registration: "Comprehensive facility certification";
      deviceTracking: "100% device-level tracking with IoT integration";
      reporting: "Real-time event streaming";
      hazardous: "Advanced treatment verification";
      materialRecovery: "Verified recovery rates with mass balance";
      downstream: "Full downstream due diligence";
      verification: "Continuous monitoring, quarterly audits";
    };
    benefits: [
      "Premium material markets access",
      "Export pre-approval",
      "Blockchain-verified certificates",
      "Circular economy platform integration"
    ];
  };
}
```

### 3.3.2 Certification Requirements

| Requirement | Basic | Standard | Premium |
|-------------|-------|----------|---------|
| Entity registration | ✓ | ✓ | ✓ |
| Facility audit | - | Annual | Quarterly |
| Environmental permits | Applicable national | Full compliance | Full + ISO 14001 |
| Worker safety | Basic training | OSHA/equivalent | OHSAS 18001 |
| Data submission | Monthly batch | Weekly + events | Real-time streaming |
| Device tracking | Weight-based | High-value items | All items |
| Downstream verification | Declaration | Documentation | Full audit trail |
| Financial assurance | - | Partial | Full bonding |
| Technology | Manual entry | API integration | IoT + automation |

### 3.3.3 Audit and Verification

```typescript
// Audit framework
interface AuditFramework {
  auditTypes: {
    registration: {
      purpose: "Initial facility/entity approval";
      scope: "Permits, capabilities, management systems";
      frequency: "One-time (renewal every 3 years)";
    };
    surveillance: {
      purpose: "Ongoing compliance verification";
      scope: "Operations, records, material flows";
      frequency: "Annual (Standard), Quarterly (Premium)";
    };
    transaction: {
      purpose: "Verify specific shipments/processing";
      scope: "Selected transactions, mass balance";
      frequency: "Random sampling";
    };
    complaint: {
      purpose: "Investigate reported issues";
      scope: "Specific allegation";
      frequency: "As needed";
    };
  };

  verificationMethods: {
    documentReview: "Permits, contracts, invoices";
    physicalInspection: "Facility walkthrough, equipment check";
    processObservation: "Watch actual operations";
    recordsAudit: "Cross-check system records with physical";
    massBalance: "Input/output reconciliation";
    sampling: "Material testing for composition/contamination";
    interviews: "Staff interviews on procedures";
    iotVerification: "Sensor data validation";
  };

  nonconformanceHandling: {
    minor: {
      definition: "Administrative gaps, isolated incidents";
      response: "Corrective action within 30 days";
      impact: "Warning recorded";
    };
    major: {
      definition: "Systematic failures, significant gaps";
      response: "Corrective action within 14 days";
      impact: "Conditional certification";
    };
    critical: {
      definition: "Environmental release, fraud, safety hazard";
      response: "Immediate suspension";
      impact: "Certification revoked, regulatory notification";
    };
  };
}
```

---

## 3.4 Stakeholder Roles

### 3.4.1 Stakeholder Ecosystem

```
E-Waste Stakeholder Ecosystem:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  PRODUCERS                                                          │
│  ├─ Assign WIA Device IDs at production                            │
│  ├─ Submit material composition data                               │
│  ├─ Fund collection/recycling (EPR)                                │
│  └─ Design for recyclability                                        │
│                                                                     │
│  RETAILERS/DISTRIBUTORS                                             │
│  ├─ Register first sale                                            │
│  ├─ Operate take-back points                                       │
│  └─ Consumer education                                              │
│                                                                     │
│  CONSUMERS                                                          │
│  ├─ Return devices to collection points                            │
│  ├─ Optional: register devices for tracking                        │
│  └─ Receive recycling certificates                                 │
│                                                                     │
│  COLLECTORS                                                         │
│  ├─ Operate collection points/services                             │
│  ├─ Record collection events                                       │
│  ├─ Initial sorting and routing                                    │
│  └─ Transfer to processors                                         │
│                                                                     │
│  PROCESSORS/RECYCLERS                                               │
│  ├─ Disassembly and material separation                            │
│  ├─ Hazardous substance treatment                                  │
│  ├─ Material recovery and quality certification                    │
│  └─ Residual waste management                                       │
│                                                                     │
│  REFURBISHERS                                                       │
│  ├─ Assess reuse potential                                         │
│  ├─ Repair and upgrade devices                                     │
│  ├─ Quality testing and certification                              │
│  └─ Remarketing                                                     │
│                                                                     │
│  REGULATORS                                                         │
│  ├─ Set collection/recovery targets                                │
│  ├─ Monitor compliance                                             │
│  ├─ Enforce regulations                                            │
│  └─ Accept WIA reports for compliance                               │
│                                                                     │
│  CERTIFICATION BODIES                                               │
│  ├─ Audit facilities                                               │
│  ├─ Issue certifications                                           │
│  └─ Maintain public registry                                       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.4.2 Responsibility Matrix

| Activity | Producer | Collector | Processor | Regulator | WIA |
|----------|----------|-----------|-----------|-----------|-----|
| Device ID assignment | R | - | - | - | S |
| Material composition | R | - | - | - | S |
| Collection event | - | R | - | M | S |
| Custody transfer | I | R | R | M | S |
| Processing documentation | - | - | R | M | S |
| Material recovery reporting | - | - | R | M | A |
| Compliance verification | A | A | A | R | S |
| Standard maintenance | C | C | C | C | R |

*R = Responsible, A = Accountable, S = Support, C = Consulted, I = Informed, M = Monitor*

### 3.4.3 Data Access Rights

| Data Type | Producer | Collector | Processor | Regulator | Public |
|-----------|----------|-----------|-----------|-----------|--------|
| Device composition | Full | Summary | Summary | Full | - |
| Collection volumes | Own | Own | - | Full | Aggregate |
| Processing data | Own | - | Full | Full | Aggregate |
| Custody chain | Own devices | Own transactions | Own transactions | Full | - |
| Recovery rates | - | - | Own | Full | Certified |
| Compliance status | Own | Own | Own | Full | Public |

---

## 3.5 Regulatory Mapping

### 3.5.1 EU WEEE Directive Alignment

```typescript
// Mapping WIA standard to EU WEEE requirements
interface WEEEMapping {
  collectionTargets: {
    weeRequirement: "65% of EEE placed on market or 85% of WEEE generated";
    wiaSupport: "Real-time collection tracking, automated target calculation";
    dataMapping: {
      weeCategory: "WIA device category",
      weightReported: "WIA batch weight records",
      collectionPoint: "WIA facility registration"
    };
  };

  recoveryTargets: {
    weeRequirement: "75-85% recovery by category";
    wiaSupport: "Material balance tracking, recovery rate calculation";
    verification: "Audit trail from input to output materials";
  };

  producerResponsibility: {
    weeRequirement: "Financing of collection and treatment";
    wiaSupport: "EPR fee tracking, material flow attribution to producers";
    reporting: "Automated producer compliance reports";
  };

  treatmentStandards: {
    weeRequirement: "Annex VII treatment requirements";
    wiaSupport: "Facility certification against treatment standards";
    verification: "Process monitoring, hazardous treatment verification";
  };

  transboundaryRules: {
    weeRequirement: "Prior notification for export outside EU";
    wiaSupport: "Export pre-notification API, receiving facility verification";
    documentation: "Chain of custody to final treatment";
  };

  reporting: {
    weeRequirement: "Annual reporting to competent authority";
    wiaSupport: "Automated report generation in EU format";
    frequency: "Real-time data, annual summary";
  };
}
```

### 3.5.2 Multi-Jurisdiction Compliance

| WIA Feature | EU WEEE | US State Laws | Japan HAL | Basel Convention |
|-------------|---------|---------------|-----------|------------------|
| Device tracking | Collection weight | Varies by state | Consumer fee tracking | Export documentation |
| Producer reporting | Automated | Format varies | N/A | Export notifications |
| Facility certification | Treatment standards | R2/e-Stewards accepted | Licensed recyclers | Environmentally sound |
| Export controls | Pre-notification | Limited | Strict | Prior informed consent |
| Recovery rates | Category-specific | Varies | By appliance | N/A |

### 3.5.3 Compliance Automation

```typescript
// Automated compliance report generation
class ComplianceReportGenerator {
  // Generate EU WEEE compliance report
  generateWEEEReport(
    producerId: string,
    reportingPeriod: DateRange,
    memberState: string
  ): WEEEComplianceReport {
    // Fetch data from WIA system
    const placedOnMarket = this.getPlacedOnMarket(producerId, reportingPeriod);
    const collected = this.getCollected(producerId, reportingPeriod);
    const treated = this.getTreated(producerId, reportingPeriod);

    // Map to WEEE categories
    const categoryData = this.mapToWEEECategories(placedOnMarket, collected, treated);

    // Calculate compliance metrics
    const collectionRate = this.calculateCollectionRate(categoryData);
    const recoveryRates = this.calculateRecoveryRates(categoryData);

    // Generate report in member state format
    return {
      producerId,
      reportingPeriod,
      memberState,
      categories: categoryData,
      metrics: {
        collectionRate,
        recoveryRates,
        complianceStatus: this.evaluateCompliance(collectionRate, recoveryRates)
      },
      supportingDocuments: this.generateSupportingDocs(),
      submissionFormat: this.getMemberStateFormat(memberState)
    };
  }

  // Map WIA device categories to WEEE categories
  private mapToWEEECategories(
    pom: DeviceRecord[],
    collected: CollectionEvent[],
    treated: TreatmentRecord[]
  ): WEEECategoryData[] {
    const categoryMap = {
      "smartphone": "IT_TELECOM",
      "laptop": "IT_TELECOM",
      "desktop": "IT_TELECOM",
      "television": "CONSUMER_ELECTRONICS",
      "refrigerator": "LARGE_HOUSEHOLD",
      "washing_machine": "LARGE_HOUSEHOLD",
      "vacuum_cleaner": "SMALL_HOUSEHOLD",
      "led_lamp": "LIGHTING"
      // ... additional mappings
    };

    // Aggregate by WEEE category
    // ... implementation
  }
}
```

---

## 3.6 Review Questions

### Question 1
Describe the four phases of the WIA E-Waste Management Standard and explain how each phase addresses a specific challenge identified in Chapter 2.

### Question 2
A device goes through collection, assessment, refurbishment, and resale. List the lifecycle events that would be recorded and identify the actor responsible for each.

### Question 3
Compare the Basic, Standard, and Premium compliance levels. What factors should a recycling facility consider when choosing which level to pursue?

### Question 4
Explain how the WIA standard supports automated compliance reporting for EU WEEE Directive requirements. What data is needed and how is it mapped?

### Question 5
A producer wants to verify that devices they put on the market in 2024 were properly recycled. Describe the chain of custody data that would allow this verification.

---

## 3.7 Key Takeaways

| Component | Purpose | Key Features |
|-----------|---------|--------------|
| 4-Phase Architecture | Comprehensive coverage | Data, API, Protocol, Integration |
| Lifecycle Framework | End-to-end tracking | 6 stages, event-driven model |
| Compliance Levels | Scalable adoption | Basic, Standard, Premium tiers |
| Stakeholder Roles | Clear responsibilities | RACI matrix for all activities |
| Regulatory Mapping | Multi-jurisdiction compliance | Automated reporting, format conversion |

### Architecture Highlights
- **WIA Device ID (WDID)**: Universal product identifier
- **Event-driven model**: Every lifecycle action recorded
- **Tiered compliance**: Accessible to all operator sizes
- **API-first design**: Interoperability by default
- **Audit trail**: Complete chain of custody

### Next Chapter Preview

Chapter 4 details the data format specifications, including JSON schemas for device records, material composition, lifecycle events, and compliance documentation.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
