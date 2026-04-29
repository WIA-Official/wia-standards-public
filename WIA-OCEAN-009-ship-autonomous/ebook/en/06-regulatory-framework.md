# Chapter 6: Regulatory and Legal Framework

## Navigating Uncharted Legal Waters

Autonomous ships challenge centuries of maritime law built on the assumption of human crews aboard vessels. International organizations, flag states, and classification societies are developing new regulatory frameworks to enable safe, legally compliant autonomous shipping.

### The International Maritime Organization (IMO)

The IMO, a United Nations specialized agency, governs international shipping regulations.

**IMO MASS (Maritime Autonomous Surface Ships) Framework:**

Adopted by the Maritime Safety Committee (MSC) in June 2021, the framework defines four degrees of autonomy:

```typescript
enum MASSAutonomyLevel {
  DEGREE_ONE = 1,   // Ship with automated processes and decision support
  DEGREE_TWO = 2,   // Remotely controlled ship with seafarers on board
  DEGREE_THREE = 3, // Remotely controlled ship without seafarers on board
  DEGREE_FOUR = 4   // Fully autonomous ship
}

interface MASSVessel {
  imoNumber: number;
  autonomyDegree: MASSAutonomyLevel;

  // Compliance requirements vary by degree
  complianceRequirements: {
    safety: SafetyRequirements;
    security: SecurityRequirements;
    environmental: EnvironmentalRequirements;
    liability: LiabilityRequirements;
  };

  // Operational restrictions
  operationalLimits: {
    geographicRestrictions?: GeoFence[];
    weatherLimits?: WeatherLimits;
    trafficDensityLimits?: TrafficDensityRestriction;
    timeRestrictions?: TimeRestriction;
  };

  // Certification
  certificates: {
    classification: ClassificationCertificate;
    flagState: FlagStateApproval;
    portState: PortStateAcceptance[];
  };
}
```

**Key IMO Conventions Affected by MASS:**

**SOLAS (Safety of Life at Sea):**
- Chapter V: Safety of Navigation
  - Regulation 10: Ships' routing
  - Regulation 19: Carriage requirements for shipborne navigational systems
  - **MASS Impact:** Autonomous navigation systems must provide equivalent safety to traditional crewed navigation

**COLREGS (Collision Regulations):**
- Rule 5: Look-out
- Rule 8: Action to avoid collision
- **MASS Impact:** Autonomous systems must demonstrate compliance with all collision avoidance rules

**STCW (Standards of Training, Certification and Watchkeeping):**
- **MASS Impact:** New certifications required for:
  - Remote operators
  - Shore control center personnel
  - Autonomous system engineers
  - AI safety auditors

**MARPOL (Marine Pollution):**
- **MASS Impact:** Autonomous systems must monitor and prevent pollution, without crew oversight

### Flag State Regulations

Ships register under a flag state, which has jurisdiction over the vessel.

**Leading Flag States for MASS:**

**Norway:**
```typescript
interface NorwegianMASSRegulations {
  authority: "Norwegian Maritime Authority (NMA)";

  approvalProcess: {
    step1: "Concept approval (preliminary design review)";
    step2: "Prototype testing (controlled waters)";
    step3: "Trial operations (limited routes)";
    step4: "Full operational approval";
  };

  requirements: {
    riskAssessment: "Formal Safety Assessment (FSA) required";
    trialing: "Minimum 12 months supervised operation";
    redundancy: "Triple-redundant critical systems";
    cybersecurity: "IEC 62443 compliance";
    remoteControl: "Shore control center approval";
    emergencyResponse: "Demonstrated safe-state capability";
  };

  operationalRestrictions: {
    initialOperations: "Coastal waters only";
    trafficDensity: "Moderate or light traffic";
    weather: "Good visibility (>5nm), sea state <4";
    monitoring: "Continuous shore monitoring required";
  };
}
```

**Singapore:**
```typescript
interface SingaporeMASSFramework {
  authority: "Maritime and Port Authority of Singapore (MPA)";

  maritimeAutonomyTechnologySandbox: {
    purpose: "Facilitate MASS trials in Singapore waters";
    duration: "Up to 2 years";
    requirements: {
      riskAssessment: "Comprehensive hazard analysis";
      trialPlan: "Detailed test scenarios";
      safetyObserver: "Qualified personnel on board during trials";
      insurance: "Adequate coverage for autonomous operations";
    };
  };

  trialAreas: {
    area1: "Southern Singapore waters (designated zone)";
    area2: "Port of Singapore (restricted operations)";
    restrictions: "AIS broadcasting, restricted hours, weather limits";
  };
}
```

**Japan:**
```typescript
interface JapanMASSRegulations {
  authority: "Ministry of Land, Infrastructure, Transport and Tourism (MLIT)";

  guidelines: {
    phase1: "Remote monitoring (Degree 1-2)";
    phase2: "Remote control (Degree 3)";
    phase3: "Full autonomy (Degree 4) - under development";
  };

  requirements: {
    systemVerification: "Third-party verification required";
    operatorQualification: "Licensed ship officers for remote operations";
    communicationBackup: "Multiple redundant communication systems";
    electronicChart: "Dual electronic chart systems";
    cybersecurity: "Guidelines for Cybersecurity Onboard Ships compliance";
  };

  targetRoutes: {
    domestic: "Coastal cargo routes (priority)";
    international: "Asia-Pacific container routes (future)";
  };
}
```

### Classification Societies

Classification societies (DNV, Lloyd's Register, ABS, etc.) provide technical standards and certification.

**DNV (Det Norske Veritas) MASS Classification:**

```typescript
interface DNVMASSClassification {
  className: "AUTONOMOUS";

  notations: {
    "AUTONOMOUS(0)": "Ship with automated processes";
    "AUTONOMOUS(1)": "Remotely controlled, crew on board";
    "AUTONOMOUS(2)": "Remotely controlled, no crew";
    "AUTONOMOUS(3)": "Fully autonomous";
  };

  requirements: {
    design: {
      redundancy: "Critical systems: triple-redundant";
      failSafe: "Defined safe-state for all failure modes";
      cyber: "IEC 62443-3-3 security level 2 minimum";
      humanMachine: "Intuitive operator interface";
    };

    testing: {
      factoryAcceptance: "Full system integration test";
      seaTrials: "Minimum 200 hours supervised operation";
      scenarios: "100+ test scenarios covering all operational conditions";
      validation: "Independent V&V (Verification & Validation)";
    };

    documentation: {
      operatingManual: "Complete autonomous system documentation";
      maintenanceManual: "Preventive and corrective maintenance procedures";
      emergencyProcedures: "Comprehensive emergency response plans";
      training: "Training program for operators and maintainers";
    };
  };

  annualSurveys: {
    systemHealth: "Autonomous system functional tests";
    softwareUpdates: "Review and approval of software changes";
    incidentReview: "Analysis of any operational incidents";
    performanceMetrics: "Safety performance monitoring";
  };
}
```

**Lloyd's Register MASS Code:**

```typescript
interface LRShipCodeForMASSCompliance {
  code: "Lloyd's Register ShipRight Design and Construction (Autonomous Ships)";

  principlesBasedApproach: {
    principle1: "Equivalent level of safety to conventional ships";
    principle2: "Systematic risk-based approach";
    principle3: "Verification through testing and simulation";
    principle4: "Continuous monitoring and improvement";
  };

  technicalRequirements: {
    sensing: {
      sensors: "Multi-sensor fusion (radar, lidar, cameras, AIS)";
      reliability: "99.9% availability for critical sensors";
      coverage: "360° awareness with no blind spots";
      validation: "Sensor accuracy verification in all conditions";
    };

    decisionMaking: {
      colreg: "Demonstrated COLREGs compliance";
      pathPlanning: "Verified safe path generation";
      emergencyResponse: "Automated emergency procedures";
      logging: "Complete decision logging for audit";
    };

    communication: {
      bandwidth: "Sufficient for operational needs";
      latency: "Acceptable for control mode";
      redundancy: "Multiple independent communication paths";
      cybersecurity: "Encrypted, authenticated communication";
    };

    powerAndPropulsion: {
      reliability: "Enhanced reliability for autonomous operation";
      monitoring: "Real-time system health monitoring";
      failover: "Automatic failover to backup systems";
    };
  };
}
```

### Liability and Insurance

Autonomous operation creates new liability questions:

**Who is Liable for an Autonomous Ship Accident?**

```typescript
interface MASSLiabilityFramework {
  potentialLiableParties: {
    shipowner: {
      liability: "Strict liability for vessel operations";
      coverage: "P&I (Protection & Indemnity) insurance";
      limits: "Unlimited for pollution, limited for property damage";
    };

    systemManufacturer: {
      liability: "Product liability for system defects";
      coverage: "Product liability insurance";
      limits: "Contract-based limitations";
    };

    softwareDeveloper: {
      liability: "Negligence if software defects cause accident";
      coverage: "Errors & Omissions (E&O) insurance";
      limits: "Contract-based limitations";
    };

    remoteOperator: {
      liability: "Professional negligence";
      coverage: "Professional indemnity insurance";
      limits: "Similar to ship officers";
    };

    flagState: {
      liability: "Regulatory compliance oversight";
      coverage: "Sovereign immunity (typically)";
    };

    classificationSociety: {
      liability: "Limited (certification errors)";
      coverage: "Professional indemnity";
      limits: "Contractual limitations";
    };
  };

  liabilityDetermination: {
    factors: [
      "Was the autonomous system operating as designed?",
      "Did remote operators follow procedures?",
      "Were there software or hardware defects?",
      "Did the shipowner maintain the system properly?",
      "Was the accident foreseeable and preventable?"
    ];

    investigationProcess: {
      step1: "Accident investigation (flag state/coastal state)";
      step2: "Data analysis (VDR/autonomous system logs)";
      step3: "Expert review (technical and legal)";
      step4: "Liability apportionment";
      step5: "Insurance claims and litigation";
    };
  };
}
```

**Insurance Products for MASS:**

```typescript
interface MASSInsuranceProducts {
  hullAndMachinery: {
    coverage: "Physical damage to vessel and equipment";
    premiums: "10-30% lower than crewed vessels (fewer crew accidents)";
    exclusions: "Cyber attacks may be excluded or separately covered";
  };

  protectionAndIndemnity: {
    coverage: "Third-party liability (collision, pollution, cargo)";
    premiums: "Variable based on autonomy degree and operational record";
    requiredData: "Detailed operational logs, system performance data";
  };

  cyberRisk: {
    coverage: "Losses from cyber attacks, system hacking";
    premiums: "Significant (new risk area)";
    requirements: "Cybersecurity audit, compliance with standards";
  };

  lossOfHire: {
    coverage: "Income loss during repairs or system failures";
    premiums: "Similar to conventional vessels";
    considerations: "System downtime may differ from mechanical failures";
  };
}
```

### Cybersecurity Regulations

Autonomous ships are vulnerable to cyber attacks.

**IMO Resolution MSC.428(98) - Maritime Cyber Risk Management:**

```typescript
interface MaritimeCyberSecurityRequirements {
  objective: "Safeguard shipping from current and emerging cyber threats";

  riskManagement: {
    identify: "Identify systems and data that need protection";
    protect: "Implement protective measures";
    detect: "Detect cybersecurity events";
    respond: "Respond to and recover from incidents";
    recover: "Restore capabilities after incident";
  };

  specificMASSRequirements: {
    systemSegmentation: {
      description: "Isolate critical systems from non-critical";
      implementation: "Network segmentation, air gaps for critical systems";
      example: "Navigation system isolated from crew WiFi network";
    };

    accessControl: {
      description: "Restrict access to critical systems";
      implementation: "Multi-factor authentication, role-based access";
      example: "Remote operators require certificate + password + token";
    };

    encryption: {
      description: "Encrypt data in transit and at rest";
      implementation: "AES-256 for data, TLS 1.3 for communications";
      example: "Satellite commands encrypted end-to-end";
    };

    intrusionDetection: {
      description: "Monitor for unauthorized access attempts";
      implementation: "IDS/IPS systems, anomaly detection";
      example: "Alert if unusual GPS position jump detected";
    };

    softwareIntegrity: {
      description: "Ensure software hasn't been tampered with";
      implementation: "Code signing, integrity checks";
      example: "Autonomous navigation software digitally signed";
    };

    incidentResponse: {
      description: "Plan for responding to cyber incidents";
      implementation: "Incident response team, procedures, drills";
      example: "Procedure for isolating compromised systems";
    };
  };
}
```

**IEC 62443 - Industrial Cybersecurity:**

```typescript
interface IEC62443Compliance {
  standard: "IEC 62443 - Security for industrial automation and control systems";

  securityLevels: {
    SL1: "Protection against casual or coincidental violation";
    SL2: "Protection against intentional violation using simple means";
    SL3: "Protection against intentional violation using sophisticated means";
    SL4: "Protection against intentional violation using sophisticated means with extended resources";
  };

  massRecommendation: "SL2 minimum, SL3 for critical systems";

  requirements: {
    SL2: {
      authentication: "User authentication required";
      authorization: "Role-based access control";
      dataIntegrity: "Detection of corrupted data";
      logging: "Audit logs of security events";
    };

    SL3: {
      authentication: "Multi-factor authentication";
      authorization: "Least privilege principle";
      encryption: "Encrypted communications";
      intrusionDetection: "Automated intrusion detection";
      softwareUpdates: "Secure software update mechanism";
    };
  };
}
```

### Port State Control

Ports have authority to inspect ships calling at their harbors.

**Port State Control for MASS:**

```typescript
interface PortStateMASSInspection {
  inspectionAuthority: "Port state control officer (PSCO)";

  massSpecificChecks: {
    documentation: {
      flagStateApproval: "Verify autonomous operation approval";
      classificationCertificate: "Verify MASS class notation";
      operatorCertification: "Verify remote operator qualifications";
      insuranceCertificate: "Verify adequate coverage for autonomous ops";
    };

    systemFunctional: {
      sensorVerification: "Test radar, lidar, cameras operational";
      communicationTest: "Verify shore communication functional";
      emergencyProcedures: "Review emergency shutdown capability";
      cybersecurity: "Verify security measures in place";
    };

    operationalRecords: {
      incidentLogs: "Review any incidents during voyage";
      systemLogs: "Check for system anomalies or failures";
      maintenanceRecords: "Verify system maintenance up to date";
    };
  };

  detentionCriteria: {
    reasonsForDetention: [
      "Inadequate documentation",
      "Critical system failures",
      "Cybersecurity vulnerabilities",
      "Insufficient operator qualifications",
      "Non-compliance with flag state restrictions",
      "Evidence of unsafe operations"
    ];
  };
}
```

### National Legislation Examples

**United States - AUTONOMOUS Act (Proposed):**

```typescript
interface USAutonomousShippingLegislation {
  fullName: "Advancing U.S. Technology to Optimize Navigation and Operations for Maritime Ships Act";
  status: "Proposed legislation (2024)";

  keyProvisions: {
    coastGuardAuthority: "U.S. Coast Guard to develop MASS regulations";
    testingZones: "Designated waters for MASS trials";
    federalPreemption: "Federal regulations preempt state/local rules";
    reportToC Congress: "Annual reports on MASS safety and economic impact";
  };

  requirements: {
    usFlagVessels: {
      approval: "Coast Guard approval required for autonomous operations";
      operators: "U.S. licensed mariners for remote operation";
      buildRequirement: "Jones Act compliance (U.S. built, flagged, crewed)";
    };

    foreignFlagVessels: {
      portAccess: "Subject to port state control";
      restrictions: "May be restricted from certain ports/routes";
    };
  };
}
```

**European Union - MASS Regulations:**

```typescript
interface EUMASSRegulations {
  framework: "EU Maritime Safety Policy";

  keyDirectives: {
    portStateControl: "Directive 2009/16/EC (amended for MASS)";
    vesselTrafficMonitoring: "Directive 2002/59/EC (SafeSeaNet integration)";
    marineEquipment: "Directive 2014/90/EU (autonomous systems approval)";
  };

  requirements: {
    euWatersOperation: {
      notification: "Prior notification to coastal states";
      monitoring: "AIS/LRIT mandatory";
      environmentalCompliance: "Full MARPOL compliance";
    };

    dataSharing: {
      safeSeaNet: "Real-time position and status reporting";
      incidentReporting: "Immediate reporting of incidents";
      performanceData: "Aggregated safety data for EU analysis";
    };
  };
}
```

### Future Regulatory Developments

**IMO Roadmap (2025-2030):**

```typescript
interface IMOMASSRoadmap {
  phase1_2025: {
    goals: [
      "Complete regulatory scoping exercise",
      "Identify required amendments to IMO conventions",
      "Develop interim guidelines for MASS trials"
    ];
  };

  phase2_2026_2028: {
    goals: [
      "Adopt amendments to SOLAS, COLREGS, STCW",
      "Develop MASS Code (similar to Polar Code)",
      "Establish performance standards for autonomous systems"
    ];
  };

  phase3_2029_2030: {
    goals: [
      "Entry into force of MASS regulations",
      "Global harmonization of flag state requirements",
      "International MASS certification framework"
    ];
  };
}
```

### Philosophy: 弘益人間

Sound regulation embodies 弘益人間 by:
- Ensuring autonomous ships are as safe as (or safer than) crewed vessels
- Protecting seafarers, passengers, and coastal communities
- Enabling technological progress while managing risks
- Creating international standards that benefit all nations
- Building public trust in autonomous maritime technology

---

**Next Chapter:** Commercial Autonomous Vessels - container ships, tankers, and bulk carriers going autonomous.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
