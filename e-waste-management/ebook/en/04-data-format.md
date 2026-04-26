# Chapter 4: Data Format Specifications

## Learning Objectives

After completing this chapter, you will be able to:

1. Implement the WIA Device ID (WDID) schema for product identification
2. Design material composition data structures
3. Create lifecycle event records
4. Build chain of custody documentation
5. Generate compliance reporting formats

---

## 4.1 Device Identification Schema

### 4.1.1 WIA Device ID (WDID) Structure

The WIA Device ID provides universal identification for electronic devices throughout their lifecycle:

```typescript
// WIA Device ID structure
interface WIADeviceId {
  // Format: WIA-[PRODUCER]-[CATEGORY]-[YEAR]-[SERIAL]-[CHECK]
  // Example: WIA-SAMSUNG-SM-2024-A1B2C3D4-7X

  prefix: "WIA";                    // Standard prefix
  producerCode: string;             // 3-8 character manufacturer code
  categoryCode: string;             // 2-4 character category code
  productionYear: number;           // 4-digit year
  serialNumber: string;             // 8-12 character unique identifier
  checkDigit: string;               // 2-character verification code
}

// Device ID generation
function generateWDID(params: DeviceParams): string {
  const producerCode = normalizeProducerCode(params.manufacturer);
  const categoryCode = getCategoryCode(params.deviceType);
  const year = new Date().getFullYear();
  const serial = generateSerial(params);
  const check = calculateCheckDigit(producerCode, categoryCode, year, serial);

  return `WIA-${producerCode}-${categoryCode}-${year}-${serial}-${check}`;
}

// Category codes
const CategoryCodes = {
  // IT and Telecommunications
  SM: "Smartphone",
  TB: "Tablet",
  LP: "Laptop",
  DT: "Desktop Computer",
  MN: "Monitor",
  PR: "Printer",
  NW: "Networking Equipment",
  SV: "Server",

  // Consumer Electronics
  TV: "Television",
  AU: "Audio Equipment",
  GM: "Gaming Console",
  CM: "Camera",

  // Large Household
  RF: "Refrigerator",
  WM: "Washing Machine",
  DW: "Dishwasher",
  AC: "Air Conditioner",
  OV: "Oven",

  // Small Household
  VC: "Vacuum Cleaner",
  KA: "Kitchen Appliance",
  PC: "Personal Care",

  // Lighting
  LE: "LED Lighting",
  FL: "Fluorescent Lamp",

  // Batteries
  LI: "Lithium-ion Battery",
  LA: "Lead-acid Battery",
  NC: "Nickel-cadmium Battery"
};
```

### 4.1.2 Device Master Record Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia-standards.org/e-waste/device-master.json",
  "title": "WIA E-Waste Device Master Record",
  "type": "object",
  "required": ["deviceId", "producer", "category", "production", "materials"],
  "properties": {
    "deviceId": {
      "type": "string",
      "pattern": "^WIA-[A-Z0-9]{3,8}-[A-Z]{2,4}-[0-9]{4}-[A-Z0-9]{8,12}-[A-Z0-9]{2}$",
      "description": "WIA Device ID"
    },
    "producer": {
      "type": "object",
      "required": ["id", "name", "country"],
      "properties": {
        "id": {"type": "string", "description": "WIA Producer Registry ID"},
        "name": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"},
        "brand": {"type": "string"},
        "model": {"type": "string"},
        "modelNumber": {"type": "string"}
      }
    },
    "category": {
      "type": "object",
      "required": ["wiaCategory", "weeeCategory"],
      "properties": {
        "wiaCategory": {"type": "string"},
        "weeeCategory": {"type": "integer", "minimum": 1, "maximum": 10},
        "productType": {"type": "string"},
        "subCategory": {"type": "string"}
      }
    },
    "production": {
      "type": "object",
      "required": ["date", "facility"],
      "properties": {
        "date": {"type": "string", "format": "date"},
        "facility": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"},
        "batchNumber": {"type": "string"},
        "serialNumber": {"type": "string"}
      }
    },
    "physical": {
      "type": "object",
      "properties": {
        "weightKg": {"type": "number", "minimum": 0},
        "dimensionsCm": {
          "type": "object",
          "properties": {
            "length": {"type": "number"},
            "width": {"type": "number"},
            "height": {"type": "number"}
          }
        },
        "volumeLiters": {"type": "number"}
      }
    },
    "materials": {
      "$ref": "#/$defs/materialComposition"
    },
    "hazardous": {
      "$ref": "#/$defs/hazardousSubstances"
    },
    "recyclability": {
      "type": "object",
      "properties": {
        "score": {"type": "integer", "minimum": 0, "maximum": 100},
        "grade": {"type": "string", "enum": ["A", "B", "C", "D", "F"]},
        "disassemblyTime": {"type": "integer", "description": "Minutes"},
        "recyclablePercent": {"type": "number"},
        "recoverablePercent": {"type": "number"},
        "notes": {"type": "string"}
      }
    },
    "documentation": {
      "type": "object",
      "properties": {
        "rohsDeclaration": {"type": "string", "format": "uri"},
        "reachCompliance": {"type": "string", "format": "uri"},
        "disassemblyGuide": {"type": "string", "format": "uri"},
        "materialSafetyData": {"type": "string", "format": "uri"}
      }
    }
  }
}
```

### 4.1.3 Producer Registry

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| producerId | string | Yes | WIA registry ID |
| legalName | string | Yes | Legal entity name |
| tradeName | string | No | Trading/brand name |
| country | string | Yes | ISO 2-letter country |
| registrationNumber | string | Yes | National registration |
| eprRegistrations | array | Yes | EPR scheme memberships |
| contactInfo | object | Yes | Contact details |
| certifications | array | No | Active certifications |
| status | string | Yes | active/suspended/revoked |

---

## 4.2 Material Composition Schema

### 4.2.1 Material Breakdown Structure

```typescript
// Material composition interface
interface MaterialComposition {
  // Summary
  totalWeightKg: number;
  measurementMethod: "actual" | "calculated" | "estimated";
  measurementDate: string;
  measurementUncertainty: number;  // Percentage

  // Material breakdown
  materials: MaterialEntry[];

  // Aggregate categories
  aggregates: {
    metals: {
      ferrous: number;           // kg
      nonFerrousBase: number;    // kg (copper, aluminum, zinc)
      preciousMetals: number;    // kg (gold, silver, palladium, platinum)
      rareEarth: number;         // kg
    };
    plastics: {
      total: number;
      byType: PlasticBreakdown[];
    };
    glass: number;
    ceramics: number;
    wood: number;
    other: number;
  };

  // Components
  components: ComponentEntry[];
}

interface MaterialEntry {
  materialId: string;            // WIA material code
  materialName: string;
  category: MaterialCategory;
  weightKg: number;
  percentage: number;
  location: string;              // Where in device
  recyclable: boolean;
  recoveryMethod: string;        // Recommended recovery
  hazardous: boolean;
  notes?: string;
}

interface ComponentEntry {
  componentId: string;
  componentType: string;         // battery, pcb, display, etc.
  description: string;
  weightKg: number;
  materials: MaterialEntry[];
  removable: boolean;
  removalDifficulty: "easy" | "medium" | "difficult" | "destructive";
  hazardousContent: boolean;
  specialHandling?: string;
}
```

### 4.2.2 Material Composition JSON Example

```json
{
  "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
  "materialComposition": {
    "totalWeightKg": 0.187,
    "measurementMethod": "actual",
    "measurementDate": "2024-01-15",
    "measurementUncertainty": 3.0,

    "aggregates": {
      "metals": {
        "ferrous": 0.025,
        "nonFerrousBase": 0.042,
        "preciousMetals": 0.00035,
        "rareEarth": 0.0008
      },
      "plastics": {
        "total": 0.028,
        "byType": [
          {"type": "PC", "weightKg": 0.015},
          {"type": "ABS", "weightKg": 0.008},
          {"type": "TPU", "weightKg": 0.005}
        ]
      },
      "glass": 0.035,
      "ceramics": 0.002,
      "other": 0.054
    },

    "components": [
      {
        "componentId": "BAT-001",
        "componentType": "battery",
        "description": "Lithium-ion battery pack",
        "weightKg": 0.048,
        "materials": [
          {"materialId": "LI-001", "materialName": "Lithium", "weightKg": 0.003},
          {"materialId": "CO-001", "materialName": "Cobalt", "weightKg": 0.008},
          {"materialId": "AL-001", "materialName": "Aluminum", "weightKg": 0.012},
          {"materialId": "CU-001", "materialName": "Copper", "weightKg": 0.006}
        ],
        "removable": false,
        "removalDifficulty": "destructive",
        "hazardousContent": true,
        "specialHandling": "Lithium battery handling protocols required"
      },
      {
        "componentId": "PCB-001",
        "componentType": "pcb",
        "description": "Main logic board",
        "weightKg": 0.025,
        "materials": [
          {"materialId": "AU-001", "materialName": "Gold", "weightKg": 0.00025},
          {"materialId": "AG-001", "materialName": "Silver", "weightKg": 0.0008},
          {"materialId": "PD-001", "materialName": "Palladium", "weightKg": 0.00002},
          {"materialId": "CU-001", "materialName": "Copper", "weightKg": 0.008},
          {"materialId": "SN-001", "materialName": "Tin", "weightKg": 0.002},
          {"materialId": "FR4", "materialName": "FR-4 Substrate", "weightKg": 0.012}
        ],
        "removable": true,
        "removalDifficulty": "medium",
        "hazardousContent": true,
        "specialHandling": "May contain brominated flame retardants"
      },
      {
        "componentId": "DSP-001",
        "componentType": "display",
        "description": "AMOLED display with touch digitizer",
        "weightKg": 0.038,
        "materials": [
          {"materialId": "GL-001", "materialName": "Glass", "weightKg": 0.028},
          {"materialId": "IN-001", "materialName": "Indium", "weightKg": 0.00002},
          {"materialId": "OLED", "materialName": "OLED Materials", "weightKg": 0.003}
        ],
        "removable": true,
        "removalDifficulty": "difficult",
        "hazardousContent": false
      }
    ]
  }
}
```

### 4.2.3 Hazardous Substance Declaration

```typescript
// Hazardous substance tracking
interface HazardousSubstances {
  rohsCompliant: boolean;
  rohsVersion: "2011/65/EU" | "2015/863/EU";
  reachCompliant: boolean;

  declarationDate: string;
  declaringParty: string;
  verificationMethod: "self-declaration" | "third-party-test" | "supplier-data";

  substances: HazardousEntry[];

  exemptions: {
    exemptionId: string;
    substance: string;
    application: string;
    expiryDate?: string;
  }[];
}

interface HazardousEntry {
  substanceId: string;
  substanceName: string;
  casNumber: string;
  category: "RoHS" | "REACH_SVHC" | "POPs" | "other";

  presence: boolean;
  concentration: {
    value: number;
    unit: "ppm" | "percent" | "mg/kg";
    threshold: number;
    compliant: boolean;
  };

  location: string;              // Where in device
  componentId: string;           // Reference to component

  handling: {
    storageRequirements: string;
    disposalMethod: string;
    protectiveEquipment: string[];
    emergencyProcedures: string;
  };
}
```

### 4.2.4 Material Code Reference

| Code | Material | Category | Recovery Method |
|------|----------|----------|-----------------|
| AU-001 | Gold | Precious Metal | Hydrometallurgy |
| AG-001 | Silver | Precious Metal | Hydrometallurgy |
| PD-001 | Palladium | Precious Metal | Hydrometallurgy |
| PT-001 | Platinum | Precious Metal | Hydrometallurgy |
| CU-001 | Copper | Base Metal | Pyrometallurgy |
| AL-001 | Aluminum | Base Metal | Smelting |
| FE-001 | Iron/Steel | Ferrous Metal | Magnetic separation |
| PB-001 | Lead | Hazardous Metal | Specialized treatment |
| HG-001 | Mercury | Hazardous Metal | Distillation |
| CD-001 | Cadmium | Hazardous Metal | Specialized treatment |
| LI-001 | Lithium | Battery Material | Hydrometallurgy |
| CO-001 | Cobalt | Battery Material | Hydrometallurgy |
| ND-001 | Neodymium | Rare Earth | Specialized separation |
| PC-001 | Polycarbonate | Plastic | Mechanical recycling |
| ABS-001 | ABS | Plastic | Mechanical recycling |
| PVC-001 | PVC | Plastic | Specialized (chlorine) |
| BFR-001 | Brominated FR | Hazardous | Incineration |

---

## 4.3 Lifecycle Event Records

### 4.3.1 Event Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia-standards.org/e-waste/lifecycle-event.json",
  "title": "WIA E-Waste Lifecycle Event",
  "type": "object",
  "required": ["eventId", "eventType", "timestamp", "deviceId", "actor"],
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid"
    },
    "eventType": {
      "type": "string",
      "enum": [
        "production", "distribution", "sale", "ownership_transfer",
        "repair", "collection", "assessment", "refurbishment",
        "facility_transfer", "disassembly", "material_recovery",
        "hazardous_treatment", "final_disposal", "lifecycle_closure"
      ]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "timezone": {
      "type": "string"
    },
    "deviceId": {
      "type": "string",
      "description": "WIA Device ID or batch reference"
    },
    "batchReference": {
      "type": "object",
      "properties": {
        "batchId": {"type": "string"},
        "deviceCount": {"type": "integer"},
        "totalWeightKg": {"type": "number"}
      }
    },
    "actor": {
      "type": "object",
      "required": ["actorId", "actorType"],
      "properties": {
        "actorId": {"type": "string"},
        "actorType": {
          "type": "string",
          "enum": ["producer", "retailer", "collector", "processor", "consumer", "regulator"]
        },
        "facilityId": {"type": "string"},
        "operatorId": {"type": "string"},
        "location": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number"},
            "longitude": {"type": "number"},
            "address": {"type": "string"},
            "country": {"type": "string"}
          }
        }
      }
    },
    "eventData": {
      "type": "object",
      "description": "Event-type specific data"
    },
    "custodyTransfer": {
      "type": "object",
      "properties": {
        "fromActor": {"type": "string"},
        "toActor": {"type": "string"},
        "transferType": {
          "type": "string",
          "enum": ["sale", "donation", "collection", "processing", "disposal"]
        },
        "documentReference": {"type": "string"}
      }
    },
    "previousEventId": {
      "type": "string",
      "format": "uuid"
    },
    "verification": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["self_declared", "witnessed", "automated", "audited"]
        },
        "verifierId": {"type": "string"},
        "evidence": {
          "type": "array",
          "items": {"type": "string", "format": "uri"}
        }
      }
    },
    "signature": {
      "type": "object",
      "properties": {
        "algorithm": {"type": "string"},
        "value": {"type": "string"},
        "publicKeyId": {"type": "string"}
      }
    }
  }
}
```

### 4.3.2 Event-Specific Data Schemas

```typescript
// Collection event data
interface CollectionEventData {
  collectionMethod: "drop_off" | "pickup" | "retail_return" | "municipal";
  collectionPointId: string;
  collectionPointType: string;

  deviceCondition: {
    functional: boolean;
    physicalCondition: "excellent" | "good" | "fair" | "poor" | "damaged";
    completeness: "complete" | "missing_accessories" | "missing_parts";
    batteryPresent: boolean;
    dataWiped: boolean | "unknown";
  };

  weight: {
    measured: number;
    unit: "kg";
    method: "scale" | "estimated";
  };

  initialRouting: "reuse" | "refurbishment" | "recycling" | "assessment_needed";

  consumerInfo?: {
    consentGiven: boolean;
    recyclingCertificateRequested: boolean;
    anonymousCollection: boolean;
  };
}

// Processing event data
interface ProcessingEventData {
  processType: "disassembly" | "shredding" | "separation" | "refining";
  facilityId: string;
  processLine: string;

  input: {
    batchId: string;
    weightKg: number;
    itemCount?: number;
    categories: string[];
  };

  output: OutputFraction[];

  processMetrics: {
    duration: number;           // minutes
    energyUsed: number;         // kWh
    waterUsed: number;          // liters
    chemicalsUsed?: ChemicalUsage[];
  };

  qualityControl: {
    samplingDone: boolean;
    contaminationLevel: number; // percentage
    qualityGrade: string;
  };
}

interface OutputFraction {
  fractionId: string;
  materialType: string;
  weightKg: number;
  purity: number;              // percentage
  destination: "sold" | "further_processing" | "disposal";
  nextProcessorId?: string;
  marketValue?: number;
}

// Material recovery event data
interface MaterialRecoveryEventData {
  recoveryProcess: string;
  inputFractions: string[];    // Fraction IDs

  recoveredMaterials: {
    materialCode: string;
    materialName: string;
    weightKg: number;
    purity: number;
    qualityCertification?: string;
    buyer?: string;
    pricePerKg?: number;
  }[];

  residuals: {
    type: string;
    weightKg: number;
    disposalMethod: string;
    disposalFacility?: string;
  }[];

  recoveryRate: {
    overall: number;           // percentage
    byMaterial: {material: string; rate: number}[];
  };

  environmentalMetrics: {
    co2Avoided: number;        // kg CO2e
    energySaved: number;       // kWh
    waterRecycled: number;     // liters
  };
}
```

### 4.3.3 Event Chain Example

```json
{
  "eventChain": [
    {
      "eventId": "evt-001",
      "eventType": "production",
      "timestamp": "2024-01-15T10:30:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "actor": {"actorId": "SAMSUNG-KR", "actorType": "producer"},
      "eventData": {
        "productionFacility": "Samsung Gumi Plant",
        "batchNumber": "2024-01-BATCH-001"
      }
    },
    {
      "eventId": "evt-002",
      "eventType": "sale",
      "timestamp": "2024-02-20T14:00:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "actor": {"actorId": "BESTBUY-US-001", "actorType": "retailer"},
      "previousEventId": "evt-001",
      "custodyTransfer": {
        "fromActor": "SAMSUNG-DISTRIBUTION",
        "toActor": "CONSUMER-ANON-X7Y8"
      }
    },
    {
      "eventId": "evt-003",
      "eventType": "collection",
      "timestamp": "2026-08-15T09:30:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "actor": {"actorId": "BESTBUY-TAKEBACK-001", "actorType": "collector"},
      "previousEventId": "evt-002",
      "eventData": {
        "collectionMethod": "retail_return",
        "deviceCondition": {"functional": false, "physicalCondition": "fair"},
        "initialRouting": "recycling"
      },
      "custodyTransfer": {
        "fromActor": "CONSUMER-ANON-X7Y8",
        "toActor": "BESTBUY-TAKEBACK-001"
      }
    },
    {
      "eventId": "evt-004",
      "eventType": "facility_transfer",
      "timestamp": "2026-08-20T08:00:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "actor": {"actorId": "SIMS-RECYCLING-CA", "actorType": "processor"},
      "previousEventId": "evt-003",
      "custodyTransfer": {
        "fromActor": "BESTBUY-TAKEBACK-001",
        "toActor": "SIMS-RECYCLING-CA"
      }
    },
    {
      "eventId": "evt-005",
      "eventType": "material_recovery",
      "timestamp": "2026-09-01T16:00:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "actor": {"actorId": "SIMS-RECYCLING-CA", "actorType": "processor"},
      "previousEventId": "evt-004",
      "eventData": {
        "recoveredMaterials": [
          {"materialCode": "CU-001", "weightKg": 0.015, "purity": 99.5},
          {"materialCode": "AU-001", "weightKg": 0.00025, "purity": 99.9},
          {"materialCode": "CO-001", "weightKg": 0.008, "purity": 98.0}
        ],
        "recoveryRate": {"overall": 85.3}
      }
    },
    {
      "eventId": "evt-006",
      "eventType": "lifecycle_closure",
      "timestamp": "2026-09-01T17:00:00Z",
      "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
      "previousEventId": "evt-005",
      "eventData": {
        "closureReason": "material_recovery_complete",
        "totalLifecycleDays": 959,
        "materialRecoveryRate": 85.3,
        "certificateId": "WIA-CERT-2026-09-001234"
      }
    }
  ]
}
```

---

## 4.4 Chain of Custody Documentation

### 4.4.1 Custody Transfer Record

```typescript
// Chain of custody record
interface CustodyTransferRecord {
  transferId: string;           // UUID
  timestamp: string;
  timezone: string;

  // Parties
  transferor: {
    entityId: string;
    entityName: string;
    facilityId?: string;
    contactPerson: string;
    role: "producer" | "distributor" | "collector" | "processor" | "transporter";
  };

  transferee: {
    entityId: string;
    entityName: string;
    facilityId?: string;
    contactPerson: string;
    role: "collector" | "processor" | "refurbisher" | "final_disposal";
  };

  // Items transferred
  items: {
    deviceIds?: string[];       // For item-level tracking
    batchId?: string;           // For batch tracking
    category: string;
    description: string;
    quantity: number;
    unit: "items" | "kg";
    estimatedValue?: number;
  };

  // Transportation
  transportation: {
    carrier: string;
    vehicleId: string;
    driverName: string;
    departureTime: string;
    arrivalTime?: string;
    waybillNumber: string;
    route?: string;
  };

  // Compliance
  compliance: {
    hazardousShipment: boolean;
    dotClassification?: string;
    unNumber?: string;
    emergencyContact: string;
    insuranceCoverage: boolean;
  };

  // Signatures
  signatures: {
    transferor: {
      name: string;
      title: string;
      signature: string;        // Digital signature
      timestamp: string;
    };
    transferee: {
      name: string;
      title: string;
      signature: string;
      timestamp: string;
    };
  };

  // Verification
  verification: {
    weightVerified: boolean;
    measuredWeight?: number;
    countVerified?: boolean;
    conditionNotes?: string;
    discrepancies?: string;
    photos?: string[];
  };
}
```

### 4.4.2 Manifest Document Format

| Section | Fields | Required |
|---------|--------|----------|
| Header | Document ID, Date, Type | Yes |
| Generator | Name, Address, EPA ID, Contact | Yes |
| Transporter | Name, EPA ID, Vehicle, Driver | Yes (if applicable) |
| Receiving Facility | Name, Address, EPA ID, Permit | Yes |
| Waste Description | Category, Quantity, Weight, UN# | Yes |
| Handling Codes | DOT Class, Disposal Method | Yes |
| Signatures | Generator, Transporter, Receiver | Yes |
| Emergency Info | Contact, Spill Response | Yes (hazardous) |

---

## 4.5 Compliance Reporting Formats

### 4.5.1 Producer Compliance Report

```typescript
// Producer annual compliance report
interface ProducerComplianceReport {
  reportId: string;
  reportingPeriod: {
    start: string;
    end: string;
  };
  submissionDate: string;
  jurisdiction: string;

  producer: {
    id: string;
    name: string;
    registrationNumber: string;
    eprScheme: string;
  };

  placedOnMarket: {
    byCategory: {
      category: string;
      weeeCategory: number;
      quantity: number;
      weightKg: number;
    }[];
    totalQuantity: number;
    totalWeightKg: number;
  };

  collectionPerformance: {
    byCategory: {
      category: string;
      collectedKg: number;
      target: number;
      achieved: number;        // percentage
    }[];
    totalCollectedKg: number;
    overallRate: number;
  };

  recoveryPerformance: {
    byCategory: {
      category: string;
      recoveredKg: number;
      recycledKg: number;
      recoveryRate: number;
      recyclingRate: number;
      target: {
        recovery: number;
        recycling: number;
      };
      compliant: boolean;
    }[];
  };

  financialContribution: {
    eprFeePaid: number;
    currency: string;
    paymentDates: string[];
    feeCalculationBasis: string;
  };

  certification: {
    certifiedBy: string;
    certificationDate: string;
    auditorName: string;
    auditorRegistration: string;
    nextAuditDue: string;
  };
}
```

### 4.5.2 Processor Compliance Report

```json
{
  "reportId": "PROC-2025-001234",
  "facility": {
    "id": "WIA-FAC-US-CA-001",
    "name": "EcoRecycle California",
    "address": "123 Green Way, Oakland, CA 94601",
    "permits": ["DTSC-HW-001234", "EPA-ID-CAD123456789"]
  },
  "reportingPeriod": {
    "start": "2025-01-01",
    "end": "2025-12-31"
  },

  "inputSummary": {
    "totalWeightKg": 5420000,
    "byCategory": [
      {"category": "IT Equipment", "weightKg": 2150000},
      {"category": "Consumer Electronics", "weightKg": 1830000},
      {"category": "Large Household", "weightKg": 980000},
      {"category": "Small Household", "weightKg": 460000}
    ],
    "sources": [
      {"type": "Producer Programs", "weightKg": 3200000},
      {"type": "Municipal Collection", "weightKg": 1420000},
      {"type": "Commercial", "weightKg": 800000}
    ]
  },

  "outputSummary": {
    "materialRecovered": {
      "ferrousMetals": {"weightKg": 1084000, "purity": "95%+"},
      "aluminum": {"weightKg": 271000, "purity": "90%+"},
      "copper": {"weightKg": 325200, "purity": "98%+"},
      "preciousMetals": {"weightKg": 542, "types": ["Au", "Ag", "Pd"]},
      "plastics": {"weightKg": 813000, "recycled": 650400},
      "glass": {"weightKg": 380000}
    },
    "reused": {
      "devices": 45200,
      "components": 128500,
      "estimatedValueUSD": 2340000
    },
    "hazardousTreated": {
      "batteries": {"weightKg": 162600, "method": "Licensed battery recycler"},
      "crtGlass": {"weightKg": 89000, "method": "Lead smelter"},
      "mercury": {"weightKg": 45, "method": "Distillation recovery"}
    },
    "residualDisposed": {
      "weightKg": 379400,
      "percentage": 7.0,
      "method": "Licensed hazardous waste landfill"
    }
  },

  "performanceMetrics": {
    "overallRecoveryRate": 93.0,
    "recyclingRate": 87.5,
    "reuseRate": 8.2,
    "disposalRate": 7.0,
    "targetMet": true
  }
}
```

---

## 4.6 Review Questions

### Question 1
Design a WIA Device ID for a laptop computer manufactured by Dell in 2025 with serial number XY789012. Explain each component of the ID.

### Question 2
A smartphone weighs 187g and contains 0.25mg of gold, 0.8mg of silver, 3g of lithium, and 8g of cobalt. Create a material composition entry for this device.

### Question 3
Write a complete lifecycle event record for a television being collected at a retail take-back point, including all required fields and appropriate event data.

### Question 4
A batch of 500 mixed IT devices weighing 850kg is transferred from a collector to a processor. Design the chain of custody documentation for this transfer.

### Question 5
A producer placed 100,000 smartphones (15,000 kg) on the EU market and achieved 12,000 kg collection (80%). Create the producer compliance report summary showing target achievement.

---

## 4.7 Key Takeaways

| Data Type | Key Schema Elements | Primary Use |
|-----------|--------------------|--------------
| Device ID (WDID) | Producer-Category-Year-Serial-Check | Universal identification |
| Material Composition | Components, Materials, Hazardous content | Recovery planning |
| Lifecycle Events | Event type, Actor, Timestamp, Chain link | Traceability |
| Chain of Custody | Transferor/ee, Items, Transport, Signatures | Legal compliance |
| Compliance Reports | POM, Collection, Recovery, Financial | Regulatory reporting |

### Schema Design Principles
- **Unique identification**: Every device traceable
- **Composition preservation**: Material data travels with device
- **Event linking**: Unbroken chain from production to recovery
- **Verification support**: Evidence attachable to all records
- **Format flexibility**: JSON/XML interchangeable

### Next Chapter Preview

Chapter 5 covers the API Interface specifications for interacting with the WIA E-Waste Management System, including authentication, endpoints, and integration patterns.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
