# WIA-OCEAN-PLASTIC-TRACK Specification v1.0

> "Hongik Ingan: Every piece of plastic has a story. By tracking it from creation to ocean recovery, we enable humanity to heal the damage and prevent future harm."

## Table of Contents

1. [Introduction](#1-introduction)
2. [Phase 1: Plastic Registration](#2-phase-1-plastic-registration)
3. [Phase 2: Supply Chain Tracking](#3-phase-2-supply-chain-tracking)
4. [Phase 3: Ocean Operations](#4-phase-3-ocean-operations)
5. [Phase 4: Certification & Reporting](#5-phase-4-certification--reporting)
6. [Data Structures](#6-data-structures)
7. [API Reference](#7-api-reference)
8. [Compliance](#8-compliance)

---

## 1. Introduction

### 1.1 Purpose

WIA-OCEAN-PLASTIC-TRACK establishes a comprehensive framework for tracking plastic throughout its lifecycle, from production through ocean recovery and recycling. This standard enables:

- Complete plastic traceability
- Corporate plastic footprint accountability
- Ocean cleanup coordination
- Recycling verification

### 1.2 Scope

```
LIFECYCLE COVERAGE
==================

[PRODUCTION] --> [DISTRIBUTION] --> [CONSUMPTION]
      |                                    |
      v                                    v
[PACKAGING]                          [DISPOSAL]
                                          |
            +------------+----------------+
            |            |                |
            v            v                v
       [RECYCLE]    [LANDFILL]    [OCEAN LEAKAGE]
            ^                              |
            |                              v
            +<---------- [OCEAN RECOVERY] <+
```

### 1.3 Definitions

| Term | Definition |
|------|------------|
| Plastic Footprint | Total plastic produced/consumed by an entity |
| Ocean Positive | More plastic recovered than leaked |
| EPR | Extended Producer Responsibility |
| PCR | Post-Consumer Recycled content |
| PIR | Post-Industrial Recycled content |

---

## 2. Phase 1: Plastic Registration

### 2.1 Production Registration

```typescript
interface PlasticProduction {
  batch_id: WIA_ID;
  producer: {
    id: WIA_ID;
    facility: string;
    country: ISO3166_Country;
  };

  material: {
    type: PlasticType;
    grade: string;
    weight_kg: number;

    composition: {
      virgin_ratio: number;      // 0-1
      pcr_ratio: number;         // Post-Consumer Recycled
      pir_ratio: number;         // Post-Industrial Recycled
      ocean_plastic_ratio: number; // Ocean-recovered plastic
    };

    additives: Additive[];
    colorants: Colorant[];
  };

  environmental: {
    carbon_footprint_kg_co2e: number;
    water_usage_liters: number;
    energy_kwh: number;
    renewable_energy_ratio: number;
  };

  quality: {
    recyclability_score: number;  // 0-100
    degradation_time_years: number;
    marine_safe: boolean;
    certifications: string[];
  };

  timestamp: ISO8601;
  proof: CryptographicProof;
}

enum PlasticType {
  PET = '1',      // Polyethylene Terephthalate
  HDPE = '2',     // High-Density Polyethylene
  PVC = '3',      // Polyvinyl Chloride
  LDPE = '4',     // Low-Density Polyethylene
  PP = '5',       // Polypropylene
  PS = '6',       // Polystyrene
  OTHER = '7',    // Other plastics
  BIO = 'BIO',    // Bioplastics
  OCEAN = 'OCP'   // Ocean-recovered plastic
}
```

### 2.2 Recyclability Assessment

```
RECYCLABILITY SCORING
=====================

[SCORE 90-100] Excellent
  - Mono-material
  - No problematic additives
  - Clear/natural color
  - Established recycling stream

[SCORE 70-89] Good
  - Minor additives
  - Light colors
  - Recyclable in most facilities

[SCORE 50-69] Moderate
  - Multi-layer but separable
  - Some additives
  - Limited recycling options

[SCORE 30-49] Poor
  - Mixed materials
  - Dark colors
  - Difficult to recycle

[SCORE 0-29] Non-recyclable
  - Complex multi-material
  - Contaminated
  - No recycling pathway
```

### 2.3 Production Verification

```typescript
production.verify({
  checks: [
    'material_composition',
    'weight_accuracy',
    'recycled_content_certification',
    'environmental_data'
  ],

  verification_methods: {
    material: 'spectroscopy',
    weight: 'certified_scale',
    recycled_content: 'chain_of_custody',
    environmental: 'lca_audit'
  },

  frequency: 'per_batch',
  auditor: 'third_party_certified'
});
```

---

## 3. Phase 2: Supply Chain Tracking

### 3.1 Product Registration

```typescript
interface PlasticProduct {
  product_id: WIA_ID;

  plastic_components: [{
    batch_id: WIA_ID;           // Links to production batch
    component: string;          // e.g., 'bottle', 'cap', 'label'
    weight_g: number;
    plastic_type: PlasticType;
  }];

  total_plastic_weight_g: number;

  brand: {
    id: WIA_ID;
    name: string;
  };

  category: ProductCategory;

  end_of_life: {
    recyclable: boolean;
    instructions: string;
    collection_program: string;
  };

  qr_code: string;  // Consumer scanning
}
```

### 3.2 Distribution Tracking

```
SUPPLY CHAIN FLOW
=================

[Producer] ---> [Converter] ---> [Brand]
    |               |               |
    v               v               v
 Resin           Product        Packaged
 Pellets         Components     Goods
    |               |               |
    +-------+-------+-------+-------+
            |               |
            v               v
      [Distributor]    [Retailer]
            |               |
            +-------+-------+
                    |
                    v
              [Consumer]
```

### 3.3 Point of Sale Integration

```typescript
interface ConsumerTouchpoint {
  transaction_id: string;
  product_id: WIA_ID;

  location: {
    retailer: WIA_ID;
    country: ISO3166_Country;
    region: string;
  };

  timestamp: ISO8601;

  consumer_info: {
    anonymous_id: string;  // Privacy-preserving
    opted_in_tracking: boolean;
  };

  // Expected end-of-life
  expected_disposal: {
    method: 'recycle' | 'landfill' | 'incineration' | 'unknown';
    collection_available: boolean;
  };
}
```

### 3.4 Disposal Tracking

```typescript
interface DisposalEvent {
  product_id: WIA_ID;

  disposal: {
    method: DisposalMethod;
    facility: WIA_ID | null;
    location: GeoCoordinates;
    timestamp: ISO8601;
  };

  outcome: {
    recycled: boolean;
    recycled_weight_g: number;
    landfilled_weight_g: number;
    incinerated_weight_g: number;
    leaked_weight_g: number;  // Environmental leakage
  };
}

enum DisposalMethod {
  RECYCLING_FACILITY = 'recycled',
  LANDFILL = 'landfill',
  INCINERATION = 'incinerated',
  COMPOSTING = 'composted',
  OCEAN_LEAKAGE = 'ocean_leaked',
  LAND_LEAKAGE = 'land_leaked',
  UNKNOWN = 'unknown'
}
```

---

## 4. Phase 3: Ocean Operations

### 4.1 Ocean Plastic Categories

```
OCEAN PLASTIC CLASSIFICATION
============================

[MACROPLASTIC] > 25mm
  - Bottles, containers
  - Fishing gear
  - Packaging
  - Identifiable items

[MESOPLASTIC] 5mm - 25mm
  - Fragments
  - Caps, lids
  - Partially degraded items

[MICROPLASTIC] < 5mm
  - Pellets (nurdles)
  - Fragments
  - Fibers
  - Difficult to recover

[GHOST GEAR]
  - Abandoned fishing nets
  - Traps, lines
  - FADs (Fish Aggregating Devices)
```

### 4.2 Cleanup Robot Standard

```typescript
interface CleanupRobot {
  robot_id: WIA_ID;

  specifications: {
    type: 'surface' | 'underwater' | 'beach' | 'riverine';
    operator: WIA_ID;
    capacity_kg: number;
    operational_depth_m: number;
    battery_hours: number;
  };

  sensors: {
    plastic_detection: boolean;
    gps: boolean;
    camera: boolean;
    sonar: boolean;
    spectrometer: boolean;
  };

  certifications: string[];

  // Real-time data stream
  telemetry: {
    position: GeoCoordinates;
    status: 'active' | 'collecting' | 'returning' | 'charging' | 'maintenance';
    collected_kg: number;
    battery_percent: number;
  };
}
```

### 4.3 Collection Record

```typescript
interface OceanCollection {
  collection_id: WIA_ID;

  robot_id: WIA_ID;
  operation: {
    start_time: ISO8601;
    end_time: ISO8601;
    area: {
      center: GeoCoordinates;
      radius_km: number;
      ocean_zone: string;  // e.g., 'Pacific Garbage Patch'
    };
  };

  collected: {
    total_weight_kg: number;

    by_type: [{
      type: PlasticType | 'ghost_gear' | 'mixed' | 'unknown';
      weight_kg: number;
      count_estimate: number;
    }];

    by_size: {
      macro_kg: number;
      meso_kg: number;
      micro_kg: number;
    };

    condition: 'fresh' | 'degraded' | 'heavily_degraded';
  };

  identification: {
    brands_identified: [{
      brand: string;
      count: number;
      confidence: number;
    }];

    origin_countries: [{
      country: ISO3166_Country;
      percentage: number;
    }];
  };

  chain_of_custody: {
    collected_by: WIA_ID;
    transported_to: WIA_ID;
    processed_at: WIA_ID;
  };

  photos: string[];
  verification: CryptographicProof;
}
```

### 4.4 Plastic Identification

```typescript
plasticIdentification.configure({
  methods: {
    visual: {
      ai_model: 'plastic_classifier_v3',
      brand_recognition: true,
      confidence_threshold: 0.8
    },

    spectroscopy: {
      nir_analysis: true,
      polymer_identification: true
    },

    barcode: {
      scan_damaged: true,
      database_lookup: true
    }
  },

  attribution: {
    brand_confidence_required: 0.9,
    origin_country_inference: true,
    age_estimation: true
  }
});
```

---

## 5. Phase 4: Certification & Reporting

### 5.1 Recycling Verification

```typescript
interface RecyclingCertificate {
  certificate_id: WIA_ID;

  input: {
    source: 'ocean' | 'post_consumer' | 'post_industrial';
    collection_ids: WIA_ID[];
    total_weight_kg: number;
  };

  process: {
    facility: WIA_ID;
    method: 'mechanical' | 'chemical' | 'hybrid';
    timestamp: ISO8601;
  };

  output: {
    product: 'pellets' | 'flakes' | 'feedstock';
    weight_kg: number;
    quality_grade: string;
    yield_rate: number;
  };

  verification: {
    auditor: WIA_ID;
    method: string;
    date: ISO8601;
  };

  traceability: {
    input_to_output_mapping: boolean;
    mass_balance_verified: boolean;
  };
}
```

### 5.2 Corporate Plastic Footprint

```typescript
interface PlasticFootprint {
  company: WIA_ID;
  period: {
    year: number;
    quarter?: 'Q1' | 'Q2' | 'Q3' | 'Q4';
  };

  production: {
    total_plastic_kg: number;
    by_type: Map<PlasticType, number>;
    virgin_kg: number;
    recycled_kg: number;
    ocean_plastic_kg: number;
  };

  packaging: {
    total_kg: number;
    recyclable_kg: number;
    non_recyclable_kg: number;
  };

  recovery: {
    collected_kg: number;
    recycled_kg: number;
    ocean_recovered_kg: number;
  };

  leakage: {
    estimated_kg: number;
    ocean_leaked_kg: number;
    methodology: string;
  };

  net_plastic: {
    produced: number;
    recovered: number;
    net: number;  // produced - recovered
    ocean_positive: boolean;  // net < 0
  };

  targets: {
    year: number;
    reduction_percent: number;
    recycled_content_percent: number;
    ocean_recovery_kg: number;
  };
}
```

### 5.3 Ocean Positive Certification

```
CERTIFICATION LEVELS
====================

[BRONZE] Ocean Aware
  - Plastic footprint calculated
  - Reduction targets set
  - Some ocean plastic used

[SILVER] Ocean Committed
  - 25%+ recycled content
  - Active collection program
  - Contributing to cleanup

[GOLD] Ocean Positive
  - More recovered than leaked
  - 50%+ recycled content
  - Major cleanup funding

[PLATINUM] Ocean Champion
  - Net zero plastic footprint
  - 75%+ recycled content
  - Industry leadership
```

### 5.4 Certification Process

```typescript
certification.apply({
  company: 'wia:company.brand_001',
  level_requested: 'gold',

  submission: {
    footprint_report: WIA_ID,
    recycled_content_proof: WIA_ID[],
    collection_program_docs: string[],
    cleanup_contributions: WIA_ID[]
  },

  audit: {
    type: 'third_party',
    auditor: 'wia:auditor.certified_001',
    scope: ['production', 'packaging', 'recovery', 'leakage']
  },

  validity: {
    period: '1_year',
    surveillance: 'quarterly',
    recertification: 'annual'
  }
});
```

---

## 6. Data Structures

### 6.1 Global Plastic Registry

```typescript
interface GlobalPlasticRegistry {
  // All registered plastic batches
  productions: Map<WIA_ID, PlasticProduction>;

  // Product catalog
  products: Map<WIA_ID, PlasticProduct>;

  // Ocean collections
  ocean_collections: Map<WIA_ID, OceanCollection>;

  // Recycling certificates
  recycling_certs: Map<WIA_ID, RecyclingCertificate>;

  // Company footprints
  footprints: Map<WIA_ID, PlasticFootprint>;

  // Cleanup robots
  robots: Map<WIA_ID, CleanupRobot>;
}
```

### 6.2 Traceability Chain

```
TRACEABILITY EXAMPLE
====================

Production Batch: batch_001
         |
         v
Product: water_bottle_001
         |
         v
Consumer: purchased_seoul_2025_01_15
         |
         v
Disposal: ocean_leaked (estimated)
         |
         v
Collection: pacific_robot_007_collection_234
         |
         v
Recycling: facility_kr_recycle_cert_567
         |
         v
New Product: recycled_fiber_jacket_890
```

### 6.3 Statistics Aggregation

```typescript
interface GlobalStatistics {
  total_plastic_registered_kg: number;
  total_ocean_collected_kg: number;
  total_recycled_kg: number;

  by_region: Map<string, {
    produced: number;
    leaked: number;
    recovered: number;
  }>;

  by_company: Map<WIA_ID, {
    footprint: number;
    recovery: number;
    net: number;
  }>;

  trends: {
    production_trend: number;  // % change
    leakage_trend: number;
    recovery_trend: number;
  };
}
```

---

## 7. API Reference

### 7.1 Production API

```
POST /api/v1/production
  Register new plastic production batch

GET /api/v1/production/{batch_id}
  Get production details

GET /api/v1/production?producer={id}&from={date}&to={date}
  Query production history
```

### 7.2 Product API

```
POST /api/v1/products
  Register product with plastic components

GET /api/v1/products/{product_id}
  Get product details with plastic breakdown

GET /api/v1/products/{product_id}/lifecycle
  Get full lifecycle from production to disposal
```

### 7.3 Ocean Operations API

```
POST /api/v1/ocean/collections
  Record ocean plastic collection

GET /api/v1/ocean/collections?robot={id}&zone={zone}
  Query collections

GET /api/v1/ocean/robots
  List registered cleanup robots

GET /api/v1/ocean/robots/{robot_id}/telemetry
  Real-time robot status

GET /api/v1/ocean/hotspots
  Get plastic concentration hotspots
```

### 7.4 Certification API

```
POST /api/v1/certification/apply
  Apply for Ocean Positive certification

GET /api/v1/certification/{company_id}
  Get company certification status

POST /api/v1/recycling/verify
  Submit recycling for verification

GET /api/v1/recycling/certificates/{cert_id}
  Get recycling certificate
```

### 7.5 Reporting API

```
GET /api/v1/footprint/{company_id}?year={year}
  Get corporate plastic footprint

GET /api/v1/statistics/global
  Get global statistics

GET /api/v1/statistics/region/{region}
  Get regional statistics

GET /api/v1/leaderboard
  Get ocean positive leaderboard
```

---

## 8. Compliance

### 8.1 Regulatory Alignment

| Regulation | Alignment |
|------------|-----------|
| UNEP Plastic Treaty | Full support for tracking requirements |
| EU Single-Use Plastics Directive | EPR reporting capability |
| Extended Producer Responsibility laws | Footprint calculation |
| National plastic taxes | Data for tax calculation |

### 8.2 Data Standards

- ISO 22095: Chain of custody for plastic
- GS1 standards: Product identification
- W3C Verifiable Credentials: Certificate format
- GeoJSON: Location data

### 8.3 Privacy Requirements

```typescript
privacy.configure({
  consumer_data: {
    collection: 'opt_in_only',
    anonymization: 'required',
    retention: '2_years_max'
  },

  company_data: {
    public_reporting: 'aggregated',
    detailed_data: 'company_access_only',
    audit_access: 'authorized_auditors'
  },

  location_data: {
    precision: 'region_level_public',
    exact: 'authorized_only'
  }
});
```

### 8.4 Verification Requirements

- Production data: Third-party audit annually
- Ocean collections: GPS + photo evidence
- Recycling: Chain of custody + mass balance
- Certifications: Independent auditor verification

---

## Appendix A: Plastic Type Reference

| Code | Type | Common Products | Recyclability |
|------|------|-----------------|---------------|
| 1 | PET | Bottles, containers | High |
| 2 | HDPE | Jugs, pipes | High |
| 3 | PVC | Pipes, cards | Low |
| 4 | LDPE | Bags, films | Moderate |
| 5 | PP | Containers, caps | Moderate |
| 6 | PS | Foam, cups | Low |
| 7 | Other | Mixed | Very Low |

## Appendix B: Ocean Zones

Major accumulation zones covered:
- North Pacific Garbage Patch
- South Pacific Gyre
- North Atlantic Gyre
- South Atlantic Gyre
- Indian Ocean Gyre
- Mediterranean Sea
- Southeast Asian Waters

---

*WIA-OCEAN-PLASTIC-TRACK v1.0 - Healing Our Oceans Together*
