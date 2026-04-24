# WIA-BATTERY-PASSPORT: Phase 1 - Data Format

**EU 배터리 여권 데이터 형식**
*Battery lifecycle tracking for EU Regulation 2027*

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

EU Battery Regulation (2023/1542) mandates digital battery passports for:
- EV batteries > 2 kWh (from 2027)
- Industrial batteries > 2 kWh (from 2027)
- LMT batteries (from 2028)

This specification defines the data format for WIA-compliant battery passports.

---

## 2. Core Data Structures

### 2.1 BatteryPassport (Root)

```typescript
interface BatteryPassport {
  // Identifier
  id: UUID;                          // WIA passport ID
  qr_code: string;                   // Physical QR link
  version: string;                   // Spec version "1.0.0"

  // Timestamps
  created_at: ISO8601;
  updated_at: ISO8601;

  // Core data
  identity: BatteryIdentity;
  manufacturer: ManufacturerInfo;
  specifications: BatterySpecifications;
  materials: MaterialComposition;
  carbon_footprint: CarbonFootprint;
  supply_chain: SupplyChainRecord[];
  health: BatteryHealth;
  lifecycle: LifecycleEvent[];
  recycling: RecyclingInfo;
  certifications: Certification[];

  // Digital signature
  signature: DigitalSignature;
}
```

### 2.2 BatteryIdentity

```typescript
interface BatteryIdentity {
  // EU required identifiers
  unique_identifier: string;         // EU Battery Passport ID
  global_trade_item_number: string;  // GTIN (if applicable)

  // Battery classification
  category: BatteryCategory;
  chemistry: BatteryChemistry;

  // Product info
  model: string;
  serial_number: string;
  batch_number: string;

  // Physical characteristics
  weight_kg: number;
  dimensions: Dimensions;

  // Manufacturing
  production_date: ISO8601;
  production_country: ISO3166Alpha2;
  production_facility_id: string;
}

enum BatteryCategory {
  EV = "ev",                         // Electric Vehicle
  LMT = "lmt",                       // Light Means of Transport
  INDUSTRIAL = "industrial",
  STATIONARY = "stationary",         // Energy Storage Systems
  PORTABLE = "portable",
  SLI = "sli"                        // Starting, Lighting, Ignition
}

enum BatteryChemistry {
  // Lithium-based
  LFP = "lfp",                       // Lithium Iron Phosphate
  NMC = "nmc",                       // Nickel Manganese Cobalt
  NCA = "nca",                       // Nickel Cobalt Aluminum
  LCO = "lco",                       // Lithium Cobalt Oxide
  LMO = "lmo",                       // Lithium Manganese Oxide
  LTO = "lto",                       // Lithium Titanate

  // Solid-state
  SOLID_STATE = "solid_state",

  // Sodium-based
  SODIUM_ION = "sodium_ion",

  // Lead-acid
  LEAD_ACID = "lead_acid",
  AGM = "agm",

  // Other
  NIMH = "nimh",                     // Nickel Metal Hydride
  OTHER = "other"
}

interface Dimensions {
  length_mm: number;
  width_mm: number;
  height_mm: number;
  form_factor: FormFactor;
}

enum FormFactor {
  CYLINDRICAL = "cylindrical",       // 18650, 21700, 4680
  PRISMATIC = "prismatic",
  POUCH = "pouch",
  PACK = "pack"
}
```

### 2.3 ManufacturerInfo

```typescript
interface ManufacturerInfo {
  // Company info
  name: string;
  legal_name: string;
  registration_number: string;       // Business registration

  // Contact
  address: Address;
  contact_email: string;
  contact_phone: string;
  website: string;

  // EU Representative (for non-EU manufacturers)
  eu_representative?: EURepresentative;

  // Facility info
  production_facilities: Facility[];
}

interface EURepresentative {
  name: string;
  address: Address;
  registration_number: string;
}

interface Facility {
  id: string;
  name: string;
  country: ISO3166Alpha2;
  address: Address;
  certifications: string[];          // ISO 14001, etc.
}
```

### 2.4 BatterySpecifications

```typescript
interface BatterySpecifications {
  // Electrical
  nominal_voltage_v: number;
  min_voltage_v: number;
  max_voltage_v: number;
  nominal_capacity_ah: number;       // Ampere-hours
  rated_capacity_wh: number;         // Watt-hours
  energy_density_wh_kg: number;
  power_density_w_kg: number;

  // Performance
  c_rate_charge: number;             // Max charging rate (e.g., 2C)
  c_rate_discharge: number;          // Max discharge rate
  round_trip_efficiency: number;     // % (typically 90-98%)
  self_discharge_rate: number;       // % per month

  // Temperature
  operating_temp_min_c: number;
  operating_temp_max_c: number;
  storage_temp_min_c: number;
  storage_temp_max_c: number;

  // Cycle life
  expected_cycle_life: number;       // Cycles to 80% SOH
  calendar_life_years: number;
  warranty_years: number;
  warranty_cycles: number;

  // Safety
  ip_rating: string;                 // e.g., "IP67"
  un38_3_certified: boolean;
  thermal_runaway_protection: boolean;
}
```

### 2.5 MaterialComposition

```typescript
interface MaterialComposition {
  // Critical raw materials (EU CRM list)
  cobalt: MaterialInfo;
  lithium: MaterialInfo;
  nickel: MaterialInfo;
  manganese: MaterialInfo;
  graphite: MaterialInfo;

  // Recycled content
  recycled_cobalt_percent: number;
  recycled_lithium_percent: number;
  recycled_nickel_percent: number;
  recycled_lead_percent: number;     // For lead-acid

  // Hazardous substances
  hazardous_substances: HazardousSubstance[];

  // Full bill of materials
  bill_of_materials: BOMEntry[];
}

interface MaterialInfo {
  weight_kg: number;
  percentage: number;
  source_country: ISO3166Alpha2[];
  recycled_content_percent: number;
  responsible_sourcing: ResponsibleSourcing;
}

interface ResponsibleSourcing {
  certified: boolean;
  certification_scheme: string;      // "RMI", "IRMA", "ASI", etc.
  certificate_id: string;
  audit_date: ISO8601;
  due_diligence_report_url: string;
}

interface HazardousSubstance {
  name: string;
  cas_number: string;                // Chemical Abstracts Service
  concentration_ppm: number;
  reach_compliant: boolean;          // EU REACH regulation
  rohs_compliant: boolean;           // EU RoHS directive
}

interface BOMEntry {
  component: string;
  material: string;
  weight_kg: number;
  supplier: string;
  country_of_origin: ISO3166Alpha2;
}
```

### 2.6 CarbonFootprint

```typescript
interface CarbonFootprint {
  // Total lifecycle
  total_kg_co2e: number;             // Total CO2 equivalent
  per_kwh_kg_co2e: number;           // Per kWh capacity

  // Breakdown by phase
  raw_material_acquisition: number;   // kg CO2e
  manufacturing: number;
  transport: number;

  // Performance class (EU requirement)
  performance_class: CarbonClass;

  // Calculation methodology
  methodology: string;               // "ISO 14067", "PEF", etc.
  calculation_date: ISO8601;
  third_party_verified: boolean;
  verifier: string;
  verification_report_url: string;

  // Data quality
  data_quality_rating: DataQuality;
}

enum CarbonClass {
  A = "A",    // < 50 kg CO2e/kWh
  B = "B",    // 50-65 kg CO2e/kWh
  C = "C",    // 65-80 kg CO2e/kWh
  D = "D",    // 80-95 kg CO2e/kWh
  E = "E"     // > 95 kg CO2e/kWh
}

enum DataQuality {
  MEASURED = "measured",             // Primary data
  CALCULATED = "calculated",         // Secondary data
  ESTIMATED = "estimated",           // Default values
  MIXED = "mixed"
}
```

### 2.7 BatteryHealth

```typescript
interface BatteryHealth {
  // State indicators
  state_of_health_percent: number;   // SOH (0-100%)
  state_of_charge_percent: number;   // SOC (0-100%)

  // Capacity
  original_capacity_ah: number;
  current_capacity_ah: number;
  capacity_fade_percent: number;

  // Resistance
  internal_resistance_mohm: number;
  resistance_increase_percent: number;

  // Usage statistics
  total_energy_throughput_kwh: number;
  full_cycle_equivalents: number;
  partial_cycles: number;

  // Operating history
  max_temperature_reached_c: number;
  min_temperature_reached_c: number;
  time_above_45c_hours: number;
  time_below_0c_hours: number;

  // Charging history
  fast_charge_count: number;         // DC fast charging
  slow_charge_count: number;         // AC charging
  avg_charge_rate_c: number;

  // Predictions
  remaining_useful_life_months: number;
  expected_eol_date: ISO8601;

  // Last update
  last_bms_sync: ISO8601;
  data_source: HealthDataSource;
}

enum HealthDataSource {
  BMS = "bms",                       // Direct from Battery Management System
  DIAGNOSTIC = "diagnostic",          // Diagnostic tool
  ESTIMATED = "estimated",           // Model-based estimation
  MANUAL = "manual"                  // Manual entry
}
```

### 2.8 LifecycleEvent

```typescript
interface LifecycleEvent {
  id: UUID;
  event_type: LifecycleEventType;
  timestamp: ISO8601;

  // Location
  facility_id?: string;
  country: ISO3166Alpha2;

  // Parties involved
  actor: ActorInfo;

  // Event-specific data
  data: LifecycleEventData;

  // Verification
  verified: boolean;
  verifier?: string;

  // Documentation
  documents: Document[];
}

enum LifecycleEventType {
  // Production
  MANUFACTURED = "manufactured",
  QUALITY_TESTED = "quality_tested",

  // Distribution
  SHIPPED = "shipped",
  RECEIVED = "received",
  SOLD = "sold",

  // First use
  INSTALLED = "installed",
  ACTIVATED = "activated",

  // Service
  MAINTAINED = "maintained",
  REPAIRED = "repaired",
  DIAGNOSED = "diagnosed",
  UPDATED = "updated",               // Firmware/software

  // Ownership
  OWNERSHIP_TRANSFERRED = "ownership_transferred",
  LEASED = "leased",
  RETURNED = "returned",

  // Second life
  REPURPOSED = "repurposed",         // EV → Stationary storage
  RECERTIFIED = "recertified",

  // End of life
  DECOMMISSIONED = "decommissioned",
  COLLECTED = "collected",
  RECYCLED = "recycled",
  DISPOSED = "disposed"
}

interface ActorInfo {
  type: ActorType;
  name: string;
  id: string;                        // Registration/license number
  country: ISO3166Alpha2;
}

enum ActorType {
  MANUFACTURER = "manufacturer",
  DISTRIBUTOR = "distributor",
  DEALER = "dealer",
  OWNER = "owner",
  OPERATOR = "operator",
  SERVICE_CENTER = "service_center",
  RECYCLER = "recycler",
  REGULATOR = "regulator"
}

type LifecycleEventData =
  | ManufacturedData
  | ShippedData
  | InstalledData
  | MaintenanceData
  | RepurposedData
  | RecycledData;

interface ManufacturedData {
  production_line: string;
  quality_grade: string;
  test_results: TestResult[];
}

interface ShippedData {
  origin: Address;
  destination: Address;
  carrier: string;
  tracking_number: string;
  transport_mode: string;
}

interface InstalledData {
  vehicle_vin?: string;              // For EV batteries
  system_id?: string;                // For stationary
  installation_type: string;
}

interface MaintenanceData {
  maintenance_type: string;
  description: string;
  parts_replaced: string[];
  soh_before: number;
  soh_after: number;
}

interface RepurposedData {
  original_application: string;
  new_application: string;
  soh_at_repurpose: number;
  recertification_id: string;
}

interface RecycledData {
  recycler_license: string;
  process_type: string;              // "Pyrometallurgical", "Hydrometallurgical"
  materials_recovered: MaterialRecovery[];
  recovery_efficiency_percent: number;
}

interface MaterialRecovery {
  material: string;
  weight_kg: number;
  purity_percent: number;
}
```

### 2.9 RecyclingInfo

```typescript
interface RecyclingInfo {
  // Recyclability
  recyclability_score: number;       // 0-100
  design_for_recycling: boolean;
  disassembly_manual_url: string;

  // Material recovery targets (EU requirements)
  cobalt_recovery_target: number;    // % (90% by 2031)
  lithium_recovery_target: number;   // % (80% by 2031)
  nickel_recovery_target: number;    // % (90% by 2031)
  copper_recovery_target: number;    // % (90% by 2031)

  // Hazardous material handling
  hazard_classification: string;     // UN hazard class
  special_handling_required: boolean;
  handling_instructions: string;

  // Collection points
  take_back_scheme: string;
  collection_points_url: string;
}
```

### 2.10 Certification

```typescript
interface Certification {
  type: CertificationType;
  name: string;
  issuer: string;
  certificate_id: string;
  issue_date: ISO8601;
  expiry_date: ISO8601;
  verification_url: string;
  status: CertificationStatus;
}

enum CertificationType {
  // Safety
  UN38_3 = "un38_3",                 // UN transport safety
  IEC62619 = "iec62619",             // Safety requirements
  IEC62660 = "iec62660",             // Performance testing
  UL2580 = "ul2580",                 // EV battery safety

  // Quality
  ISO9001 = "iso9001",
  IATF16949 = "iatf16949",           // Automotive quality

  // Environmental
  ISO14001 = "iso14001",
  ISO14067 = "iso14067",             // Carbon footprint

  // Responsible sourcing
  RMI = "rmi",                       // Responsible Minerals Initiative
  IRMA = "irma",                     // Initiative for Responsible Mining
  ASI = "asi",                       // Aluminium Stewardship Initiative

  // EU specific
  CE_MARKING = "ce_marking",
  EU_TYPE_APPROVAL = "eu_type_approval"
}

enum CertificationStatus {
  VALID = "valid",
  EXPIRED = "expired",
  REVOKED = "revoked",
  PENDING = "pending"
}
```

---

## 3. JSON Schema

### 3.1 Complete Battery Passport Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/battery-passport/v1.0.0",
  "title": "WIA Battery Passport",
  "type": "object",
  "required": [
    "id",
    "version",
    "identity",
    "manufacturer",
    "specifications",
    "materials",
    "carbon_footprint",
    "health"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "qr_code": {
      "type": "string",
      "pattern": "^WIABAT:[A-Z0-9]{16}$"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "identity": {
      "$ref": "#/definitions/BatteryIdentity"
    },
    "specifications": {
      "$ref": "#/definitions/BatterySpecifications"
    },
    "materials": {
      "$ref": "#/definitions/MaterialComposition"
    },
    "carbon_footprint": {
      "$ref": "#/definitions/CarbonFootprint"
    },
    "health": {
      "$ref": "#/definitions/BatteryHealth"
    }
  }
}
```

---

## 4. Binary Format (.wiabat)

### 4.1 File Structure

```
┌─────────────────────────────────────┐
│ Magic Number (4 bytes): "WBAT"      │
├─────────────────────────────────────┤
│ Version (2 bytes): 0x0100           │
├─────────────────────────────────────┤
│ Header Length (4 bytes)             │
├─────────────────────────────────────┤
│ Header (variable)                   │
│  - Passport ID (16 bytes UUID)      │
│  - Created timestamp (8 bytes)      │
│  - Checksum (32 bytes SHA-256)      │
├─────────────────────────────────────┤
│ Sections                            │
│  - Identity section                 │
│  - Specs section                    │
│  - Materials section                │
│  - Carbon section                   │
│  - Health section                   │
│  - Lifecycle section                │
├─────────────────────────────────────┤
│ Digital Signature (variable)        │
└─────────────────────────────────────┘
```

### 4.2 Section Format

```
┌─────────────────────────────────────┐
│ Section Type (2 bytes)              │
│  0x0001 = Identity                  │
│  0x0002 = Specifications            │
│  0x0003 = Materials                 │
│  0x0004 = Carbon Footprint          │
│  0x0005 = Health                    │
│  0x0006 = Lifecycle                 │
│  0x0007 = Certifications            │
├─────────────────────────────────────┤
│ Section Length (4 bytes)            │
├─────────────────────────────────────┤
│ Compressed Data (MessagePack + LZ4) │
└─────────────────────────────────────┘
```

---

## 5. QR Code Format

### 5.1 URL Format

```
https://battery.wia.org/p/{passport_id}
```

### 5.2 Compact Data Format (for offline)

```
WIABAT:1.0:{base64url_encoded_essential_data}
```

Essential data includes:
- Passport ID
- Manufacturer
- Chemistry type
- Capacity (Wh)
- SOH percentage
- Carbon class
- Production date

Example:
```
WIABAT:1.0:eyJpZCI6IjAxOTM1YTJiLTNjNGQiLCJtZnIiOiJDQVRMIiwiY2hlbSI6Im5tYyIsImNhcCI6NzUwMDAsInNvaCI6OTgsImNhcmIiOiJCIiwicHJvZCI6IjIwMjUtMDEtMTV9
```

---

## 6. Data Exchange Formats

### 6.1 EU Battery Passport Interoperability

WIA format maps to EU Battery Passport data model:
- Global Battery Alliance (GBA) schema compatible
- Catena-X Battery Pass compatible

### 6.2 Export Formats

| Format | Use Case | Extension |
|--------|----------|-----------|
| JSON | API exchange | .json |
| Binary | Efficient storage | .wiabat |
| PDF | Human-readable report | .pdf |
| XML | Legacy systems | .xml |

---

## 7. Data Integrity

### 7.1 Required Hashes

```typescript
interface DataIntegrity {
  // Overall passport hash
  passport_hash: string;             // SHA-256

  // Section hashes
  identity_hash: string;
  specs_hash: string;
  materials_hash: string;
  carbon_hash: string;
  health_hash: string;
  lifecycle_hash: string;

  // Merkle root for lifecycle events
  lifecycle_merkle_root: string;
}
```

### 7.2 Digital Signature

```typescript
interface DigitalSignature {
  algorithm: "Ed25519" | "ECDSA-P256";
  public_key: string;
  signature: string;
  signed_at: ISO8601;
  signer_did: string;                // DID of manufacturer/authority
}
```

---

## 8. Compliance Mapping

### 8.1 EU Battery Regulation (2023/1542) Mapping

| EU Requirement | WIA Field |
|----------------|-----------|
| Unique identifier | `identity.unique_identifier` |
| Manufacturer info | `manufacturer.*` |
| Carbon footprint | `carbon_footprint.*` |
| Recycled content | `materials.recycled_*_percent` |
| SOH data | `health.state_of_health_percent` |
| Expected lifetime | `specifications.expected_cycle_life` |
| Due diligence | `materials.*.responsible_sourcing` |

---

**Document ID**: WIA-BATTERY-PASSPORT-PHASE-1
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
