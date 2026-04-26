# WIA-AUTO-004 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-004
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 11. Data Formats

### 11.1 Vehicle Configuration

```json
{
  "vehicle": {
    "id": "EV-2024-001",
    "manufacturer": "Example Motors",
    "model": "E-Sedan",
    "year": 2024,
    "type": "BEV",
    "battery": {
      "capacity_kwh": 75.0,
      "usable_capacity_kwh": 72.0,
      "chemistry": "NMC811",
      "voltage_nominal": 355,
      "voltage_max": 403,
      "voltage_min": 240,
      "cell_type": "cylindrical_21700",
      "configuration": "96s4p",
      "cooling_type": "liquid"
    },
    "motor": {
      "type": "PMSM",
      "count": 1,
      "location": "rear",
      "power_rated_kw": 150,
      "power_peak_kw": 200,
      "torque_rated_nm": 310,
      "torque_peak_nm": 400,
      "max_rpm": 14000,
      "efficiency_percent": 96
    },
    "powertrain": {
      "inverter_type": "SiC",
      "inverter_efficiency_percent": 98,
      "gearbox_type": "single_speed",
      "gearbox_ratio": 9.0,
      "gearbox_efficiency_percent": 97,
      "drivetrain": "RWD"
    },
    "vehicle_specs": {
      "mass_kg": 1800,
      "drag_coefficient": 0.24,
      "frontal_area_m2": 2.3,
      "rolling_resistance": 0.008,
      "tire_radius_m": 0.34
    },
    "charging": {
      "ac_max_kw": 11,
      "dc_max_kw": 150,
      "ac_connector": "Type2",
      "dc_connector": "CCS2"
    }
  }
}
```

### 11.2 Drive Profile

```json
{
  "drive_profile": {
    "timestamp": "2024-12-26T10:00:00Z",
    "duration_seconds": 3600,
    "distance_km": 85.3,
    "average_speed_kmh": 85.3,
    "energy_consumed_kwh": 14.5,
    "efficiency_kwh_per_100km": 17.0,
    "conditions": {
      "ambient_temp_c": 22,
      "hvac_power_kw": 1.2,
      "terrain": "rolling",
      "weather": "clear"
    },
    "soc_start_percent": 85,
    "soc_end_percent": 66,
    "regenerative_energy_kwh": 2.1,
    "regenerative_efficiency_percent": 78,
    "segments": [
      {
        "type": "urban",
        "distance_km": 12.5,
        "average_speed_kmh": 42,
        "energy_kwh": 2.1
      },
      {
        "type": "highway",
        "distance_km": 68.8,
        "average_speed_kmh": 110,
        "energy_kwh": 11.2
      },
      {
        "type": "rural",
        "distance_km": 4.0,
        "average_speed_kmh": 68,
        "energy_kwh": 1.2
      }
    ]
  }
}
```

### 11.3 Charging Session

```json
{
  "charging_session": {
    "session_id": "CHG-20241226-123456",
    "start_time": "2024-12-26T14:30:00Z",
    "end_time": "2024-12-26T14:58:00Z",
    "duration_minutes": 28,
    "charger_id": "STATION-042-DC-02",
    "charger_type": "DC_FAST",
    "max_power_kw": 150,
    "standard": "CCS2",
    "energy_delivered_kwh": 52.3,
    "cost_total": 18.31,
    "cost_currency": "USD",
    "soc_start_percent": 18,
    "soc_end_percent": 82,
    "battery_temp_start_c": 28,
    "battery_temp_end_c": 35,
    "power_profile": [
      {"time": "14:30", "power_kw": 145, "soc": 18, "temp_c": 28},
      {"time": "14:35", "power_kw": 148, "soc": 28, "temp_c": 30},
      {"time": "14:40", "power_kw": 142, "soc": 38, "temp_c": 32},
      {"time": "14:45", "power_kw": 125, "soc": 52, "temp_c": 34},
      {"time": "14:50", "power_kw": 95, "soc": 68, "temp_c": 35},
      {"time": "14:55", "power_kw": 62, "soc": 78, "temp_c": 35},
      {"time": "14:58", "power_kw": 42, "soc": 82, "temp_c": 35}
    ],
    "average_power_kw": 112,
    "peak_power_kw": 148,
    "efficiency_percent": 94
  }
}
```

---

## 12. API Interface

### 12.1 Core Functions

#### 12.1.1 Range Calculation

```typescript
interface RangeCalculationRequest {
  batteryCapacity: number;      // kWh
  currentSoC: number;            // %
  averageSpeed: number;          // km/h
  ambientTemp: number;           // °C
  driveProfile: 'urban' | 'highway' | 'mixed';
  hvacEnabled: boolean;
  terrainType: 'flat' | 'rolling' | 'mountainous';
}

interface RangeCalculationResponse {
  estimatedRange: number;        // km
  rangeMin: number;              // km (worst case)
  rangeMax: number;              // km (best case)
  energyConsumption: number;     // kWh/100km
  confidence: number;            // 0-1
  factors: {
    baseConsumption: number;
    temperatureImpact: number;
    hvacImpact: number;
    terrainImpact: number;
  };
}

function calculateRange(request: RangeCalculationRequest): RangeCalculationResponse;
```

#### 12.1.2 Charging Time Estimation

```typescript
interface ChargingTimeRequest {
  batteryCapacity: number;       // kWh
  currentSoC: number;             // %
  targetSoC: number;              // %
  chargerPower: number;           // kW
  chargerType: 'AC_L1' | 'AC_L2' | 'DC_FAST';
  batteryTemp: number;            // °C
}

interface ChargingTimeResponse {
  estimatedTime: number;          // minutes
  energyToDeliver: number;        // kWh
  averagePower: number;           // kW
  chargingCurve: Array<{
    time: number;                 // minutes
    soc: number;                  // %
    power: number;                // kW
  }>;
  preConditionTime?: number;      // minutes (if battery too cold/hot)
}

function calculateChargingTime(request: ChargingTimeRequest): ChargingTimeResponse;
```

#### 12.1.3 Energy Consumption

```typescript
interface EnergyConsumptionRequest {
  distance: number;               // km
  averageSpeed: number;           // km/h
  vehicleMass: number;            // kg
  dragCoefficient: number;        // dimensionless
  frontalArea: number;            // m²
  rollingResistance: number;      // dimensionless
  grade: number;                  // % (positive = uphill)
  acceleration: number;           // m/s²
  ambientTemp: number;            // °C
  hvacPower: number;              // kW
}

interface EnergyConsumptionResponse {
  totalEnergy: number;            // kWh
  energyPerKm: number;            // kWh/km
  breakdown: {
    aerodynamic: number;          // kWh
    rolling: number;              // kWh
    grade: number;                // kWh
    acceleration: number;         // kWh
    hvac: number;                 // kWh
    accessories: number;          // kWh
    drivetrain_loss: number;      // kWh
  };
  efficiency: number;             // % (useful energy / total energy)
}

function calculateEnergyConsumption(request: EnergyConsumptionRequest): EnergyConsumptionResponse;
```

#### 12.1.4 Regenerative Braking

```typescript
interface RegenerativeBrakingRequest {
  vehicleMass: number;            // kg
  initialSpeed: number;           // km/h
  finalSpeed: number;             // km/h
  decelerationRate: number;       // m/s²
  motorEfficiency: number;        // 0-1
  inverterEfficiency: number;     // 0-1
  batteryEfficiency: number;      // 0-1
  currentSoC: number;             // %
  batteryTemp: number;            // °C
}

interface RegenerativeBrakingResponse {
  kineticEnergy: number;          // kWh
  recoverableEnergy: number;      // kWh
  actualRecovered: number;        // kWh
  recoveryEfficiency: number;     // %
  limitations: string[];          // e.g., ["battery_soc_high", "battery_temp_low"]
  rangeExtension: number;         // km
}

function calculateRegenerativeEnergy(request: RegenerativeBrakingRequest): RegenerativeBrakingResponse;
```

### 12.2 Battery Management

```typescript
interface BatteryStatus {
  soc: number;                    // %
  soh: number;                    // %
  voltage: number;                // V
  current: number;                // A
  temperature: number;            // °C
  power: number;                  // kW (positive = discharge, negative = charge)
  maxChargePower: number;         // kW
  maxDischargePower: number;      // kW
  cellVoltages: number[];         // V (per cell/module)
  cellTemperatures: number[];     // °C (per cell/module)
  cycleCount: number;
  estimatedRemainingLife: number; // years
}

interface BatteryHealthPrediction {
  currentSoH: number;             // %
  predictedSoH: Array<{
    years: number;
    soh: number;                  // %
    confidence: number;           // 0-1
  }>;
  degradationRate: number;        // % per year
  factorsInfluencingHealth: {
    temperature_stress: number;   // 0-1 (higher = more stress)
    cycle_depth_stress: number;
    fast_charge_frequency: number;
    calendar_aging: number;
  };
  recommendations: string[];
}

function getBatteryStatus(): BatteryStatus;
function predictBatteryHealth(currentStatus: BatteryStatus, usagePattern: UsagePattern): BatteryHealthPrediction;
```

### 12.3 Motor Control

```typescript
interface MotorCommand {
  torqueRequest: number;          // N·m
  speedLimit: number;             // RPM
  efficiencyMode: 'eco' | 'normal' | 'sport';
}

interface MotorStatus {
  actualTorque: number;           // N·m
  speed: number;                  // RPM
  power: number;                  // kW
  efficiency: number;             // %
  temperature: {
    stator: number;               // °C
    rotor: number;                // °C
    windings: number;             // °C
  };
  current: {
    d_axis: number;               // A
    q_axis: number;               // A
  };
  voltage: {
    d_axis: number;               // V
    q_axis: number;               // V
  };
}

function setMotorCommand(command: MotorCommand): void;
function getMotorStatus(): MotorStatus;
```

---

## 13. Safety Protocols

### 13.1 High Voltage Safety

#### 13.1.1 Isolation Monitoring

**Requirements**:
```
Minimum isolation resistance: 100 Ω/V of system voltage
Example: 400V system → 40 kΩ minimum

Continuous monitoring:
- Measure resistance between HV+ and chassis
- Measure resistance between HV- and chassis
- Alert if <100 Ω/V
- Disconnect HV system if <50 Ω/V (critical fault)
```

**Detection Method**:
```
Inject low-frequency AC signal (1-20 Hz, <1 mA)
Measure current leakage to chassis
Calculate isolation resistance from Ohm's law
Update every 100-500 ms
```

#### 13.1.2 Automatic Disconnection

**Conditions for HV Disconnect**:
1. Isolation fault (R < threshold)
2. Over-voltage (cell voltage > 4.25V for Li-ion)
3. Under-voltage (cell voltage < 2.5V)
4. Over-current (>C-rate limit)
5. Over-temperature (>60°C for most chemistries)
6. Crash detection (accelerometer >5g)
7. Service mode activation
8. Emergency stop button

**Disconnection Mechanism**:
```
Primary: HV contactors (relays) in series with HV+
Backup: Pyrofuses (explosive fuses) for crash scenarios
Response time: <10 ms for crash, <100 ms for electrical faults
```

#### 13.1.3 Service Disconnect

**Manual Service Disconnect**:
- Physical disconnect mechanism
- Removes power from HV system
- Required for service work
- Interlocked (cannot remove while HV active)
- Storage in safe location when removed

### 13.2 Battery Protection

#### 13.2.1 Cell-Level Protection

**Voltage Limits**:
```
Over-voltage:
- Warning: >4.15V (for NMC, 3.6V nominal)
- Error: >4.20V
- Critical: >4.25V (disconnect)

Under-voltage:
- Warning: <3.0V
- Error: <2.7V
- Critical: <2.5V (disconnect, risk of copper dissolution)
```

**Current Limits**:
```
Discharge:
- Continuous: 2-4C (e.g., 150-300 kW for 75 kWh)
- Peak (10s): 4-8C (e.g., 300-600 kW)
- Absolute max: Hardware limit at 10C

Charge:
- Continuous: 0.5-1C (e.g., 37.5-75 kW)
- Peak (DC fast): 1.5-3C (e.g., 112-225 kW, with thermal management)
- Absolute max: Hardware limit at 4C
```

**Temperature Limits**:
```
Discharge Operation:
- Optimal: 20-35°C
- Acceptable: -10 to 50°C
- Limited performance: -20 to -10°C, 50 to 55°C
- Prohibited: <-20°C, >55°C (disconnect)

Charge Operation:
- Optimal: 20-35°C
- Acceptable: 0 to 45°C
- Limited (slow): -10 to 0°C
- Prohibited: <-10°C, >45°C (prevent Li plating and degradation)

Storage:
- Optimal: 20-25°C at 50% SoC
- Acceptable: 0-30°C
```

#### 13.2.2 Thermal Runaway Prevention

**Detection**:
```
Temperature Rise Rate:
- Normal: <1°C/min
- Warning: >2°C/min
- Critical: >5°C/min (possible thermal runaway)

Voltage Drop:
- Sudden drop >0.5V in <1s indicates internal short

Gas Sensors:
- Detect venting gases (electrolyte vapor)
```

**Mitigation**:
```
1. Immediate HV disconnect
2. Activate aggressive cooling (max flow rate)
3. Isolate affected module (if architecture allows)
4. Alert driver and emergency services
5. Prevent propagation to adjacent cells (thermal barriers)
```

**Thermal Barriers**:
```
- Phase change materials between cells/modules
- Mica or ceramic insulation
- Air gaps
- Goal: Delay propagation >5 minutes (allow evacuation)
```

### 13.3 Crash Safety

#### 13.3.1 Crash Detection

**Sensors**:
```
Accelerometers: Detect >5g impact (any direction)
Pressure sensors: Detect cabin intrusion
Roll sensors: Detect rollover

Fusion: Combine multiple signals for robustness
Response time: <2 ms (faster than airbag deployment)
```

**Actions**:
```
1. Fire pyrofuses (disconnect HV battery) <10 ms
2. Open HV contactors <10 ms
3. Disable charging system
4. Activate emergency flashers
5. Unlock doors
6. Cut fuel pump (if PHEV)
7. Send crash notification (if connected)
```

#### 13.3.2 Battery Pack Structural Protection

**Design Requirements**:
```
Side impact: Withstand 50 km/h pole impact
Front/rear: Withstand 60 km/h offset frontal impact
Bottom: Withstand road debris, curb strikes

Methods:
- Reinforced battery enclosure (aluminum or steel)
- Crush zones around pack
- Elevated mounting (ground clearance)
- Underbody shield
```

**Post-Crash Safety**:
```
- HV components de-energized automatically
- Emergency responders can confirm via indicator (e.g., flashing LED)
- Manual disconnect easily accessible and labeled
- First responder guide with HV component locations
```

### 13.4 Electromagnetic Compatibility (EMC)

**Requirements**:
```
Emissions (conducted and radiated):
- Must meet CISPR 25 (automotive EMC standard)
- Frequency range: 150 kHz to 2.5 GHz
- Limit interference with radio, navigation, safety systems

Immunity:
- Withstand external electromagnetic fields
- From: Radio transmitters, power lines, other vehicles
- No malfunction or safety issue

Shielding:
- HV cables: Shielded and grounded
- Inverter: Metal enclosure, EMI filters
- Battery: Conductive enclosure (Faraday cage)
```

### 13.5 Functional Safety (ISO 26262)

**ASIL Levels** (Automotive Safety Integrity Level):
```
ASIL D (highest): HV safety, crash detection, braking
ASIL C: Motor control, battery management
ASIL B: Charging control, thermal management
ASIL A: Infotainment, comfort features
QM (Quality Management): Non-safety features

Requirements:
- Redundant sensors and controllers for ASIL D
- Fail-safe defaults (e.g., open HV contactors on fault)
- Continuous self-diagnostics
- Error logging and reporting
```

### 13.6 Charging Safety

**Grid Fault Detection**:
```
- Ground fault detection (GFCI)
- Over/under voltage detection
- Over-current protection
- Arc fault detection

Response: Immediately stop charging, open contactors
```

**Pilot Signal** (SAE J1772):
```
Function: Communication between vehicle and EVSE
- Charger readiness
- Maximum available current (PWM duty cycle)
- Charging permission (vehicle requests)

Voltage Levels:
- +12V: Standby (charger ready, vehicle not connected)
- +9V: Vehicle detected
- +6V: Vehicle ready, charging authorized
- +3V: Ventilation required (rare, DC fast)
- -12V: Fault condition
```

**Interlock Circuit**:
```
Purpose: Prevent removal of plug while energized
Mechanism: Solenoid locks connector in place during charging
Release: Only when HV de-energized
```

---

## 14. References

### 14.1 Standards and Regulations

**International**:
- ISO 6469: Electric Road Vehicles - Safety Specifications
- ISO 17409: Electric Vehicles - Conductive Power Transfer - Safety Requirements
- ISO 15118: Vehicle-to-Grid Communication Interface
- IEC 61851: Electric Vehicle Conductive Charging System
- IEC 62196: Plugs, Socket-Outlets, Vehicle Connectors and Vehicle Inlets - Conductive Charging

**North America**:
- SAE J1772: AC Level 1 and Level 2 Charging
- SAE J3068: AC Level 2 Charging for Fleets
- SAE J3400: NACS (North American Charging Standard)
- SAE J2954: Wireless Power Transfer
- UL 2202: Electric Vehicle Charging Equipment
- FMVSS 305: Electric Powered Vehicles - Electrolyte Spillage and Electrical Shock Protection

**Europe**:
- ECE R100: Electric Powertrains - Safety Requirements
- EN 62196: Charging Connectors
- VDA: German Association Standards

**China**:
- GB/T 18384: Electric Vehicles - Safety Requirements
- GB/T 20234: Connection Set for Conductive Charging
- GB/T 27930: Communication Protocol

### 14.2 Battery Research

1. Goodenough, J.B., Park, K.S. (2013). "The Li-Ion Rechargeable Battery: A Perspective". *Journal of the American Chemical Society*.

2. Blomgren, G.E. (2017). "The Development and Future of Lithium Ion Batteries". *Journal of The Electrochemical Society*.

3. USABC (United States Advanced Battery Consortium). "Battery Technology Roadmap".


### 14.3 Motor and Power Electronics

1. Chan, C.C., Chau, K.T. (2001). "Modern Electric Vehicle Technology". Oxford University Press.

2. Emadi, A. (2015). "Advanced Electric Drive Vehicles". CRC Press.

3. Krishnan, R. (2010). "Permanent Magnet Synchronous and Brushless DC Motor Drives". CRC Press.

4. Mohan, N., Undeland, T.M. (2007). "Power Electronics: Converters, Applications, and Design". Wiley.

### 14.4 Vehicle Dynamics and Efficiency

1. Gillespie, T.D. (1992). "Fundamentals of Vehicle Dynamics". SAE International.

2. Guzzella, L., Sciarretta, A. (2013). "Vehicle Propulsion Systems". Springer.

3. EPA: "Electric Vehicle and Fuel Cell Vehicle Dynamometer Testing".

### 14.5 Thermal Management

1. Pesaran, A.A. (2001). "Battery Thermal Management in EVs and HEVs". *Advanced Automotive Battery Conference*.


### 14.6 WIA Standards

- WIA-INTENT: Intent-based vehicle control interfaces
- WIA-OMNI-API: Universal API for automotive systems
- WIA-SOCIAL: V2V and V2X communication protocols
- WIA-ENERGY: Smart grid and V2G integration
- WIA-CLIMATE: Carbon accounting and sustainability metrics

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-004 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
