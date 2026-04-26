# WIA-AUTO-004 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-004
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

**Single Loop** (simple):
```
Battery → Motor → Inverter → Radiator → Pump → Battery
Pros: Simple, low cost
Cons: Compromised temperatures, single point of failure
```

**Dual Loop** (common):
```
Loop 1: Battery → Battery Chiller/Heater → Pump → Battery
Loop 2: Motor + Inverter → Radiator → Pump → Motor/Inverter
Pros: Optimized temperatures for each component
Cons: Higher complexity and cost
```

**Integrated with HVAC** (premium):
```
Heat Pump System:
- Summer: Battery/Motor heat → Rejected to ambient, Cabin cooled
- Winter: Ambient heat + Waste heat → Battery/Motor warming, Cabin heating
Advantages: Maximum efficiency, fast warm-up
Complexity: High
Examples: Tesla Model Y, BMW iX
```

---

## 8. Range Calculation

### 8.1 Fundamental Range Equation

```
Range (km) = (E_battery × DoD × η_total) / E_consumption

Where:
- E_battery = Total battery capacity (kWh)
- DoD = Usable Depth of Discharge (0.8-0.95)
- η_total = Overall drivetrain efficiency (0.85-0.95)
- E_consumption = Energy consumption per km (kWh/km)
```

**Example** (75 kWh battery, 0.18 kWh/km consumption):
```
Range = (75 × 0.9 × 0.92) / 0.18
      = 62.1 / 0.18
      = 345 km
```

### 8.2 Energy Consumption Modeling

#### 8.2.1 Driving Resistance Forces

**Total Resistance**:
```
F_total = F_aero + F_roll + F_grade + F_accel

Where:
- F_aero = Aerodynamic drag
- F_roll = Rolling resistance
- F_grade = Gravitational resistance on slope
- F_accel = Inertial resistance during acceleration
```

**1. Aerodynamic Drag**:
```
F_aero = 0.5 × ρ × C_d × A × v²

Where:
- ρ = Air density (1.225 kg/m³ at sea level, 15°C)
- C_d = Drag coefficient (0.20-0.35 for EVs)
- A = Frontal area (1.8-2.5 m² for sedans)
- v = Velocity (m/s)

Power required:
P_aero = F_aero × v = 0.5 × ρ × C_d × A × v³

Note: Power increases with cube of velocity!

Example (v = 120 km/h = 33.3 m/s, C_d = 0.24, A = 2.3 m²):
F_aero = 0.5 × 1.225 × 0.24 × 2.3 × 33.3²
       = 313 N
P_aero = 313 × 33.3 = 10.4 kW
```

**2. Rolling Resistance**:
```
F_roll = C_rr × m × g × cos(α)

Where:
- C_rr = Rolling resistance coefficient (0.006-0.012 for EVs)
- m = Vehicle mass (kg)
- g = Gravitational acceleration (9.81 m/s²)
- α = Road grade angle (degrees)

Power required:
P_roll = F_roll × v

Example (m = 1800 kg, C_rr = 0.008, flat road):
F_roll = 0.008 × 1800 × 9.81 × 1
       = 141 N
P_roll (at 120 km/h) = 141 × 33.3 = 4.7 kW
```

**3. Grade Resistance**:
```
F_grade = m × g × sin(α)

Where:
- α = Road grade angle

For small angles: sin(α) ≈ grade (%)
Example: 5% grade ≈ sin(2.86°) ≈ 0.05

Power required:
P_grade = F_grade × v

Example (m = 1800 kg, 5% grade, 50 km/h):
F_grade = 1800 × 9.81 × 0.05 = 883 N
P_grade = 883 × (50/3.6) = 12.3 kW
```

**4. Acceleration Force**:
```
F_accel = m_eff × a

Where:
- m_eff = m × (1 + k_rot)
- k_rot = Rotational inertia factor (0.05-0.1 for EVs)
- a = Acceleration (m/s²)

Power required:
P_accel = F_accel × v

Example (m = 1800 kg, a = 1 m/s², v = 50 km/h):
m_eff = 1800 × 1.08 = 1944 kg
F_accel = 1944 × 1 = 1944 N
P_accel = 1944 × (50/3.6) = 27.0 kW
```

#### 8.2.2 Total Power Requirement

**At Wheels**:
```
P_wheel = (F_aero + F_roll + F_grade + F_accel) × v

Example (120 km/h, flat, steady):
P_wheel = (313 + 141) × 33.3 = 15.1 kW
```

**At Battery** (including losses):
```
P_battery = P_wheel / (η_motor × η_inverter × η_gearbox)

Typical efficiencies:
- η_motor: 0.92-0.96
- η_inverter: 0.96-0.98
- η_gearbox: 0.96-0.98
- Combined: 0.85-0.92

Example:
P_battery = 15.1 / (0.94 × 0.97 × 0.97)
          = 15.1 / 0.885
          = 17.1 kW
```

**Energy Consumption**:
```
E_consumption (kWh/km) = P_battery (kW) / v (km/h)

Example:
E_consumption = 17.1 / 120 = 0.143 kWh/km
              = 14.3 kWh/100km
```

### 8.3 Drive Cycle Analysis

#### 8.3.1 Standard Drive Cycles

**1. EPA Combined** (55% City, 45% Highway):
```
City (FTP-75):
- Average speed: 34 km/h
- Max speed: 91 km/h
- Distance: 17.77 km
- Duration: 31.2 min
- Stops: 23

Highway (HWFET):
- Average speed: 77 km/h
- Max speed: 97 km/h
- Distance: 16.45 km
- Duration: 12.75 min
- Stops: 0

Combined:
E_combined = 0.55 × E_city + 0.45 × E_highway
```

**2. WLTP** (Worldwide Harmonized Light Vehicle Test Procedure):
```
Low: 0-15 min, max 56 km/h
Medium: 15-23 min, max 76 km/h
High: 23-28 min, max 97 km/h
Extra High: 28-30 min, max 131 km/h

More aggressive than EPA → higher consumption
```

**3. NEDC** (obsolete but reference):
```
Less realistic, lower consumption than EPA/WLTP
```

#### 8.3.2 Real-World Factors

**Temperature Effects**:
```
Cabin Heating/Cooling:
- Heating (0°C ambient): +3-7 kW → 30-50% range reduction
- Cooling (35°C ambient): +1-3 kW → 10-20% range reduction

Battery Performance:
- At -10°C: -20% capacity, -30% power → 25-35% range reduction
- At -20°C: -30% capacity, -50% power → 40-50% range reduction
```

**Driving Style**:
```
Aggressive (rapid accel/decel): +20-40% consumption
Moderate: Reference
Eco (smooth, anticipatory): -10-20% consumption
```

**Auxiliary Loads**:
```
Baseline (always on): 200-500 W (computers, lights)
HVAC: 1-7 kW (climate dependent)
Heated seats/steering: 100-300 W
Headlights: 50-150 W (LED)
Infotainment: 50-100 W
```

**Terrain**:
```
Flat: Reference
Rolling hills: +10-20% consumption (energy lost to braking, partial regen recovery)
Mountainous: +20-40% consumption (going up) or -10-20% (going down with regen)
```

**Speed**:
```
Urban (30-50 km/h): Low aero drag, frequent stop/start, moderate consumption
Highway (100-120 km/h): High aero drag, steady speed, moderate-high consumption
Autobahn (150-180 km/h): Very high aero drag (cubic!), highest consumption

Consumption vs. Speed (typical sedan):
50 km/h: 12 kWh/100km
80 km/h: 14 kWh/100km
100 km/h: 16 kWh/100km
120 km/h: 19 kWh/100km
140 km/h: 23 kWh/100km
160 km/h: 28 kWh/100km
```

### 8.4 Range Estimation Algorithm

```python
def calculate_range(battery_capacity, soc, avg_speed, ambient_temp,
                   drive_style, terrain, hvac_use):
    """
    Comprehensive range estimation
    """
    # Base energy available
    energy_available = battery_capacity * (soc / 100) * USABLE_DOD

    # Base consumption from vehicle characteristics
    base_consumption = calculate_base_consumption(avg_speed)

    # Temperature adjustment
    temp_factor = temperature_factor(ambient_temp)

    # HVAC load
    hvac_power = calculate_hvac_power(ambient_temp, hvac_use)
    hvac_consumption = hvac_power / avg_speed

    # Driving style adjustment
    style_factor = {
        'eco': 0.85,
        'normal': 1.0,
        'sport': 1.25
    }[drive_style]

    # Terrain adjustment
    terrain_factor = {
        'flat': 1.0,
        'rolling': 1.15,
        'mountainous': 1.30
    }[terrain]

    # Total consumption
    total_consumption = (base_consumption * temp_factor * style_factor *
                        terrain_factor + hvac_consumption)

    # Estimated range
    estimated_range = energy_available / total_consumption

    # Confidence interval
    uncertainty = 0.15  # ±15%
    range_min = estimated_range * (1 - uncertainty)
    range_max = estimated_range * (1 + uncertainty)

    return {
        'range': estimated_range,
        'range_min': range_min,
        'range_max': range_max,
        'consumption': total_consumption,
        'energy_available': energy_available
    }
```

---

## 9. Energy Efficiency

### 9.1 Powertrain Efficiency

**Component Efficiencies**:
```
Battery discharge efficiency: 95-98%
Inverter efficiency: 95-98%
Motor efficiency: 90-97%
Gearbox efficiency: 96-98%
Differential efficiency: 97-99%

Tank-to-Wheel Efficiency: 85-95%

vs. ICE Vehicle:
Engine efficiency: 20-30% (gasoline), 30-40% (diesel)
Transmission efficiency: 85-90%
Total: 17-27% (gasoline), 25-36% (diesel)

EV Advantage: 3-5× more efficient!
```

### 9.2 Energy Flow Sankey Diagram

```
100 kWh from Battery
├─ 2 kWh: Battery internal resistance (2%)
├─ 2 kWh: Inverter losses (2%)
├─ 6 kWh: Motor losses (6%)
├─ 2 kWh: Gearbox/differential (2%)
└─ 88 kWh: To wheels (88% efficiency)

At wheels:
├─ 50 kWh: Aerodynamic drag (56%)
├─ 20 kWh: Rolling resistance (23%)
├─ 10 kWh: Braking (11%, partially recovered)
└─ 8 kWh: Accessories (9%)

Note: Distribution varies with speed
- Low speed: Rolling resistance dominant
- High speed: Aerodynamic drag dominant
```

### 9.3 Efficiency Optimization Strategies

**1. Motor Operating Point Optimization**:
```
- Operate motor in high-efficiency region (sweet spot)
- Typical: 75-85% of rated torque, 2000-6000 RPM
- Efficiency map-based control
```

**2. Regenerative Braking Maximization**:
```
- Predictive deceleration
- One-pedal driving mode
- Route-based optimization (GPS + map data)
```

**3. Thermal Management**:
```
- Preheat/precool while charging
- Minimize HVAC power:
  * Seat heating instead of cabin (1/10 power)
  * Heat pump instead of resistive heating (1/3 power)
  * Smart ventilation (use outside air when possible)
```

**4. Aerodynamic Enhancements**:
```
- Active grille shutters (close when cooling not needed)
- Auto-lowering suspension at high speed
- Wheel covers/aero wheels
- Underbody panels
```

**5. Weight Reduction**:
```
- Aluminum/carbon fiber body
- Structural battery pack
- Lightweight wheels
```

**6. Tire Optimization**:
```
- Low rolling resistance tires (C_rr = 0.006-0.008)
- Proper inflation (under-inflation increases C_rr 10-20%)
- Narrower tires (lower drag, but less grip)
```

### 9.4 Efficiency Comparison

| Vehicle Type | Tank-to-Wheel Efficiency | Well-to-Wheel Efficiency* |
|--------------|-------------------------|--------------------------|
| EV (Grid) | 85-95% | 30-45% (depends on grid mix) |
| EV (Solar) | 85-95% | 75-85% |
| ICE Gasoline | 17-27% | 12-20% |
| ICE Diesel | 25-36% | 18-27% |
| Hybrid | 30-40% | 22-30% |
| PHEV | 40-60% | 30-45% |
| Hydrogen FCV | 40-60% | 15-25% (H2 from electrolysis) |

*Well-to-Wheel includes fuel production and delivery

---

## 10. Charging Systems

### 10.1 AC Charging (On-Board Charger)

#### 10.1.1 Level 1 (120V, 12-16A)
```
Voltage: 120V AC
Current: 12-16A
Power: 1.4-1.9 kW
Connector: J1772 (US), Type 2 (EU)
Charging Time (75 kWh): 40-50 hours (0-100%)
Use Case: Emergency, overnight (PHEV)
```

#### 10.1.2 Level 2 (240V, 16-80A)
```
Voltage: 208-240V AC (single-phase)
Current: 16-80A
Power: 3.3-19.2 kW
Connector: J1772 (US), Type 2 (EU)
Charging Time (75 kWh): 4-23 hours (0-100%)
Use Cases:
- Home charging (typical: 7.2 kW, 11-11 hours)
- Workplace charging
- Public L2 chargers
- Hotel/shopping center

Common Configurations:
- 3.3 kW: 14A @ 240V (basic)
- 7.2 kW: 30A @ 240V (most common home)
- 11 kW: 48A @ 230V (3-phase, Europe)
- 22 kW: 32A @ 400V (3-phase, EU, rare for vehicles)
```

**Charging Curve** (Level 2, 7.2 kW):
```
0-80% SoC: Constant current (7.2 kW)
80-95% SoC: Transition to constant voltage
95-100% SoC: Constant voltage (power tapers to <1 kW)

Time to 80%: ~7.5 hours (from empty, 75 kWh battery)
Time to 100%: ~11 hours
```

### 10.2 DC Fast Charging

#### 10.2.1 Charging Standards

**CCS (Combined Charging System)** - Most common globally
```
CCS1 (North America):
- Connector: J1772 + 2 DC pins
- Power: 50-350 kW
- Voltage: 200-920V DC
- Current: 200-500A
- Vehicles: Most non-Tesla EVs in US

CCS2 (Europe):
- Connector: Type 2 + 2 DC pins
- Power: 50-350 kW
- Same specs as CCS1
- Vehicles: All EVs in Europe (including Tesla Model 3/Y)
```

**CHAdeMO** - Japanese standard
```
Connector: Dedicated DC connector
Power: 50-400 kW (400 kW: CHAdeMO 3.0)
Voltage: 50-500V (up to 1000V for v3.0)
Current: 125-400A (up to 600A for v3.0)
Vehicles: Nissan Leaf, older Japanese EVs
Status: Declining market share outside Japan
```

**Tesla Supercharger**
```
North America (before 2024):
- Connector: Proprietary Tesla connector
- Power: 72-250 kW
- Voltage: 50-500V
- Current: 300-630A

V3 Supercharger:
- Power: Up to 250 kW (limited by vehicle)
- Vehicles: Model 3 Performance ~250 kW, Model S/X ~200 kW

V4 Supercharger (2023+):
- Power: Up to 350 kW
- Connector: Longer cable, CCS adapter support

NACS (North American Charging Standard):
- Tesla connector adopted as SAE J3400
- Ford, GM, Rivian, etc. adopting for 2025+ vehicles
```

**GB/T** - Chinese standard
```
Connector: GB/T DC connector
Power: 37.5-237.5 kW (up to 900 kW proposed)
Voltage: 200-750V
Current: 80-250A (up to 600A proposed)
Vehicles: All EVs sold in China
```

#### 10.2.2 Charging Power Levels

```
Level 3 / DC Fast:
- 50 kW: Early DC fast chargers, ~45 min (10-80%)
- 100 kW: Common public chargers, ~30 min (10-80%)
- 150 kW: Modern highway chargers, ~25 min (10-80%)
- 250 kW: Tesla V3, Ionity, ~18 min (10-80%)
- 350 kW: Ultra-fast chargers, ~15 min (10-80%, if vehicle supports)

Note: Actual charging time depends on:
1. Battery capacity
2. Battery chemistry and thermal management
3. Current SoC
4. Battery temperature
5. Charger power vs. vehicle acceptance rate
```

#### 10.2.3 Charging Curve

**Typical DC Fast Charging Profile**:
```
Phase 1: Preconditioning (if needed)
- If battery too cold (<15°C) or hot (>45°C)
- Active heating/cooling to optimal range (20-35°C)
- Power limited (e.g., 50 kW) during preconditioning
- Duration: 5-15 min

Phase 2: Constant Power / Constant Current (0-50% SoC)
- Maximum power delivery
- Limited by: min(Charger_Power, Battery_Power, Cable_Current)
- Example: 150 kW charger, battery accepts 175 kW → 150 kW
- Duration: 10-15 min

Phase 3: Power Ramp (50-80% SoC)
- Power gradually reduces to protect battery
- Voltage approaches maximum (e.g., 410V for 400V system)
- Transition to constant voltage
- Duration: 10-15 min

Phase 4: Constant Voltage (80-100% SoC)
- Voltage held constant at maximum
- Current (and power) decrease exponentially
- Battery balancing occurs
- Very slow above 90% SoC
- Duration: 15-30 min (80-100%)

Recommendation: Charge to 80% for fastest charging, stop at 90% for optimal time/energy
```

**Charging Power vs. SoC** (Example: 800V vehicle, 350 kW charger):
```
SoC    Power   Voltage  Current  Duration
0-10%  50 kW   320V     156A     ~6 min  (precondition)
10-40% 300 kW  400V     750A     ~8 min
40-60% 270 kW  450V     600A     ~6 min
60-75% 200 kW  500V     400A     ~6 min
75-80% 150 kW  520V     288A     ~3 min
80-90% 75 kW   540V     139A     ~8 min
90-100% 25 kW  550V     45A      ~20 min

10-80%: ~29 minutes
10-100%: ~57 minutes
```

### 10.3 Charging Infrastructure

#### 10.3.1 Communication Protocols

**ISO 15118** (Plug & Charge):
```
Features:
- Automatic authentication (no RFID card needed)
- Dynamic pricing
- Bidirectional communication (V2G ready)
- Encrypted communication
- Smart charging (load management)

Adoption: CCS and Tesla (in some regions)
```

**OCPP** (Open Charge Point Protocol):
```
Purpose: Charger ↔ Network communication
Features:
- Remote monitoring
- Firmware updates
- Load balancing
- Transaction management
- Widely adopted for network management
```

#### 10.3.2 Smart Charging

**Load Management**:
```
Dynamic load balancing across multiple charging points
Example: 100 kW total capacity, 4 vehicles:
- If 1 vehicle: 100 kW
- If 2 vehicles: 50 kW each
- If 4 vehicles: 25 kW each
```

**Time-of-Use Optimization**:
```
Charge when electricity is cheap (off-peak)
Typical: Night (9 PM - 7 AM) at 50-70% of peak price
Smart: Delay charging or modulate power based on grid signal
```

**V2G (Vehicle-to-Grid)**:
```
Bidirectional charging: Vehicle can discharge to grid
Applications:
- Grid stabilization (frequency regulation)
- Peak shaving (reduce demand during high-price periods)
- Backup power
- Renewable integration (solar/wind buffering)

Revenue potential: $500-1500/year per vehicle (varies by market)
Battery degradation: Minimal if managed properly (<2% extra degradation)
```

### 10.4 Wireless Charging (WPT)

**Technology**: Inductive coupling (magnetic resonance)

**Efficiency**: 85-93% (vs. 95-98% for wired)

**Power Levels**:
```
Level 1 (≤3.7 kW): Residential (SAE J2954)
Level 2 (3.7-11 kW): Commercial, public
Level 3 (>11 kW): Rapid wireless (in development)
```

**Alignment Tolerance**: ±10-15 cm (modern systems)

**Advantages**:
- Convenience (no plug)
- Automatic charging
- Weather-proof
- Suitable for autonomous vehicles

**Disadvantages**:
- Lower efficiency
- Higher cost
- Requires precise alignment
- Foreign object detection needed

**Status**: Limited commercial deployment (BMW, Genesis)

---


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
