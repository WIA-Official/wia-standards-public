# WIA-AUTO-004 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-004
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

```
3-phase ABC → 2-phase αβ (Clarke) → 2-phase dq (Park)

Clarke Transform:
I_α = I_a
I_β = (2×I_b + I_a) / √3

Park Transform:
I_d = I_α×cos(θ) + I_β×sin(θ)  (flux component)
I_q = -I_α×sin(θ) + I_β×cos(θ) (torque component)
```

**Control Loop**:
```
Torque Command → I_q Reference → PI Controller → V_q
Flux Command → I_d Reference → PI Controller → V_d
→ Inverse Park → Inverse Clarke → PWM → Inverter
```

#### 4.5.2 Maximum Torque Per Ampere (MTPA)

For PMSM:
```
Optimal I_d and I_q to maximize torque for given stator current

I_d = -(ψ_m / 2×L_d) + √[(ψ_m / 2×L_d)² + I_s²]
I_q = √(I_s² - I_d²)
```

#### 4.5.3 Field Weakening

At high speed, back-EMF limits voltage. Reduce flux to extend speed range:

```
Base Speed: ω_base = V_max / ψ_m
Field Weakening: I_d < 0 (negative d-axis current)
Max Speed: ω_max = 2-5 × ω_base (depending on motor type)
```

**Constant Power Region**:
```
P = T × ω = constant
T ∝ 1/ω (torque decreases inversely with speed)
```

---

## 5. Power Electronics

### 5.1 Inverter (DC/AC Converter)

#### 5.1.1 Topology

**3-Phase 2-Level Voltage Source Inverter** (most common):
```
Configuration: 6 switches (IGBTs or MOSFETs) + 6 freewheeling diodes
Input: DC from battery (300-800V)
Output: 3-phase AC (variable voltage and frequency)
Switching Frequency: 10-20 kHz
```

**Power Module Components**:
- Power switches: IGBT (for high voltage >650V) or SiC MOSFET (next-gen)
- Freewheeling diodes: Fast recovery diodes or SiC Schottky
- Gate drivers: Isolated drivers for each switch
- DC-link capacitor: Filter and energy storage (100-500 µF)

#### 5.1.2 Space Vector Modulation (SVM)

**Principle**: Generate 3-phase AC by switching between 8 voltage vectors

**Switching States**:
```
8 vectors: V0, V1, V2, V3, V4, V5, V6, V7
- V0, V7: Zero vectors (all switches on or off)
- V1-V6: Active vectors (at 60° intervals)
```

**Duty Cycle Calculation**:
```
T_a = T_s × m × sin(π/3 - θ)
T_b = T_s × m × sin(θ)
T_0 = T_s - T_a - T_b

Where:
- T_s = switching period
- m = modulation index (0-1)
- θ = angle within sector (0-60°)
```

**Modulation Index**:
```
m = V_out / (V_dc / √3)

Maximum without overmodulation: m = 0.907
```

#### 5.1.3 Switching Losses

**Conduction Losses**:
```
P_cond = I_rms² × R_on × D

Where:
- I_rms = RMS current
- R_on = on-state resistance
- D = duty cycle
```

**Switching Losses**:
```
P_sw = f_sw × (E_on + E_off)

Where:
- f_sw = switching frequency
- E_on = turn-on energy loss
- E_off = turn-off energy loss
```

**Total Inverter Efficiency**:
```
η = P_out / (P_out + P_cond + P_sw + P_driver)
Typical: 95-98% for Si IGBT, 98-99% for SiC MOSFET
```

### 5.2 DC/DC Converter

#### 5.2.1 High-Voltage to Low-Voltage (HV→12V/48V)

**Topology**: Isolated DC/DC converter (flyback, full-bridge, LLC resonant)

**Specifications**:
```
Input: 300-800V DC (from HV battery)
Output: 12-14V DC (for accessories)
Power: 2-3 kW (continuous)
Efficiency: 90-95%
Isolation: >1000V
```

**Applications**:
- 12V battery charging
- Accessories (lights, infotainment, controls)
- Replaces alternator in conventional vehicle

#### 5.2.2 Bidirectional DC/DC Converter

**Topology**: Dual active bridge (DAB)

**Applications**:
- 48V hybrid systems
- V2L (Vehicle-to-Load) applications
- Auxiliary battery management

### 5.3 On-Board Charger (OBC)

#### 5.3.1 AC/DC Conversion

**Topology**: Power Factor Correction (PFC) + DC/DC converter

**Stages**:
```
1. AC Input Filter: EMI suppression
2. PFC Stage: Boost converter (unity power factor)
3. DC/DC Stage: Isolated converter (output regulation)
4. Output Filter: Ripple reduction
```

**Specifications**:
```
Input: 120-240V AC, 50/60 Hz (single-phase)
       208-480V AC (3-phase for high-power OBC)
Output: 300-800V DC (to battery)
Power: 3.3-22 kW (single-phase), up to 50 kW (3-phase)
Efficiency: 92-95%
Power Factor: >0.99
THD: <5%
```

**Charging Profiles**:
```
1. Constant Current (CC): I = I_max until V = V_max
2. Constant Voltage (CV): V = V_max, I decreases exponentially
3. Taper: Transition from CC to CV

Typical: CC to 80% SoC, CV from 80-100%
```

### 5.4 Next-Generation Power Semiconductors

#### 5.4.1 Silicon Carbide (SiC)

**Advantages over Si**:
- Higher breakdown voltage: 10× vs. Si
- Lower on-resistance: 50% reduction
- Higher switching frequency: 50-100 kHz vs. 10-20 kHz
- Higher temperature operation: 200°C vs. 150°C
- Lower switching losses: 75% reduction

**Impact on EV**:
- Inverter efficiency: 98-99% vs. 95-97% (Si)
- Size reduction: 40-50% smaller and lighter
- Range improvement: 5-10% from lower losses
- Cost: Currently 2-3× Si, decreasing rapidly

**Adoption**:
- Tesla Model 3/Y (SiC inverter)
- Lucid Air
- Most new EVs from 2023+

#### 5.4.2 Gallium Nitride (GaN)

**Characteristics**:
- Even lower losses than SiC
- Very high switching frequency: 100-500 kHz
- Lower cost potential than SiC
- Currently limited to lower voltages (<650V)

**EV Applications**:
- On-board chargers
- DC/DC converters
- 48V systems
- Future: HV inverters (in development)

---

## 6. Regenerative Braking

### 6.1 Operating Principle

**Concept**: Electric motor operates as generator during deceleration

**Energy Flow**:
```
Kinetic Energy → Motor (Generator) → Inverter (Rectifier) → Battery
```

**Torque Control**:
```
Regenerative Torque = min(T_requested, T_motor_limit, T_battery_limit)

Where:
- T_requested: Driver brake input
- T_motor_limit: Max generator torque at current speed
- T_battery_limit: Max charge current × motor constant
```

### 6.2 Energy Recovery

#### 6.2.1 Kinetic Energy Available

```
E_kinetic = (1/2) × m × v²

Where:
- m = vehicle mass (kg)
- v = velocity (m/s)

Example (1800 kg vehicle, 100 km/h → 0):
E_kinetic = 0.5 × 1800 × (100/3.6)²
         = 0.5 × 1800 × 27.78²
         = 694,440 J = 0.193 kWh
```

#### 6.2.2 Recoverable Energy

```
E_recovered = E_kinetic × η_motor × η_inverter × η_battery

Typical efficiencies:
- Motor (as generator): 85-90%
- Inverter (AC→DC): 95-97%
- Battery charging: 95-98%
- Overall: 77-85%

Example:
E_recovered = 0.193 × 0.85 × 0.96 × 0.96
            = 0.151 kWh (78% recovery)
```

#### 6.2.3 Limitations

**Motor Speed Limits**:
- At very low speed (<5 km/h), motor torque insufficient → friction brakes engage
- At very high speed, battery current limit → reduced regen torque

**Battery Charge Limits**:
- At high SoC (>90%), charge current limited → reduced regen
- At low temperature (<0°C), charge acceptance reduced → limited regen
- C-rate limit (typically 0.5-1C continuous)

**Power Electronics Limits**:
- Maximum inverter current
- DC-link voltage limits

### 6.3 Blended Braking

**Coordination**: Seamless blend between regenerative and friction braking

**Strategies**:

**1. Serial Braking** (most common):
```
if (Brake_Request < Regen_Capacity):
    Friction_Brake = 0
    Regen_Brake = Brake_Request
else:
    Regen_Brake = Regen_Capacity
    Friction_Brake = Brake_Request - Regen_Capacity
```

**2. Parallel Braking**:
```
Regen_Brake = k × Brake_Request
Friction_Brake = (1-k) × Brake_Request

Where k varies with speed, SoC, temperature
```

**3. One-Pedal Driving**:
```
Accelerator_Lifted → Strong_Regen (up to 0.2-0.3g decel)
Brake_Pedal → Additional friction braking if needed

Allows driving with minimal brake pedal use in city
```

### 6.4 Regenerative Braking Efficiency

**Real-World Energy Recovery**:
```
Urban Driving: 15-25% of total energy recovered
Highway Driving: 5-10% (less braking)
Mixed Driving: 10-20%
```

**Range Extension**:
```
EPA City Range Improvement: 20-30% vs. no regen
EPA Highway Range Improvement: 5-10% vs. no regen
```

**Factors Affecting Recovery**:
- Driving pattern (stop-and-go vs. steady speed)
- Battery SoC (limited at high SoC)
- Temperature (limited in cold weather)
- Motor/inverter efficiency
- Driver behavior (smooth vs. aggressive braking)

---

## 7. Thermal Management

### 7.1 Battery Thermal Management

#### 7.1.1 Temperature Requirements

**Optimal Operating Range**:
```
Ideal: 20-35°C
Acceptable: 15-45°C
Critical: >60°C (risk of damage)
           <0°C (reduced performance and capacity)
```

**Temperature Effects**:
```
Capacity:
- At -20°C: 50-70% of rated capacity
- At 0°C: 80-90% of rated capacity
- At 25°C: 100% (nominal)
- At 45°C: 100% but accelerated degradation

Resistance:
- At -20°C: 3-5× nominal
- At 0°C: 1.5-2× nominal
- At 25°C: 1× (nominal)
- At 45°C: 0.9-1× nominal

Power:
- At -20°C: 30-50% of rated power
- At 0°C: 60-80% of rated power
- At 25°C: 100% (nominal)
- At 45°C: 100% but thermal limits may apply
```

#### 7.1.2 Cooling Methods

**1. Passive Air Cooling**:
```
Method: Natural convection or forced air flow
Heat Transfer: Air → Cell surface
Thermal Resistance: 20-50 K/W

Pros:
- Simple, low cost
- No additional power consumption (natural)
- Minimal complexity

Cons:
- Poor heat transfer (air: 0.026 W/m·K)
- Uneven temperature distribution
- Insufficient for high-power applications

Applications:
- Low-power EVs (e.g., early Nissan Leaf)
- Mild climates
```

**2. Active Air Cooling**:
```
Method: Forced air circulation with fans/blowers
Heat Transfer: Air (forced) → Cell surface
Thermal Resistance: 10-30 K/W
Power Consumption: 100-500 W

Improvements over passive:
- Better heat transfer (forced convection)
- Controllable cooling rate
- Lower cost than liquid

Limitations:
- Still limited heat transfer
- Fan noise
- Not sufficient for fast charging

Applications:
- Budget EVs
- Mild performance requirements
```

**3. Liquid Cooling**:
```
Method: Coolant circulation (water-glycol, 50:50 mix)
Heat Transfer: Coolant → Cold plate → Cell
Thermal Resistance: 1-5 K/W
Power Consumption: 200-1000 W (pump + radiator fan)
Coolant: 50% water, 50% glycol (freezing point: -37°C)
Flow Rate: 5-20 L/min

Cooling Plate Designs:
a) Serpentine channels
b) Parallel channels
c) Mini-channel heat exchanger

Pros:
- Excellent heat transfer (water: 0.6 W/m·K)
- Uniform temperature distribution
- Enables fast charging
- Heating capability (with PTC or heat pump)

Cons:
- Higher cost and complexity
- Additional weight (~10-20 kg)
- Risk of coolant leakage
- Requires maintenance

Applications:
- Most modern EVs
- High-performance and long-range vehicles
- DC fast charging capable
```

**4. Refrigerant Cooling** (Direct Refrigeration):
```
Method: Refrigerant (R134a or R1234yf) phase change cooling
Heat Transfer: Refrigerant evaporation → Cell
Thermal Resistance: 0.5-2 K/W
COP (Coefficient of Performance): 2-4

Pros:
- Highest cooling performance
- Can achieve sub-ambient temperatures
- Very uniform temperature

Cons:
- Highest complexity and cost
- Shares AC system (may impact cabin comfort)
- Higher power consumption

Applications:
- High-performance EVs (some Tesla, Porsche Taycan)
- Extreme climates (very hot regions)
```

**5. Phase Change Material (PCM)**:
```
Method: PCM (wax, salt hydrate) absorbs heat via phase transition
Phase Change Temperature: 25-35°C (matched to optimal battery temp)
Latent Heat: 150-250 kJ/kg

Pros:
- Passive thermal buffering
- No power consumption
- Uniform temperature during phase change
- Compact

Cons:
- Limited heat capacity (saturates)
- Requires external cooling to reset
- Added weight
- Not standalone solution

Applications:
- Supplementary to active cooling
- Racing/high-performance (thermal buffering during sprint)
```

#### 7.1.3 Heating Methods

**Cold Weather Challenges**:
- Reduced battery capacity and power
- Reduced charging acceptance
- Increased internal resistance

**Heating Solutions**:

**1. Resistive Heating** (PTC - Positive Temperature Coefficient):
```
Method: Electric resistance heater in coolant loop
Power: 3-7 kW
Efficiency: ~100% (electrical → heat)
Energy Consumption: High (from battery)

Pros:
- Simple, reliable
- Fast heating
- Low cost

Cons:
- High energy consumption (reduces range 20-40% in cold weather)
- Inefficient

Applications:
- Most current EVs
```

**2. Heat Pump**:
```
Method: Reverse refrigeration cycle
COP: 2-4 (2-4× more efficient than resistive)
Power: 1-3 kW electrical input → 2-12 kW heat output
Operating Range: Down to -10°C to -20°C (ambient)

Pros:
- 2-4× more efficient than resistive
- Can also cool
- Reduces range loss in cold weather

Cons:
- Higher cost
- Complexity
- Reduced COP at very low ambient temperatures

Applications:
- Premium EVs (Tesla Model Y, Nissan Ariya, Hyundai Ioniq 5)
- Cold climate markets
```

**3. Coolant Pre-conditioning**:
```
Method: Heat battery while plugged in (grid power)
Timing: Before departure (scheduled or remote activated)

Pros:
- Zero impact on driving range
- Battery at optimal temp for departure
- Enables faster charging

Cons:
- Requires grid connection
- Time required (20-60 min)
```

### 7.2 Motor Thermal Management

#### 7.2.1 Heat Generation

**Losses in Motor**:
```
Total Loss = Copper Loss + Iron Loss + Mechanical Loss + Stray Loss

Copper Loss (I²R):
P_copper = 3 × I²_rms × R_phase
Dominant at low speed (high current)

Iron Loss (hysteresis + eddy current):
P_iron = k_h × f × B² + k_e × f² × B²
Dominant at high speed (high frequency)

Mechanical Loss:
P_mech = friction + windage
Increases with speed

Typical Distribution at Rated Load:
- Copper loss: 50-60%
- Iron loss: 20-30%
- Mechanical loss: 10-15%
- Stray loss: 5-10%
```

**Heat Distribution**:
```
Winding (hottest): 80-120°C (continuous)
                   150-180°C (peak, short-term)
Magnets (PMSM): 80-120°C (risk of demagnetization >150°C)
Stator core: 60-100°C
Rotor: 60-100°C
```

#### 7.2.2 Cooling Methods

**Liquid Cooling** (most common):
```
Method: Coolant jacket around stator
Coolant: Water-glycol (same as battery, often shared loop)
Flow Rate: 3-10 L/min
Thermal Resistance: 0.1-0.5 K/W
Power Removal: 5-15 kW (at rated load)

Cooling Paths:
1. Stator jacket cooling (most common)
2. Oil spray cooling on windings (high-performance)
3. Shaft cooling (water channels through hollow shaft)
```

**Oil Cooling** (next-gen):
```
Method: Direct oil spray on windings (ATF - Automatic Transmission Fluid)
Advantages:
- Direct cooling of hottest components (windings)
- Better heat transfer than water jacket
- Also lubricates bearings
- Enables higher power density

Disadvantages:
- Requires oil filtration and cooling
- More complex sealing
- Higher cost

Applications:
- High-performance EVs (Porsche Taycan)
- Race applications
```

### 7.3 Power Electronics Thermal Management

**Critical Components**:
- IGBTs/MOSFETs: Junction temperature <150°C (Si), <175°C (SiC)
- Diodes: Similar limits as switches
- Capacitors: Temperature-sensitive (lifetime halves per 10°C increase)

**Cooling Methods**:
```
Liquid Cold Plate:
- Integrated into power module baseplate
- Coolant: Water-glycol
- Thermal Interface: Thermal grease or pad (0.5-2 K/W)
- Heat Sink: Aluminum cold plate with micro-channels
- Thermal Resistance: Junction to coolant: 0.1-0.5 K/W
```

**Power Derating**:
```
If (T_junction > T_max - margin):
    P_max = P_rated × (T_max - T_ambient) / (T_junction - T_ambient)
```

### 7.4 Integrated Thermal Management

**System Integration**:
```
Components sharing cooling loops:
1. Battery + Motor + Inverter (most common)
2. Separate battery loop (if different temp requirements)
3. Cabin HVAC integration (heat pump systems)
```

**Coolant Loops**:


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
