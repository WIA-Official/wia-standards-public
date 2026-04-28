# WIA-QUA-007 — Phase 3: Protocol

> Superconducting canonical Phase 3: operational protocols (magnet + maglev + power transmission + cryogenics).

# WIA-QUA-007: Superconducting - Complete Specification v1.0

> **Standard:** WIA-QUA-007
> **Title:** Superconducting
> **Version:** 1.0.0
> **Status:** Active
> **Date:** 2025-12-26
> **Authors:** WIA Quantum Research Group
> **Category:** Future Technology / Quantum / Physics

---

## Abstract

This specification defines a comprehensive framework for superconducting systems - materials and devices that exhibit zero electrical resistance and perfect diamagnetism below a critical temperature. The standard encompasses theoretical foundations (BCS theory, Cooper pairs), material classifications (Type I, Type II, HTSC), device implementations (qubits, Josephson junctions, SQUIDs), and practical applications (quantum computing, MRI, maglev, power transmission).

**弘益人間 (Benefit All Humanity)** - This standard serves humanity by enabling revolutionary technologies through superconductivity: efficient energy systems, powerful quantum computers, advanced medical imaging, and ultra-sensitive scientific instruments.

---


## 8. Superconducting Magnets

### 8.1 MRI Magnets

#### 8.1.1 Design Specifications

```typescript
interface MRIMagnet {
  // Field parameters
  fieldStrength: number; // Tesla (1.5, 3, 7 typical)
  homogeneity: number; // ppm over DSV
  dsvDiameter: number; // cm (diameter of spherical volume)

  // Coil design
  coils: {
    material: 'NbTi' | 'Nb3Sn';
    wireDiameter: number; // mm
    totalLength: number; // km
    turns: number;
    layers: number;
  };

  // Current and stored energy
  operatingCurrent: number; // A (typically 100-500)
  storedEnergy: number; // MJ (megajoules)
  inductance: number; // H

  // Cryogenics
  cryostat: {
    liquidHelium: number; // Liters
    boilOffRate: number; // L/day
    cryocooler: boolean; // Zero-boiloff?
  };

  // Quench protection
  quenchDetection: QuenchDetector;
  quenchHeaters: number; // Number of heaters
  dumpResistor: number; // Ω
}
```

#### 8.1.2 Field Homogeneity

Shimming to achieve <1 ppm over 40 cm DSV:

```
ΔB/B₀ < 1 × 10⁻⁶
```

Methods:
- **Passive shimming**: Iron pieces
- **Active shimming**: Superconducting shim coils
- **Dynamic shimming**: Real-time correction

### 8.2 Particle Accelerator Magnets

#### 8.2.1 LHC Dipole Magnets

```typescript
interface LHCDipole {
  // Specifications
  fieldStrength: 8.33; // Tesla
  length: 14.3; // meters
  aperture: 0.056; // meters (56 mm)

  // Superconductor
  conductor: {
    material: 'NbTi';
    filaments: 6300; // Number of SC filaments
    filamentDiameter: 7e-6; // 7 μm
    copperToSCRatio: 1.65;
  };

  // Operating parameters
  current: 11850; // Amperes
  temperature: 1.9; // Kelvin (superfluid He)
  storedEnergy: 7; // MJ

  // Total for LHC
  totalMagnets: 1232; // Dipoles
  totalWeight: 27; // tonnes each
  totalCurrent: 11850; // A (series connection)
}
```

---



## 9. Maglev Systems

### 9.1 Electromagnetic Suspension (EMS)

```typescript
interface EMSMaglev {
  type: 'electromagnetic-suspension';

  // Electromagnets
  magnets: {
    type: 'conventional-electromagnet';
    material: 'copper-or-aluminum';
    current: number; // A
    force: number; // N
  }[];

  // Control system
  control: {
    feedback: 'active'; // Continuously adjusted
    sensors: ProximitySensor[];
    updateRate: number; // Hz
  };

  // Performance
  levitationGap: 0.008; // 8 mm
  maxSpeed: 450; // km/h (Transrapid)
  power: 'moderate'; // Needs constant power
}
```

### 9.2 Electrodynamic Suspension (EDS)

```typescript
interface EDSMaglev {
  type: 'electrodynamic-suspension';

  // Superconducting magnets on vehicle
  magnets: {
    type: 'superconducting';
    material: 'NbTi' | 'YBCO';
    field: number; // Tesla
    current: number; // A (persistent mode)
  }[];

  // Track coils
  track: {
    coilType: 'figure-8' | 'null-flux';
    spacing: number; // meters
    material: 'aluminum' | 'copper';
  };

  // Performance
  levitationGap: 0.10; // 10 cm
  maxSpeed: 603; // km/h (SCMaglev record)
  power: 'zero-for-levitation'; // Only needs propulsion power

  // Cryogenics
  cryostat: CryogenicSystem;
}
```

### 9.3 High-Tc Maglev

Using YBCO bulk superconductors:

```typescript
interface HTSCMaglev {
  // HTSC bulks
  bulks: {
    material: 'YBCO';
    size: number; // cm diameter
    count: number;
    trappedField: number; // Tesla
  };

  // Permanent magnet track
  track: {
    magnetType: 'NdFeB' | 'SmCo';
    fieldStrength: number; // Tesla
    spacing: number; // cm
  };

  // Cooling
  cooling: {
    method: 'liquid-nitrogen' | 'cryocooler';
    temperature: 77; // Kelvin
    power: number; // Watts
  };

  // Self-stability
  stable: true; // No active control needed!
  restoring force: number; // N/mm
}
```

---



## 10. Power Transmission

### 10.1 Superconducting Cables

#### 10.1.1 HTS Power Cables

```typescript
interface HTSPowerCable {
  // Cable design
  design: 'coaxial' | 'tri-axial' | 'warm-dielectric';

  // HTS tapes
  tapes: {
    material: 'BSCCO-2223' | 'YBCO';
    width: number; // mm
    thickness: number; // μm
    criticalCurrent: number; // A at 77K
  };

  // Cable parameters
  length: number; // meters (km-scale demos)
  diameter: number; // cm
  capacity: number; // MW
  current: number; // kA (1-10 typical)
  voltage: number; // kV

  // Cryogenic system
  cooling: {
    cryogen: 'liquid-nitrogen';
    flowRate: number; // L/min
    refrigeration: number; // kW
  };

  // Performance
  losses: {
    conductor: number; // W/m (near zero)
    dielectric: number; // W/m
    thermal: number; // W/m (into cryostat)
    total: number; // 50-70% less than conventional
  };
}
```

#### 10.1.2 Economic Analysis

```typescript
interface PowerCableEconomics {
  // Capital costs
  capitalCost: {
    cable: number; // $/m
    cryogenics: number; // $
    installation: number; // $
    total: number; // $
  };

  // Operating costs
  operatingCost: {
    refrigeration: number; // $/year
    maintenance: number; // $/year
    losses: number; // $/year (energy cost of losses)
    total: number; // $/year
  };

  // Comparison to conventional
  comparison: {
    powerLossSavings: number; // % reduction
    rightOfWaySavings: number; // m² (compact design)
    paybackPeriod: number; // years
  };
}
```

---



## 11. Cryogenic Systems

### 11.1 Cooling Technologies

#### 11.1.1 Temperature Ranges

```typescript
const CRYOGENIC_COOLING = {
  liquidNitrogen: {
    temperature: 77, // Kelvin
    boilingPoint: 77.4, // K at 1 atm
    applications: ['HTSC', 'Cryocooler precooling'],
    cost: 'low', // ~$0.50/liter
    availability: 'excellent'
  },

  liquidHelium: {
    temperature: 4.2, // Kelvin
    boilingPoint: 4.22, // K at 1 atm
    applications: ['LTS', 'MRI', 'NMR'],
    cost: 'high', // ~$5-10/liter
    availability: 'limited' // Helium shortage
  },

  superfluidHelium: {
    temperature: 1.9, // Kelvin (LHC)
    method: 'pumped-helium-4',
    applications: ['High-field magnets', 'Particle accelerators'],
    cost: 'very-high',
    complexity: 'high'
  },

  helium3: {
    temperature: 0.3, // Kelvin
    method: 'He3-refrigerator',
    applications: ['Research'],
    cost: 'extreme', // He3 very rare
    complexity: 'very-high'
  },

  dilutionRefrigerator: {
    temperature: 0.010, // 10 mK
    method: 'He3-He4-mixing',
    applications: ['Quantum computing'],
    cost: 'extreme',
    complexity: 'extreme'
  }
};
```

#### 11.1.2 Cryocooler Systems

```typescript
interface Cryocooler {
  type: 'Gifford-McMahon' | 'pulse-tube' | 'Stirling' | 'Joule-Thomson';

  // Performance
  coolingPower: number; // Watts at temperature
  temperature: number; // Kelvin
  efficiency: number; // % of Carnot

  // Operating parameters
  inputPower: number; // Watts (electrical)
  refrigerationCOP: number; // Coefficient of performance
  coolingCapacity: {
    at77K: number; // Watts
    at20K: number; // Watts
    at4K: number; // Watts
  };

  // Physical
  weight: number; // kg
  vibration: number; // μm (pulse-tube: very low)
  maintenance: number; // hours between service
}
```

### 11.2 Thermal Management

#### 11.2.1 Heat Load Sources

```typescript
interface HeatLoad {
  // Conduction
  conduction: {
    supports: number; // W (through support structures)
    wires: number; // W (current leads)
    radiation: number; // W (thermal radiation)
  };

  // Convection
  convection: {
    residualGas: number; // W (if not high vacuum)
  };

  // System operation
  operation: {
    jouleHeating: number; // W (connections, joints)
    acLosses: number; // W (if AC currents)
    eddyCurrents: number; // W (time-varying fields)
  };

  // Total
  total: number; // Sum of all sources
}
```

---




---

## A.1 Magnet-operation protocol

Persistent-mode magnet operation: ramp current to set point under regulated voltage, close the persistent-current switch (PCS) by warming and re-cooling its superconducting bridge, verify zero ramp-rate drift over 60 minutes, then switch off the power supply. Quench protection: voltage-tap pairs across each pancake or layer, with comparators set to detect Zone-1 and Zone-2 normal regions; on detection the energy-extraction circuit dumps stored energy into an external dump resistor sized to keep peak hot-spot temperature below the design limit (typically 200 K for NbTi, 500 K for HTS).

## A.2 Maglev levitation protocol

EDS (electrodynamic suspension) maglev (e.g., SCMaglev, JR Central): on-board superconducting magnets generate the lift force via induced eddy currents in the guideway null-flux coils. EMS (electromagnetic suspension) maglev (e.g., Transrapid) uses room-temperature electromagnets and is therefore out of scope here. The EDS protocol covers cryostat operation in motion (microphonic vibration mitigation, on-board cryocooler load), guideway propulsion synchronisation, and emergency landing on auxiliary wheels at sub-100 km/h.

## A.3 Power-transmission protocol

Superconducting power cables (HTS) operate in three architectures: cold-dielectric coaxial, warm-dielectric, and tri-axial. The protocol covers cable-end terminations (current-lead heat-load budget), in-line joints with the contact-resistance budget, and the cryogenic loop (LN2 sub-cooling at 65–77 K with overpressure 0.3–0.5 MPa for two-phase suppression).

## A.4 Cryogenic-system protocol

Cryogenic systems are described by their thermodynamic cycle (Joule-Thomson, Stirling, Gifford-McMahon, pulse-tube, dilution), heat-load budget at each stage, and the redundancy class. Class-A systems (qubit-research, medical MRI) require a cold-spare cryocooler ready to assume load within minutes; Class-B systems (industrial maglev, power transmission) accept seconds-to-minutes warming with safe-state procedures; Class-C systems are research-only.

## A.5 Quench-detection and protection protocol

Quench detection primary signal: voltage difference across paired voltage taps that bounds the inductive transformation voltage. Threshold and validation window are tuned per coil-time-constant; modern HTS-magnet protection uses no-insulation (NI) winding and accepts longer detection times in exchange for self-protecting current redistribution. Secondary signals: differential pressure across the cryogen reservoir, temperature spike in the resistive zone, and acoustic emission from training events.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane traffic. Telemetry traffic uses mTLS with per-channel monotonic counters; replay attempts are detected and dropped at the broker.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/superconducting/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-superconducting-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/superconducting-host:1.0.0` ships every superconducting envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/superconducting.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Superconducting deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
