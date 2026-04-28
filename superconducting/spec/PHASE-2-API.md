# WIA-QUA-007 — Phase 2: API Interface

> Superconducting canonical Phase 2: device API (Josephson junction + qubit + SQUID).

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


## 5. Josephson Junctions

### 5.1 Josephson Effect

#### 5.1.1 DC Josephson Effect

Current flows with zero voltage:

```
I = Ic × sin(φ)
```

Where:
- `I` = Supercurrent through junction (A)
- `Ic` = Critical current (A)
- `φ` = Phase difference between superconductors (radians)

#### 5.1.2 AC Josephson Effect

Under applied voltage:

```
φ(t) = φ₀ + (2e/ℏ) × V × t

f = 2eV / h ≈ 483.6 MHz/μV
```

Where:
- `V` = Applied voltage (volts)
- `f` = Josephson frequency (Hz)
- `e` = Elementary charge
- `h` = Planck constant

#### 5.1.3 RCSJ Model

Resistively and Capacitively Shunted Junction:

```typescript
interface RCSJModel {
  // Circuit parameters
  criticalCurrent: number; // Ic (A)
  resistance: number; // R (Ω)
  capacitance: number; // C (F)

  // Characteristic frequencies
  plasmaFrequency: number; // ωp = √(2eIc / ℏC)
  characteristicFrequency: number; // ωc = 2eIcR / ℏ

  // Damping parameter
  stewartMcCumberParameter: number; // βc = 2πIcR²C / Φ₀

  // Dynamics
  dynamics: 'underdamped' | 'overdamped' | 'critically-damped';
}
```

### 5.2 Junction Fabrication

#### 5.2.1 Junction Types

```typescript
interface JosephsonJunction {
  type: 'SIS' | 'SNS' | 'SInsS' | 'dayem-bridge' | 'variable-thickness';

  // Physical structure
  structure: {
    superconductor1: string; // e.g., 'Al', 'Nb'
    barrier: {
      material: string; // 'AlOx', 'MgO', 'normal metal'
      thickness: number; // nanometers
    };
    superconductor2: string;
  };

  // Junction parameters
  area: number; // m²
  criticalCurrent: number; // A
  normalResistance: number; // Ω
  icRnProduct: number; // V (quality metric)

  // Fabrication
  fabricationMethod: 'shadow-evaporation' | 'SNAP' | 'ion-milling' | 'bridge';
  tolerance: number; // % variation in Ic
}
```

#### 5.2.2 Critical Current Control

```
Ic = Jc × A

Jc ≈ (π × Δ) / (2 × e × Rn × A)
```

Where:
- `Jc` = Critical current density (A/m²)
- `A` = Junction area (m²)
- `Δ` = Superconducting gap (eV)
- `Rn` = Normal state resistance (Ω)

---



## 6. Superconducting Qubits

### 6.1 Qubit Types

#### 6.1.1 Transmon Qubit

Most common superconducting qubit:

```typescript
interface TransmonQubit {
  // Circuit parameters
  josephsonEnergy: number; // EJ (GHz)
  chargingEnergy: number; // EC (GHz)
  ejEcRatio: number; // EJ/EC >> 1 (charge noise insensitive)

  // Qubit parameters
  frequency: number; // f01 ≈ √(8EJEC) / h (GHz)
  anharmonicity: number; // α = EC (negative, ~-200 to -400 MHz)

  // Coherence times
  t1: number; // Energy relaxation (μs)
  t2: number; // Phase coherence (μs)
  t2_echo: number; // Echo coherence (μs)

  // Control
  driveLine: 'capacitive' | 'inductive';
  readoutResonator: ResonatorSpecs;
}

const TYPICAL_TRANSMON = {
  frequency: 5.2e9, // 5.2 GHz
  anharmonicity: -300e6, // -300 MHz
  t1: 50e-6, // 50 μs
  t2: 70e-6, // 70 μs
  ejEcRatio: 50,
  josephsonEnergy: 20e9, // 20 GHz
  chargingEnergy: 400e6 // 400 MHz
};
```

#### 6.1.2 Flux Qubit

Tunable frequency qubit:

```typescript
interface FluxQubit {
  // Circuit topology
  squidLoop: {
    junctions: JosephsonJunction[];
    inductance: number; // H
    area: number; // m²
  };

  // External flux control
  externalFlux: number; // Φext / Φ₀
  fluxBias: number; // Φ₀ units

  // Tunable frequency
  frequencyRange: {
    min: number; // GHz
    max: number; // GHz
    tunability: number; // % range
  };

  // Coherence
  t1: number; // μs
  t2: number; // μs (flux noise limited)

  // Sweet spot
  sweetSpot: {
    flux: number; // Φ₀ units (usually 0.5)
    dephasing: number; // Reduced flux noise
  };
}
```

#### 6.1.3 Phase Qubit

Large anharmonicity:

```typescript
interface PhaseQubit {
  // Tilted washboard potential
  josephsonEnergy: number; // EJ
  biasCurrent: number; // Fraction of Ic

  // Quantum levels in well
  level0: number; // Ground state (GHz)
  level1: number; // First excited (GHz)
  level2: number; // Second excited (GHz)

  // Large anharmonicity
  anharmonicity: number; // ~1 GHz (large!)

  // Escape rate
  escapeRate: number; // Tunneling out of well (s⁻¹)

  // Readout
  readoutMethod: 'escape-measurement';
  fidelity: number; // Readout fidelity
}
```

### 6.2 Qubit Control

#### 6.2.1 Single-Qubit Gates

```typescript
interface QuantumGate {
  name: 'I' | 'X' | 'Y' | 'Z' | 'H' | 'S' | 'T' | 'Rx' | 'Ry' | 'Rz';

  // Gate parameters
  duration: number; // nanoseconds
  amplitude: number; // Control amplitude
  phase: number; // Control phase
  frequency: number; // Drive frequency

  // Performance
  fidelity: number; // 0-1 (>0.999 for transmon)
  errorRate: number; // 1 - fidelity

  // Pulse shape
  pulseShape: 'gaussian' | 'DRAG' | 'square' | 'cosine';
  envelope: ComplexFunction;
}

const SINGLE_QUBIT_GATES = {
  X_90: {
    name: 'Rx',
    angle: Math.PI / 2,
    duration: 20e-9, // 20 ns
    fidelity: 0.9995
  },
  X_180: {
    name: 'X',
    angle: Math.PI,
    duration: 40e-9, // 40 ns
    fidelity: 0.9993
  },
  Hadamard: {
    name: 'H',
    duration: 40e-9,
    fidelity: 0.9992
  }
};
```

#### 6.2.2 Two-Qubit Gates

```typescript
interface TwoQubitGate {
  type: 'CNOT' | 'CZ' | 'iSWAP' | 'CPHASE';

  // Control mechanism
  coupling: 'capacitive' | 'inductive' | 'resonator';
  couplingStrength: number; // MHz

  // Gate parameters
  duration: number; // nanoseconds
  fidelity: number; // 0-1 (>0.99 target)

  // Tuning
  tuningMethod: 'frequency' | 'flux' | 'coupler';
  tuningSpeed: number; // MHz/ns
}

const TWO_QUBIT_GATES = {
  CNOT: {
    type: 'CNOT',
    coupling: 'capacitive',
    duration: 100e-9, // 100 ns
    fidelity: 0.993,
    couplingStrength: 5e6 // 5 MHz
  },
  CZ: {
    type: 'CZ',
    coupling: 'resonator',
    duration: 200e-9, // 200 ns
    fidelity: 0.995,
    couplingStrength: 10e6 // 10 MHz
  }
};
```

### 6.3 Qubit Readout

#### 6.3.1 Dispersive Readout

```typescript
interface DispersiveReadout {
  // Resonator parameters
  resonator: {
    frequency: number; // GHz (typically 6-8 GHz)
    quality: number; // Q factor (1000-10000)
    coupling: number; // κ (MHz)
  };

  // Qubit-resonator coupling
  coupling: number; // g (MHz)
  dispersiveShift: number; // χ = g²/Δ (MHz)
  cooperativity: number; // C = 4g²/(κγ)

  // Readout performance
  fidelity: number; // 0-1 (>0.99 typical)
  duration: number; // nanoseconds
  snr: number; // Signal-to-noise ratio (dB)

  // Measurement
  measurementOperator: 'Z';
  outcomes: [0, 1]; // Qubit states
}
```

---



## 7. SQUID Magnetometers

### 7.1 DC SQUID

#### 7.1.1 Device Structure

```typescript
interface DcSQUID {
  // Two Josephson junctions in parallel
  junctions: {
    junction1: JosephsonJunction;
    junction2: JosephsonJunction;
    symmetry: number; // Ic1/Ic2 (ideally 1)
  };

  // SQUID loop
  loop: {
    inductance: number; // L (pH, typically 100-500)
    area: number; // m² (determines flux coupling)
    geometry: 'square' | 'washer' | 'gradiometer';
  };

  // Flux sensitivity
  sensitivity: number; // Tesla/√Hz (1e-15 typical)
  bandwidth: number; // Hz
  noiseFloor: number; // Tesla

  // Operating point
  workingPoint: {
    biasCurrent: number; // Fraction of Ic
    fluxBias: number; // Φ₀ units
  };
}
```

#### 7.1.2 Voltage-Flux Relation

```
V(Φ) = (R/2) × √(Ic² - (π × Φ / Φ₀)²)
```

For small flux variations:

```
∂V/∂Φ = (π/2Φ₀) × R × Ic × sin(π × Φ / Φ₀)
```

Maximum sensitivity at Φ = Φ₀/2 (quadrature point).

#### 7.1.3 Flux-Locked Loop

```typescript
interface FluxLockedLoop {
  // Feedback system
  feedback: {
    gain: number; // V/Φ₀
    bandwidth: number; // Hz
    lockRange: number; // Φ₀ units
  };

  // Modulation
  modulation: {
    frequency: number; // kHz (typically 100-500)
    amplitude: number; // Φ₀ units
    waveform: 'sine' | 'square' | 'triangle';
  };

  // Performance
  dynamicRange: number; // Φ₀ units
  slew rate: number; // Φ₀/s
  linearity: number; // % deviation
}
```

### 7.2 Applications

#### 7.2.1 Magnetoencephalography (MEG)

```typescript
interface MEGSystem {
  // SQUID array
  sensors: DcSQUID[];
  sensorCount: number; // 300+ typical
  spacing: number; // cm between sensors

  // Sensitivity
  fieldSensitivity: 1e-15; // Tesla/√Hz
  gradientSensitivity: 1e-14; // Tesla/m/√Hz

  // Shielding
  shieldedRoom: {
    layers: number; // mu-metal layers
    attenuation: number; // dB at 1 Hz
  };

  // Signal processing
  bandwidth: number; // Hz (0.1-1000 typical)
  samplingRate: number; // Hz
  channels: number;
}
```

---




---

## A.1 Endpoint reference

```http
POST /superconducting/v1/materials                # register material record
GET  /superconducting/v1/materials/{id}           # fetch material record
POST /superconducting/v1/junctions                # register Josephson junction
GET  /superconducting/v1/qubits/{id}/coherence    # latest coherence numbers
POST /superconducting/v1/squids                   # register SQUID sensor
WS   /superconducting/v1/telemetry/stream         # cryostat telemetry
```

Every endpoint follows the discovery convention at `/.well-known/wia-superconducting`.

## A.2 Material-record API

`POST /materials` accepts the Phase 1 §A.1 envelope and returns a stable `materialId`. Subsequent characterisation reports reference the `materialId`. Updates to the canonical record produce a new version with the prior version preserved in the version history.

## A.3 Josephson-junction API

`POST /junctions` registers a junction with the parameters listed in Phase 1 §A.4. Junctions intended for qubit fabrication MUST cross-reference the qubit envelope (Phase 3 §A.2). Read endpoints expose the live `T1/T2` history at the device level so SDKs can plot drift over fabrication batches.

## A.4 SQUID and detector API

`POST /squids` and `POST /detectors` register flux- and photon-sensitive devices respectively. The endpoints accept calibration envelopes (flux-to-voltage transfer curve, dark-count vs. bias point sweep, photon-number resolving probability matrix) so downstream consumers can compensate without re-measuring.

## A.5 Cryostat telemetry WebSocket

The telemetry WebSocket multiplexes per-stage temperatures (50 K, 4 K, still, cold-plate, mixing chamber for dilution units), pressures, magnet currents, and quench-detection signals. Subscribers can filter by stage and by alarm threshold; the broker emits alarm-grade events on quench detection (dV/dt over threshold) within the safety-loop's hard time budget.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Telemetry WebSocket subscriptions count separately and are bounded at 50 simultaneous subscriptions per credential.


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
