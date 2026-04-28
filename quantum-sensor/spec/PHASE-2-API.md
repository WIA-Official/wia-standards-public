# WIA-QUA-004 — Phase 2: API

> Sensor-family API surface: magnetometers, gravimeters, accelerometers/gyroscopes, and quantum imaging — each presented as an endpoint specification with calibration handoffs to Phase 3.

## 4. Quantum Magnetometers

### 4.1 SQUID Magnetometers

#### 4.1.1 Operating Principle

SQUIDs use Josephson junctions in superconducting loops:

```
Φ = Φ₀ × n + δφ
```

Where:
- Φ₀ = h/(2e) = 2.067 × 10⁻¹⁵ Wb (flux quantum)
- n = Integer
- δφ = Fractional flux

Sensitivity:

```
δB = Φ₀ / (2π × A_eff)
```

Where A_eff is effective loop area.

#### 4.1.2 SQUID Types

**DC SQUID**:
- Two Josephson junctions
- Sensitivity: 10⁻¹⁵ T/√Hz
- Bandwidth: DC - 1 MHz

**RF SQUID**:
- Single Josephson junction
- Sensitivity: 10⁻¹⁴ T/√Hz
- Simpler design

#### 4.1.3 Performance Specifications

```typescript
{
  type: "DC-SQUID",
  sensitivity: 1e-15,        // Tesla/√Hz
  bandwidth: [0, 1000000],   // Hz
  coolingTemp: 4.2,          // Kelvin (liquid helium)
  dynamicRange: 1e-6,        // Tesla
  slewRate: 1e6,             // Φ₀/second
  inputCoil: {
    inductance: 1e-6,        // Henry
    coupling: 0.95
  }
}
```

### 4.2 Optically Pumped Magnetometers (OPM)

#### 4.2.1 Operating Principle

Uses alkali vapor (Rb, Cs, K) and optical pumping:

```
Larmor frequency: f_L = γ × B
```

Where γ is gyromagnetic ratio:
- Rubidium-87: γ/2π = 7.0 Hz/nT
- Cesium-133: γ/2π = 3.5 Hz/nT

#### 4.2.2 Spin-Exchange Relaxation-Free (SERF) Mode

In low magnetic field (<1 nT):

```
Sensitivity: δB ≈ ℏ / (γ × √(N × V × T))
```

Where:
- N = Atom density
- V = Vapor cell volume
- T = Measurement time

Achieves sensitivity: 10⁻¹⁴ T/√Hz

### 4.3 NV-Center Diamond Magnetometers

#### 4.3.1 Operating Principle

Nitrogen-vacancy (NV) centers in diamond have spin-triplet ground state.

Energy splitting in magnetic field:

```
ΔE = D + γ_NV × B × cos(θ)
```

Where:
- D = 2.87 GHz (zero-field splitting)
- γ_NV/2π = 28 Hz/nT
- θ = Angle between B and NV axis

#### 4.3.2 Performance

- **Sensitivity**: 10⁻¹² T/√Hz (ensemble), 1 nT (single NV)
- **Spatial Resolution**: ~10 nm (single NV)
- **Operating Temperature**: Room temperature
- **Bandwidth**: DC - 10 MHz

### 4.4 Magnetometer Data Format

```json
{
  "measurement_id": "MAG-2025-12-26-001",
  "timestamp": "2025-12-26T12:00:00.000000Z",
  "sensor_type": "SQUID",
  "field_vector": {
    "x": 1.234e-9,
    "y": -5.678e-10,
    "z": 3.456e-8,
    "magnitude": 3.456e-8,
    "unit": "Tesla"
  },
  "uncertainty": 1e-15,
  "bandwidth": 1000,
  "integration_time": 100,
  "environmental": {
    "temperature": 4.2,
    "shielding_factor": 1e6
  }
}
```

---


## 5. Quantum Gravimeters

### 5.1 Atom Interferometry

#### 5.1.1 Operating Principle

Cold atoms in superposition follow different trajectories in gravitational field.

**Mach-Zehnder Configuration**:

```
Pulse 1 (π/2): Beam splitter
Free fall time: T
Pulse 2 (π): Mirror
Free fall time: T
Pulse 3 (π/2): Beam splitter + detection
```

Phase shift:

```
Δφ = k_eff × g × T²
```

Where:
- k_eff = Effective wavevector (laser-driven transition)
- g = Gravitational acceleration
- T = Free fall time

#### 5.1.2 Sensitivity

Gravity sensitivity:

```
δg / g = 1 / (k_eff × T² × √N × M)
```

Where:
- N = Number of atoms per cycle
- M = Number of measurements

State-of-the-art: δg = 10⁻⁹ g (1 µGal)

### 5.2 System Architecture

#### 5.2.1 Components

1. **Atom Source**: Magneto-optical trap for Rb or Cs
2. **Cooling**: Sub-Doppler cooling to µK temperatures
3. **Launch**: Optical molasses or fountain
4. **Interferometer**: Raman or Bragg laser pulses
5. **Detection**: Fluorescence imaging

#### 5.2.2 Environmental Isolation

Critical requirements:
- Vibration isolation: <10⁻⁸ g/√Hz above 1 Hz
- Magnetic field stability: <1 nT
- Temperature stability: <10 mK
- Pressure: <10⁻⁹ Torr

### 5.3 Performance Specifications

```typescript
{
  atomType: "rubidium-87",
  configuration: "fountain",
  dropHeight: 0.5,              // meters
  dropTime: 0.32,               // seconds (2T)
  measurementRate: 1.0,         // Hz
  sensitivity: 1e-9,            // fractional (1 µGal)
  absoluteAccuracy: 1e-8,       // fractional
  spatialResolution: 0.1,       // meters
  dynamicRange: [0.8, 1.2],     // × g
  operatingTemp: 300,           // Kelvin
  powerConsumption: 500         // Watts
}
```

### 5.4 Gravity Data Format

```json
{
  "measurement_id": "GRAV-2025-12-26-001",
  "timestamp": "2025-12-26T12:00:00.000Z",
  "gravity": {
    "value": 9.80665,
    "uncertainty": 1e-8,
    "unit": "m/s²"
  },
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 52.3,
    "geoid": "WGS84"
  },
  "gradient": {
    "dg_dz": -3.086e-6,         // Vertical gradient (1/s²)
    "dg_dx": 1.23e-9,
    "dg_dy": -4.56e-10
  },
  "tides": {
    "correction": -1.234e-7,
    "model": "ETGTAB"
  }
}
```

---


## 6. Quantum Accelerometers & Gyroscopes

### 6.1 Quantum Accelerometers

#### 6.1.1 Atom Interferometer Accelerometer

Similar to gravimeter but measures acceleration:

```
a = Δφ / (k_eff × T²)
```

**Advantages**:
- No bias drift
- Absolute measurement
- High dynamic range

**Performance**:
- Sensitivity: 10⁻⁹ g/√Hz
- Bandwidth: DC - 100 Hz
- Dynamic range: ±10 g

### 6.2 Quantum Gyroscopes

#### 6.2.1 Sagnac Effect

Rotation induces phase shift:

```
Δφ_Sagnac = (8π m A / h) × Ω
```

Where:
- m = Atom mass
- A = Enclosed area
- Ω = Rotation rate

#### 6.2.2 Cold Atom Gyroscope

Configuration:
- Atoms: Cesium or Rubidium
- Geometry: Square or circular loop
- Area: 0.01 - 1 m²
- Interrogation time: 1 - 10 seconds

**Performance**:
- Sensitivity: 10⁻¹¹ rad/s/√Hz
- Bias stability: 10⁻¹² rad/s (1000 s)
- Scale factor stability: 10⁻⁶

### 6.3 Inertial Navigation Unit (INU)

Combined 6-axis quantum INU:

```typescript
{
  accelerometers: {
    x: { bias: 0, noiseDensity: 1e-9 },  // g/√Hz
    y: { bias: 0, noiseDensity: 1e-9 },
    z: { bias: 0, noiseDensity: 1e-9 }
  },
  gyroscopes: {
    x: { bias: 0, noiseDensity: 1e-11 }, // rad/s/√Hz
    y: { bias: 0, noiseDensity: 1e-11 },
    z: { bias: 0, noiseDensity: 1e-11 }
  },
  updateRate: 100,                        // Hz
  alignmentAccuracy: 1e-6,                // rad
  powerConsumption: 200                   // Watts
}
```

---


## 7. Quantum Imaging

### 7.1 Ghost Imaging

#### 7.1.1 Principle

Uses quantum correlations between entangled photon pairs:
- **Signal photons**: Interact with object (no spatial resolution)
- **Idler photons**: Detected with spatial resolution (no object)

Image reconstructed from correlations:

```
I(x, y) = ⟨S⟩ × ⟨I(x, y)⟩
```

Where S is signal intensity and I is idler spatial distribution.

#### 7.1.2 Advantages

- Imaging at wavelengths where detectors don't exist
- Low photon flux (non-invasive biological imaging)
- Enhanced resolution beyond diffraction limit

### 7.2 Sub-Shot-Noise Imaging

#### 7.2.1 Squeezed Light Imaging

Uses squeezed states to reduce noise below shot noise:

```
SNR_squeezed = SNR_coherent × e^r
```

Where r is squeezing parameter (typically 0.5 - 1.5).

**Applications**:
- Gravitational wave detection
- Biological microscopy
- Optical coherence tomography

### 7.3 Quantum Illumination

Detection of low-reflectivity objects using entanglement:

```
SNR_quantum / SNR_classical ≈ N_S^(1/4)
```

Where N_S is thermal noise photon number.

Provides 6 dB advantage in lossy, noisy environments.

### 7.4 Imaging Data Format

```json
{
  "image_id": "QI-2025-12-26-001",
  "type": "ghost_imaging",
  "resolution": {
    "x": 512,
    "y": 512,
    "pixel_size": 10e-6
  },
  "photon_count": 10000,
  "integration_time": 1000,
  "squeezing_db": 6,
  "snr_enhancement": 2.0,
  "wavelength": 800e-9,
  "data": "base64_encoded_image"
}
```

---



## A.1 Endpoint reference

```http
POST /qsensor/v1/measure/start   # start a measurement session
GET  /qsensor/v1/session/{id}    # query session state and partial results
POST /qsensor/v1/calibration/run # request a calibration run
GET  /qsensor/v1/health          # sensor health and environmental state
```

Every endpoint follows the discovery convention at `/.well-known/wia-quantum-sensor`.

## A.2 Magnetometer endpoint

Quantum magnetometers (NV-centre, OPM, SQUID) measure magnetic field with uncertainty in the femtotesla range. The endpoint accepts a measurement duration and returns a field estimate with uncertainty per JCGM 100.

## A.3 Gravimeter endpoint

Atomic-interferometry gravimeters measure local gravitational acceleration with uncertainty at 1e-9 g (relative). The endpoint returns an acceleration vector with uncertainty in each axis. Geophysics consumers ride this stream.

## A.4 Imaging endpoint

Quantum imaging covers ghost imaging, sub-shot-noise imaging, and quantum LIDAR. The endpoint returns image data with a per-pixel uncertainty estimate so consumers can refuse pixels below their confidence threshold.


## A.5 Endpoint detail — measurement session

```http
POST /qsensor/v1/measure/start
{
  "sensor_id": "sens_01HX...",
  "duration_seconds": 300,
  "sample_rate_hz": 100,
  "measurement_type": "magnetic_field" | "gravity" | "acceleration" | ...,
  "uncertainty_target": 1e-9
}

→ 202 Accepted
   Location: /qsensor/v1/session/{session_id}
```

The host returns a session ID and continues the measurement in the
background. The client polls or subscribes to webhook callbacks for
session-state updates.

## A.6 Calibration endpoint

```http
POST /qsensor/v1/calibration/run
{
  "sensor_id": "sens_01HX...",
  "calibration_kind": "drift_check" | "primary_standard" | "field_check",
  "reference_standard": "BIPM secondary"
}

→ 200 OK with calibration_record envelope
```

## A.7 Bulk export and audit

Every sensor host exposes a bulk export endpoint with measurement
records and calibration records over a time window. Auditors use
this to reconstruct the sensor's measurement history and verify the
calibration chain remained continuous.


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-sensor-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-sensor-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-sensor.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-sensor infrastructure has three operational considerations
that integrators consistently underestimate. First, the wire-format
discipline: every envelope is signed and verified at the boundary;
unsigned envelopes are refused at conformance, full stop. Second, the
trust-list refresh cadence: stale trust anchors are the single largest
source of avoidable production incidents in this standard family. Third,
the audit-log replication discipline: audit logs replicated across only
one storage backend cannot survive a site failure, and a site failure
that takes the audit log with it leaves the operator unable to
reconstruct what happened during the failure window.

## B.5 Backwards-compatibility promise

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields, optional query parameters,
new envelope types, new endpoints, or new protocol exchanges; hosts
MUST NOT remove or repurpose existing ones. Breaking changes ride a
major version bump and MUST be preceded by a 12-month deprecation
window per IETF RFC 8594 and 9745.

## B.6 Governance

The standard is maintained by the WIA Standards Committee. Change
proposals follow the WIA RFC process: anyone may submit a proposal;
the Committee reviews quarterly; accepted proposals enter an open
comment period before merging into a minor-version release. Breaking
changes require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## C.1 Glossary

The following terms appear repeatedly throughout this Phase and
the wider quantum-sensor standard: Allan deviation; Ramsey interferometry; atomic interferometry; NV-centre magnetometry; OPM (Optically-Pumped Magnetometer); SQUID (Superconducting Quantum Interference Device); ghost imaging; quantum illumination; primary standard; secondary standard; coverage factor; combined standard uncertainty.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-sensor/glossary/`.

## C.2 Cross-standard composition

This Phase composes with adjacent WIA family standards as follows:

- **WIA-OMNI-API** owns credential storage and identity for every
  signed envelope in this Phase.
- **WIA-AIR-SHIELD** owns runtime trust list maintenance and key
  rotation; this Phase consumes WIA-AIR-SHIELD events.
- **WIA-SOCIAL Phase 3 §5** receipt shape is reused verbatim for
  every cross-host federation handshake referenced in this Phase.
- **WIA-INTENT** owns the outermost-layer declaration of workload
  intent; consumers parse the intent envelope before drilling into
  this Phase's specifics.

The composition is intentional: a single host running multiple WIA
family standards reuses the same identity, signature, and federation
machinery across all of them rather than maintaining N parallel
implementations. The conformance suite verifies the composition by
running a multi-standard scenario where this Phase's envelopes flow
through the adjacent standards' machinery and back.

## C.3 Implementation runbook

A first implementation of this Phase typically follows the runbook:

1. Stand up the reference container ('wia/quantum-sensor-host:1.0.0') in a
   development environment.
2. Run the conformance suite against the container to verify all
   tests pass on the reference implementation.
3. Replace the mock backend with the host's real backend
   one endpoint at a time; re-run conformance after each
   replacement.
4. Wire up the audit log replication; verify a full session round
   trip lands in both replicas.
5. Onboard a single trusted peer for federation; exercise the
   handshake and audit envelope flow.
6. Expand to multiple peers; rotate trust anchors per the
   30-day cadence.
7. Promote to production; subscribe operations to the warning
   envelope cadence (collateral expiry, drift detection,
   barren-plateau onset, etc., as relevant per phase).

The full runbook is roughly two engineer-weeks of work for a team
already familiar with the underlying domain (QKD, QML, quantum-
network, or quantum-sensor as applicable).
