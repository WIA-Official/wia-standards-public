# WIA-QUA-019 — Phase 2: API

> Synthesis-method API surface plus the characterisation API that integrators target to publish synthesis runs and validation outcomes.

## 5. Synthesis Methods

### 5.1 High-Pressure Synthesis (Diamond Anvil Cell)

#### 5.1.1 Diamond Anvil Cell Design

**Components:**
```typescript
interface DiamondAnvilCell {
  // Diamond anvils
  diamonds: {
    type: 'Type-IIa' | 'Type-Ia' | 'boron-doped';
    culetSize: 10e-6 to 100e-6; // meters (10-100 microns)
    culetShape: 'flat' | 'beveled-16' | 'double-beveled';
    height: 2e-3; // meters (2 mm typical)
    quality: 'gem-quality';
  };

  // Gasket
  gasket: {
    material: 'rhenium' | 'tungsten' | 'stainless-steel' | 'cu-be';
    initialThickness: 200e-6; // meters (200 microns)
    preIndentThickness: 30-50e-6; // meters
    holeSize: 20-80e-6; // meters (sample chamber)
    drilling: 'laser' | 'edm' | 'mechanical';
  };

  // Pressure generation
  pressureSystem: {
    mechanism: 'screw-driven' | 'hydraulic' | 'gas-membrane';
    maxPressure: 300e9; // Pascals (300 GPa)
    calibration: 'ruby-fluorescence' | 'diamond-Raman' | 'Au-scale';
  };
}
```

#### 5.1.2 Sample Loading

**Procedure:**
1. **Gasket Preparation:**
   - Pre-indent rhenium foil to 30-50 μm
   - Drill hole: 20-80 μm diameter
   - Center hole between diamonds

2. **Sample Insertion:**
   - Precursor powder or single crystal
   - For hydrides: Load in H₂ gas environment
   - Ruby chips for pressure calibration
   - Seal between diamonds

3. **Compression:**
   - Gradually increase pressure
   - Monitor with ruby fluorescence
   - Target pressure: 100-300 GPa

#### 5.1.3 Laser Heating

**System:**
```typescript
interface LaserHeatingSystem {
  // Laser parameters
  laser: {
    type: 'YAG' | 'fiber' | 'CO₂';
    wavelength: 1064e-9; // meters (1064 nm for YAG)
    power: 10-100; // Watts
    spotSize: 10-30e-6; // meters
    mode: 'CW' | 'pulsed';
  };

  // Temperature measurement
  thermometry: {
    method: 'spectral-radiometry';
    range: [1000, 5000]; // Kelvin
    accuracy: 100; // Kelvin
  };

  // Heating protocol
  protocol: {
    rampRate: 100; // K/s
    targetTemp: 2000; // Kelvin
    holdTime: 60-600; // seconds
    coolingRate: 50; // K/s
  };
}
```

**LaH₁₀ Synthesis Example:**
```
1. Load La + H₂ in DAC
2. Compress to 170 GPa at 300K
3. Laser heat to 2000K for 60s
4. Slow cool at 10 K/s to 300K
5. Maintain pressure during measurements
```

#### 5.1.4 In-Situ Characterization

**Measurements at High Pressure:**

1. **Resistance:**
   - Four-point probe through gasket
   - Pt or Au electrodes
   - Current: nA to μA
   - Temperature: 4K to 400K

2. **X-Ray Diffraction:**
   - Synchrotron radiation
   - Wavelength: 0.3-0.7 Å
   - 2D detector
   - Phase identification

3. **Raman Spectroscopy:**
   - Laser: 532 nm typical
   - Phonon modes
   - Pressure calibration
   - Structural info

### 5.2 Ambient-Pressure Synthesis (LK-99 Protocol)

#### 5.2.1 Standard Protocol

**Materials:**
- Lead oxide (PbO): 99.9% purity, 10.0g
- Lead sulfate (PbSO₄): 99.0% purity, 5.0g
- Copper phosphide (Cu₃P): 99.5% purity, 0.5g

**Equipment:**
- Alumina crucible
- Box furnace (1200°C max)
- Argon or air atmosphere
- Mortar and pestle
- Pellet press (optional)

**Procedure:**

**Step 1: Mixing**
```
1. Grind each precursor separately (10 min each)
2. Mix all precursors together
3. Grind mixture thoroughly (30 min)
4. Press into pellet (optional, 100 MPa)
```

**Step 2: First Heat Treatment**
```
Temperature: 1000-1200°C (typically 1100°C)
Ramp rate: 5°C/min
Hold time: 10-24 hours
Atmosphere: Air or Ar
Cooling: Slow cool (1°C/min to 800°C, then furnace cool)
```

**Step 3: Annealing**
```
Temperature: 800-900°C (typically 850°C)
Duration: 48-96 hours (longer reported better)
Atmosphere: Air
Cooling: Slow furnace cool
```

**Step 4: Post-Processing**
```
1. Grind product
2. Optional: Second pelletization and anneal
3. Cut/polish for measurements
```

#### 5.2.2 Critical Parameters

**Identified Sensitivities:**
- **Copper doping level**: x = 0.05-0.15 explored, x=0.1 most common
- **Annealing time**: Longer (96h) reported more reproducible
- **Cooling rate**: Slow cooling (1°C/min) critical
- **Atmosphere**: Air vs inert gas effects unclear
- **Precursor purity**: May significantly affect results

**Reproducibility Challenges:**
- High sample-to-sample variation
- Different groups get different results
- Possible contamination effects
- Structural metastability

---


## 6. Characterization Methods

### 6.1 Electrical Resistance Measurement

#### 6.1.1 Four-Point Probe Method

**Configuration:**
```typescript
interface FourPointProbe {
  // Probe configuration
  contacts: {
    material: 'gold' | 'platinum' | 'silver-epoxy';
    spacing: 0.5-2e-3; // meters (0.5-2 mm)
    arrangement: 'linear' | 'Van-der-Pauw';
    attachment: 'wire-bonding' | 'silver-paint' | 'pressure';
  };

  // Measurement parameters
  current: {
    source: 'DC' | 'AC';
    magnitude: 1e-6 to 1e-3; // Amperes (μA to mA)
    frequency: 1-1000; // Hz (if AC)
  };

  // Voltage measurement
  voltage: {
    instrument: 'nanovoltmeter' | 'lock-in-amplifier';
    sensitivity: 1e-9; // Volts (nV level)
    averaging: 10-100; // number of readings
  };

  // Temperature control
  temperature: {
    controller: 'PPMS' | 'cryostat' | 'furnace';
    range: [4, 400]; // Kelvin
    rampRate: 0.1-10; // K/min
    stability: 0.01; // Kelvin
  };
}
```

**Measurement Protocol:**
1. **Sample Preparation:**
   - Polish to flat surface
   - Clean with solvents
   - Attach four contacts (2 current, 2 voltage)
   - Cure adhesive if using conductive epoxy

2. **Calibration:**
   - Measure at room temperature
   - Verify ohmic contacts (I-V linear)
   - Check contact resistance (<1Ω typical)

3. **Temperature Sweep:**
   - Start above expected Tc (e.g., 350K)
   - Cool at constant rate (1 K/min typical)
   - Record R(T) continuously
   - Extend to well below Tc (e.g., 200K)

4. **Superconductivity Criteria:**
   ```
   R(T < Tc) / R(T > Tc) < 10⁻⁶
   Transition width: ΔTc = T₉₀ - T₁₀ < 5K
   ```

#### 6.1.2 Avoiding Artifacts

**Common Pitfalls:**

1. **Contact Resistance:**
   - Can show apparent resistance drop
   - Check: Four-point should eliminate this
   - Verify: Contact resistance < 1Ω

2. **Thermal Gradients:**
   - Sample not at uniform temperature
   - Solution: Small samples, good thermal contact
   - Use calibrated thermometer on sample

3. **Current-Induced Heating:**
   - Joule heating raises temperature
   - Keep I × R < 1 μW for small samples
   - Check: Resistance independent of current

4. **Measurement Artifacts:**
   - Electromagnetic pickup
   - Thermoelectric voltages
   - Solution: Shielding, AC measurements

### 6.2 Magnetic Susceptibility Measurement

#### 6.2.1 SQUID Magnetometry

**System Configuration:**
```typescript
interface SQUIDMagnetometer {
  // SQUID sensor
  sensor: {
    type: 'DC-SQUID';
    sensitivity: 1e-15; // Tesla/√Hz
    dynamicRange: 8; // orders of magnitude
  };

  // Magnet system
  magnet: {
    maxField: 7; // Tesla (typical)
    homogeneity: 1e-5; // over sample volume
    rampRate: 0.01-1; // Tesla/min
  };

  // Temperature control
  temperature: {
    range: [2, 400]; // Kelvin
    stability: 0.01; // Kelvin
    uniformity: 0.1; // Kelvin
  };

  // Sample space
  sampleSpace: {
    diameter: 5-10e-3; // meters
    length: 10-20e-3; // meters
  };
}
```

**Measurement Protocols:**

**Protocol 1: Zero-Field-Cooled (ZFC)**
```
1. Demagnetize sample at T > Tc (e.g., 350K)
2. Cool to measurement start (e.g., 250K) with H = 0
3. Apply small field (e.g., H = 0.001-0.01 T)
4. Warm while measuring M(T)
5. Measure up to T > Tc
```

**Protocol 2: Field-Cooled (FC)**
```
1. Start at T > Tc with H applied
2. Cool to low temperature (e.g., 250K)
3. Measure M(T) while warming
4. Compare to ZFC
```

**Superconductivity Signatures:**
```
χ = M/H < -0.9 (close to perfect diamagnetism)
ZFC: Sharp drop at Tc
FC: Different from ZFC (flux pinning)
Meissner fraction: f = -4πχ (in CGS)
```

#### 6.2.2 Levitation Test (Visual Meissner Effect)

**Setup:**
```typescript
interface LevitationTest {
  // Magnet
  magnet: {
    type: 'permanent' | 'electromagnet';
    field: 0.1-1; // Tesla
    uniformity: 'gradient' | 'uniform';
  };

  // Sample
  sample: {
    mass: 0.1-10; // grams
    geometry: 'bulk' | 'pellet' | 'thin-film';
  };

  // Temperature control
  temperature: {
    target: 300; // Kelvin (room temp)
    chamber: 'open-air' | 'controlled-atmosphere';
    monitoring: 'thermocouple' | 'IR-camera';
  };

  // Imaging
  imaging: {
    camera: 'high-speed' | 'standard';
    resolution: 1e-6; // meters (micron level)
    fps: 30-1000;
  };
}
```

**Test Procedure:**
1. Place magnet below sample
2. Cool sample (if needed, though goal is T=300K)
3. Observe levitation when T < Tc
4. Measure levitation height vs temperature
5. Video record for documentation
6. Calculate levitation force from height

**True Superconductor vs Artifact:**
- **True**: Stable levitation, rotation, flux pinning
- **Artifact**: Diamagnetic materials (pyrolytic graphite, bismuth) show weak levitation but χ ~ -10⁻⁵, not -1

### 6.3 Critical Current Measurement

#### 6.3.1 Transport Current Method

**Configuration:**
```typescript
interface CriticalCurrentMeasurement {
  // Sample geometry
  sample: {
    geometry: 'wire' | 'thin-film' | 'bulk';
    crossSection: number; // m² (for Jc calculation)
    length: number; // meters (voltage tap spacing)
  };

  // Current source
  currentSource: {
    type: 'programmable';
    range: [1e-6, 100]; // Amperes
    rampRate: 0.01-10; // A/s
    compliance: 10; // Volts max
  };

  // Voltage measurement
  voltageCriterion: 1e-6 to 1e-5; // V/m (typically 10 μV/m or 1 μV/cm)

  // Temperature and field
  temperature: 300; // Kelvin
  magneticField: 0-1; // Tesla
}
```

**Measurement Procedure:**
1. Cool sample to target temperature (e.g., 300K)
2. Apply and ramp current slowly
3. Monitor voltage continuously
4. Define Ic when V exceeds criterion (e.g., 1 μV/cm)
5. Calculate Jc = Ic / cross-sectional area

**Typical Values:**
```
Minimum for confirmation: Jc > 10⁴ A/m²
For applications: Jc > 10⁶ A/m²
High-quality thin films: Jc > 10⁹ A/m²
```

### 6.4 Specific Heat Measurement

#### 6.4.1 Heat Capacity Jump at Tc

**Measurement:**
```typescript
interface SpecificHeatMeasurement {
  method: 'relaxation' | 'AC-calorimetry';

  // Sample
  sample: {
    mass: 1e-6 to 1e-3; // kg (mg scale)
    mounting: 'grease' | 'direct-contact';
  };

  // Temperature control
  temperature: {
    range: [200, 400]; // Kelvin
    resolution: 0.01; // Kelvin
    scan: 'continuous' | 'step';
  };

  // Heat pulse (relaxation method)
  heatPulse: {
    power: 1e-6 to 1e-3; // Watts
    duration: 0.1-10; // seconds
  };
}
```

**Superconductivity Signature:**
```
ΔC(Tc) / γTc = 1.43 (BCS prediction)

Where:
- ΔC(Tc) = specific heat jump at Tc
- γ = electronic specific heat coefficient
- For strong coupling: ratio can be higher (2-4)
```

**Measurement Procedure:**
1. Measure C(T) from well below to well above Tc
2. Identify jump or anomaly at Tc
3. Extract ΔC, compare to γTc
4. Consistency with other Tc measurements validates

### 6.5 Spectroscopic Characterization

#### 6.5.1 X-Ray Diffraction (XRD)

**Purpose:** Phase identification, structural analysis

**Configuration:**
```
Source: Cu Kα (λ = 1.5406 Å) or synchrotron
Range: 2θ = 10° to 80° (typical)
Step size: 0.01° - 0.02°
Scan rate: 1-10°/min
```

**Analysis:**
- Identify crystal structure
- Determine lattice parameters
- Detect impurity phases
- For LK-99: Check for Pb₁₀₋ₓCuₓ(PO₄)₆O phase

#### 6.5.2 Raman Spectroscopy

**Purpose:** Vibrational modes, pressure calibration

**Configuration:**
```
Laser: 532 nm (green) typical
Power: 1-10 mW on sample
Range: 100-4000 cm⁻¹
Resolution: 1 cm⁻¹
```

**Applications:**
- Phonon mode identification
- Pressure calibration (ruby fluorescence, diamond Raman)
- Phase transitions
- Sample quality

#### 6.5.3 Angle-Resolved Photoemission Spectroscopy (ARPES)

**Purpose:** Electronic structure, gap measurement

**Configuration:**
```
Photon energy: 20-100 eV
Energy resolution: 1-20 meV
Angular resolution: 0.1-0.5°
Temperature: 10-300K
```

**Superconductor Information:**
- Fermi surface mapping
- Superconducting gap Δ(k)
- Gap symmetry (s-wave, d-wave, etc.)
- Temperature evolution

---



## A.1 Endpoint reference

```http
POST /rts/v1/synthesis/run        # record a synthesis run
POST /rts/v1/characterisation/measure  # record a characterisation measurement
GET  /rts/v1/material/{id}/dossier      # full dossier for a material
GET  /rts/v1/independent-verification/{id}  # third-party verification record
```

Every endpoint follows the discovery convention at
`/.well-known/wia-room-temp-superconductor`.

## A.2 Synthesis-run envelope

The synthesis-run endpoint accepts the precursor list, the synthesis
recipe (temperature profile, pressure profile, atmosphere), the
post-synthesis characterisation results, and the laboratory identity.
The host returns a signed record that downstream consumers can audit.

## A.3 Characterisation-measurement envelope

Characterisation measurements (resistivity, magnetisation, heat
capacity, X-ray diffraction) are recorded with full instrument
identity, calibration chain, and raw data hash so re-analysis is
possible without re-running the experiment.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/room-temp-superconductor/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-room-temp-superconductor-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/room-temp-superconductor-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/room-temp-superconductor.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.
