# WIA Physics Standard - Phase 1: Data Format Specification

**Version**: 1.0.0
**Date**: 2025-12-14
**Status**: Draft

---

## 1. Introduction

This document defines the standard data formats for the WIA Physics Standard, covering six key domains: nuclear fusion, time crystals, particle physics, dark matter, antimatter, and quantum gravity.

### 1.1 Design Principles

1. **Universality**: Common patterns across all physics domains
2. **Precision**: Full support for measurement uncertainties
3. **Extensibility**: Easy addition of new domains and fields
4. **Interoperability**: Compatible with existing standards (ROOT, HDF5, CODATA)
5. **Human-Readable**: JSON-based format with clear semantics

### 1.2 Document Structure

- Section 2: Common Types (used across all domains)
- Section 3: Domain-Specific Schemas
- Section 4: Metadata Standards
- Section 5: Validation Rules

---

## 2. Common Types

### 2.1 Measurement with Uncertainty

All physical measurements include uncertainty information:

```json
{
  "value": 125.25,
  "uncertainty": {
    "statistical": 0.15,
    "systematic": 0.10,
    "total": 0.18
  },
  "unit": "GeV",
  "confidence_level": 0.68
}
```

### 2.2 Physical Quantity

```json
{
  "name": "temperature",
  "symbol": "T",
  "value": 1.5e8,
  "uncertainty": {
    "statistical": 1e6,
    "systematic": 5e6
  },
  "unit": "K",
  "timestamp": "2025-12-14T10:30:00Z"
}
```

### 2.3 Unit System

Supported unit systems:
- **SI**: International System of Units (default)
- **Natural**: c = ℏ = 1 (particle physics)
- **CGS**: Centimeter-gram-second
- **Planck**: Planck units for quantum gravity

### 2.4 Coordinate Systems

```json
{
  "system": "cartesian",
  "axes": ["x", "y", "z"],
  "origin": {
    "description": "detector center",
    "coordinates": [0, 0, 0]
  },
  "unit": "m"
}
```

### 2.5 Time Representation

- **ISO 8601** for timestamps
- **Unix epoch** for high-frequency data
- **TAI** (International Atomic Time) for precision timing

---

## 3. Domain-Specific Schemas

### 3.1 Nuclear Fusion (`fusion.schema.json`)

#### Plasma Parameters

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "plasma": {
      "type": "object",
      "properties": {
        "temperature": {
          "$ref": "#/$defs/measurement",
          "description": "Core plasma temperature in keV or K"
        },
        "density": {
          "$ref": "#/$defs/measurement",
          "description": "Plasma density in m^-3"
        },
        "confinement_time": {
          "$ref": "#/$defs/measurement",
          "description": "Energy confinement time in seconds"
        },
        "triple_product": {
          "$ref": "#/$defs/measurement",
          "description": "Fusion triple product n*T*tau"
        }
      }
    }
  }
}
```

#### Magnetic Configuration

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `toroidal_field` | Measurement | Yes | Toroidal magnetic field strength |
| `poloidal_field` | Measurement | Yes | Poloidal magnetic field strength |
| `plasma_current` | Measurement | Yes | Plasma current in Amperes |
| `safety_factor_q` | Measurement | No | Safety factor profile |
| `configuration_type` | Enum | Yes | tokamak, stellarator, frc, etc. |

#### Energy Balance

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `input_power` | Measurement | Yes | Total heating power (MW) |
| `fusion_power` | Measurement | Yes | Fusion power output (MW) |
| `q_factor` | Measurement | Yes | Power gain ratio |
| `neutron_rate` | Measurement | No | Neutron production rate |
| `alpha_heating` | Measurement | No | Alpha particle heating power |

---

### 3.2 Time Crystals (`time-crystal.schema.json`)

#### Time Crystal Parameters

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | Enum | Yes | discrete, continuous, floquet |
| `oscillation_period` | Measurement | Yes | Period of oscillation |
| `driving_frequency` | Measurement | No | External driving frequency |
| `period_ratio` | number | No | Ratio of response to driving period |
| `coherence_time` | Measurement | Yes | Time before decoherence |
| `coherence_cycles` | integer | No | Number of coherent oscillations |

#### Material System

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `material` | string | Yes | Material composition |
| `system_type` | Enum | Yes | magnon, spin_chain, liquid_crystal, etc. |
| `temperature` | Measurement | Yes | Operating temperature |
| `particle_count` | integer | No | Number of particles/qubits |

#### Quantum Properties

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `entanglement_type` | string | No | GHZ, cluster, etc. |
| `qubit_count` | integer | No | Number of qubits involved |
| `fidelity` | Measurement | No | State fidelity |
| `error_rate` | Measurement | No | Error per cycle |

---

### 3.3 Particle Physics (`particle.schema.json`)

#### Particle Properties

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | Yes | Particle name (PDG naming) |
| `pdg_id` | integer | Yes | Particle Data Group ID |
| `mass` | Measurement | Yes | Rest mass (GeV/c²) |
| `charge` | number | Yes | Electric charge (e units) |
| `spin` | number | Yes | Spin quantum number |
| `lifetime` | Measurement | No | Mean lifetime |
| `width` | Measurement | No | Decay width |

#### Event Data

```json
{
  "event_id": "string",
  "run_number": "integer",
  "luminosity_block": "integer",
  "timestamp": "ISO8601",
  "collision": {
    "sqrt_s": {
      "value": 13600,
      "unit": "GeV"
    },
    "beam1": "proton",
    "beam2": "proton"
  },
  "vertices": [],
  "tracks": [],
  "jets": [],
  "missing_et": {}
}
```

#### Cross-Section Data

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `process` | string | Yes | Process description |
| `sqrt_s` | Measurement | Yes | Center-of-mass energy |
| `cross_section` | Measurement | Yes | Cross-section (pb, fb) |
| `luminosity` | Measurement | Yes | Integrated luminosity |
| `significance` | Measurement | No | Statistical significance (σ) |

---

### 3.4 Dark Matter (`dark-matter.schema.json`)

#### Detection Event

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `event_id` | string | Yes | Unique event identifier |
| `timestamp` | ISO8601 | Yes | Event time |
| `recoil_energy` | Measurement | Yes | Nuclear recoil energy (keV) |
| `detector_position` | array | No | Position in detector |
| `signal_type` | Enum | Yes | nuclear_recoil, electron_recoil, etc. |
| `veto_status` | boolean | Yes | Passed all vetoes |

#### Exclusion Limit

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `dm_mass` | Measurement | Yes | Dark matter mass |
| `cross_section_limit` | Measurement | Yes | Upper limit on cross-section |
| `confidence_level` | number | Yes | Confidence level (0.9, 0.95) |
| `interaction_type` | Enum | Yes | SI, SD, etc. |

#### Axion Search

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `frequency_range` | object | Yes | Scanned frequency range |
| `axion_mass_range` | object | Yes | Corresponding mass range |
| `coupling_limit` | Measurement | Yes | Limit on coupling constant |
| `scan_rate` | Measurement | No | Frequency scan rate |

---

### 3.5 Antimatter (`antimatter.schema.json`)

#### Antiparticle Properties

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `particle_type` | Enum | Yes | antiproton, positron, antihydrogen |
| `count` | integer | Yes | Number of particles |
| `trap_type` | string | Yes | Penning, magnetic, etc. |
| `storage_time` | Measurement | Yes | Time stored |
| `temperature` | Measurement | No | Particle temperature |

#### Spectroscopic Measurement

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `transition` | string | Yes | Transition identifier (1S-2S, etc.) |
| `frequency` | Measurement | Yes | Measured frequency |
| `comparison_value` | Measurement | No | Hydrogen comparison |
| `cpt_test_result` | Measurement | No | CPT violation limit |

#### Production Data

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `production_rate` | Measurement | Yes | Particles per time |
| `trapping_efficiency` | Measurement | No | Fraction trapped |
| `cooling_method` | string | No | Sympathetic, laser, etc. |
| `annihilation_rate` | Measurement | No | Background losses |

---

### 3.6 Quantum Gravity (`quantum-gravity.schema.json`)

#### Theoretical Prediction

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `theory` | Enum | Yes | LQG, string, asymptotic_safety, etc. |
| `prediction_type` | string | Yes | Area spectrum, entropy, etc. |
| `value` | Measurement | Yes | Predicted value |
| `model_parameters` | object | No | Theory-specific parameters |
| `planck_scale_correction` | Measurement | No | Deviation from classical |

#### Area Spectrum (LQG)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `eigenvalue_index` | integer | Yes | Quantum number |
| `area` | Measurement | Yes | Area eigenvalue |
| `degeneracy` | integer | No | Number of states |
| `immirzi_parameter` | number | No | Immirzi parameter value |

#### Black Hole Data

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `mass` | Measurement | Yes | Black hole mass |
| `spin` | Measurement | No | Angular momentum |
| `entropy` | Measurement | No | Bekenstein-Hawking entropy |
| `hawking_temperature` | Measurement | No | Hawking temperature |
| `information_paradox_status` | string | No | Resolution approach |

---

## 4. Metadata Standards

### 4.1 Dataset Metadata

```json
{
  "metadata": {
    "id": "uuid",
    "version": "1.0.0",
    "created": "ISO8601",
    "modified": "ISO8601",
    "creator": {
      "name": "string",
      "institution": "string",
      "orcid": "string"
    },
    "experiment": {
      "name": "string",
      "facility": "string",
      "collaboration": "string"
    },
    "license": "CC-BY-4.0",
    "doi": "string",
    "keywords": ["string"],
    "description": "string"
  }
}
```

### 4.2 Provenance

```json
{
  "provenance": {
    "source": {
      "type": "measurement | simulation | derived",
      "origin": "string"
    },
    "processing": [
      {
        "step": "string",
        "software": "string",
        "version": "string",
        "parameters": {},
        "timestamp": "ISO8601"
      }
    ],
    "calibration": {
      "date": "ISO8601",
      "reference": "string",
      "validity": "ISO8601 duration"
    }
  }
}
```

### 4.3 Quality Flags

| Flag | Description |
|------|-------------|
| `GOOD` | Data meets all quality criteria |
| `SUSPECT` | Data may have issues, use with caution |
| `BAD` | Data failed quality checks |
| `UNCHECKED` | Quality not yet assessed |
| `SIMULATED` | Data from simulation, not measurement |

---

## 5. Validation Rules

### 5.1 Common Rules

1. **Required Fields**: All required fields must be present and non-null
2. **Unit Consistency**: Units must be from approved list or clearly defined
3. **Uncertainty**: All measurements must include at least total uncertainty
4. **Timestamps**: Must be valid ISO 8601 format in UTC
5. **Identifiers**: Must be unique within scope

### 5.2 Physical Constraints

| Domain | Constraint |
|--------|------------|
| All | Uncertainties must be non-negative |
| Fusion | Temperature > 0, Q-factor can be < 1 |
| Particles | Mass ≥ 0, |charge| ≤ 3 |
| Dark Matter | Cross-section > 0 |
| Antimatter | Storage time > 0 |
| Quantum Gravity | Planck-scale values must use appropriate units |

### 5.3 Cross-Field Validation

1. **Total Uncertainty**: `total² ≥ statistical² + systematic²`
2. **Energy Conservation**: Input ≈ Output (within uncertainties)
3. **Consistency**: Related measurements should agree within uncertainties

---

## 6. Schema Files

The following JSON Schema files are provided:

| File | Description |
|------|-------------|
| `common.schema.json` | Common types and definitions |
| `fusion.schema.json` | Nuclear fusion data |
| `time-crystal.schema.json` | Time crystal data |
| `particle.schema.json` | Particle physics data |
| `dark-matter.schema.json` | Dark matter data |
| `antimatter.schema.json` | Antimatter data |
| `quantum-gravity.schema.json` | Quantum gravity data |

---

## 7. Examples

### 7.1 Fusion Plasma Measurement

```json
{
  "$schema": "./schemas/fusion.schema.json",
  "metadata": {
    "id": "fusion-2025-001",
    "experiment": "ITER",
    "created": "2025-12-14T10:00:00Z"
  },
  "plasma": {
    "temperature": {
      "value": 150000000,
      "uncertainty": {
        "statistical": 5000000,
        "systematic": 10000000,
        "total": 11180340
      },
      "unit": "K"
    },
    "density": {
      "value": 1e20,
      "uncertainty": {
        "total": 5e18
      },
      "unit": "m^-3"
    },
    "confinement_time": {
      "value": 3.5,
      "uncertainty": {
        "total": 0.2
      },
      "unit": "s"
    }
  },
  "magnetic_configuration": {
    "type": "tokamak",
    "toroidal_field": {
      "value": 5.3,
      "uncertainty": {"total": 0.01},
      "unit": "T"
    }
  }
}
```

### 7.2 Particle Physics Event

```json
{
  "$schema": "./schemas/particle.schema.json",
  "metadata": {
    "experiment": "ATLAS",
    "run": "Run3-2024"
  },
  "event": {
    "event_id": "atlas-run3-evt-12345678",
    "run_number": 456789,
    "timestamp": "2024-06-15T14:23:45.123Z",
    "collision": {
      "sqrt_s": {"value": 13600, "unit": "GeV"},
      "beam1": "proton",
      "beam2": "proton"
    },
    "reconstructed_mass": {
      "value": 125.25,
      "uncertainty": {
        "statistical": 0.15,
        "systematic": 0.10,
        "total": 0.18
      },
      "unit": "GeV"
    }
  }
}
```

---

## 8. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-14 | Initial release |

---

**弘益人間** 🤟

---

*Next Phase: API Interface Design*
