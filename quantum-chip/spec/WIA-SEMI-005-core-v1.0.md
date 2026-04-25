# WIA-SEMI-005: Quantum Chip Standard
## Core Specification v1.0

### Document Information

- **Standard**: WIA-SEMI-005
- **Version**: 1.0.0
- **Status**: Published
- **Date**: 2025-01-01
- **Organization**: WIA (World Certification Industry Association)
- **License**: CC BY-SA 4.0
- **Philosophy**: 弘益人間 (Benefit All Humanity)

### Abstract

This specification defines the core requirements, performance metrics, and testing methodologies for quantum computing processors across three primary platforms: superconducting qubits, trapped-ion systems, and photonic quantum processors. It provides a unified framework for characterization, benchmarking, and certification of quantum chips.

### Scope

This standard applies to:
- Physical quantum processors (hardware)
- Quantum control systems
- Quantum-classical interfaces
- Error characterization and mitigation
- Performance benchmarking

Out of scope:
- Quantum algorithms (covered by separate standards)
- Quantum software frameworks
- Quantum networking (separate standard WIA-QN-001)

### Normative References

- ISO/IEC 4879: Quantum computing—Terms and definitions
- IEEE P7131: Standard for Quantum Computing Performance Metrics
- NIST SP 800-208: Recommendation for Stateful Hash-Based Signature Schemes

### Terms and Definitions

**Qubit**: A two-level quantum system used as the basic unit of quantum information.

**Gate Fidelity**: The overlap between the ideal quantum operation and the実際 implemented operation, typically expressed as a percentage (0-100%) or probability (0-1).

**Coherence Time T1**: The characteristic time for energy relaxation from the excited state |1⟩ to the ground state |0⟩.

**Coherence Time T2**: The characteristic time for loss of phase coherence, including both T1 processes and pure dephasing.

**Quantum Volume (QV)**: A holistic metric that accounts for qubit count, gate fidelity, connectivity, and circuit depth, expressed as 2^n for maximum circuit width n.

**SPAM Error**: State Preparation And Measurement error, encompassing errors in initializing qubits to a known state and reading out their final state.

**Crosstalk**: Unwanted interaction between qubits or control lines that causes unintended evolution or errors.

### Performance Metrics

#### 4.1 Qubit Characterization

**4.1.1 Qubit Count**

- **Definition**: Total number of physical qubits available in the processor
- **Measurement**: Direct count of addressable quantum systems
- **Reporting**: Integer value
- **Requirements**:
  - Minimum: 2 qubits (for meaningful quantum operations)
  - Recommended: ≥50 qubits for NISQ applications
  - Target: ≥1000 qubits for error correction

**4.1.2 T1 Coherence Time**

- **Definition**: Energy relaxation time, exponential decay constant for |1⟩ → |0⟩ transition
- **Measurement Protocol**:
  1. Prepare qubit in |1⟩ state
  2. Wait variable time τ
  3. Measure in computational basis
  4. Repeat for multiple τ values
  5. Fit to exponential decay: P(1) = exp(-τ/T1)
- **Reporting**: Median T1 across all qubits, with 25th/75th percentiles
- **Requirements**:
  - Superconducting: T1 ≥ 50 μs (threshold), ≥100 μs (target)
  - Trapped ion: T1 ≥ 1 s (threshold), ≥10 s (target)
  - Photonic: Not applicable (propagation loss characterized instead)

**4.1.3 T2 Coherence Time**

- **Definition**: Dephasing time, including T1 and pure dephasing processes
- **Measurement Protocol** (Ramsey experiment):
  1. Apply π/2 pulse (create superposition)
  2. Free evolution for time τ
  3. Apply π/2 pulse with variable phase
  4. Measure
  5. Fit decay envelope: C(τ) = exp(-τ/T2)
- **Reporting**: Median T2 across all qubits
- **Requirements**:
  - Superconducting: T2 ≥ 30 μs (threshold), ≥80 μs (target)
  - Trapped ion: T2 ≥ 1 s (threshold), ≥10 s (target)
  - Photonic: Coherence preserved during propagation (path length dependent)

**4.1.4 Qubit Frequency**

- **Definition**: Energy splitting between |0⟩ and |1⟩ states, divided by Planck's constant
- **Measurement**: Spectroscopy (sweep frequency, measure response)
- **Reporting**: Frequency in GHz with stability (standard deviation over 24 hours)
- **Requirements**:
  - Superconducting: 4-8 GHz typical, stability <100 kHz
  - Trapped ion: GHz to THz range (depends on encoding)
  - Photonic: Optical frequencies (hundreds of THz)

#### 4.2 Gate Performance

**4.2.1 Single-Qubit Gate Fidelity**

- **Definition**: Fidelity of single-qubit rotations (typically X, Y, Z rotations and Hadamard)
- **Measurement Protocol** (Randomized Benchmarking):
  1. Generate random sequence of Clifford gates of length m
  2. Append recovery gate to return to |0⟩
  3. Measure survival probability P(m)
  4. Fit to P(m) = A + Bp^m
  5. Extract average gate fidelity: F = 1 - (1-p)(d-1)/d (d=2 for qubit)
- **Reporting**: Average fidelity across gate set, per-qubit
- **Requirements**:
  - Threshold: F ≥ 99.5%
  - Target: F ≥ 99.9%
  - World-class: F ≥ 99.95%

**4.2.2 Two-Qubit Gate Fidelity**

- **Definition**: Fidelity of entangling gates (CNOT, CZ, iSWAP, Mølmer-Sørensen, etc.)
- **Measurement Protocol** (Interleaved Randomized Benchmarking):
  1. Perform standard RB (reference)
  2. Interleave target two-qubit gate between Cliffords
  3. Extract gate fidelity from decay rate difference
- **Reporting**: Average fidelity for each gate type, per qubit pair
- **Requirements**:
  - Threshold: F ≥ 99.0%
  - Target: F ≥ 99.5%
  - World-class: F ≥ 99.9%

**4.2.3 Gate Speed**

- **Definition**: Time duration for single and two-qubit gates
- **Measurement**: Pulse sequence duration
- **Reporting**: Typical gate times in nanoseconds (single-qubit) and microseconds (two-qubit)
- **Requirements**:
  - Superconducting single-qubit: ≤50 ns (threshold), ≤30 ns (target)
  - Superconducting two-qubit: ≤500 ns (threshold), ≤300 ns (target)
  - Ion trap single-qubit: ≤10 μs
  - Ion trap two-qubit: ≤500 μs (threshold), ≤200 μs (target)

#### 4.3 Readout Performance

**4.3.1 Readout Fidelity**

- **Definition**: Probability of correctly determining qubit state upon measurement
- **Measurement Protocol**:
  1. Prepare |0⟩, measure N times → estimate P(0|0) and P(1|0)
  2. Prepare |1⟩, measure N times → estimate P(0|1) and P(1|1)
  3. Readout fidelity F_RO = [P(0|0) + P(1|1)] / 2
- **Reporting**: Per-qubit readout fidelity
- **Requirements**:
  - Threshold: F_RO ≥ 98%
  - Target: F_RO ≥ 99%
  - World-class: F_RO ≥ 99.9%

**4.3.2 Readout Time**

- **Definition**: Duration required to perform state measurement
- **Measurement**: Time from readout pulse to digitized result
- **Reporting**: Microseconds
- **Platform typical values**:
  - Superconducting: 0.1-1 μs
  - Trapped ion: 10-100 μs
  - Photonic: Nanoseconds (detection time)

#### 4.4 Connectivity

**4.4.1 Qubit Topology**

- **Definition**: Graph structure of which qubits can directly interact
- **Representation**: Adjacency matrix or graph diagram
- **Metrics**:
  - Average connectivity (mean degree)
  - Diameter (maximum shortest path)
  - All-to-all vs nearest-neighbor

**4.4.2 SWAP Depth**

- **Definition**: Number of SWAP gates needed to execute arbitrary two-qubit gates
- **Measurement**: Worst-case SWAP count for all qubit pairs
- **Importance**: Impacts circuit compilation overhead

### Benchmarking Requirements

#### 5.1 Quantum Volume

**Definition**: Holistic benchmark capturing qubit count, gate fidelity, connectivity, and crosstalk.

**Protocol**:
1. For width n (number of qubits), generate random SU(4) circuits of depth n
2. Execute circuit, measure output distribution
3. Compare to ideal: heavy output generation (HOG) metric
4. Success criterion: HOG > 2/3 with statistical significance
5. Quantum Volume = 2^n for maximum successful n

**Reporting**: Achieved Quantum Volume (power of 2)

**Requirements**: Systems should report QV annually with methodology

#### 5.2 Circuit Layer Fidelity

**Definition**: Average fidelity per layer of parallel gates

**Measurement**:
1. Execute circuits with varying numbers of layers
2. Measure output fidelity vs ideal
3. Extract per-layer fidelity from exponential decay

**Reporting**: Percentage (0-100%)

**Target**: ≥99% per layer for useful algorithms

#### 5.3 Application-Specific Benchmarks

Systems may report performance on:
- Quantum chemistry (molecular energy accuracy)
- Optimization (approximation ratio)
- Sampling (classical hardness)
- Machine learning (classification accuracy)

**Requirement**: Clear problem statement, classical baseline, reproducibility

### Error Characterization

#### 6.1 Error Budget

Systems must provide an error budget breaking down contributions:
- Single-qubit gate errors
- Two-qubit gate errors
- Measurement errors
- State preparation errors
- Idling/decoherence errors
- Crosstalk errors
- Leakage errors (to non-computational states)

**Format**: Table with error source, rate, and contribution to total

#### 6.2 Crosstalk Measurement

**Protocol**:
1. Apply gate to target qubit
2. Measure state change on spectator qubits
3. Quantify unwanted rotation or phase

**Reporting**: Crosstalk matrix (all qubit pairs)

**Threshold**: Crosstalk-induced errors <10% of direct gate errors

### Calibration and Stability

#### 7.1 Calibration Frequency

- **Definition**: How often system requires recalibration
- **Reporting**: Typical time between calibrations (hours)
- **Recommendation**: Automated daily calibration minimum

#### 7.2 Performance Stability

- **Measurement**: Track key metrics (gate fidelity, T1, T2) over 24-hour period
- **Reporting**: Mean and standard deviation
- **Requirement**: <5% drift in fidelities over 24 hours

#### 7.3 Uptime

- **Definition**: Percentage of time system is available for computation
- **Calculation**: (Total time - downtime) / Total time × 100%
- **Target**: ≥90% uptime for production systems

### Compliance and Certification

#### 8.1 Compliance Levels

**Level 1: Basic Compliance**
- Report qubit count, T1, T2
- Single and two-qubit gate fidelities (RB)
- Readout fidelity
- Topology

**Level 2: Standard Compliance**
- All Level 1 requirements
- Quantum Volume measurement
- Error budget
- 24-hour stability data

**Level 3: Full Compliance**
- All Level 2 requirements
- Application benchmarks
- Third-party verification
- Open data sharing

#### 8.2 Certification Process

1. **Self-Assessment**: Vendor performs measurements
2. **Documentation**: Submit results with methodology
3. **Review**: WIA technical committee reviews
4. **Verification**: Optional third-party testing
5. **Certification**: WIA-SEMI-005 compliance certificate issued
6. **Renewal**: Annual recertification required

#### 8.3 Non-Compliance

Systems not meeting threshold requirements may still report results but cannot claim compliance. Aspirational targets provided for development roadmaps.

### Platform-Specific Requirements

#### 9.1 Superconducting Qubits

**Additional Metrics**:
- Operating temperature: <100 mK
- Anharmonicity: >200 MHz
- Resonator coupling strength

**Calibration**:
- Frequency tracking (drift <100 kHz/day)
- Pulse optimization (DRAG, optimal control)
- Mixer calibration

#### 9.2 Trapped-Ion Systems

**Additional Metrics**:
- Ion species and transitions
- Motional heating rate: <1000 quanta/s (threshold), <100 quanta/s (target)
- Laser linewidth: <1 MHz
- Rabi frequency: 1-100 MHz

**Calibration**:
- Motional mode frequency tracking
- Rabi frequency optimization
- Individual ion addressing

#### 9.3 Photonic Processors

**Additional Metrics**:
- Single-photon purity: g^(2)(0) <0.1
- Indistinguishability (HOM visibility): >95%
- Detection efficiency: >90%
- Photon loss per component: <0.5 dB

**For Continuous-Variable**:
- Squeezing level: >15 dB
- Mode purity: >95%

### Documentation Requirements

Compliant systems must provide:
1. **Technical datasheet**: All required metrics
2. **Calibration procedures**: Detailed protocols
3. **Benchmark results**: Raw data and analysis
4. **Access information**: How to use system (cloud/on-premise)
5. **Changelog**: Updates and improvements

### Security and Privacy

- **Access control**: Authentication and authorization
- **Data protection**: Encryption of user circuits and data
- **Isolation**: Multi-user environment isolation
- **Audit logging**: Tracking of system access and usage

### Environmental and Safety

- **Power consumption**: Report total system power (including cooling)
- **Cryogenic safety**: For dilution refrigerators
- **Laser safety**: Class ratings for ion trap and photonic systems
- **Electromagnetic compatibility**: EMC standards

### Future Roadmap

This standard (v1.0) focuses on NISQ-era systems. Future versions will address:
- **v1.1 (2026)**: Error-corrected logical qubit metrics
- **v2.0 (2028)**: Fault-tolerant quantum computing standards
- **v3.0 (2030+)**: Quantum networking integration

### Annexes

**Annex A**: Randomized Benchmarking Protocol (detailed)
**Annex B**: Quantum Volume Implementation Guide
**Annex C**: Error Budget Template
**Annex D**: Glossary of Terms
**Annex E**: Measurement Uncertainty Guidelines

---

**Document Control**

- **Prepared by**: WIA Quantum Standards Working Group
- **Approved by**: WIA Standards Board
- **Next Review**: 2026-01-01
- **Feedback**: standards@wia.org

**Copyright © 2025 WIA / SmileStory Inc.**
弘益人間 · Benefit All Humanity
