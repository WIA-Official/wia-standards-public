# WIA-SEMI-005: Testing and Verification Procedures
## Version 1.0

### Introduction

This document specifies standard testing and verification procedures for quantum chip systems complying with WIA-SEMI-005 core standard. All measurements must follow these protocols to ensure consistency and comparability across different platforms and vendors.

### Randomized Benchmarking (RB)

**Purpose**: Measure average gate fidelity for Clifford gates

**Single-Qubit RB Protocol**:
1. Select sequence length m from [1, 2, 5, 10, 20, 50, 100, 200, ...]
2. Generate K random sequences of m Clifford gates
3. For each sequence:
   a. Compute net Clifford operation C = C_m · C_{m-1} · ... · C_1
   b. Append recovery gate C^(-1) to return to |0⟩
   c. Execute: |0⟩ → C_m ... C_1 → C^(-1) → measure
4. Repeat N shots per sequence (N ≥ 100)
5. Calculate survival probability P(m) = fraction of |0⟩ outcomes
6. Fit to exponential decay: P(m) = A + Bp^m where p = (d-1)/d + (1-r)
7. Extract r (average error per Clifford gate)
8. Convert to average gate fidelity: F_avg = 1 - r(d-1)/d (d=2 for qubit)

**Parameters**:
- K ≥ 30 random sequences per length
- N ≥ 100 shots per sequence
- At least 10 different sequence lengths
- Maximum length: limited by coherence time

**Two-Qubit Interleaved RB**:
1. Perform standard RB → extract p_ref
2. Perform RB with target gate G interleaved between each Clifford → extract p_G
3. Gate fidelity: F_G = 1 - (1 - p_G/p_ref)(d²-1)/d² (d=2)

**Reporting**:
- Average Clifford fidelity with 95% confidence interval
- Decay curve plots
- Raw data (P vs m)

### Quantum Process Tomography (QPT)

**Purpose**: Full characterization of quantum gate

**Protocol**:
1. Prepare input state from basis {|0⟩, |1⟩, |+⟩, |+i⟩} (4 states for 1 qubit)
2. Apply gate G
3. Perform state tomography on output
4. Reconstruct process matrix χ
5. Calculate process fidelity F_process = Tr(χ_ideal χ_measured)

**Requirements**:
- 4^n input states for n qubits
- 3^n measurement settings
- Minimum 1000 shots per setting

**Use Case**: Detailed gate characterization, less common for routine benchmarking

### Quantum State Tomography (QST)

**Purpose**: Reconstruct density matrix of quantum state

**Protocol** (1 qubit):
1. Prepare state ρ
2. Measure in 3 bases: Z (computational), X (±), Y (±i)
3. From measurement statistics, reconstruct ρ using maximum likelihood or linear inversion
4. Calculate fidelity F = Tr(√(√ρ_ideal ρ √ρ_ideal))

**Multi-Qubit**:
- Requires 3^n measurement settings
- Exponentially expensive (use compressive sensing for >5 qubits)

### Coherence Time Measurements

**T1 Measurement**:
```
1. Initialize to |1⟩: Apply X gate
2. Wait time τ (vary from 0 to 5×T1_estimate)
3. Measure in Z basis
4. Repeat for τ ∈ [0, Δτ, 2Δτ, ..., 5T1_est] with Δτ = T1_est/20
5. Fit P(1|τ) = exp(-τ/T1) + offset
6. Extract T1
```

**T2* Measurement** (Ramsey):
```
1. Apply π/2 pulse (X or Y)
2. Free evolution for time τ
3. Apply π/2 pulse (same axis)
4. Measure
5. Vary τ, measure oscillations
6. Fit amplitude envelope: A(τ) = exp(-τ/T2*)
```

**T2 (Echo) Measurement**:
```
1. π/2 pulse
2. Free evolution τ/2
3. π pulse (refocusing)
4. Free evolution τ/2
5. π/2 pulse
6. Measure
7. Extract T2 from decay (typically T2 > T2* due to refocusing low-frequency noise)
```

**Reporting Requirements**:
- Fit parameters with uncertainties
- Plots showing data and fits
- Temperature and time-of-day (for drift tracking)

### Quantum Volume Measurement

**Detailed Protocol**:
1. Choose width n (number of qubits to test)
2. Generate circuit:
   a. Initialize to |0...0⟩
   b. For each layer d=1 to n:
      - Partition n qubits into n/2 pairs randomly
      - Apply random SU(4) gate to each pair
   c. Total depth: n layers
3. Execute circuit, measure all qubits
4. Collect N_shot measurement outcomes
5. Calculate Heavy Output Generation (HOG):
   - Compute ideal probability distribution {p_i}
   - Define median probability: p_med such that Σ_{p_i > p_med} p_i ≈ 0.5
   - HOG = (measured frequency of heavy outputs) / N_shot
6. Statistical test: HOG > 2/3 with confidence >97% (2σ)
7. Repeat for N_circuits ≥ 100 different random circuits
8. If success rate ≥ 97%, Quantum Volume achieved: QV = 2^n
9. Increment n and repeat

**Implementation Details**:
- Random SU(4) generated via Haar measure or fixed gate set compilation
- Use efficient circuit compilation
- N_shot ≥ 1000 per circuit recommended
- Report both achieved QV and maximum attempted

### Readout Confusion Matrix

**Measurement**:
```
For each qubit q:
  1. Prepare |0⟩, measure N times → f_00 = P(measure 0 | prepared 0)
                                    f_01 = P(measure 1 | prepared 0)
  2. Prepare |1⟩, measure N times → f_10 = P(measure 0 | prepared 1)
                                    f_11 = P(measure 1 | prepared 1)
  3. Confusion matrix M = [f_00, f_01]
                          [f_10, f_11]
```

**Mitigation**:
- Invert M to correct measured counts: n_corrected = M^(-1) n_measured
- Report corrected and uncorrected results

### Crosstalk Characterization

**Single-Qubit Crosstalk**:
1. Apply X gate to target qubit i
2. Measure spectator qubit j (should remain in |0⟩)
3. Error rate e_ij = P(j flips | operate on i)
4. Repeat for all pairs (i,j)
5. Construct crosstalk matrix

**Frequency Collision Detection**:
1. Sweep qubit i frequency
2. Monitor spectator qubit j coherence/frequency
3. Identify collision points (avoided crossings)
4. Document and avoid in operation

### Gate Set Tomography (GST)

**Purpose**: Overcome SPAM errors in gate characterization

**Protocol** (simplified):
1. Prepare fiducial states
2. Apply varying sequences of gates
3. Measure in fiducial basis
4. Use overcomplete set to reconstruct gates and SPAM jointly
5. Self-consistent characterization immune to SPAM

**Computational Cost**: Higher than RB but more accurate

**Software**: pyGSTi recommended

### Cross-Entropy Benchmarking

**For Random Circuit Sampling**:
1. Generate random circuit (varying depths)
2. Simulate classically to get ideal distribution P_ideal(x)
3. Execute on quantum hardware, sample outcomes {x_i}
4. Calculate cross-entropy: XEB = 2^n <P_ideal(x_i)> - 1
5. XEB = 1 indicates perfect sampling, XEB = 0 is random

**Interpretation**: Quantifies fidelity of sampling distribution

### Application-Specific Benchmarks

**Quantum Chemistry (VQE)**:
- Molecule: H2, LiH, BeH2 (standard test cases)
- Metric: Energy accuracy vs classical (CCSD, FCI)
- Report: Final energy, number of iterations, circuit depth

**Optimization (QAOA)**:
- Problem: MaxCut on random graphs (specify size and edge probability)
- Metric: Approximation ratio vs optimal
- Report: Success probability, required depth p

### Environmental Conditions

All measurements must report:
- Temperature (for cryogenic systems)
- Time and date
- Calibration age (time since last calibration)
- Any anomalies or system issues

### Statistical Requirements

- **Confidence Intervals**: Report 95% CI for all metrics
- **Sample Size**: Minimum shots per measurement specified per protocol
- **Outlier Handling**: Document and justify any data exclusion
- **Reproducibility**: Repeat measurements on different days, report variance

### Data Formats

**Standardized JSON Schema** for Results:
```json
{
  "measurement": "RandomizedBenchmarking",
  "qubit_id": [0],
  "timestamp": "2025-01-01T12:00:00Z",
  "parameters": {
    "sequence_lengths": [1, 2, 5, 10, 20, 50, 100],
    "num_sequences": 30,
    "shots_per_sequence": 100
  },
  "results": {
    "survival_probabilities": [0.995, 0.990, 0.975, ...],
    "fitted_fidelity": 0.9995,
    "confidence_interval": [0.9993, 0.9997]
  }
}
```

### Verification and Auditing

**Third-Party Verification**:
- Independent execution of protocols
- Access to raw data
- Reproducibility confirmation

**Self-Certification**:
- Automated testing suite
- Continuous monitoring
- Drift detection and alerts

### Conclusion

Adherence to these testing procedures ensures:
- Comparability across platforms
- Reproducibility of results
- Confidence in reported metrics
- Industry-wide standardization

---

© 2025 WIA / SmileStory Inc.
弘益人間 · Benefit All Humanity
