# WIA-SEMI-005: Platform-Specific Extensions
## Version 1.0

### Overview

This document provides platform-specific requirements and metrics for the three primary quantum computing technologies covered by WIA-SEMI-005.

## Part 1: Superconducting Qubit Systems

### 1.1 Additional Performance Metrics

**Anharmonicity (α)**:
- Definition: Energy difference between |1⟩→|2⟩ and |0⟩→|1⟩ transitions
- Measurement: Two-tone spectroscopy
- Typical values: 200-400 MHz (transmon)
- Requirement: α > 150 MHz for selective addressing

**Resonator Parameters**:
- Readout resonator frequency: 6-8 GHz typical
- Quality factor Q: >10^4 (loaded)
- Dispersive shift χ: 1-5 MHz

**Qubit-Resonator Coupling**:
- Coupling strength g: 50-200 MHz
- Cooperativity C = g²/(κγ): >10

### 1.2 Control Requirements

**Microwave Control**:
- Frequency range: 4-8 GHz
- Power range: -80 to 0 dBm
- Timing resolution: <1 ns
- Phase stability: <1 degree over gate time
- AWG sampling rate: >1 GS/s

**DC Bias (Flux Control)**:
- Resolution: <1 mA (corresponds to ~MHz frequency tuning)
- Stability: <0.1% over 1 hour
- Noise: <10 μA RMS

**Pulse Shaping**:
- DRAG parameter optimization
- Derivative terms for leakage reduction
- Gaussian or DRAG pulse envelopes standard

### 1.3 Cryogenic System

**Temperature Requirements**:
- Mixing chamber: 10-20 mK
- Cold plate: <100 mK
- Still: 600-800 mK

**Stability**:
- Temperature fluctuations: <±1 mK at mixing chamber
- Vibration isolation: <1 μm displacement

**Heat Budget**:
- Available cooling power at base: 10-100 μW
- Dissipation per qubit: <1 μW

### 1.4 Fabrication Standards

**Substrate**:
- Material: High-resistivity silicon (>10 kΩ·cm) or sapphire
- Thickness: 300-500 μm (silicon), 430 μm (sapphire)
- Surface roughness: <1 nm RMS

**Josephson Junctions**:
- Critical current: 10-100 nA
- Junction resistance: 1-10 kΩ
- Area: 0.01-1 μm²
- Fabrication: Double-angle evaporation or bridge technique

**Superconducting Material**:
- Aluminum (most common): Tc = 1.2 K, Δ/h ≈ 90 GHz
- Niobium (alternative): Tc = 9.2 K
- Film thickness: 50-200 nm
- Purity: 99.999%+

### 1.5 Noise Characterization

**Flux Noise**:
- Measurement: Ramsey spectroscopy, echo decay
- Typical magnitude: 1-10 μΦ0/√Hz at 1 Hz
- Spectrum: ~1/f

**Charge Noise**:
- Less critical for transmons (charge-insensitive design)
- Can affect tunable couplers and other circuit elements

**Dielectric Loss**:
- Loss tangent tan δ < 10^(-6) desired
- Characterized via resonator Q measurements

### 1.6 Topology-Specific Requirements

**Square Lattice**:
- Connectivity: 4 (interior qubits)
- Frequency allocation: Alternating pattern to minimize collisions
- Use case: Surface code error correction

**Heavy-Hex**:
- Connectivity: 3
- Optimized for specific error correction codes
- IBM's preferred topology

**Custom Topologies**:
- Long-range couplers: Resonator-mediated, tunable
- Application-optimized designs

## Part 2: Trapped-Ion Systems

### 2.1 Ion Species and Transitions

**Common Ion Species**:

**Ytterbium-171 (¹⁷¹Yb⁺)**:
- Hyperfine qubit: |F=0, mF=0⟩ ↔ |F=1, mF=0⟩
- Transition frequency: 12.642812 GHz
- Wavelengths: 369.5 nm (cooling), 935 nm (repump), 355 nm (ionization)
- Advantages: Commercial laser availability, hyperfine clock state

**Barium-137 (¹³⁷Ba⁺)**:
- Similar properties to Yb
- Used by Quantinuum

**Calcium-40 (⁴⁰Ca⁺)**:
- Simpler level structure
- Wavelengths: 397 nm (cooling), 866 nm (repump), 729 nm (qubit)

### 2.2 Trap Parameters

**Paul Trap RF**:
- Frequency: 10-100 MHz
- Voltage: 100-500 V amplitude
- Secular frequencies: 1-10 MHz

**Trap Depth**:
- Minimum: 0.1 eV (>1000 K equivalent)
- Typical: 0.3-1 eV

**Ion-Electrode Distance**:
- Macroscopic traps: 1-2 mm
- Surface traps: 50-100 μm

**Motional Heating Rate**:
- Threshold: <1000 quanta/s
- Target: <100 quanta/s
- World-class: <10 quanta/s
- Scaling: ~1/d^4 (d = ion-electrode distance)

### 2.3 Laser Requirements

**Wavelength Stability**:
- Qubit laser: <1 kHz linewidth
- Cooling laser: <1 MHz acceptable

**Intensity Stability**:
- Rabi frequency fluctuations: <1%
- Power stabilization via AOM

**Beam Parameters**:
- Waist: 1-10 μm (individual addressing)
- Pointing stability: <0.1 μm
- Wavefront quality: Near diffraction-limited

**Rabi Frequency**:
- Single-qubit gates: 10-100 kHz
- Two-qubit gates: 1-10 kHz (motional coupling)

### 2.4 Vacuum System

**Pressure**:
- Operating: <10^(-11) torr
- Background gas collision rate: <0.1/s

**Vacuum Components**:
- Ion pumps
- NEG (non-evaporable getter) pumps
- UHV-compatible materials only

### 2.5 Readout and Detection

**Fluorescence Collection**:
- Numerical aperture: NA >0.6
- Collection efficiency: >10%
- PMT or EMCCD detection

**Discrimination Fidelity**:
- Single measurement: >99.9% achievable
- Multi-measurement averaging: >99.99%

### 2.6 QCCD Architecture (Advanced)

**Multi-Zone Design**:
- Junction zones: Splitting/merging ion chains
- Memory zones: Long-term storage
- Entangling zones: High-fidelity gates
- Readout zones: Fluorescence detection

**Ion Shuttling**:
- Transport fidelity: >99.99%
- Speed: 1-10 m/s
- Maintain cooling and coherence during transport

## Part 3: Photonic Quantum Processors

### 3.1 Single-Photon Sources

**Purity**:
- Second-order correlation: g^(2)(0) <0.1 (threshold), <0.01 (target)
- Multi-photon probability: <5%

**Indistinguishability**:
- Hong-Ou-Mandel visibility: >95% (threshold), >99% (world-class)
- Spectrum: Narrow linewidth or matched filtering

**Brightness**:
- Generation rate: >10^6 photons/s (threshold), >10^8/s (target)
- Collection efficiency: >80%

**Source Types**:
- SPDC (Spontaneous Parametric Down-Conversion): Probabilistic, pure
- Quantum dots: Near-deterministic, requires cryogenic operation (4-30 K)
- Parametric oscillators: High rate, mode matching critical

### 3.2 Detection

**SNSPDs (Superconducting Nanowire Single-Photon Detectors)**:
- Detection efficiency: >90% (target), 98%+ (world-class)
- Dark count rate: <1 Hz
- Timing jitter: <50 ps
- Recovery time (dead time): <10 ns
- Operating temperature: 1-4 K

**Si APDs (Avalanche Photodiodes)**:
- Efficiency: 50-70%
- Dark counts: kHz range
- Room temperature operation

### 3.3 Integrated Photonic Circuits

**Platform Materials**:
- Silicon: High index contrast, CMOS compatible, loss 1-5 dB/cm
- Silicon nitride: Lower loss (<0.1 dB/cm), broader transparency
- Lithium niobate: Electro-optic modulation, nonlinear optics

**Component Specs**:
- Waveguide loss: <1 dB/cm (Si), <0.1 dB/cm (SiN)
- Beam splitter ratio accuracy: ±2%
- Phase shifter insertion loss: <0.5 dB
- Switching speed: >10 MHz

**Coupling Efficiency**:
- Fiber-to-chip: >80%
- Mode mismatch: <5%

### 3.4 Interferometer Stability

**Phase Stability**:
- <0.1 rad drift over measurement time
- Active stabilization required for >10 components

**Temperature Control**:
- ±0.01°C stability for passive compensation
- Thermo-optic phase shifters for active correction

### 3.5 Continuous-Variable (CV) Platforms

**Squeezing Requirements**:
- Level: >15 dB (target), >20 dB (world-class)
- Bandwidth: Match gate duration
- Anti-squeezing: Managed via filtering

**Mode Purity**:
- Overlap with local oscillator: >95%
- Spurious modes: <5% power

**Homodyne Detection**:
- Efficiency: >90%
- Electronic noise: Below shot noise level
- Bandwidth: >100 MHz

### 3.6 Photonic Quantum Volume

**Alternative Metric for Photonics**:
- Photon count: Number of detected photons
- Mode count: Spatial/temporal modes
- Hong-Ou-Mandel visibility
- Classical simulation hardness

## Part 4: Emerging Platforms (Informative)

### 4.1 Neutral Atoms

- Atom count: 50-1000+ (Rydberg arrays)
- Trap type: Optical tweezers
- Gate mechanism: Rydberg blockade
- T2 coherence: >1 second
- Two-qubit fidelity: 97-99.5%

### 4.2 Silicon Spin Qubits

- Qubit size: <50 nm
- Operating temperature: <1 K
- Gate fidelity: 99%+ (single), 95-99% (two-qubit improving)
- Fabrication: CMOS-compatible

### 4.3 Topological (Majorana)

- Status: Research phase, no qubits demonstrated
- Predicted advantage: Inherent error protection

---

© 2025 WIA / SmileStory Inc.
弘益人間 · Benefit All Humanity
