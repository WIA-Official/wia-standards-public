# WIA-SEMI-005: Glossary and Reference Guide
## Version 1.0

### Acronyms and Abbreviations

- **CNOT**: Controlled-NOT gate
- **CPMG**: Carr-Purcell-Meiboom-Gill (dynamical decoupling sequence)
- **DRAG**: Derivative Removal by Adiabatic Gate
- **FTQC**: Fault-Tolerant Quantum Computing
- **GHZ**: Greenberger-Horne-Zeilinger (maximally entangled state)
- **GRAPE**: Gradient Ascent Pulse Engineering
- **GST**: Gate Set Tomography
- **HOG**: Heavy Output Generation
- **LDPC**: Low-Density Parity-Check (error correction codes)
- **MBQC**: Measurement-Based Quantum Computing
- **NISQ**: Noisy Intermediate-Scale Quantum
- **PEC**: Probabilistic Error Cancellation
- **QCCD**: Quantum Charge-Coupled Device (ion trap architecture)
- **QPT**: Quantum Process Tomography
- **QST**: Quantum State Tomography
- **QV**: Quantum Volume
- **RB**: Randomized Benchmarking
- **SPAM**: State Preparation And Measurement
- **SPDC**: Spontaneous Parametric Down-Conversion
- **SNSPD**: Superconducting Nanowire Single-Photon Detector
- **VQE**: Variational Quantum Eigensolver
- **ZNE**: Zero-Noise Extrapolation

### Key Terms

**Algorithmic Qubits (#AQ)**: IonQ's metric representing the number of qubits effectively usable for algorithms, accounting for fidelity and connectivity.

**Barren Plateau**: Phenomenon in variational quantum algorithms where gradients vanish exponentially with system size, preventing optimization.

**Bloch Sphere**: Geometric representation of a single qubit's quantum state as a point on a unit sphere.

**Circuit Depth**: Number of sequential layers of gates in a quantum circuit.

**Clifford Gates**: Set of gates (H, S, CNOT) that map Pauli operators to Pauli operators. Efficiently classically simulable (Gottesman-Knill theorem).

**Coherence**: Preservation of quantum superposition and phase relationships.

**Decoherence**: Loss of quantum coherence due to environmental interaction.

**Dephasing**: Loss of relative phase between quantum states (affects T2).

**Dispersive Readout**: Measurement technique where qubit state shifts a coupled resonator's frequency (superconducting qubits).

**Entanglement**: Quantum correlation between particles that cannot be described by local hidden variables.

**Error Syndrome**: Pattern of measurement outcomes indicating which error occurred (in quantum error correction).

**Fidelity**: Measure of closeness between two quantum states or operations (0 to 1, or 0% to 100%).

**Gate**: Unitary operation on qubits (X, Y, Z, H, CNOT, etc.).

**Heavy Output**: Bit string with probability above median in a distribution (quantum volume measurement).

**Indistinguishability**: Degree to which two photons are identical (critical for photonic quantum computing).

**Josephson Junction**: Superconducting device consisting of two superconductors separated by thin insulator, core of superconducting qubits.

**Leakage**: Transition to states outside computational subspace (e.g., |2⟩ in transmon).

**Logical Qubit**: Error-corrected qubit encoded in multiple physical qubits.

**Magic State**: Resource state used to achieve non-Clifford gates in fault-tolerant quantum computing.

**Mølmer-Sørensen Gate**: Trapped-ion entangling gate mediated by collective motional modes.

**No-Cloning Theorem**: Fundamental principle that arbitrary quantum states cannot be copied.

**Pauli Operators**: X, Y, Z single-qubit operators (bit-flip, bit-phase-flip, phase-flip).

**Quantum Advantage**: Demonstration that quantum computer outperforms classical for specific task.

**Quantum Error Correction (QEC)**: Techniques to protect quantum information from errors using redundancy.

**Quantum Supremacy**: Original term for quantum advantage (less preferred due to connotations).

**Qubit**: Two-level quantum system, unit of quantum information.

**Rabi Frequency**: Rate of oscillation between quantum states under resonant driving.

**Randomized Benchmarking**: Protocol to measure average gate fidelity by randomizing errors.

**Shot Noise**: Statistical fluctuation in measurement outcomes due to finite sampling.

**Stabilizer**: Operator used in quantum error correction that commutes with logical operators.

**Surface Code**: Leading quantum error correction code with 2D nearest-neighbor connectivity requirement.

**SWAP Gate**: Operation that exchanges states of two qubits.

**T Gate**: Non-Clifford gate (π/8 rotation about Z axis), resource for universal quantum computing.

**Threshold**: Error rate below which quantum error correction can achieve arbitrarily low logical error rates.

**Transmon**: Transmission line shunted plasma oscillation qubit, most common superconducting qubit type.

**Trotterization**: Approximation of time evolution by splitting Hamiltonian into commutable parts.

**Universal Quantum Computer**: System capable of approximating any quantum operation to arbitrary precision.

### Mathematical Notation

- |ψ⟩: Quantum state (Dirac ket notation)
- ⟨ψ|: Dual vector (bra)
- ⟨ψ|φ⟩: Inner product
- ρ: Density matrix (mixed state representation)
- U: Unitary operator
- H: Hamiltonian (energy operator)
- I, X, Y, Z: Pauli operators
- H: Hadamard gate
- S: Phase gate (π/2 rotation)
- T: T gate (π/4 rotation)
- CNOT: Controlled-NOT gate
- F: Fidelity
- ε: Error rate
- τ: Time variable
- χ: Process matrix (QPT)

### Physical Constants

- h: Planck constant = 6.626 × 10^(-34) J·s
- ħ: Reduced Planck constant = h/(2π)
- k_B: Boltzmann constant = 1.381 × 10^(-23) J/K
- e: Elementary charge = 1.602 × 10^(-19) C
- Φ_0: Magnetic flux quantum = h/(2e) ≈ 2.068 × 10^(-15) Wb

### Units

- **Frequency**: Hz, GHz, MHz
- **Time**: s, ms, μs, ns
- **Temperature**: K, mK (kelvin, millikelvin)
- **Energy**: J (joule), eV (electronvolt), ħω (angular frequency units)
- **Power**: W, mW, μW, dBm
- **Magnetic Field**: T (tesla), G (gauss)
- **Pressure**: torr, Pa

### Useful Conversions

- 1 GHz ≈ 48 μeV ≈ 0.56 K (h × frequency → energy → temperature)
- 10 mK (dilution fridge temp) ≈ 0.21 GHz thermal frequency
- 0 dBm = 1 mW
- -10 dBm = 0.1 mW
- 1 torr = 133.3 Pa

### Common Quantum States

- |0⟩: Ground state, computational basis
- |1⟩: Excited state, computational basis
- |+⟩ = (|0⟩ + |1⟩)/√2: +eigenstate of X
- |-⟩ = (|0⟩ - |1⟩)/√2: -eigenstate of X
- |+i⟩ = (|0⟩ + i|1⟩)/√2: +eigenstate of Y
- |-i⟩ = (|0⟩ - i|1⟩)/√2: -eigenstate of Y
- Bell states: (|00⟩ ± |11⟩)/√2, (|01⟩ ± |10⟩)/√2 (maximally entangled)
- GHZ state: (|000...⟩ + |111...⟩)/√2 (n-qubit entanglement)
- W state: (|100...⟩ + |010...⟩ + ... + |...001⟩)/√n

### References and Standards

- ISO/IEC 4879: Quantum computing—Terms and definitions
- IEEE P7131: Quantum Computing Performance Metrics  
- NIST Quantum Information Science publications
- arXiv quantum physics (quant-ph) preprints

### Software and Tools

**Quantum Development Frameworks**:
- Qiskit (IBM)
- Cirq (Google)
- PennyLane (Xanadu)
- Q# (Microsoft)
- PyQuil (Rigetti)

**Simulation**:
- QuTiP: Quantum Toolbox in Python
- Qiskit Aer: High-performance simulators
- ProjectQ: Compiler framework

**Benchmarking**:
- pyGSTi: Gate Set Tomography
- Quantum Benchmark (Keysight)

**Optimization**:
- QuTech: Quantum compilers
- XACC: Language-agnostic quantum programming framework

### Organizations

- **WIA**: World Certification Industry Association (this standard)
- **IEEE**: Institute of Electrical and Electronics Engineers
- **ISO/IEC**: International Organization for Standardization / International Electrotechnical Commission
- **NIST**: National Institute of Standards and Technology (USA)
- **QED-C**: Quantum Economic Development Consortium

### Educational Resources

- Nielsen & Chuang: "Quantum Computation and Quantum Information" (textbook)
- Qiskit Textbook: Online, free quantum computing course
- IBM Quantum Learning Platform
- Microsoft Quantum Katas
- Xanadu Quantum Codebook

### Contact and Support

- **WIA Standards Office**: standards@wia.org
- **Technical Questions**: quantum-tech@wia.org
- **Certification**: certification@wia.org
- **Website**: https://wiabooks.store/tag/wia-quantum-chip/

---

© 2025 WIA / SmileStory Inc.
弘益人間 · Benefit All Humanity
