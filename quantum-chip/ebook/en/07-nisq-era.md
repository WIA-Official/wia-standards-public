# Chapter 7: The NISQ Era
## Noisy Intermediate-Scale Quantum Computing

### Introduction

We currently live in what physicist John Preskill termed the "NISQ era" (Noisy Intermediate-Scale Quantum) in his influential 2018 paper. This epoch is characterized by quantum processors with 50-1000 qubits that are too small and error-prone for full fault-tolerant quantum error correction, yet potentially powerful enough to solve certain problems beyond classical reach.

The NISQ era represents both an opportunity and a challenge: Can we extract useful computation from imperfect quantum hardware before achieving full fault tolerance? This chapter explores NISQ algorithms, applications, error mitigation techniques, and the fundamental question of quantum advantage in the near term.

### Defining Characteristics of NISQ Devices

**Scale:**
- 50-1000 physical qubits
- Below threshold for economical fault tolerance
- Larger than classical simulation (for some problems)

**Noise:**
- Gate errors: 0.1-1% per two-qubit gate
- Decoherence during computation
- Measurement errors
- State preparation errors
- Crosstalk and unwanted interactions

**Circuit Depth:**
- Limited by coherence times
- Typical: 10-1000 two-qubit gates
- Depends on error rates and problem structure

**Computational Model:**
- Direct ("analog") quantum simulation
- Hybrid quantum-classical algorithms
- Error mitigation rather than correction
- Approximate rather than exact solutions

### NISQ Algorithms

**Variational Quantum Algorithms (VQAs)**

The workhorse of NISQ computing:

**General Structure:**
1. Prepare parameterized quantum state |ψ(θ)⟩
2. Measure observable ⟨O⟩
3. Classical optimizer updates parameters θ
4. Repeat until convergence

**Advantages:**
- Shallow circuits (reduce error accumulation)
- Noise-resilient (optimization can adapt)
- Flexible problem encoding

**Challenges:**
- Barren plateaus (optimization landscapes)
- Local minima
- Shot noise in measurements
- Classical optimization overhead

**Variational Quantum Eigensolver (VQE)**

Find ground state energy of Hamiltonian H:

**Application:** Quantum chemistry, materials science

**Algorithm:**
1. Prepare ansatz state |ψ(θ)⟩ (problem-specific or hardware-efficient)
2. Measure energy E(θ) = ⟨ψ(θ)|H|ψ(θ)⟩
3. Classical optimizer minimizes E(θ)
4. Iterate to convergence

**Ansatz Design:**
- **Chemistry-inspired:** UCC (Unitary Coupled Cluster)
- **Hardware-efficient:** Layers of parameterized gates
- **Problem-agnostic:** QAOA-like structures

**Demonstrations:**
- Small molecules (H₂, LiH, BeH₂, H₂O)
- Bond dissociation curves
- Electronic structure calculations
- Current limit: ~10-20 qubits for chemistry

**Challenges:**
- Ansatz expressibility vs trainability
- Measurement overhead (many Pauli terms in H)
- Noise accumulation
- Classical computational cost

**Quantum Approximate Optimization Algorithm (QAOA)**

Solve combinatorial optimization problems:

**Algorithm:**
1. Encode problem in cost Hamiltonian H_C
2. Prepare initial state |s⟩ (usually uniform superposition)
3. Apply p layers of:
   - Cost operator e^(-iγH_C)
   - Mixer operator e^(-iβH_M)
4. Measure in computational basis
5. Optimize angles (γ, β)

**Applications:**
- MaxCut
- Graph coloring
- Traveling salesman
- Portfolio optimization
- Resource allocation

**Depth-Quality Tradeoff:**
- Low p (depth): Fast but approximate
- High p: Better solutions but more noise
- Optimal p depends on problem and hardware

**Performance:**
- Demonstration on problems up to ~30-40 variables
- Approximation ratios competitive with classical heuristics
- Debate: Does quantum help? Mixed results.

**Quantum Machine Learning (QML)**

Controversial but active area:

**Approaches:**
- **Quantum feature maps:** Encode classical data in quantum states
- **Quantum neural networks:** Parameterized quantum circuits
- **Quantum kernels:** Compute inner products in exponentially large space

**Claimed Advantages:**
- Exponentially large Hilbert space
- Efficient representation of certain functions
- Quantum sampling for generative models

**Skepticism:**
- Loading classical data is bottleneck
- Measurement destroys quantum advantage
- Barren plateaus severe in QML
- Classical ML advancing rapidly

**Demonstrated Applications:**
- Classification on small datasets
- Generative models (quantum GANs)
- Reinforcement learning (early stage)

**Verdict (2025):** Niche applications possible, general quantum advantage unproven

### Quantum Simulation

Most natural application for quantum computers:

**Principle:** Simulate quantum systems with quantum hardware (Feynman's original vision)

**Analog Simulation:**
- Direct implementation of target Hamiltonian
- Minimal gate overhead
- Limited to specific models

**Digital Simulation:**
- Trotterization: e^(iHt) ≈ [e^(iH₁Δt)e^(iH₂Δt)...]^(t/Δt)
- General Hamiltonians
- Error accumulation with Trotter steps

**Applications:**

**Condensed Matter Physics:**
- Fermi-Hubbard model (high-T_c superconductivity)
- Spin systems
- Topological phases
- Many-body localization

**Demonstrations:**
- Google: Time crystals (2021)
- Various groups: Quantum phase transitions

**High-Energy Physics:**
- Lattice gauge theories
- Quantum field theory simulation
- Parton shower modeling

**Chemistry:**
- Reaction dynamics
- Excited state properties
- Time-dependent phenomena

**Quantum Advantage:**
- Classical simulation: Exponential cost for generic systems
- Quantum simulation: Polynomial cost
- Challenge: Verification (can't classically check answer!)

### Quantum Sampling Problems

Demonstrate quantum advantage via sampling tasks:

**Gaussian Boson Sampling**

**Protocol:**
- Input: Squeezed states into interferometer
- Evolution: Linear optics (beam splitters, phase shifters)
- Measurement: Photon number in output modes
- Output: Sample from complex probability distribution

**Claimed Advantages:**
- Classical sampling: #P-hard (conjectured)
- Quantum sampling: Efficient
- Verification possible (cross-entropy, etc.)

**Demonstrations:**
- Xanadu Borealis: 216 modes (2022)
- USTC Jiuzhang: 113-144 modes (2020-2023)

**Criticisms:**
- Verification relies on unproven complexity assumptions
- No practical application identified
- Classical algorithms improving

**Random Circuit Sampling**

**Protocol:**
- Apply random quantum gates
- Measure in computational basis
- Compare distribution to theoretical prediction

**Google Sycamore (2019):**
- 53 qubits
- Random circuit depth 20
- Claimed 10,000 years classical vs 200 seconds quantum
- IBM disputed: ~2.5 days on classical supercomputer with better algorithm

**Status (2025):**
- Classical algorithms continue improving
- Quantum systems getting larger (beyond classical simulation)
- Arms race between quantum hardware and classical algorithms

### Error Mitigation Techniques

Bridge the gap to fault tolerance:

**Zero-Noise Extrapolation (ZNE)**

**Idea:** Amplify noise, measure at multiple noise levels, extrapolate to zero noise

**Implementation:**
1. Run circuit at native noise level → E₀
2. Artificially increase noise → E₁, E₂, ... (pulse stretching, gate repetition)
3. Fit polynomial: E(λ) = a₀ + a₁λ + ...
4. Extrapolate to λ=0: E(0) = a₀

**Effectiveness:**
- Can improve errors by 2-10×
- Limited by extrapolation accuracy
- Increased circuit depth/shots

**Probabilistic Error Cancellation (PEC)**

**Idea:** Express noisy operation as linear combination of implementable operations

**Process:**
1. Characterize noise: Ñ = N + ε
2. Invert: N = Ñ - ε = Σᵢ cᵢ Õᵢ
3. Sample operations Õᵢ with probability |cᵢ|
4. Weight results by sign(cᵢ)

**Cost:**
- Sampling overhead ~exp(circuit depth × error rate)
- Exponential in depth (limits applicability)
- Can achieve exact error-free distribution (in principle)

**Measurement Error Mitigation**

**Simple approach:**
1. Characterize readout errors (confusion matrix)
2. Invert matrix to correct measured counts

**Effectiveness:**
- Easy to implement
- Significant improvement for high SPAM errors
- Linear overhead

**Dynamical Decoupling**

**Idea:** Apply sequences of gates to average out noise

**Implementation:**
- Insert π-pulse sequences during idle times
- XY4, CPMG, etc. sequences
- Refocus certain error types

**Benefits:**
- Extend coherence times
- Suppress low-frequency noise
- Compatible with other techniques

**Combining Techniques:**
- ZNE + measurement mitigation
- Clifford data regression
- Virtual distillation

**Overall Impact:**
- Can extend NISQ regime capabilities
- Not a replacement for error correction
- Diminishing returns with circuit depth

### Benchmarking NISQ Devices

**Quantum Volume (QV)**

IBM's metric combining qubit count, connectivity, and errors:

**Protocol:**
1. Random quantum circuits (SU(4) layers)
2. Heavy output generation (above median probability strings)
3. Success threshold: >2/3 heavy outputs

**QV = 2ⁿ** where n is largest width circuit passing threshold

**Current Records (2025):**
- Quantinuum: 65,536 (2¹⁶)
- IBM: 512 (2⁹)
- IonQ: Similar range

**Criticisms:**
- Compiler-dependent
- Favors all-to-all connectivity
- Doesn't capture all aspects of performance

**Algorithmic Qubits (#AQ) - IonQ**

Number of qubits usable for algorithm with target fidelity:

**Calculation:**
- Run benchmark algorithms
- Measure success probability
- Account for connectivity overhead

**Transparency:**
- More interpretable than QV
- Direct measure of usable qubits

**CLOPS (Circuit Layer Operations Per Second)**

IBM's throughput metric:

**Measures:**
- Speed of executing quantum circuits
- Includes classical processing time
- Relevant for variational algorithms

**Typical Values (2025):**
- 10,000-50,000 CLOPS
- Improving with better control systems

**Application-Specific Benchmarks:**
- Chemistry: Molecular energy accuracy
- Optimization: Approximation ratio
- Sampling: Classical hardness

### Near-Term Applications with Commercial Interest

**Drug Discovery and Chemistry:**

**Companies:**
- Roche + Cambridge Quantum (now Quantinuum)
- Merck + IBM
- JSR Corporation + IonQ/IBM

**Use Cases:**
- Ground state energy calculations
- Reaction pathway analysis
- Catalyst optimization
- Drug-target binding

**Current Status:**
- Small molecules (<20 qubits)
- Proof-of-concept demonstrations
- Not yet outperforming classical methods
- Building quantum chemistry libraries

**Finance:**

**Applications:**
- Portfolio optimization
- Risk analysis (Monte Carlo)
- Derivative pricing
- Fraud detection

**Partners:**
- Goldman Sachs + multiple quantum vendors
- JPMorgan + IBM, IonQ
- BBVA + various providers

**Status:**
- Algorithm development
- Hybrid approaches (quantum + classical)
- Exploring quantum advantage boundaries

**Materials Science:**

**Focus:**
- Battery materials (lithium compounds)
- Catalysts
- High-temperature superconductors
- Novel alloys

**Partnerships:**
- Mercedes-Benz + IBM (battery research)
- Mitsubishi Chemical + IBM/IonQ
- ExxonMobil + IBM (catalyst design)

**Timeline:**
- 3-5 years to practical quantum advantage (optimistic)
- 5-10 years (realistic)

**Logistics and Optimization:**

**Problems:**
- Vehicle routing
- Supply chain optimization
- Production scheduling
- Traffic flow

**Demonstrations:**
- Volkswagen + D-Wave (traffic optimization)
- Airbus + various providers (flight optimization)
- DHL + IBM (logistics)

**Reality:**
- Current classical algorithms very good
- Quantum advantage requires significant scaling
- Hybrid approaches most promising

### The Quantum Advantage Question

**What constitutes quantum advantage?**

**Definitions:**

1. **Computational:** Solve problem faster than any classical algorithm
2. **Economic:** Cost-effective compared to classical solutions
3. **Practical:** Provides value in real application

**Status by Application:**

| Application | Advantage Demonstrated? | Timeline to Practical Advantage |
|-------------|------------------------|--------------------------------|
| Sampling tasks | Claimed (disputed) | N/A (no application) |
| Quantum simulation | Limited cases | 2-5 years |
| Optimization | No | 5-10 years |
| Machine learning | No | Uncertain |
| Cryptography (Shor) | Theoretical | 10-20 years |
| Chemistry | No | 3-7 years |

**Challenges:**

1. **Moving Target:** Classical algorithms improving
2. **Verification:** Hard to verify quantum results classically
3. **Noise:** Limits depth and accuracy
4. **Scale:** Not enough qubits yet for many problems

**Optimism:**
- Hardware improving exponentially
- Algorithm development accelerating
- Error mitigation extending NISQ regime

**Skepticism:**
- No killer app yet found
- Classical computers not standing still
- Fault tolerance may be necessary

### The NISQ-to-FTQC Transition

**Current State (2025):**
- ~100-1000 qubit processors
- Below/near error correction threshold
- Limited error-corrected demonstrations

**Transition Phase (2025-2030):**
- Early error-corrected logical qubits
- Hybrid NISQ + error-corrected systems
- Thousands of physical qubits
- Hundreds of logical qubits

**Post-NISQ (2030+):**
- Full fault tolerance
- Thousands of logical qubits
- Arbitrary computation depth
- Clear quantum advantage in multiple domains

### WIA-SEMI-005 NISQ Standards

**Performance Reporting Requirements:**

1. **Hardware Characterization:**
   - Qubit count
   - Gate fidelities (single and two-qubit)
   - Coherence times
   - Connectivity graph

2. **Benchmark Scores:**
   - Quantum volume
   - Or alternative standardized benchmark
   - Test conditions and compiler settings

3. **Application Performance:**
   - Problem-specific metrics
   - Comparison to classical baselines
   - Error mitigation techniques used

4. **Transparency:**
   - Full methodology disclosure
   - Data availability
   - Reproducibility information

### Conclusion

The NISQ era represents quantum computing's adolescence—beyond proof-of-concept but not yet mature. We have demonstrated that quantum computers can execute complex algorithms, sometimes venture into regimes hard for classical simulation, and begin to explore practical applications.

Whether NISQ devices will find "killer applications" before fault-tolerant systems arrive remains an open question. The smart money is hedging: pursue NISQ applications while aggressively developing error correction. Some problems may yield to NISQ approaches; others will require full fault tolerance.

The WIA-SEMI-005 standard ensures that NISQ-era claims are rigorous, performance is transparent, and progress is measurable. As we transition from NISQ to fault-tolerant quantum computing, these standards will evolve while maintaining continuity and comparability.

The next few years will be decisive in determining the NISQ era's legacy: a productive period of early applications, or a transitional phase to be quickly superseded by error-corrected systems.

---

**References:**
1. Preskill, "Quantum Computing in the NISQ era and beyond," Quantum (2018)
2. Bharti et al., "Noisy intermediate-scale quantum algorithms," Reviews of Modern Physics (2022)
3. Cerezo et al., "Variational quantum algorithms," Nature Reviews Physics (2021)
4. Aaronson & Chen, "Complexity-Theoretic Foundations of Quantum Supremacy Experiments," CCC (2017)

**Resources:**
- Algorithm repositories
- Benchmarking platforms
- Error mitigation libraries
- Application case studies
