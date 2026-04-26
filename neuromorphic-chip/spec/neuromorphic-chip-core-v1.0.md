# WIA-SEMI-007: Neuromorphic Chip Core Specification v1.0

## Abstract

This specification defines the core requirements, interfaces, and protocols for neuromorphic computing systems based on spiking neural networks (SNNs) and event-driven architectures. It establishes standardized methods for spike encoding, network communication, hardware interfaces, and performance benchmarking.

**Status:** Stable
**Version:** 1.0.0
**Date:** 2025-01-15
**Authors:** WIA Technical Committee on Neuromorphic Computing

## 1. Scope

This specification covers:
- Neuron and synapse models
- Spike encoding/decoding protocols
- Hardware abstraction layers
- Performance metrics and benchmarking
- Safety and reliability requirements

## 2. Neuron Models

### 2.1 Leaky Integrate-and-Fire (LIF)

**Mathematical Definition:**
```
τ_m * dV/dt = -(V - V_rest) + R * I(t) + I_syn(t)

if V(t) ≥ V_threshold:
    V(t+) ← V_reset
    emit spike at time t
    enter refractory period for τ_ref
```

**Required Parameters:**
- `τ_m`: Membrane time constant (default: 20 ms, range: 1-100 ms)
- `V_rest`: Resting potential (default: -70 mV)
- `V_threshold`: Spike threshold (default: -55 mV)
- `V_reset`: Reset potential (default: -70 mV)
- `R`: Membrane resistance (default: 10 MΩ)
- `τ_ref`: Refractory period (default: 2 ms, range: 0-10 ms)

**Implementation Requirements:**
- MUST support fixed-point arithmetic (minimum 16-bit)
- SHOULD support configurable precision (8-bit to 32-bit)
- MUST handle overflow/underflow conditions
- MUST implement refractory period enforcement

### 2.2 Adaptive Models

Implementations MAY support extended models:
- Izhikevich neuron
- Adaptive Exponential Integrate-and-Fire (AdEx)
- Hodgkin-Huxley (for research applications)

## 3. Synaptic Models

### 3.1 Static Synapses

**Definition:**
```
I_syn(t) = w * Σ g(t - t_spike)
where g(t) = exp(-t / τ_syn) for t ≥ 0
```

**Parameters:**
- `w`: Synaptic weight (real number, typically [-1, 1])
- `τ_syn`: Synaptic time constant (default: 5 ms)

### 3.2 Plastic Synapses (STDP)

**Standard STDP Rule:**
```
if t_post - t_pre > 0:  # Pre before post
    Δw = A_plus * exp(-(t_post - t_pre) / τ_plus)
else:  # Post before pre
    Δw = -A_minus * exp((t_post - t_pre) / τ_minus)

w_new = clip(w_old + Δw, w_min, w_max)
```

**Parameters:**
- `A_plus`: Potentiation amplitude (default: 0.005)
- `A_minus`: Depression amplitude (default: 0.00525)
- `τ_plus`: Potentiation time window (default: 20 ms)
- `τ_minus`: Depression time window (default: 20 ms)
- `w_min`, `w_max`: Weight bounds (default: 0, 1)

**Requirements:**
- Implementations MUST support weight clipping
- Implementations SHOULD support triplet STDP for enhanced learning
- Weight updates MUST be atomic on hardware platforms

## 4. Spike Encoding

### 4.1 Rate Coding

**Definition:**
```
firing_rate(x) = k * x + b
where x ∈ [0, 1], firing_rate ∈ [0, f_max]
```

**Parameters:**
- `f_max`: Maximum firing rate (default: 200 Hz)
- `duration`: Encoding window (recommended: 50-200 ms)

**Requirements:**
- MUST use Poisson process for spike generation
- MAY use regular spike trains for deterministic encoding

### 4.2 Latency Coding

**Definition:**
```
spike_time(x) = (1 - x) * T_window
where x ∈ [0, 1], spike_time ∈ [0, T_window]
```

**Parameters:**
- `T_window`: Time window (default: 20 ms, range: 1-50 ms)
- `threshold`: Minimum value to encode (default: 0.1)

**Requirements:**
- Higher values MUST spike earlier
- Values below threshold MAY not generate spikes

### 4.3 Population Coding

**Definition:**
```
response_i(x) = f_max * exp(-((x - μ_i)^2) / (2 * σ^2))
where μ_i is the preferred value of neuron i
```

**Parameters:**
- `n_neurons`: Population size (minimum: 10)
- `σ`: Tuning curve width (default: 0.1)
- `value_range`: [min, max] range to encode

## 5. Hardware Interface Protocol

### 5.1 Address Event Representation (AER)

**Packet Format:**
```
struct SpikePacket {
    uint32_t neuron_id;      // Global neuron identifier
    uint32_t timestamp;      // Time in microseconds
    uint8_t  polarity;       // 0 or 1 for inhibitory/excitatory
    uint8_t  reserved[3];    // Future use
}
```

**Requirements:**
- Packets MUST be transmitted in temporal order
- Timestamp resolution MUST be ≤ 1 μs
- Neuron IDs MUST be unique within network
- Implementations MUST handle packet loss gracefully

### 5.2 Memory-Mapped Interface

**Register Map:**
```
0x0000-0x0FFF: Control registers
0x1000-0x1FFF: Neuron parameters
0x2000-0x2FFF: Synaptic weights (write)
0x3000-0x3FFF: Spike output (read)
0x4000-0x4FFF: Status and diagnostics
```

**Control Registers:**
- `0x0000`: CONTROL (start/stop/reset)
- `0x0004`: STATUS (running/idle/error)
- `0x0008`: TIMESTEP (simulation time step in μs)
- `0x000C`: NUM_NEURONS (number of active neurons)

## 6. Performance Metrics

### 6.1 Energy Efficiency

**Definition:**
```
Energy_Efficiency = Total_Synaptic_Operations / Energy_Consumed

Units: SOPS/W (Synaptic Operations Per Second per Watt)
```

**Measurement Requirements:**
- MUST measure total energy including idle power
- SHOULD measure per-inference energy
- MUST report average, min, max values

**Benchmarks:**
- Minimum acceptable: 100 GOPS/W
- Target: 10 TOPS/W
- State-of-art: 100 TOPS/W

### 6.2 Latency

**Components:**
```
Total_Latency = Encoding_Time + Processing_Time + Decoding_Time
```

**Requirements:**
- MUST report end-to-end latency
- SHOULD break down by component
- MUST use 95th percentile (P95) for reporting

**Benchmarks:**
- Rate coding: < 100 ms acceptable
- Latency coding: < 10 ms target
- Real-time: < 1 ms for critical applications

### 6.3 Accuracy

**Definition:**
```
Accuracy = Correct_Predictions / Total_Predictions
```

**Requirements:**
- MUST test on standard benchmarks (N-MNIST, DVS-Gesture)
- SHOULD report confusion matrix
- MUST use held-out test set

**Benchmarks:**
- N-MNIST: > 95% minimum, > 98% target
- DVS-Gesture: > 90% minimum, > 95% target

## 7. Safety and Reliability

### 7.1 Error Handling

**Requirements:**
- Implementations MUST detect arithmetic overflow
- Implementations MUST handle invalid neuron parameters
- Implementations SHOULD provide error recovery mechanisms

### 7.2 Fault Tolerance

**Stuck-at Faults:**
- Systems SHOULD detect stuck-at-0 and stuck-at-1 faults
- Systems MAY implement fault mapping strategies
- Degradation SHOULD be graceful (< 5% accuracy loss per 1% fault rate)

### 7.3 Determinism

**Requirements:**
- Given identical inputs and parameters, systems SHOULD produce identical outputs
- Non-deterministic sources (e.g., Poisson encoding) MUST be seedable
- Implementations MUST document sources of non-determinism

## 8. Compliance

### 8.1 Conformance Levels

**Level 1 (Basic):**
- LIF neuron support
- Static synapses
- Rate coding
- AER interface

**Level 2 (Standard):**
- Level 1 +
- STDP learning
- Latency coding
- Memory-mapped interface

**Level 3 (Advanced):**
- Level 2 +
- Multiple neuron models
- Population coding
- Hardware acceleration

### 8.2 Testing

**Required Tests:**
- Single neuron dynamics (spike generation, refractory period)
- Synaptic transmission (delay, weight multiplication)
- Learning rules (STDP convergence)
- Encoding/decoding (accuracy, latency)

**Recommended Tests:**
- Benchmark datasets (N-MNIST, DVS-Gesture)
- Energy consumption profiles
- Fault injection tests

## 9. References

### 9.1 Normative References

- [1] IEEE Std 754-2019 — IEEE Standard for Floating-Point Arithmetic.
- [2] WIA-OMNI-API Specification v1.0.
- [3] Address Event Representation (AER) — community-standard event-encoding format used by neuromorphic-vision sensors and processors.

### 9.2 Vendor / Project Documentation (Informative)

- [4] Intel Neuromorphic Research Community — Intel Loihi 2 architecture documentation: https://intel.com/content/www/us/en/research/neuromorphic-computing.html
- [5] IBM Research — TrueNorth chip official architecture documentation.
- [6] University of Manchester APT Group — SpiNNaker / SpiNNaker2 project page: https://apt.cs.manchester.ac.uk/projects/SpiNNaker/

### 9.3 Open Toolchains and Datasets

- [7] Norse — PyTorch-based open-source spiking-neural-network library.
- [8] Nengo — Applied Brain Research neural-simulation framework.
- [9] N-MNIST, DVS128 Gesture, CIFAR10-DVS — public neuromorphic-vision benchmark datasets.

## 10. Appendix A: Example Implementation

**Minimal LIF Neuron (Python):**
```python
class LIFNeuron:
    def __init__(self, tau_m=20, v_threshold=-55, v_reset=-70):
        self.tau_m = tau_m
        self.v_threshold = v_threshold
        self.v_reset = v_reset
        self.v = v_reset
        self.t_last_spike = -float('inf')

    def step(self, current, dt=0.1, t=0):
        if t - self.t_last_spike < 2:  # Refractory period
            return False

        dv = (-(self.v - (-70)) + 10 * current) / self.tau_m
        self.v += dv * dt

        if self.v >= self.v_threshold:
            self.v = self.v_reset
            self.t_last_spike = t
            return True
        return False
```

## 11. Change Log

**v1.0.0 (2025-01-15):**
- Initial release
- Core neuron and synapse models
- Standard encoding protocols
- Performance metrics
- Hardware interfaces

---

© 2025 WIA-Official
弘益人間 (Hongik Ingan) - Benefit All Humanity

**License:** Creative Commons Attribution 4.0 International (CC BY 4.0)
**Status:** Stable
**Maintainer:** WIA Technical Committee
**Contact:** standards@wia-official.org
