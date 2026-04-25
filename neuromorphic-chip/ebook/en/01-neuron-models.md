# Chapter 1: Neuron Models and Synaptic Dynamics

## Understanding the Building Blocks of Neuromorphic Computing

The foundation of any neuromorphic system is the neuron model - a mathematical abstraction of how biological neurons process and transmit information. This chapter explores the spectrum of neuron models, from simple threshold units to biologically detailed conductance-based models, helping you choose the right model for your application.

## The Biological Neuron

Before diving into mathematical models, let's understand what we're trying to emulate:

**Biological Neuron Structure:**
```
Dendrites → Soma (Cell Body) → Axon → Synaptic Terminals
   ↑           (integrates)      ↓         (outputs)
 (inputs)       (fires if        (spike      (to other
                threshold)     propagates)    neurons)
```

**Key Processes:**
1. **Integration:** Dendrites receive spikes from thousands of other neurons
2. **Summation:** Soma integrates postsynaptic potentials (PSPs)
3. **Threshold:** When voltage exceeds threshold (~-55mV), neuron fires
4. **Spike Generation:** All-or-nothing action potential (~100mV amplitude)
5. **Refractory Period:** Brief period (1-5ms) where neuron cannot fire again
6. **Reset:** Membrane potential returns to resting state (~-70mV)

**Biological Parameters:**
- Resting potential: -70 mV
- Threshold potential: -55 mV
- Spike amplitude: ~100 mV
- Spike duration: ~1 ms
- Refractory period: 1-5 ms
- Membrane time constant: 10-20 ms
- Synaptic delay: 0.5-2 ms

## 1. Leaky Integrate-and-Fire (LIF) Model

The LIF model is the workhorse of neuromorphic computing, offering an excellent balance between biological plausibility and computational efficiency.

### Mathematical Formulation

**Differential Equation:**
```
τ_m * dV/dt = -(V - V_rest) + R * I(t)

If V ≥ V_threshold:
    - Emit spike
    - V ← V_reset
    - Enter refractory period for τ_ref
```

**Parameters:**
- `τ_m`: Membrane time constant (10-20 ms)
- `V`: Membrane potential
- `V_rest`: Resting potential (-70 mV)
- `V_threshold`: Firing threshold (-55 mV)
- `V_reset`: Reset potential (-70 mV)
- `R`: Membrane resistance (10 MΩ)
- `I(t)`: Input current
- `τ_ref`: Refractory period (2 ms)

### Implementation

**Python Implementation:**
```python
import numpy as np

class LIFNeuron:
    def __init__(self, tau_m=20.0, v_rest=-70.0, v_threshold=-55.0,
                 v_reset=-70.0, tau_ref=2.0, resistance=10.0):
        self.tau_m = tau_m  # ms
        self.v_rest = v_rest  # mV
        self.v_threshold = v_threshold  # mV
        self.v_reset = v_reset  # mV
        self.tau_ref = tau_ref  # ms
        self.R = resistance  # MΩ

        self.v = v_rest  # Current voltage
        self.t_last_spike = -np.inf  # Last spike time

    def step(self, current, dt=0.1):
        """
        Simulate one time step
        current: Input current in nA
        dt: Time step in ms
        Returns: True if spike occurred
        """
        # Check if in refractory period
        if (self.t - self.t_last_spike) < self.tau_ref:
            return False

        # Update voltage using Euler method
        dv = (-(self.v - self.v_rest) + self.R * current) / self.tau_m
        self.v += dv * dt

        # Check for spike
        if self.v >= self.v_threshold:
            self.v = self.v_reset
            self.t_last_spike = self.t
            return True

        return False

    def simulate(self, current_func, duration, dt=0.1):
        """
        Simulate neuron over time period
        current_func: Function that returns current at time t
        duration: Simulation duration in ms
        dt: Time step in ms
        """
        steps = int(duration / dt)
        times = np.arange(0, duration, dt)
        voltages = np.zeros(steps)
        spikes = []

        for i, t in enumerate(times):
            self.t = t
            current = current_func(t)

            if self.step(current, dt):
                spikes.append(t)

            voltages[i] = self.v

        return times, voltages, spikes
```

**Example Usage:**
```python
# Create neuron
neuron = LIFNeuron()

# Constant current input
def constant_current(t):
    return 1.5  # nA

# Simulate for 100ms
times, voltages, spikes = neuron.simulate(constant_current, duration=100)

print(f"Number of spikes: {len(spikes)}")
print(f"Firing rate: {len(spikes) / 0.1:.2f} Hz")
print(f"Spike times: {spikes}")
```

### Analytical Solutions

For constant input current, we can derive the firing rate analytically:

**Steady-State Voltage:**
```
V_∞ = V_rest + R * I
```

**Inter-Spike Interval (ISI):**
```
ISI = τ_m * ln((V_∞ - V_rest) / (V_∞ - V_threshold)) + τ_ref
```

**Firing Rate:**
```
f = 1 / ISI  (if V_∞ > V_threshold)
f = 0        (otherwise)
```

### Hardware Implementation

**Digital Circuit (Simplified):**
```verilog
module lif_neuron #(
    parameter VOLTAGE_BITS = 16,
    parameter CURRENT_BITS = 16
) (
    input wire clk,
    input wire rst,
    input wire [CURRENT_BITS-1:0] current_in,
    output reg spike_out
);

    // Parameters (scaled for fixed-point)
    localparam TAU_M = 16'd20;
    localparam V_REST = 16'd0;      // Shifted to 0 for simplicity
    localparam V_THRESHOLD = 16'd15;
    localparam V_RESET = 16'd0;
    localparam RESISTANCE = 16'd10;

    // State
    reg [VOLTAGE_BITS-1:0] voltage;
    reg [7:0] refractory_counter;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            voltage <= V_REST;
            spike_out <= 1'b0;
            refractory_counter <= 8'd0;
        end else begin
            spike_out <= 1'b0;

            if (refractory_counter > 0) begin
                // In refractory period
                refractory_counter <= refractory_counter - 1;
            end else begin
                // Update voltage: V += (-(V - V_rest) + R*I) / tau_m
                wire signed [VOLTAGE_BITS-1:0] leak = voltage - V_REST;
                wire signed [VOLTAGE_BITS-1:0] input_term = (RESISTANCE * current_in) >> 8;
                wire signed [VOLTAGE_BITS-1:0] delta = (input_term - leak) / TAU_M;

                voltage <= voltage + delta;

                // Check threshold
                if (voltage >= V_THRESHOLD) begin
                    voltage <= V_RESET;
                    spike_out <= 1'b1;
                    refractory_counter <= 8'd2;  // 2 clock cycles refractory
                end
            end
        end
    end
endmodule
```

## 2. Izhikevich Neuron Model

The Izhikevich model provides rich dynamics while maintaining computational efficiency, capable of reproducing 20+ different firing patterns observed in biological neurons.

### Mathematical Formulation

**Coupled Differential Equations:**
```
dv/dt = 0.04v² + 5v + 140 - u + I
du/dt = a(bv - u)

If v ≥ 30 mV:
    v ← c
    u ← u + d
```

**Parameters:**
- `v`: Membrane potential
- `u`: Recovery variable
- `I`: Input current
- `a`: Recovery time scale (0.01-0.1)
- `b`: Sensitivity of u to v (0.2-0.3)
- `c`: Reset value for v (-65 to -50 mV)
- `d`: Reset increment for u (2-8)

### Neuron Types

By adjusting parameters, we can create different neuron types:

**Regular Spiking (RS) - Cortical Pyramidal Neurons:**
```python
a, b, c, d = 0.02, 0.2, -65, 8
```

**Intrinsically Bursting (IB):**
```python
a, b, c, d = 0.02, 0.2, -55, 4
```

**Chattering (CH):**
```python
a, b, c, d = 0.02, 0.2, -50, 2
```

**Fast Spiking (FS) - Interneurons:**
```python
a, b, c, d = 0.1, 0.2, -65, 2
```

**Low-Threshold Spiking (LTS):**
```python
a, b, c, d = 0.02, 0.25, -65, 2
```

### Python Implementation

```python
class IzhikevichNeuron:
    def __init__(self, a=0.02, b=0.2, c=-65, d=8):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

        self.v = -65  # Initial voltage
        self.u = b * self.v  # Initial recovery

    def step(self, current, dt=0.1):
        """
        Simulate one time step
        current: Input current
        dt: Time step in ms
        Returns: True if spike occurred
        """
        # Update using Euler method
        for _ in range(int(1/dt)):  # Sub-stepping for stability
            dv = (0.04 * self.v**2 + 5 * self.v + 140 - self.u + current) * dt
            du = self.a * (self.b * self.v - self.u) * dt

            self.v += dv
            self.u += du

        # Check for spike
        if self.v >= 30:
            spike = True
            self.v = self.c
            self.u += self.d
        else:
            spike = False

        return spike

# Example: Create different neuron types
neurons = {
    'Regular Spiking': IzhikevichNeuron(0.02, 0.2, -65, 8),
    'Fast Spiking': IzhikevichNeuron(0.1, 0.2, -65, 2),
    'Bursting': IzhikevichNeuron(0.02, 0.2, -55, 4),
}

# Simulate with constant current
for name, neuron in neurons.items():
    spikes = []
    for t in range(0, 100):
        if neuron.step(current=10, dt=0.1):
            spikes.append(t * 0.1)
    print(f"{name}: {len(spikes)} spikes")
```

## 3. Hodgkin-Huxley Model

The Hodgkin-Huxley model is the most biologically accurate, modeling ion channel dynamics that generate action potentials. While computationally expensive, it's essential for detailed neural simulations.

### Mathematical Formulation

**Main Equation:**
```
C_m * dV/dt = I - I_Na - I_K - I_L

where:
I_Na = g_Na * m³ * h * (V - E_Na)  [Sodium current]
I_K = g_K * n⁴ * (V - E_K)         [Potassium current]
I_L = g_L * (V - E_L)              [Leak current]
```

**Gating Variables:**
```
dm/dt = α_m(V) * (1 - m) - β_m(V) * m
dh/dt = α_h(V) * (1 - h) - β_h(V) * h
dn/dt = α_n(V) * (1 - n) - β_n(V) * n
```

**Parameters:**
- `C_m`: Membrane capacitance (1 μF/cm²)
- `g_Na`: Max sodium conductance (120 mS/cm²)
- `g_K`: Max potassium conductance (36 mS/cm²)
- `g_L`: Leak conductance (0.3 mS/cm²)
- `E_Na`: Sodium reversal potential (+50 mV)
- `E_K`: Potassium reversal potential (-77 mV)
- `E_L`: Leak reversal potential (-54.4 mV)

### Implementation

```python
import numpy as np

class HodgkinHuxleyNeuron:
    def __init__(self):
        # Parameters
        self.C_m = 1.0      # μF/cm²
        self.g_Na = 120.0   # mS/cm²
        self.g_K = 36.0     # mS/cm²
        self.g_L = 0.3      # mS/cm²
        self.E_Na = 50.0    # mV
        self.E_K = -77.0    # mV
        self.E_L = -54.4    # mV

        # State variables
        self.V = -65.0
        self.m = 0.05
        self.h = 0.6
        self.n = 0.32

    def alpha_m(self, V):
        return 0.1 * (V + 40) / (1 - np.exp(-(V + 40) / 10))

    def beta_m(self, V):
        return 4.0 * np.exp(-(V + 65) / 18)

    def alpha_h(self, V):
        return 0.07 * np.exp(-(V + 65) / 20)

    def beta_h(self, V):
        return 1.0 / (1 + np.exp(-(V + 35) / 10))

    def alpha_n(self, V):
        return 0.01 * (V + 55) / (1 - np.exp(-(V + 55) / 10))

    def beta_n(self, V):
        return 0.125 * np.exp(-(V + 65) / 80)

    def step(self, I_ext, dt=0.01):
        # Calculate currents
        I_Na = self.g_Na * self.m**3 * self.h * (self.V - self.E_Na)
        I_K = self.g_K * self.n**4 * (self.V - self.E_K)
        I_L = self.g_L * (self.V - self.E_L)

        # Update voltage
        dV = (I_ext - I_Na - I_K - I_L) / self.C_m
        self.V += dV * dt

        # Update gating variables
        dm = (self.alpha_m(self.V) * (1 - self.m) - self.beta_m(self.V) * self.m)
        dh = (self.alpha_h(self.V) * (1 - self.h) - self.beta_h(self.V) * self.h)
        dn = (self.alpha_n(self.V) * (1 - self.n) - self.beta_n(self.V) * self.n)

        self.m += dm * dt
        self.h += dh * dt
        self.n += dn * dt

        return self.V
```

## 4. Synaptic Dynamics

Synapses are the connections between neurons, implementing learning and memory through plasticity.

### Exponential Synapse Model

**Postsynaptic Current:**
```
I_syn(t) = w * Σ_spikes exp(-(t - t_spike) / τ_syn)

where:
- w: Synaptic weight
- τ_syn: Synaptic time constant (1-10 ms)
- t_spike: Spike arrival times
```

**Implementation:**
```python
class ExponentialSynapse:
    def __init__(self, weight, tau_syn=5.0):
        self.weight = weight
        self.tau_syn = tau_syn
        self.g = 0.0  # Conductance

    def receive_spike(self):
        """Called when presynaptic spike arrives"""
        self.g += 1.0

    def step(self, dt=0.1):
        """Update synaptic conductance"""
        self.g *= np.exp(-dt / self.tau_syn)
        return self.weight * self.g
```

### Spike-Timing-Dependent Plasticity (STDP)

STDP is the primary learning rule in neuromorphic systems:

**Weight Update Rule:**
```
Δw = A+ * exp(-Δt / τ+)  if Δt > 0 (pre before post)
Δw = -A- * exp(Δt / τ-)  if Δt < 0 (post before pre)

where:
- Δt = t_post - t_pre
- A+, A-: Learning rates
- τ+, τ-: Time constants
```

**Implementation:**
```python
class STDPSynapse:
    def __init__(self, weight_init=0.5, A_plus=0.005, A_minus=0.00525,
                 tau_plus=20.0, tau_minus=20.0, w_min=0.0, w_max=1.0):
        self.weight = weight_init
        self.A_plus = A_plus
        self.A_minus = A_minus
        self.tau_plus = tau_plus
        self.tau_minus = tau_minus
        self.w_min = w_min
        self.w_max = w_max

        self.trace_pre = 0.0
        self.trace_post = 0.0

    def pre_spike(self, t):
        """Presynaptic spike occurred"""
        # Update weight based on postsynaptic trace
        self.weight -= self.A_minus * self.trace_post
        self.weight = np.clip(self.weight, self.w_min, self.w_max)

        # Update presynaptic trace
        self.trace_pre += 1.0

    def post_spike(self, t):
        """Postsynaptic spike occurred"""
        # Update weight based on presynaptic trace
        self.weight += self.A_plus * self.trace_pre
        self.weight = np.clip(self.weight, self.w_min, self.w_max)

        # Update postsynaptic trace
        self.trace_post += 1.0

    def step(self, dt=0.1):
        """Decay traces"""
        self.trace_pre *= np.exp(-dt / self.tau_plus)
        self.trace_post *= np.exp(-dt / self.tau_minus)
```

## 5. Model Selection Guide

**Choose LIF when:**
- Energy efficiency is critical
- Hardware implementation needed
- Large-scale networks (>10K neurons)
- Real-time processing required

**Choose Izhikevich when:**
- Diverse firing patterns needed
- Moderate biological realism required
- Medium-scale networks (100-10K neurons)
- Researching neural dynamics

**Choose Hodgkin-Huxley when:**
- Maximum biological accuracy needed
- Studying ion channel dynamics
- Small networks (<100 neurons)
- Pharmaceutical/medical research

## Summary

This chapter covered the fundamental neuron models used in neuromorphic computing:
- **LIF:** Simple, efficient, hardware-friendly
- **Izhikevich:** Rich dynamics, moderate complexity
- **Hodgkin-Huxley:** Biologically accurate, computationally expensive
- **Synapses:** Exponential dynamics and STDP learning

In the next chapter, we'll build complete spiking neural networks using these components.

---

**Exercises:**
1. Implement a LIF neuron in your favorite programming language
2. Compare firing patterns of different Izhikevich neuron types
3. Implement STDP and observe weight changes with different spike patterns
4. Design a hardware circuit for a simplified LIF neuron

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
