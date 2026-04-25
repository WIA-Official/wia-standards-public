# Chapter 3: Memristor Crossbars and Neuromorphic Hardware

## The Physical Substrate for Brain-Inspired Computing

While previous chapters focused on algorithms and network architectures, this chapter dives into the hardware foundations that make neuromorphic computing truly revolutionary: memristor-based crossbar arrays and specialized neuromorphic chips.

## Memristor Fundamentals

### What is a Memristor?

A **memristor** (memory + resistor) is the fourth fundamental circuit element, predicted by Leon Chua in 1971 and first physically realized by HP Labs in 2008.

**The Four Fundamental Elements:**
1. **Resistor:** V = IR (Ohm's Law)
2. **Capacitor:** Q = CV
3. **Inductor:** Φ = LI
4. **Memristor:** Φ = M(q)q (resistance depends on charge history)

**Key Properties:**
- **Non-volatile:** Retains state without power
- **Analog states:** Continuous resistance values (not just 0/1)
- **Bidirectional programming:** Can increase or decrease resistance
- **Nanoscale:** Can be fabricated at 10-100nm scale
- **Low energy:** Femtojoule-scale operations

### Physical Mechanisms

**Metal-Oxide Memristors (Most Common):**
```
Top Electrode (Pt)
     |
  TiO₂ (oxygen-rich)
     |
  TiO₂₋ₓ (oxygen-poor)
     |
Bottom Electrode (Pt)
```

**Operation:**
1. **SET (Low Resistance):** Positive voltage drives oxygen vacancies, forms conductive filament
2. **RESET (High Resistance):** Negative voltage ruptures filament
3. **Intermediate States:** Partial filament formation enables analog resistance

**Resistance Range:**
- High Resistance State (HRS): 1-100 MΩ
- Low Resistance State (LRS): 1-100 kΩ
- On/Off Ratio: 10-1000×
- Number of levels: 16-256 distinct states

### Memristor as Synapse

**Perfect Match for Synapses:**
```
Biological Synapse          Memristor Synapse
------------------          -----------------
Synaptic weight      ↔     Conductance (1/R)
Strengthen (LTP)     ↔     Decrease R (SET)
Weaken (LTD)        ↔     Increase R (RESET)
Analog strength     ↔     Continuous R values
Non-volatile        ↔     Retains state
```

**Synaptic Operation:**
```python
class MemristorSynapse:
    def __init__(self, R_init=10e3, R_on=1e3, R_off=100e3,
                 k_on=1e-5, k_off=1e-5):
        """
        R_init: Initial resistance (Ω)
        R_on: Minimum resistance (Ω)
        R_off: Maximum resistance (Ω)
        k_on, k_off: Programming rates
        """
        self.R = R_init
        self.R_on = R_on
        self.R_off = R_off
        self.k_on = k_on
        self.k_off = k_off

    def apply_voltage(self, V, duration):
        """
        Apply voltage pulse to program memristor
        V: Voltage (V), positive for SET, negative for RESET
        duration: Pulse duration (s)
        """
        if V > 0:  # SET operation
            # Exponential approach to R_on
            self.R = self.R_on + (self.R - self.R_on) * np.exp(-self.k_on * V * duration)
        elif V < 0:  # RESET operation
            # Exponential approach to R_off
            self.R = self.R_off + (self.R - self.R_off) * np.exp(-self.k_off * abs(V) * duration)

        return self.R

    def get_conductance(self):
        """Return conductance (synaptic weight)"""
        return 1.0 / self.R

    def compute_current(self, V_input):
        """Compute output current (multiply operation)"""
        return V_input / self.R
```

## Crossbar Array Architecture

### Basic Crossbar Structure

A crossbar array is a grid of wires with memristors at each intersection:

```
         Column Lines (Input Voltages)
              V₁  V₂  V₃  V₄
               |   |   |   |
    Row 1  ----M---M---M---M---- → I₁ = Σ(V_j / R₁ⱼ)
               |   |   |   |
    Row 2  ----M---M---M---M---- → I₂ = Σ(V_j / R₂ⱼ)
               |   |   |   |
    Row 3  ----M---M---M---M---- → I₃ = Σ(V_j / R₃ⱼ)
               |   |   |   |

    M = Memristor
    Output currents = Matrix-Vector Multiplication!
```

**Mathematical Operation:**
```
I_row = Σ(V_col / R) = Σ(V_col × G)

where G = 1/R is conductance (synaptic weight)

This is exactly: I = W × V  (matrix-vector multiply)
```

### Implementing Neural Networks

**Fully Connected Layer:**
```
Input neurons → Voltage encoding
    ↓
Crossbar array (N×M memristors)
    ↓
Output currents → Current sensing → Spike generation
```

**Python Simulation:**
```python
class CrossbarArray:
    def __init__(self, n_rows, n_cols):
        """
        Create crossbar array
        n_rows: Number of output neurons
        n_cols: Number of input neurons
        """
        self.n_rows = n_rows
        self.n_cols = n_cols

        # Initialize memristors with random conductances
        self.conductances = np.random.rand(n_rows, n_cols) * 1e-3  # Siemens

        # Non-ideal effects
        self.wire_resistance = 100  # Ω per memristor
        self.sneak_paths = True  # Model parasitic currents

    def forward(self, input_voltages):
        """
        Compute crossbar output
        input_voltages: Array of input voltages (V)
        Returns: Array of output currents (A)
        """
        if not self.sneak_paths:
            # Ideal case: simple matrix multiply
            output_currents = self.conductances @ input_voltages
        else:
            # Non-ideal: solve Kirchhoff's laws
            output_currents = self._solve_with_parasitic(input_voltages)

        return output_currents

    def _solve_with_parasitic(self, V_in):
        """
        Solve crossbar with wire resistance and sneak paths
        Uses nodal analysis
        """
        # This is computationally expensive but more accurate
        # For large arrays, typically use approximate methods

        # Build conductance matrix including wire resistance
        n_nodes = (self.n_rows + 1) * (self.n_cols + 1)
        G_matrix = np.zeros((n_nodes, n_nodes))

        # Add memristor conductances and wire conductances
        # ... (complex nodal analysis)

        # For simplicity, use ideal model with resistance penalty
        wire_conductance = 1.0 / self.wire_resistance
        effective_conductance = (1.0 / self.conductances + self.wire_resistance)**(-1)

        return effective_conductance @ V_in

    def train_write(self, weight_matrix, precision=8):
        """
        Program memristors to target weights
        weight_matrix: Target synaptic weights
        precision: Bit precision
        """
        # Quantize weights to available resistance levels
        w_min, w_max = weight_matrix.min(), weight_matrix.max()
        levels = 2 ** precision

        # Map weights to conductance range
        G_min, G_max = 1e-6, 1e-3  # Siemens
        normalized = (weight_matrix - w_min) / (w_max - w_min)
        target_G = G_min + normalized * (G_max - G_min)

        # Quantize to discrete levels
        quantized_G = np.round(normalized * (levels - 1)) / (levels - 1)
        quantized_G = G_min + quantized_G * (G_max - G_min)

        self.conductances = quantized_G

    def read_weights(self):
        """Read current conductance values"""
        return self.conductances.copy()

# Example: Implement 784→128 layer for MNIST
crossbar = CrossbarArray(n_rows=128, n_cols=784)

# Train using backprop, then write weights
trained_weights = train_ann_layer(...)  # From your ML framework
crossbar.train_write(trained_weights, precision=8)

# Inference
for image in test_images:
    # Encode pixels as voltages (0-1V range)
    input_voltages = image.flatten() * 1.0  # 784 voltages

    # Crossbar computes weighted sum in one operation
    output_currents = crossbar.forward(input_voltages)

    # Convert currents to spikes
    spikes = (output_currents > threshold)
```

### Non-Ideal Effects and Mitigation

**1. Device Variability**
- **Problem:** Manufacturing variations cause resistance spread
- **Impact:** 10-30% variation in conductance
- **Solution:** Write-verify programming, calibration

**2. Wire Resistance**
- **Problem:** Voltage drops along wires
- **Impact:** Non-uniform voltage across array
- **Solution:** Segmentation, peripheral circuits, compensation

**3. Sneak Paths**
- **Problem:** Current flows through unintended paths
- **Impact:** Reduced on/off ratio, crosstalk
- **Solution:** Selector devices (1T1M, 1S1M), differential encoding

**4. Stuck-at Faults**
- **Problem:** Devices stuck in high or low resistance
- **Impact:** Incorrect computations
- **Solution:** Fault-tolerant mapping, error correction

**5. Endurance/Retention**
- **Problem:** Limited write cycles (~10⁶), state drift over time
- **Impact:** Weight degradation
- **Solution:** Wear leveling, periodic refresh

**Mitigation Strategies:**
```python
class RobustCrossbar(CrossbarArray):
    def __init__(self, n_rows, n_cols):
        super().__init__(n_rows, n_cols)

        # Add variability
        self.variability = np.random.randn(n_rows, n_cols) * 0.1

        # Track write cycles
        self.write_counts = np.zeros((n_rows, n_cols))
        self.max_writes = 1e6

    def forward_with_noise(self, input_voltages, temperature=300):
        """
        Include device variations and thermal noise
        """
        # Device-to-device variation
        effective_G = self.conductances * (1 + self.variability)

        # Thermal noise (Johnson-Nyquist)
        k_B = 1.38e-23  # Boltzmann constant
        bandwidth = 1e9  # Hz
        noise_current = np.sqrt(4 * k_B * temperature * effective_G * bandwidth)

        # Nominal output
        output = effective_G @ input_voltages

        # Add noise
        output += np.random.randn(self.n_rows) * noise_current

        return output

    def adaptive_write(self, target_weights, max_iterations=10):
        """
        Write-verify programming to combat variability
        """
        for iteration in range(max_iterations):
            # Write target weights
            self.train_write(target_weights)

            # Verify by reading
            actual_weights = self.read_weights()

            # Calculate error
            error = target_weights - actual_weights

            # If error small enough, done
            if np.max(np.abs(error)) < 0.01:
                break

            # Otherwise, adjust and retry
            target_weights += 0.5 * error

            self.write_counts += 1

        # Check for wear-out
        worn_devices = self.write_counts > self.max_writes
        if np.any(worn_devices):
            print(f"Warning: {np.sum(worn_devices)} devices worn out")
```

## Advanced Crossbar Techniques

### Differential Encoding

Use pairs of memristors to represent positive and negative weights:

```
Weight W = G⁺ - G⁻

Positive weight: G⁺ high, G⁻ low
Negative weight: G⁺ low, G⁻ high
Zero weight: G⁺ ≈ G⁻
```

**Implementation:**
```python
class DifferentialCrossbar:
    def __init__(self, n_rows, n_cols):
        # Use two crossbars
        self.positive_crossbar = CrossbarArray(n_rows, n_cols)
        self.negative_crossbar = CrossbarArray(n_rows, n_cols)

    def forward(self, input_voltages):
        # Compute difference of currents
        I_plus = self.positive_crossbar.forward(input_voltages)
        I_minus = self.negative_crossbar.forward(input_voltages)
        return I_plus - I_minus

    def train_write(self, signed_weights):
        # Decompose into positive and negative parts
        W_plus = np.maximum(signed_weights, 0)
        W_minus = np.maximum(-signed_weights, 0)

        self.positive_crossbar.train_write(W_plus)
        self.negative_crossbar.train_write(W_minus)
```

### Multi-Bit Encoding

Increase precision using multiple memristors per synapse:

```
Weight = 2⁰×G₀ + 2¹×G₁ + 2²×G₂ + ... + 2ⁿ×Gₙ
```

### 3D Integration

Stack multiple crossbar layers for higher density:

```
     Layer 3 (crossbar)
          |
     Layer 2 (crossbar)
          |
     Layer 1 (crossbar)
          |
     CMOS circuits
```

**Advantages:**
- 10-100× higher synaptic density
- Shorter interconnects → lower latency
- Reduced footprint

**Challenges:**
- Thermal management (heat dissipation)
- Vertical interconnect fabrication
- Yield issues

## Neuromorphic Chip Architectures

### Intel Loihi 2

**Architecture Overview:**
```
┌─────────────────────────────────┐
│     Loihi 2 Chip                │
│                                 │
│  ┌──────┐  ┌──────┐  ┌──────┐  │
│  │ Core │  │ Core │  │ Core │  │
│  │  1   │  │  2   │  │ ...  │  │ 128 cores
│  └──────┘  └──────┘  └──────┘  │
│       │         │         │     │
│  ├────────────────────────────┤ │
│  │   Asynchronous NoC          │ │ Network-on-Chip
│  ├────────────────────────────┤ │
│  │   Learning Engines          │ │ On-chip STDP
│  └────────────────────────────┘ │
└─────────────────────────────────┘
```

**Specifications:**
- **Process:** 14nm (Intel 4 technology for Loihi 2)
- **Cores:** 128 neuromorphic cores
- **Neurons:** 1 million neurons total (~8K per core)
- **Synapses:** 120 million programmable synapses
- **Power:** <100mW typical, <1W peak
- **Latency:** <1ms for inter-core communication

**Core Architecture:**
```python
class LoihiCore:
    def __init__(self, n_neurons=1024):
        self.n_neurons = n_neurons

        # Neuron parameters (programmable LIF, Izhikevich, etc.)
        self.neuron_type = 'LIF'
        self.v_threshold = np.ones(n_neurons) * (-55)
        self.tau_m = np.ones(n_neurons) * 20

        # Synaptic memory (4K synapses per neuron max)
        self.synapse_memory = {}

        # Learning rules (on-chip STDP)
        self.learning_enabled = True
        self.stdp_params = {...}

    def process_spike_packet(self, spike_packet):
        """
        Process incoming spike packet
        spike_packet: (source_neuron_id, timestamp)
        """
        source_id = spike_packet['id']
        timestamp = spike_packet['time']

        # Look up synaptic connections
        if source_id in self.synapse_memory:
            for target_id, weight in self.synapse_memory[source_id]:
                # Deliver weighted spike
                self.neurons[target_id].receive_spike(weight)

                # Update synaptic weight (if learning enabled)
                if self.learning_enabled:
                    self.update_weight_stdp(source_id, target_id, timestamp)
```

**Programming Model:**
```python
# Using Intel's Lava framework
from lava.lib.dl import slayer
from lava.proc.lif.process import LIF
from lava.proc.dense.process import Dense

# Define network
class LoihiNetwork(nn.Module):
    def __init__(self):
        super().__init__()

        # Input layer
        self.fc1 = Dense(in_features=784, out_features=256)
        self.lif1 = LIF(shape=(256,), du=2048, dv=2048, vth=10000)

        # Hidden layer
        self.fc2 = Dense(in_features=256, out_features=128)
        self.lif2 = LIF(shape=(128,), du=2048, dv=2048, vth=10000)

        # Output layer
        self.fc3 = Dense(in_features=128, out_features=10)
        self.lif3 = LIF(shape=(10,), du=2048, dv=2048, vth=10000)

    def forward(self, x):
        x = self.fc1(x)
        x = self.lif1(x)

        x = self.fc2(x)
        x = self.lif2(x)

        x = self.fc3(x)
        x = self.lif3(x)

        return x

# Deploy to Loihi 2
network = LoihiNetwork()
loihi_network = network.to('loihi')  # Compile and map to hardware
```

### IBM TrueNorth

**Architecture:**
```
TrueNorth Chip (4096 cores)
    ├── Core 0: 256 neurons, 256×256 synapses
    ├── Core 1: 256 neurons, 256×256 synapses
    ├── ...
    └── Core 4095: 256 neurons, 256×256 synapses

Total: 1M neurons, 256M synapses
```

**Core Structure:**
```
     Axons (256 inputs)
          │
    ┌─────────────┐
    │  Crossbar   │ 256×256 synaptic weights
    │   (SRAM)    │
    └─────────────┘
          │
    [ 256 Neurons ] (LIF)
          │
    Spike Outputs → Router → Other cores
```

**Key Features:**
- **Power:** 70mW active, 26mW standby
- **Real-time:** Biological time scale (1ms time step)
- **Fixed-point:** 16-bit activations, 1-bit weights
- **Event-driven:** Only active cores consume power

### BrainChip Akida

Commercial neuromorphic processor for edge AI:

**Features:**
- **Incremental learning:** On-device learning without retraining
- **Event-based processing:** Compatible with DVS cameras
- **Power:** <1mW for inference, <10mW with learning
- **Applications:** Keyword spotting, object detection, anomaly detection

**Programming:**
```python
# Using BrainChip MetaTF framework
import akida
import tensorflow as tf

# Train in TensorFlow
model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(32, (3,3), activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(64, (3,3), activation='relu'),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(10, activation='softmax')
])

model.compile(optimizer='adam', loss='categorical_crossentropy')
model.fit(x_train, y_train, epochs=10)

# Convert to Akida SNN
akida_model = akida.Model(model)

# Deploy to Akida hardware
device = akida.Device()
device.load_model(akida_model)

# Inference
prediction = device.predict(test_image)
```

## Summary

This chapter covered:
- **Memristors:** Physical implementation of synapses
- **Crossbar arrays:** Efficient matrix multiplication in hardware
- **Non-ideal effects:** Variability, wire resistance, solutions
- **Neuromorphic chips:** Loihi 2, TrueNorth, Akida architectures

Next: Spike encoding and decoding protocols for neuromorphic systems.

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
