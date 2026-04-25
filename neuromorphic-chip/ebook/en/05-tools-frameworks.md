# Chapter 5: Development Tools and Frameworks

## Building Neuromorphic Applications Efficiently

The neuromorphic computing ecosystem has matured significantly, offering developers a rich set of tools, simulators, and frameworks. This chapter surveys the landscape of development tools, from low-level hardware simulators to high-level machine learning frameworks.

## Simulation Frameworks

### 1. Brian2 - Flexible Spiking Neural Network Simulator

**Features:**
- Python-based with C++ code generation
- Equation-based neuron/synapse definitions
- Multiple neuron models (LIF, Izhikevich, HH)
- GPU acceleration support

**Installation:**
```bash
pip install brian2
```

**Example:**
```python
from brian2 import *

# Define parameters
tau = 10*ms
v_threshold = -50*mV
v_reset = -70*mV

# Create neuron model with equations
eqs = '''
dv/dt = (v_rest - v + R*I)/tau : volt
I : amp
v_rest : volt
R : ohm
'''

# Create neuron group
neurons = NeuronGroup(100, eqs,
                     threshold='v > v_threshold',
                     reset='v = v_reset',
                     method='euler')

# Set parameters
neurons.v = v_reset
neurons.v_rest = -70*mV
neurons.R = 10*Mohm

# Input current
neurons.I = '0.5*nA * (i < 50) + 1.5*nA * (i >= 50)'

# Monitor spikes
spike_mon = SpikeMonitor(neurons)
state_mon = StateMonitor(neurons, 'v', record=[0, 50])

# Run simulation
run(100*ms)

# Plot results
plot(spike_mon.t/ms, spike_mon.i, '.')
xlabel('Time (ms)')
ylabel('Neuron index')
show()
```

**Synapses and STDP:**
```python
# Create synaptic connections
synapses = Synapses(neurons, neurons,
                   model='''w : 1
                           dapre/dt = -apre/tau_pre : 1 (event-driven)
                           dapost/dt = -apost/tau_post : 1 (event-driven)''',
                   on_pre='''v_post += w*mV
                            apre += deltaA_pre
                            w = clip(w + apost, 0, w_max)''',
                   on_post='''apost += deltaA_post
                             w = clip(w + apre, 0, w_max)''')

synapses.connect(p=0.1)  # 10% connection probability
synapses.w = 'rand() * w_max'

# STDP parameters
tau_pre = tau_post = 20*ms
deltaA_pre = 0.01
deltaA_post = -deltaA_pre * 1.05
w_max = 1.0
```

### 2. NEST - Large-Scale Neural Simulations

**Features:**
- Designed for biological neural networks
- Excellent scalability (up to supercomputers)
- MPI parallelization
- Extensive neuron/synapse model library

**Example:**
```python
import nest

# Reset kernel
nest.ResetKernel()

# Create neurons
neurons = nest.Create('iaf_psc_alpha', 1000)

# Create Poisson input
noise = nest.Create('poisson_generator', params={'rate': 80000.0})

# Create spike recorder
spikes = nest.Create('spike_recorder')

# Connect input to neurons
nest.Connect(noise, neurons, syn_spec={'weight': 1.2})

# Connect neurons to recorder
nest.Connect(neurons, spikes)

# Recurrent connections with STDP
nest.CopyModel('static_synapse', 'stdp',
              {'weight': 1.0, 'delay': 1.0})

nest.Connect(neurons, neurons,
            conn_spec={'rule': 'fixed_indegree', 'indegree': 100},
            syn_spec={'model': 'stdp_synapse',
                     'alpha': 0.5,
                     'lambda': 0.01,
                     'mu_plus': 0.0,
                     'mu_minus': 0.0,
                     'tau_plus': 20.0})

# Simulate
nest.Simulate(1000.0)  # 1000 ms

# Get spike data
events = spikes.get('events')
print(f"Total spikes: {len(events['times'])}")
```

### 3. BindsNET - Machine Learning with SNNs

**Features:**
- PyTorch integration
- GPU acceleration
- Focus on ML applications
- Easy conversion from ANNs

**Example:**
```python
import torch
from bindsnet.network import Network
from bindsnet.network.nodes import Input, LIFNodes
from bindsnet.network.topology import Connection
from bindsnet.learning import PostPre

# Create network
network = Network(dt=1.0)  # 1ms time step

# Add layers
input_layer = Input(n=784)  # 28x28 MNIST
hidden_layer = LIFNodes(n=400, thresh=-52.0, refrac=5)
output_layer = LIFNodes(n=10, thresh=-40.0)

# Add to network
network.add_layer(layer=input_layer, name='Input')
network.add_layer(layer=hidden_layer, name='Hidden')
network.add_layer(layer=output_layer, name='Output')

# Create connections with STDP
input_hidden = Connection(
    source=input_layer,
    target=hidden_layer,
    w=0.3 * torch.rand(784, 400),
    update_rule=PostPre,
    nu=(1e-4, 1e-2),  # Learning rates
    wmin=0.0,
    wmax=1.0
)

hidden_output = Connection(
    source=hidden_layer,
    target=output_layer,
    w=0.3 * torch.rand(400, 10),
    update_rule=PostPre,
    nu=(1e-4, 1e-2)
)

network.add_connection(input_hidden, source='Input', target='Hidden')
network.add_connection(hidden_output, source='Hidden', target='Output')

# Training
for epoch in range(10):
    for image, label in mnist_loader:
        # Encode image as spikes
        spikes = {'Input': poisson_encode(image, time=100)}

        # Run network
        network.run(inputs=spikes, time=100)

        # Update weights (STDP handles this automatically)
```

### 4. SpikingJelly - PyTorch Extension for SNNs

**Features:**
- Seamless PyTorch integration
- Surrogate gradient learning
- Clock-driven and event-driven modes
- CUDA acceleration

**Example:**
```python
from spikingjelly.clock_driven import neuron, functional, surrogate, layer
import torch.nn as nn

class SpikingCNN(nn.Module):
    def __init__(self, T=4):
        super().__init__()
        self.T = T

        # Conv layers with batch normalization
        self.conv = nn.Sequential(
            layer.Conv2d(1, 128, kernel_size=3, padding=1, bias=False),
            layer.BatchNorm2d(128),
            neuron.IFNode(surrogate_function=surrogate.ATan()),
            layer.MaxPool2d(2, 2),

            layer.Conv2d(128, 128, kernel_size=3, padding=1, bias=False),
            layer.BatchNorm2d(128),
            neuron.IFNode(surrogate_function=surrogate.ATan()),
            layer.MaxPool2d(2, 2)
        )

        # Fully connected
        self.fc = nn.Sequential(
            layer.Flatten(),
            layer.Linear(128 * 7 * 7, 128, bias=False),
            neuron.IFNode(surrogate_function=surrogate.ATan()),
            layer.Linear(128, 10, bias=False),
            neuron.IFNode(surrogate_function=surrogate.ATan())
        )

    def forward(self, x):
        x = x.unsqueeze(0).repeat(self.T, 1, 1, 1, 1)  # [T, N, C, H, W]

        out_spikes_counter = 0
        for t in range(self.T):
            out_spikes_counter += self.fc(self.conv(x[t]))

        return out_spikes_counter / self.T

# Train with standard PyTorch
model = SpikingCNN(T=4)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.CrossEntropyLoss()

for epoch in range(10):
    for img, label in train_loader:
        optimizer.zero_grad()
        output = model(img)
        loss = loss_fn(output, label)
        loss.backward()
        optimizer.step()

        functional.reset_net(model)  # Reset neuron states
```

### 5. Norse - Industrial-Grade SNN Framework

**Features:**
- Production-ready PyTorch library
- Efficient sparse spike handling
- Comprehensive neuron models
- Emphasis on reproducibility

**Example:**
```python
import torch
import norse.torch as norse

# Define network
class NorseNet(torch.nn.Module):
    def __init__(self):
        super().__init__()

        self.fc1 = norse.LIFRecurrentCell(784, 256)
        self.fc2 = norse.LIFCell(256, 10)

    def forward(self, x):
        # x: [T, batch, features]
        T = x.shape[0]

        # Initialize states
        s1 = s2 = None

        voltages = []
        for t in range(T):
            z1, s1 = self.fc1(x[t], s1)
            z2, s2 = self.fc2(z1, s2)
            voltages.append(s2.v)

        return torch.stack(voltages)

model = NorseNet()

# Encode input as Poisson spikes
encoder = norse.PoissonEncoder(seq_length=100, f_max=100)
spike_input = encoder(data)

# Forward pass
output = model(spike_input)
```

## Hardware Development Tools

### Intel Loihi - Lava Framework

```python
from lava.magma.core.process.process import AbstractProcess
from lava.magma.core.process.ports.ports import InPort, OutPort
from lava.magma.core.process.variable import Var

class LIFProcess(AbstractProcess):
    """Custom LIF neuron for Loihi"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        shape = kwargs['shape']

        self.a_in = InPort(shape=shape)
        self.s_out = OutPort(shape=shape)

        self.u = Var(shape=shape, init=0)  # Membrane voltage
        self.v = Var(shape=shape, init=0)  # Current
        self.du = Var(shape=(1,), init=4096)  # Decay
        self.dv = Var(shape=(1,), init=4096)
        self.vth = Var(shape=(1,), init=100)  # Threshold

# Deploy to Loihi 2 chip
from lava.magma.core.run_configs import Loihi2HwCfg

network = create_network()
network.run(condition=RunSteps(num_steps=1000),
           run_cfg=Loihi2HwCfg())
```

### BrainChip Akida - MetaTF

```python
from akida import Model
import tensorflow as tf

# Train standard TensorFlow model
keras_model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(28, 28, 1)),
    tf.keras.layers.Conv2D(32, (3, 3), activation='relu'),
    tf.keras.layers.MaxPooling2D((2, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(10, activation='softmax')
])

# Compile and train
keras_model.compile(optimizer='adam',
                   loss='categorical_crossentropy',
                   metrics=['accuracy'])
keras_model.fit(x_train, y_train, epochs=10)

# Convert to Akida SNN
akida_model = Model(keras_model)

# Quantize to 4-bit for edge deployment
akida_model.quantize(4)

# Evaluate on Akida
accuracy = akida_model.evaluate(x_test, y_test)
print(f"Akida accuracy: {accuracy:.2%}")

# Deploy to hardware
akida_model.map()
akida_model.forward(test_sample)
```

## Dataset and Encoding Libraries

### Tonic - Event-Based Dataset Library

```python
import tonic

# Load neuromorphic MNIST dataset
dataset = tonic.datasets.NMNIST(save_to='./data',
                                train=True)

# Apply transformations
transform = tonic.transforms.Compose([
    tonic.transforms.Denoise(filter_time=10000),
    tonic.transforms.ToFrame(sensor_size=(34, 34, 2),
                            n_time_bins=10)
])

dataset = tonic.datasets.NMNIST(save_to='./data',
                                transform=transform)

# Create data loader
from torch.utils.data import DataLoader

loader = DataLoader(dataset, batch_size=32,
                   collate_fn=tonic.collation.PadTensors())

for events, label in loader:
    # events: [batch, time_bins, channels, height, width]
    output = model(events)
```

### Supported Datasets

- **N-MNIST:** Neuromorphic MNIST (DVS recorded)
- **N-Caltech101:** Object recognition
- **DVS-Gesture:** Hand gesture recognition
- **SHD:** Spiking Heidelberg Digits (audio)
- **SSC:** Spiking Speech Commands
- **POKER-DVS:** Card symbols

## Visualization and Debugging

### Spike Raster Plots

```python
import matplotlib.pyplot as plt

def plot_spikes(spike_times, spike_indices):
    """
    Plot spike raster
    spike_times: When spikes occurred
    spike_indices: Which neuron spiked
    """
    plt.figure(figsize=(10, 6))
    plt.scatter(spike_times, spike_indices, s=1, c='black')
    plt.xlabel('Time (ms)')
    plt.ylabel('Neuron Index')
    plt.title('Spike Raster Plot')
    plt.tight_layout()
    plt.show()

# From Brian2 monitor
plot_spikes(spike_mon.t/ms, spike_mon.i)
```

### Connectivity Visualization

```python
import networkx as nx

def visualize_network(weight_matrix, threshold=0.1):
    """Visualize network connectivity"""
    G = nx.DiGraph()

    # Add nodes
    n_neurons = weight_matrix.shape[0]
    G.add_nodes_from(range(n_neurons))

    # Add edges above threshold
    for i in range(n_neurons):
        for j in range(n_neurons):
            if weight_matrix[i, j] > threshold:
                G.add_edge(i, j, weight=weight_matrix[i, j])

    # Draw
    pos = nx.spring_layout(G)
    nx.draw(G, pos, node_size=300, node_color='lightblue',
           with_labels=True, arrows=True)
    plt.show()
```

## Performance Profiling

### Energy Estimation

```python
def estimate_energy(n_spikes, n_synapses, technology='loihi'):
    """
    Estimate energy consumption
    n_spikes: Total spike events
    n_synapses: Active synapses
    technology: 'loihi', 'truenorth', 'gpu', etc.
    """
    energy_per_spike = {
        'loihi': 1e-12,      # 1 pJ per spike
        'truenorth': 0.1e-12, # 0.1 pJ
        'gpu': 1e-6,         # 1 μJ
        'cpu': 10e-6         # 10 μJ
    }

    return n_spikes * energy_per_spike[technology]

# Compare platforms
spikes = 1000000
for platform in ['loihi', 'truenorth', 'gpu']:
    energy = estimate_energy(spikes, 0, platform)
    print(f"{platform}: {energy*1e6:.2f} μJ")
```

### Latency Analysis

```python
def analyze_latency(spike_train, encoding='rate', time_window=100):
    """
    Analyze network latency
    """
    if encoding == 'rate':
        # Need full integration window
        return time_window
    elif encoding == 'latency':
        # First spike latency
        if len(spike_train) > 0:
            return np.min(spike_train)
        else:
            return float('inf')
    elif encoding == 'population':
        # Time until population stabilizes
        return estimate_population_time(spike_train)
```

## Best Practices

**Development Workflow:**
1. **Prototype in Python:** Use Brian2/BindsNET for rapid development
2. **Optimize with PyTorch:** Convert to SpikingJelly/Norse for training
3. **Hardware Deployment:** Map to Loihi/Akida for production

**Performance Tips:**
- Use event-driven simulation for sparse networks
- Batch processing for throughput
- Quantize weights for hardware deployment
- Profile energy consumption early

**Debugging Strategies:**
- Visualize spike rasters
- Monitor membrane potentials
- Check weight distributions
- Verify spike rates are reasonable (1-100 Hz)

## Summary

This chapter covered:
- **Simulators:** Brian2, NEST, BindsNET, SpikingJelly, Norse
- **Hardware Tools:** Lava (Loihi), MetaTF (Akida)
- **Datasets:** Tonic library, neuromorphic datasets
- **Visualization:** Spike rasters, connectivity graphs
- **Profiling:** Energy and latency estimation

Choose tools based on your use case:
- **Research:** Brian2, NEST
- **ML/DL:** SpikingJelly, Norse, BindsNET
- **Production:** Lava, MetaTF

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
