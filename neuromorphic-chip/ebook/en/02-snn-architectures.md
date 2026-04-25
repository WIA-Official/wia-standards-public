# Chapter 2: Spiking Neural Network Architectures

## From Single Neurons to Intelligent Systems

Building on the neuron models from Chapter 1, this chapter explores how to construct complete spiking neural networks (SNNs) that can perform practical tasks like classification, pattern recognition, and control. We'll cover network topologies, learning algorithms, and conversion techniques from traditional deep learning models.

## Network Topologies

### 1. Feedforward SNNs

The simplest SNN architecture processes information in one direction from input to output.

**Architecture:**
```
Input Layer → Hidden Layer(s) → Output Layer
    |              |                 |
  Encoding    LIF Neurons      Decision/
  (spikes)    + STDP         Decoding
```

**Python Implementation:**
```python
import numpy as np

class FeedforwardSNN:
    def __init__(self, layer_sizes, dt=0.1):
        """
        layer_sizes: List of neurons per layer [input, hidden1, ..., output]
        dt: Time step in ms
        """
        self.layer_sizes = layer_sizes
        self.dt = dt
        self.n_layers = len(layer_sizes)

        # Initialize neurons for each layer
        self.neurons = []
        for size in layer_sizes:
            layer = [LIFNeuron() for _ in range(size)]
            self.neurons.append(layer)

        # Initialize synaptic weights (random initialization)
        self.weights = []
        for i in range(self.n_layers - 1):
            w = np.random.randn(layer_sizes[i], layer_sizes[i+1]) * 0.1
            self.weights.append(w)

        # Synaptic traces for STDP
        self.pre_traces = [np.zeros(size) for size in layer_sizes[:-1]]
        self.post_traces = [np.zeros(size) for size in layer_sizes[1:]]

    def forward(self, input_spikes):
        """
        Propagate spikes through network
        input_spikes: Boolean array of input layer spikes
        Returns: Output layer spikes
        """
        layer_spikes = [input_spikes]

        for layer_idx in range(self.n_layers - 1):
            # Calculate input currents for next layer
            input_currents = np.dot(layer_spikes[layer_idx],
                                   self.weights[layer_idx])

            # Update neurons and collect spikes
            next_spikes = np.zeros(self.layer_sizes[layer_idx + 1], dtype=bool)
            for i, neuron in enumerate(self.neurons[layer_idx + 1]):
                next_spikes[i] = neuron.step(input_currents[i], self.dt)

            layer_spikes.append(next_spikes)

        return layer_spikes

    def train_stdp(self, input_pattern, duration=100, learning_rate=0.001):
        """
        Train network using STDP
        input_pattern: Function that generates input spikes at time t
        duration: Training duration in ms
        learning_rate: STDP learning rate
        """
        steps = int(duration / self.dt)

        for step in range(steps):
            t = step * self.dt

            # Get input spikes for this time step
            input_spikes = input_pattern(t)

            # Forward pass
            layer_spikes = self.forward(input_spikes)

            # STDP weight updates
            for layer_idx in range(self.n_layers - 1):
                pre_spikes = layer_spikes[layer_idx]
                post_spikes = layer_spikes[layer_idx + 1]

                # Update traces
                self.pre_traces[layer_idx] *= np.exp(-self.dt / 20.0)
                self.post_traces[layer_idx] *= np.exp(-self.dt / 20.0)

                self.pre_traces[layer_idx] += pre_spikes
                self.post_traces[layer_idx] += post_spikes

                # Weight updates
                if np.any(pre_spikes):
                    # LTD: Pre-spike decreases weights based on post-trace
                    self.weights[layer_idx] -= learning_rate * np.outer(
                        pre_spikes, self.post_traces[layer_idx])

                if np.any(post_spikes):
                    # LTP: Post-spike increases weights based on pre-trace
                    self.weights[layer_idx] += learning_rate * np.outer(
                        self.pre_traces[layer_idx], post_spikes)

                # Clip weights to [0, 1]
                self.weights[layer_idx] = np.clip(self.weights[layer_idx], 0, 1)

    def predict(self, input_spikes_sequence):
        """
        Classify input by counting output spikes
        input_spikes_sequence: List of input spike arrays over time
        Returns: Index of most active output neuron
        """
        output_counts = np.zeros(self.layer_sizes[-1])

        for input_spikes in input_spikes_sequence:
            layer_spikes = self.forward(input_spikes)
            output_counts += layer_spikes[-1]

        return np.argmax(output_counts)

# Example: MNIST classification
def create_mnist_snn():
    # Architecture: 784 input → 128 hidden → 10 output
    snn = FeedforwardSNN([784, 128, 10])
    return snn
```

### 2. Recurrent SNNs

Recurrent connections enable temporal processing and memory, crucial for sequential data.

**Reservoir Computing (Liquid State Machine):**
```
Input → Random Recurrent Network → Linear Readout
        (Reservoir/Liquid)           (Trained)
```

**Key Concepts:**
- **Reservoir:** Large randomly connected recurrent network
- **Echo State Property:** Network state contains history of inputs
- **Readout:** Only output weights are trained (simple, fast)

**Implementation:**
```python
class LiquidStateMachine:
    def __init__(self, n_inputs, n_reservoir, n_outputs,
                 connectivity=0.1, spectral_radius=0.9):
        self.n_inputs = n_inputs
        self.n_reservoir = n_reservoir
        self.n_outputs = n_outputs

        # Random input weights
        self.W_in = np.random.randn(n_inputs, n_reservoir) * 0.1

        # Random recurrent weights (sparse)
        mask = np.random.rand(n_reservoir, n_reservoir) < connectivity
        W_res = np.random.randn(n_reservoir, n_reservoir) * mask

        # Scale to desired spectral radius
        eigenvalues = np.linalg.eigvals(W_res)
        W_res *= spectral_radius / np.max(np.abs(eigenvalues))
        self.W_res = W_res

        # Output weights (to be trained)
        self.W_out = np.random.randn(n_reservoir, n_outputs) * 0.01

        # Reservoir state
        self.state = np.zeros(n_reservoir)

    def step(self, input_spikes):
        """Update reservoir state and compute output"""
        # Input contribution
        input_current = np.dot(input_spikes, self.W_in)

        # Recurrent contribution
        recurrent_current = np.dot(self.state, self.W_res)

        # Update reservoir (simplified activation)
        total_current = input_current + recurrent_current
        self.state = np.tanh(total_current)  # Or use LIF neurons

        # Compute output
        output = np.dot(self.state, self.W_out)
        return output

    def train_readout(self, X_train, y_train):
        """
        Train readout weights using ridge regression
        X_train: List of input spike sequences
        y_train: Target outputs
        """
        # Collect reservoir states
        states = []
        for input_sequence in X_train:
            self.state = np.zeros(self.n_reservoir)  # Reset
            for input_spikes in input_sequence:
                self.step(input_spikes)
                states.append(self.state.copy())

        states = np.array(states)

        # Ridge regression
        ridge_param = 1e-6
        self.W_out = np.linalg.lstsq(
            states.T @ states + ridge_param * np.eye(self.n_reservoir),
            states.T @ y_train,
            rcond=None
        )[0]
```

### 3. Convolutional SNNs

Extending CNNs to spiking domain for efficient visual processing.

**Architecture:**
```
Event Camera → Conv Layers → Pooling → FC Layers → Output
   (DVS)       (Spiking)    (Max/Avg)   (Spiking)
```

**Key Adaptations:**
- **Spike-based Convolution:** Convolve binary spike trains with learned kernels
- **Temporal Coding:** Information in spike timing, not just rate
- **Event-driven Processing:** Only compute when spikes occur

**Implementation:**
```python
class SpikingConvLayer:
    def __init__(self, in_channels, out_channels, kernel_size, stride=1):
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.stride = stride

        # Initialize convolutional kernels
        self.kernels = np.random.randn(
            out_channels, in_channels, kernel_size, kernel_size
        ) * 0.1

        # LIF neurons for each output feature map position
        self.neurons = {}  # Lazy initialization

    def forward(self, input_spikes, height, width):
        """
        input_spikes: [in_channels, height, width] binary array
        Returns: [out_channels, out_h, out_w] spikes
        """
        out_h = (height - self.kernel_size) // self.stride + 1
        out_w = (width - self.kernel_size) // self.stride + 1

        output_spikes = np.zeros((self.out_channels, out_h, out_w), dtype=bool)

        for c_out in range(self.out_channels):
            for i in range(out_h):
                for j in range(out_w):
                    # Extract receptive field
                    i_start = i * self.stride
                    j_start = j * self.stride
                    receptive_field = input_spikes[
                        :,
                        i_start:i_start+self.kernel_size,
                        j_start:j_start+self.kernel_size
                    ]

                    # Compute weighted sum (spike convolution)
                    current = np.sum(receptive_field * self.kernels[c_out])

                    # Get or create neuron for this position
                    neuron_key = (c_out, i, j)
                    if neuron_key not in self.neurons:
                        self.neurons[neuron_key] = LIFNeuron()

                    # Check if neuron fires
                    output_spikes[c_out, i, j] = self.neurons[neuron_key].step(current)

        return output_spikes

class SpikingCNN:
    def __init__(self):
        # Example: Simple spiking CNN for MNIST
        self.conv1 = SpikingConvLayer(1, 32, kernel_size=5)
        self.conv2 = SpikingConvLayer(32, 64, kernel_size=5)
        self.fc = FeedforwardSNN([64 * 4 * 4, 128, 10])

    def forward(self, input_image):
        # Encode input as spikes (rate coding)
        input_spikes = (np.random.rand(*input_image.shape) < input_image).astype(bool)

        # Conv layer 1: 28x28 → 24x24
        x = self.conv1.forward(input_spikes[np.newaxis], 28, 28)

        # Max pooling: 24x24 → 12x12
        x = self.max_pool_spikes(x, pool_size=2)

        # Conv layer 2: 12x12 → 8x8
        x = self.conv2.forward(x, 12, 12)

        # Max pooling: 8x8 → 4x4
        x = self.max_pool_spikes(x, pool_size=2)

        # Flatten
        x = x.reshape(-1)

        # Fully connected layers
        output = self.fc.forward(x)

        return output

    def max_pool_spikes(self, spikes, pool_size):
        """Max pooling for spike trains"""
        c, h, w = spikes.shape
        out_h, out_w = h // pool_size, w // pool_size
        pooled = np.zeros((c, out_h, out_w), dtype=bool)

        for i in range(out_h):
            for j in range(out_w):
                pool_region = spikes[
                    :,
                    i*pool_size:(i+1)*pool_size,
                    j*pool_size:(j+1)*pool_size
                ]
                pooled[:, i, j] = np.any(pool_region, axis=(1, 2))

        return pooled
```

## Learning Algorithms

### 1. Spike-Timing-Dependent Plasticity (STDP)

**Advantages:**
- Unsupervised learning
- Local computation (no backpropagation)
- Biologically plausible
- Online learning

**Disadvantages:**
- Slower convergence than supervised methods
- Requires careful tuning
- May need supervised readout

**Triplet STDP (Enhanced):**
```python
class TripletSTDP:
    """
    Triplet STDP considers three spikes instead of pairs,
    capturing more complex temporal correlations
    """
    def __init__(self, tau_plus=20, tau_minus=20, tau_x=100, tau_y=100,
                 A2_plus=0.005, A2_minus=0.00525, A3_plus=0.001, A3_minus=0.001):
        # Time constants
        self.tau_plus = tau_plus
        self.tau_minus = tau_minus
        self.tau_x = tau_x
        self.tau_y = tau_y

        # Learning rates
        self.A2_plus = A2_plus
        self.A2_minus = A2_minus
        self.A3_plus = A3_plus
        self.A3_minus = A3_minus

        # Traces
        self.r1 = 0.0  # Short-term pre-trace
        self.r2 = 0.0  # Long-term pre-trace
        self.o1 = 0.0  # Short-term post-trace
        self.o2 = 0.0  # Long-term post-trace

    def pre_spike(self):
        """Handle presynaptic spike"""
        # Weight depression (depends on recent post-spikes)
        delta_w = -self.A2_minus * self.o1 - self.A3_minus * self.o1 * self.r2

        # Update traces
        self.r1 += 1.0
        self.r2 += 1.0

        return delta_w

    def post_spike(self):
        """Handle postsynaptic spike"""
        # Weight potentiation (depends on recent pre-spikes)
        delta_w = self.A2_plus * self.r1 + self.A3_plus * self.r1 * self.o2

        # Update traces
        self.o1 += 1.0
        self.o2 += 1.0

        return delta_w

    def step(self, dt):
        """Decay traces"""
        self.r1 *= np.exp(-dt / self.tau_plus)
        self.r2 *= np.exp(-dt / self.tau_x)
        self.o1 *= np.exp(-dt / self.tau_minus)
        self.o2 *= np.exp(-dt / self.tau_y)
```

### 2. Surrogate Gradient Learning

Enables backpropagation through spikes by using smooth approximations.

**Problem:** Spike function is non-differentiable
```
spike = 1 if V ≥ V_th else 0
```

**Solution:** Use surrogate gradient during backprop
```
∂spike/∂V ≈ σ'(V) where σ is smooth function
```

**Common Surrogate Functions:**
```python
def fast_sigmoid_surrogate(v, threshold, beta=10):
    """Fast sigmoid surrogate gradient"""
    return beta / (beta * np.abs(v - threshold) + 1) ** 2

def exponential_surrogate(v, threshold, alpha=1.0):
    """Exponential surrogate gradient"""
    return alpha * np.exp(-alpha * np.abs(v - threshold))

def triangular_surrogate(v, threshold, gamma=0.5):
    """Triangular surrogate gradient"""
    return np.maximum(0, 1 - np.abs(v - threshold) / gamma) / gamma
```

**SpikingJelly Framework Example:**
```python
import torch
import torch.nn as nn
from spikingjelly.clock_driven import neuron, functional, surrogate

class SpikingNet(nn.Module):
    def __init__(self, T=8):
        super().__init__()
        self.T = T  # Number of time steps

        # Network layers with LIF neurons
        self.fc1 = nn.Linear(784, 256)
        self.lif1 = neuron.LIFNode(
            surrogate_function=surrogate.ATan(),
            detach_reset=True
        )

        self.fc2 = nn.Linear(256, 128)
        self.lif2 = neuron.LIFNode(
            surrogate_function=surrogate.ATan(),
            detach_reset=True
        )

        self.fc3 = nn.Linear(128, 10)
        self.lif3 = neuron.LIFNode(
            surrogate_function=surrogate.ATan(),
            detach_reset=True
        )

    def forward(self, x):
        # x shape: [batch, 784]
        # Repeat input for T time steps
        x = x.unsqueeze(0).repeat(self.T, 1, 1)  # [T, batch, 784]

        spike_counts = []
        for t in range(self.T):
            x_t = self.fc1(x[t])
            x_t = self.lif1(x_t)

            x_t = self.fc2(x_t)
            x_t = self.lif2(x_t)

            x_t = self.fc3(x_t)
            x_t = self.lif3(x_t)

            spike_counts.append(x_t)

        # Sum spikes over time
        return torch.stack(spike_counts).sum(0)

# Training loop
model = SpikingNet(T=8)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
criterion = nn.CrossEntropyLoss()

for epoch in range(10):
    for images, labels in train_loader:
        optimizer.zero_grad()

        outputs = model(images)
        loss = criterion(outputs, labels)

        loss.backward()
        optimizer.step()

        # Reset neuron states
        functional.reset_net(model)
```

### 3. ANN-to-SNN Conversion

Convert trained artificial neural networks to SNNs for efficient deployment.

**Conversion Process:**
1. Train standard ANN (e.g., CNN)
2. Replace ReLU activations with LIF neurons
3. Convert weights to rate-coded spike frequencies
4. Calibrate thresholds and time steps

**Implementation:**
```python
def convert_ann_to_snn(ann_model, num_timesteps=100):
    """
    Convert ANN to SNN
    ann_model: PyTorch model with ReLU activations
    num_timesteps: Number of simulation time steps
    """
    snn_model = copy.deepcopy(ann_model)

    # Replace all ReLU layers with LIF neurons
    for name, module in snn_model.named_modules():
        if isinstance(module, nn.ReLU):
            # Replace with LIF neuron
            parent_name = name.rsplit('.', 1)[0] if '.' in name else ''
            parent = snn_model.get_submodule(parent_name) if parent_name else snn_model
            attr_name = name.split('.')[-1]

            setattr(parent, attr_name, neuron.LIFNode(
                surrogate_function=surrogate.ATan(),
                v_threshold=1.0,
                detach_reset=True
            ))

    # Normalize weights based on activation statistics
    # (Run calibration dataset through original ANN)
    normalize_weights(snn_model, calibration_data)

    return snn_model

def normalize_weights(model, calibration_data):
    """
    Normalize weights to match ANN activations with SNN firing rates
    """
    activation_scales = {}

    # Collect max activations from each layer
    hooks = []
    def get_activation(name):
        def hook(model, input, output):
            activation_scales[name] = output.abs().max().item()
        return hook

    for name, module in model.named_modules():
        if isinstance(module, (nn.Conv2d, nn.Linear)):
            hooks.append(module.register_forward_hook(get_activation(name)))

    # Run calibration
    with torch.no_grad():
        for data in calibration_data:
            model(data)

    # Remove hooks
    for hook in hooks:
        hook.remove()

    # Scale weights
    for name, module in model.named_modules():
        if isinstance(module, (nn.Conv2d, nn.Linear)) and name in activation_scales:
            scale = activation_scales[name]
            if scale > 0:
                module.weight.data /= scale
                if module.bias is not None:
                    module.bias.data /= scale
```

## Network Optimization Techniques

### Sparse Connectivity

Reduce connections to improve efficiency:

```python
def create_sparse_network(n_inputs, n_outputs, sparsity=0.1):
    """Create sparse random network"""
    mask = np.random.rand(n_inputs, n_outputs) < sparsity
    weights = np.random.randn(n_inputs, n_outputs) * mask
    return weights
```

### Weight Quantization

Reduce precision for hardware efficiency:

```python
def quantize_weights(weights, bits=8):
    """Quantize weights to fixed-point representation"""
    w_min, w_max = weights.min(), weights.max()
    levels = 2 ** bits
    scale = (w_max - w_min) / (levels - 1)

    quantized = np.round((weights - w_min) / scale)
    dequantized = quantized * scale + w_min

    return dequantized, scale, w_min
```

## Summary

This chapter covered:
- **Feedforward SNNs:** Basic architecture for classification
- **Recurrent SNNs:** Temporal processing with reservoir computing
- **Convolutional SNNs:** Efficient visual processing
- **Learning Algorithms:** STDP, surrogate gradients, ANN conversion
- **Optimization:** Sparsity and quantization techniques

Next chapter: Hardware implementation with memristors and neuromorphic chips.

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
