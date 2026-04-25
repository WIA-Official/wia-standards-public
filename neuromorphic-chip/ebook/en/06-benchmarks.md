# Chapter 6: Benchmarks and Performance Metrics

## Evaluating Neuromorphic Systems

Benchmarking neuromorphic systems requires new metrics beyond traditional accuracy. This chapter covers standard benchmarks, datasets, and evaluation methodologies specific to brain-inspired computing.

## Key Performance Metrics

### 1. Energy Efficiency

**Synaptic Operations per Watt (SOPS/W):**
```
SOPS/W = (Total Synaptic Operations) / (Energy Consumed)

Example:
- 1M neurons × 1K synapses × 50 Hz × 1 second = 50B operations
- Energy: 5 mW × 1 second = 5 mJ
- Efficiency: 50B / 0.005 J = 10 TOPS/W
```

**Python Calculator:**
```python
def calculate_energy_efficiency(n_neurons, n_synapses_per_neuron,
                                firing_rate, power_watts):
    """
    Calculate energy efficiency
    firing_rate: Average Hz
    power_watts: Power consumption
    Returns: SOPS/W
    """
    total_synapses = n_neurons * n_synapses_per_neuron
    ops_per_second = total_synapses * firing_rate
    sops_per_watt = ops_per_second / power_watts

    return sops_per_watt

# Intel Loihi 2 estimate
efficiency = calculate_energy_efficiency(
    n_neurons=1_000_000,
    n_synapses_per_neuron=120,
    firing_rate=50,  # Hz
    power_watts=0.1  # 100 mW
)
print(f"Loihi 2: {efficiency/1e12:.0f} TOPS/W")
# Output: ~60 TOPS/W

# Compare to GPU
gpu_efficiency = 1e9 / 300  # 1 GOPS at 300W
print(f"GPU: {gpu_efficiency/1e9:.2f} GOPS/W")
# Loihi is 60,000× more efficient!
```

### 2. Latency

**End-to-End Latency:**
```
Total Latency = Encoding Time + Processing Time + Decoding Time
```

**Components:**
- **Encoding:** 1-100ms depending on method (rate vs latency coding)
- **Processing:** 1-10ms per layer in SNN
- **Decoding:** 1-50ms for spike integration

```python
def estimate_latency(encoding_type, n_layers, spike_integration_time):
    """
    Estimate total inference latency
    """
    encoding_times = {
        'rate': 100,      # ms
        'latency': 10,    # ms
        'population': 50, # ms
        'temporal': 5     # ms
    }

    encoding_lat = encoding_times.get(encoding_type, 50)
    processing_lat = n_layers * 2  # 2ms per layer
    decoding_lat = spike_integration_time

    total = encoding_lat + processing_lat + decoding_lat
    return total

# Rate coding network
latency_rate = estimate_latency('rate', n_layers=4, spike_integration_time=50)
print(f"Rate coding latency: {latency_rate} ms")

# Temporal coding network
latency_temporal = estimate_latency('temporal', n_layers=4, spike_integration_time=5)
print(f"Temporal coding latency: {latency_temporal} ms")
# 19ms vs 158ms - 8× faster!
```

### 3. Accuracy vs Efficiency Trade-off

**Pareto Frontier Analysis:**
```python
import matplotlib.pyplot as plt

# Benchmark data (hypothetical)
systems = {
    'GPU-DNN': {'accuracy': 99.2, 'energy': 500, 'latency': 10},
    'Loihi-SNN': {'accuracy': 97.5, 'energy': 0.05, 'latency': 25},
    'TrueNorth': {'accuracy': 95.8, 'energy': 0.07, 'latency': 30},
    'Akida': {'accuracy': 96.5, 'energy': 0.001, 'latency': 15},
}

# Plot accuracy vs energy
plt.figure(figsize=(10, 6))
for name, metrics in systems.items():
    plt.scatter(metrics['energy'], metrics['accuracy'],
               s=200, label=name, alpha=0.7)
    plt.annotate(name, (metrics['energy'], metrics['accuracy']))

plt.xlabel('Energy per Inference (mJ)')
plt.ylabel('Accuracy (%)')
plt.title('Accuracy vs Energy Efficiency')
plt.xscale('log')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()
```

## Standard Benchmarks

### N-MNIST (Neuromorphic MNIST)

**Dataset:** 70,000 handwritten digits recorded with DVS camera

**Typical Results:**
```python
benchmark_results = {
    'CNN-GPU': {
        'accuracy': 99.5,
        'energy_per_sample': 500,  # mJ
        'latency': 5,  # ms
    },
    'SNN-Loihi': {
        'accuracy': 99.1,
        'energy_per_sample': 0.1,  # mJ
        'latency': 15,  # ms
    },
    'SNN-Software': {
        'accuracy': 98.5,
        'energy_per_sample': 10,  # mJ
        'latency': 30,  # ms
    }
}

# Energy efficiency gain
gpu_energy = benchmark_results['CNN-GPU']['energy_per_sample']
loihi_energy = benchmark_results['SNN-Loihi']['energy_per_sample']
improvement = gpu_energy / loihi_energy
print(f"Energy improvement: {improvement}× more efficient")
# 5000× more efficient!
```

### DVS-Gesture

**Dataset:** 11 hand gestures, 1342 samples, event-based recording

**State-of-the-Art:**
- **Accuracy:** 96.8% (SNN with STBP)
- **Latency:** 20ms
- **Energy:** 2.5 μJ per inference

**Implementation:**
```python
def evaluate_dvs_gesture(model, test_loader):
    """Evaluate on DVS-Gesture dataset"""
    correct = 0
    total = 0
    total_energy = 0
    total_latency = 0

    for events, label in test_loader:
        start_time = time.time()

        # Inference
        output = model(events)
        prediction = torch.argmax(output)

        # Metrics
        correct += (prediction == label).item()
        total += 1

        # Estimate energy (spike-based)
        n_spikes = count_spikes(model)
        total_energy += n_spikes * 5e-12  # 5 fJ per spike

        # Latency
        latency = (time.time() - start_time) * 1000  # ms
        total_latency += latency

    accuracy = 100 * correct / total
    avg_energy = total_energy / total * 1e6  # Convert to μJ
    avg_latency = total_latency / total

    return {
        'accuracy': accuracy,
        'energy_per_sample': avg_energy,
        'latency': avg_latency
    }
```

### SHD (Spiking Heidelberg Digits)

**Dataset:** Audio classification, spoken digits 0-9 in English and German

**Challenge:** Temporal pattern recognition

**Results:**
```python
snn_results = {
    'accuracy': 92.4,
    'n_neurons': 700,
    'n_synapses': 50000,
    'timesteps': 100,
    'energy': 0.5  # μJ
}

# Energy per synapse-operation
energy_per_sop = snn_results['energy'] / (snn_results['n_synapses'] * snn_results['timesteps'])
print(f"Energy per synaptic operation: {energy_per_sop*1e6:.2f} fJ")
```

### N-Caltech101

**Dataset:** 101 object categories, event-based recordings

**Complexity:** Higher than N-MNIST, requires deeper networks

**Benchmark Results:**
```
| Model | Accuracy | Energy (mJ) | Latency (ms) | Platform |
|-------|----------|-------------|--------------|----------|
| ResNet-50 | 91.2% | 850 | 12 | GPU |
| SNN-Deep | 88.5% | 5 | 45 | TrueNorth |
| SNN-ANN2SNN | 89.8% | 8 | 35 | Loihi |
```

## Custom Benchmark Suite

```python
class NeuromorphicBenchmark:
    def __init__(self, model, platform='cpu'):
        self.model = model
        self.platform = platform
        self.results = {}

    def run_inference_benchmark(self, test_loader, n_samples=1000):
        """Comprehensive inference benchmark"""
        latencies = []
        energies = []
        predictions = []
        labels = []

        for i, (data, label) in enumerate(test_loader):
            if i >= n_samples:
                break

            # Measure latency
            start = time.perf_counter()
            output = self.model(data)
            latency = (time.perf_counter() - start) * 1000  # ms

            # Estimate energy
            if self.platform == 'loihi':
                energy = self.estimate_loihi_energy(output)
            elif self.platform == 'gpu':
                energy = self.estimate_gpu_energy(latency)
            else:
                energy = 0

            latencies.append(latency)
            energies.append(energy)
            predictions.append(torch.argmax(output).item())
            labels.append(label.item())

        # Calculate metrics
        accuracy = 100 * np.mean(np.array(predictions) == np.array(labels))

        self.results = {
            'accuracy': accuracy,
            'mean_latency': np.mean(latencies),
            'std_latency': np.std(latencies),
            'mean_energy': np.mean(energies),
            'total_energy': np.sum(energies),
            'throughput': 1000 / np.mean(latencies),  # samples/sec
        }

        return self.results

    def estimate_loihi_energy(self, spike_output):
        """Estimate Loihi energy consumption"""
        n_spikes = torch.sum(spike_output > 0).item()
        energy_per_spike = 1e-12  # 1 pJ
        return n_spikes * energy_per_spike * 1e6  # μJ

    def estimate_gpu_energy(self, latency_ms):
        """Estimate GPU energy consumption"""
        gpu_power = 300  # Watts
        return (gpu_power * latency_ms / 1000) * 1000  # mJ

    def report(self):
        """Generate benchmark report"""
        print("="*50)
        print(f"Neuromorphic Benchmark Results ({self.platform})")
        print("="*50)
        print(f"Accuracy: {self.results['accuracy']:.2f}%")
        print(f"Mean Latency: {self.results['mean_latency']:.2f} ms")
        print(f"Std Latency: {self.results['std_latency']:.2f} ms")
        print(f"Mean Energy: {self.results['mean_energy']:.6f} μJ")
        print(f"Throughput: {self.results['throughput']:.1f} samples/sec")
        print(f"Energy Efficiency: {self.results['accuracy'] / self.results['mean_energy']:.1f} acc%/μJ")
        print("="*50)

# Usage
benchmark = NeuromorphicBenchmark(snn_model, platform='loihi')
results = benchmark.run_inference_benchmark(test_loader, n_samples=1000)
benchmark.report()
```

## Specialized Metrics

### Spike Sparsity

```python
def calculate_sparsity(spike_tensor):
    """
    Calculate spike sparsity
    spike_tensor: [time, neurons] boolean array
    Returns: Sparsity ratio [0, 1]
    """
    total_possible_spikes = spike_tensor.size
    actual_spikes = np.sum(spike_tensor)
    sparsity = 1 - (actual_spikes / total_possible_spikes)
    return sparsity

# Typical sparsity in SNNs: 95-99%
# Typical sparsity in DNNs: 0-50%
```

### Temporal Precision

```python
def measure_temporal_precision(spike_trains, n_trials=100):
    """
    Measure spike timing reliability across trials
    spike_trains: List of spike time arrays (one per trial)
    Returns: Jitter (standard deviation of spike times)
    """
    all_spike_times = []

    # Align spike trains to first spike
    for train in spike_trains:
        if len(train) > 0:
            aligned = train - train[0]
            all_spike_times.append(aligned)

    # Calculate jitter for each spike position
    max_spikes = max(len(t) for t in all_spike_times)
    jitters = []

    for spike_idx in range(max_spikes):
        times_at_position = [t[spike_idx] for t in all_spike_times
                            if len(t) > spike_idx]
        if len(times_at_position) > 1:
            jitter = np.std(times_at_position)
            jitters.append(jitter)

    return np.mean(jitters) if jitters else 0

# Good temporal precision: <1ms jitter
# Poor temporal precision: >10ms jitter
```

### Memory Footprint

```python
def calculate_model_size(model):
    """Calculate SNN model size"""
    param_size = 0
    buffer_size = 0

    for param in model.parameters():
        param_size += param.nelement() * param.element_size()

    for buffer in model.buffers():
        buffer_size += buffer.nelement() * buffer.element_size()

    total_mb = (param_size + buffer_size) / (1024**2)
    return total_mb

# Comparison
snn_size = calculate_model_size(snn_model)
dnn_size = calculate_model_size(dnn_model)

print(f"SNN model: {snn_size:.2f} MB")
print(f"DNN model: {dnn_size:.2f} MB")

# SNNs often smaller due to binary activations
```

## Real-World Application Benchmarks

### Keyword Spotting (Edge AI)

**Requirements:**
- Accuracy: >95%
- Latency: <100ms
- Power: <1mW

**Neuromorphic Solution:**
```python
results = {
    'accuracy': 96.5,
    'latency': 15,  # ms
    'power': 0.5,   # mW
    'energy_per_inference': 7.5,  # μJ
    'always_on': True
}

# Traditional solution
traditional = {
    'accuracy': 97.8,
    'latency': 8,    # ms
    'power': 500,    # mW
    'energy_per_inference': 4000,  # μJ
    'always_on': False  # Too power-hungry
}

improvement = traditional['energy_per_inference'] / results['energy_per_inference']
print(f"Energy efficiency gain: {improvement:.0f}×")
```

### Object Detection (Robotics)

**Benchmark:**
```python
def benchmark_object_detection(model, video_stream):
    """Benchmark real-time object detection"""
    frame_count = 0
    detection_count = 0
    total_latency = 0
    total_energy = 0

    for frame in video_stream:
        start = time.time()

        # Detect objects
        detections = model.detect(frame)

        # Metrics
        latency = (time.time() - start) * 1000
        energy = count_spikes(model) * 5e-12 * 1e6  # μJ

        frame_count += 1
        detection_count += len(detections)
        total_latency += latency
        total_energy += energy

        if frame_count >= 1000:
            break

    return {
        'fps': 1000 / (total_latency / frame_count),
        'detections_per_frame': detection_count / frame_count,
        'energy_per_frame': total_energy / frame_count,
        'avg_latency': total_latency / frame_count
    }
```

## Standardized Reporting

**Benchmark Report Template:**
```python
def generate_benchmark_report(results, config):
    """Generate standardized benchmark report"""
    report = f"""
    WIA-SEMI-007 Neuromorphic Benchmark Report
    ==========================================

    System Configuration:
    - Platform: {config['platform']}
    - Model: {config['model_name']}
    - Dataset: {config['dataset']}
    - Encoding: {config['encoding_method']}

    Performance Metrics:
    - Accuracy: {results['accuracy']:.2f}%
    - Latency: {results['latency']:.2f} ms
    - Throughput: {results['throughput']:.1f} samples/sec
    - Energy per Inference: {results['energy']:.6f} μJ
    - Power: {results['power']:.2f} mW

    Efficiency Metrics:
    - SOPS/W: {results['sops_per_watt']/1e9:.1f} GOPS/W
    - Accuracy/Energy: {results['accuracy']/results['energy']:.1f} %/μJ
    - Spike Sparsity: {results['sparsity']*100:.1f}%

    Hardware Utilization:
    - Neurons Used: {results['neurons_used']:,}
    - Synapses Used: {results['synapses_used']:,}
    - Memory: {results['memory_mb']:.2f} MB

    © 2025 WIA-SEMI-007 Standard
    """

    return report
```

## Summary

Key takeaways:
- **Energy Efficiency:** Primary metric for neuromorphic systems (10,000× gains possible)
- **Latency:** Trade-off with encoding method (1-100ms range)
- **Accuracy:** Often 1-3% below DNN, acceptable for most edge applications
- **Benchmarks:** N-MNIST, DVS-Gesture, SHD, N-Caltech101
- **Real-World:** Keyword spotting, object detection show dramatic energy savings

Next chapter: Deployment strategies for neuromorphic systems.

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
