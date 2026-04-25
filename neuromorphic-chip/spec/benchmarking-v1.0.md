# WIA-SEMI-007: Benchmarking Specification v1.0

## 1. Standard Benchmarks

### 1.1 N-MNIST
- Dataset: 70,000 samples
- Accuracy target: > 98%
- Energy target: < 1 mJ/sample
- Latency target: < 50ms

### 1.2 DVS-Gesture
- Dataset: 1342 samples, 11 classes
- Accuracy target: > 95%
- Energy target: < 5 μJ/sample
- Latency target: < 30ms

### 1.3 SHD (Spiking Heidelberg Digits)
- Dataset: Spoken digits 0-9
- Accuracy target: > 90%
- Energy target: < 1 μJ/sample
- Latency target: < 20ms

## 2. Performance Metrics

### 2.1 Energy Efficiency
```
SOPS/W = (neurons × synapses × firing_rate) / power_watts
```
- Minimum: 100 GOPS/W
- Target: 10 TOPS/W
- State-of-art: 100 TOPS/W

### 2.2 Throughput
```
Throughput = samples_processed / time_seconds
```
- Minimum: 100 samples/sec
- Target: 1000 samples/sec

### 2.3 Accuracy
```
Accuracy = correct_predictions / total_predictions
```
- Report: mean, std dev, min, max
- Use held-out test set

## 3. Reporting Format

### 3.1 Required Information
- Platform: hardware/software
- Model architecture: layers, neurons, synapses
- Encoding: method, parameters
- Training: dataset, epochs, learning rate

### 3.2 Results Template
```json
{
  "platform": "Intel Loihi 2",
  "benchmark": "N-MNIST",
  "accuracy": 98.5,
  "energy_per_sample_uj": 0.1,
  "latency_ms": 15,
  "throughput_samples_sec": 1000,
  "sops_per_watt": 50e12
}
```

---

© 2025 WIA-Official
