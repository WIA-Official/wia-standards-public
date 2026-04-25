# Chapter 4: Spike Encoding and Decoding Protocols

## Translating Information into Spike Trains

The bridge between traditional data and neuromorphic computing is **spike encoding** - the process of converting continuous values, images, audio, or sensor data into temporal patterns of spikes. This chapter explores various encoding schemes and their trade-offs in accuracy, latency, and energy efficiency.

## Fundamental Encoding Schemes

### 1. Rate Coding

**Principle:** Information encoded in firing frequency

**Formula:**
```
firing_rate = k × input_value
spike_probability = input_value × dt / ISI
```

**Implementation:**
```python
def rate_encode(value, duration=100, max_rate=200, dt=1.0):
    """
    Encode value as Poisson spike train
    value: Input in range [0, 1]
    duration: Encoding window (ms)
    max_rate: Maximum firing rate (Hz)
    dt: Time step (ms)
    Returns: Array of spike times
    """
    rate = value * max_rate  # Hz
    spike_times = []

    t = 0
    while t < duration:
        # Poisson process: exponential inter-spike intervals
        if rate > 0:
            isi = -np.log(np.random.rand()) / (rate / 1000)  # Convert Hz to ms⁻¹
            t += isi
            if t < duration:
                spike_times.append(t)
        else:
            break

    return np.array(spike_times)

# Example: Encode MNIST pixel (value=0.75) for 100ms
spike_times = rate_encode(0.75, duration=100, max_rate=200)
print(f"Generated {len(spike_times)} spikes")
print(f"Effective rate: {len(spike_times) / 0.1:.1f} Hz")
```

**Advantages:**
- Simple to implement
- Noise-robust (averaging over multiple spikes)
- Biologically plausible

**Disadvantages:**
- High latency (need integration window)
- Energy inefficient (many spikes)
- Low information density

**Use Cases:**
- Traditional vision sensors
- Audio processing with averaging
- Applications where latency is not critical

### 2. Temporal (Latency) Coding

**Principle:** Information encoded in precise spike timing

**Formula:**
```
spike_time = (1 - value) × time_window
```
Higher values spike earlier, lower values spike later.

**Implementation:**
```python
def latency_encode(values, time_window=20):
    """
    Encode values as spike latencies
    values: Array of inputs in [0, 1]
    time_window: Encoding window (ms)
    Returns: Array of spike times (or inf if no spike)
    """
    spike_times = (1 - values) * time_window
    # Values close to 0 spike late (or not at all)
    spike_times[values < 0.1] = np.inf  # Threshold
    return spike_times

# Example: Encode image patch
patch = np.array([0.9, 0.5, 0.2, 0.0])
times = latency_encode(patch, time_window=10)
print(f"Spike times: {times} ms")
# Output: [1.0, 5.0, 8.0, inf] ms
```

**Time-to-First-Spike (TTFS):**
```python
class TTFSEncoder:
    def __init__(self, threshold=0.5, max_time=50):
        self.threshold = threshold
        self.max_time = max_time
        self.integrate_and_fire = LIFNeuron()

    def encode(self, input_value):
        """Returns time of first spike or None"""
        current = input_value * 10  # Scale to appropriate range

        for t in np.arange(0, self.max_time, 0.1):
            if self.integrate_and_fire.step(current, dt=0.1):
                return t

        return None  # No spike within time window
```

**Advantages:**
- Very low latency (1-10ms)
- High information capacity (log₂(T/dt) bits per neuron)
- Energy efficient (single spike per neuron)

**Disadvantages:**
- Sensitive to noise
- Requires precise timing
- Difficult to implement with rate-based neurons

**Use Cases:**
- Event-based vision (DVS cameras)
- Real-time robotics
- Ultra-low latency applications

### 3. Population Coding

**Principle:** Distributed representation across multiple neurons

**Gaussian Tuning Curves:**
```python
class PopulationEncoder:
    def __init__(self, n_neurons, value_range=(0, 1), sigma=0.1):
        """
        n_neurons: Size of population
        value_range: (min, max) values to encode
        sigma: Tuning curve width
        """
        self.n_neurons = n_neurons
        self.sigma = sigma

        # Preferred values evenly distributed
        self.preferred_values = np.linspace(value_range[0],
                                           value_range[1],
                                           n_neurons)

    def encode(self, value, max_rate=100):
        """
        Encode value as population firing rates
        Returns: Array of firing rates
        """
        # Gaussian tuning curve
        responses = max_rate * np.exp(-((value - self.preferred_values)**2) /
                                      (2 * self.sigma**2))
        return responses

    def decode(self, firing_rates):
        """
        Decode value from population activity (population vector)
        firing_rates: Array of firing rates
        Returns: Decoded value
        """
        # Weighted average
        return np.sum(firing_rates * self.preferred_values) / np.sum(firing_rates)

# Example: Encode/decode angle for motor control
encoder = PopulationEncoder(n_neurons=20, value_range=(0, 360), sigma=20)

angle = 180  # degrees
rates = encoder.encode(angle, max_rate=100)

# Add noise
noisy_rates = rates + np.random.randn(20) * 5
decoded_angle = encoder.decode(noisy_rates)

print(f"Original: {angle}°, Decoded: {decoded_angle:.1f}°")
# Very robust to noise!
```

**Sparse Distributed Representation:**
```python
def sparse_population_encode(value, n_neurons=1000, sparsity=0.05):
    """
    Encode using sparse random projections
    Only ~5% of neurons active
    """
    # Random projection matrix
    W = np.random.randn(1, n_neurons) * 0.1

    # Apply non-linearity and sparsification
    activity = np.maximum(0, W @ value)
    threshold = np.percentile(activity, 100 * (1 - sparsity))
    sparse_activity = (activity > threshold).astype(float)

    return sparse_activity
```

**Advantages:**
- Extremely noise-robust
- Graceful degradation
- High capacity with large populations

**Disadvantages:**
- Requires many neurons
- Higher energy consumption
- Decoding can be complex

**Use Cases:**
- Sensory processing (grid cells, place cells)
- Motor control
- Cognitive tasks requiring robustness

### 4. Phase Coding

**Principle:** Spike timing relative to oscillatory reference

Used extensively in hippocampus (theta phase precession):

```python
class PhaseEncoder:
    def __init__(self, oscillation_freq=8, phase_range=(0, 2*np.pi)):
        """
        oscillation_freq: Reference oscillation (Hz)
        phase_range: (min_phase, max_phase)
        """
        self.freq = oscillation_freq
        self.period = 1000 / oscillation_freq  # ms
        self.phase_range = phase_range

    def encode(self, value, t):
        """
        Encode value as phase within oscillation cycle
        value: Input in [0, 1]
        t: Current time (ms)
        Returns: Spike time
        """
        # Map value to phase
        phase = self.phase_range[0] + value * (self.phase_range[1] - self.phase_range[0])

        # Current oscillation cycle
        cycle = int(t / self.period)
        cycle_start = cycle * self.period

        # Spike time within cycle
        spike_time = cycle_start + (phase / (2 * np.pi)) * self.period

        return spike_time if spike_time > t else spike_time + self.period

# Example: Theta phase coding (8 Hz)
encoder = PhaseEncoder(oscillation_freq=8)

# Different values encoded at different phases
for value in [0.0, 0.25, 0.5, 0.75, 1.0]:
    spike_t = encoder.encode(value, t=0)
    phase_deg = (spike_t / encoder.period) * 360
    print(f"Value {value:.2f} → Phase {phase_deg:.1f}°")
```

**Advantages:**
- High temporal precision
- Natural for oscillatory systems
- Can multiplex multiple values in one cycle

**Disadvantages:**
- Requires phase reference
- Complex decoding
- Sensitive to timing jitter

**Use Cases:**
- Spatial navigation (hippocampus)
- Sensory binding
- Attention modulation

## Advanced Encoding Techniques

### Event-Based Vision Encoding (DVS)

Dynamic Vision Sensors output spikes on brightness changes:

```python
class DVSEncoder:
    def __init__(self, threshold=0.15):
        """
        threshold: Brightness change threshold
        """
        self.threshold = threshold
        self.last_brightness = None

    def encode_frame(self, image, timestamp):
        """
        Generate events from image
        Returns: List of (x, y, timestamp, polarity) events
        """
        events = []

        if self.last_brightness is None:
            self.last_brightness = image
            return events

        # Calculate log brightness change
        log_change = np.log(image + 1e-6) - np.log(self.last_brightness + 1e-6)

        # Positive events (brightness increase)
        pos_events = log_change > self.threshold
        for y, x in zip(*np.where(pos_events)):
            events.append({'x': x, 'y': y, 't': timestamp, 'p': 1})

        # Negative events (brightness decrease)
        neg_events = log_change < -self.threshold
        for y, x in zip(*np.where(neg_events)):
            events.append({'x': x, 'y': y, 't': timestamp, 'p': -1})

        self.last_brightness = image
        return events

# Process video stream
dvs = DVSEncoder(threshold=0.15)
events_stream = []

for t, frame in enumerate(video_frames):
    events = dvs.encode_frame(frame, timestamp=t)
    events_stream.extend(events)

print(f"Generated {len(events_stream)} events from {len(video_frames)} frames")
# Typical compression: 100-1000× fewer events than pixels
```

### Auditory Spike Encoding

**Cochlea-Inspired Encoding:**
```python
class CochleaEncoder:
    def __init__(self, n_channels=64, sample_rate=16000):
        """
        n_channels: Number of frequency channels
        sample_rate: Audio sample rate (Hz)
        """
        self.n_channels = n_channels
        self.sample_rate = sample_rate

        # Create gammatone filterbank
        self.filters = self._create_filterbank()

    def _create_filterbank(self):
        """Create gammatone filters mimicking cochlea"""
        f_min, f_max = 50, 8000  # Hz
        center_freqs = np.logspace(np.log10(f_min), np.log10(f_max), self.n_channels)

        # ... (gammatone filter implementation)
        return filters

    def encode(self, audio_signal):
        """
        Encode audio as spike trains per frequency channel
        audio_signal: 1D numpy array
        Returns: List of spike trains for each channel
        """
        spike_trains = []

        for channel_filter in self.filters:
            # Filter audio
            filtered = signal.lfilter(channel_filter, 1, audio_signal)

            # Half-wave rectification
            rectified = np.maximum(0, filtered)

            # Leaky integrate-and-fire
            spike_train = self._lif_encode(rectified)
            spike_trains.append(spike_train)

        return spike_trains

    def _lif_encode(self, signal, threshold=1.0):
        """Encode continuous signal with LIF neuron"""
        v = 0
        tau = 0.01  # seconds
        spike_times = []

        for t, s in enumerate(signal):
            v += s
            v *= np.exp(-1 / (tau * self.sample_rate))

            if v >= threshold:
                spike_times.append(t / self.sample_rate)
                v = 0

        return np.array(spike_times)

# Example: Encode speech
encoder = CochleaEncoder(n_channels=64, sample_rate=16000)
spike_trains = encoder.encode(speech_audio)

# Each spike train represents one frequency band
for i, train in enumerate(spike_trains[:5]):
    print(f"Channel {i}: {len(train)} spikes")
```

## Decoding Strategies

### 1. Spike Count Decoding

Simply count spikes in time window:

```python
def spike_count_decode(spike_trains, time_window=100):
    """
    Decode by counting spikes
    spike_trains: List of spike time arrays for each output neuron
    time_window: Integration window (ms)
    Returns: Predicted class (neuron with most spikes)
    """
    counts = [len(train) for train in spike_trains]
    return np.argmax(counts)
```

### 2. Population Vector Decoding

Weight spikes by preferred direction:

```python
def population_vector_decode(spike_counts, preferred_values):
    """
    Decode using population vector method
    spike_counts: Firing rates/counts for each neuron
    preferred_values: What each neuron "prefers" to encode
    """
    vector_sum = np.sum(spike_counts[:, np.newaxis] * preferred_values, axis=0)
    magnitude = np.linalg.norm(vector_sum)
    return vector_sum / magnitude if magnitude > 0 else 0
```

### 3. Bayesian Decoding

Maximum likelihood estimation:

```python
def bayesian_decode(spike_counts, tuning_curves, stimuli):
    """
    Bayesian decoder using known tuning curves
    spike_counts: Observed spikes
    tuning_curves: f_i(s) for each neuron i and stimulus s
    stimuli: Possible stimulus values
    Returns: Most likely stimulus
    """
    likelihoods = []

    for stimulus in stimuli:
        # Poisson likelihood: P(spike_counts | stimulus)
        expected_rates = tuning_curves(stimulus)
        log_likelihood = np.sum(spike_counts * np.log(expected_rates + 1e-9) -
                                expected_rates)
        likelihoods.append(log_likelihood)

    return stimuli[np.argmax(likelihoods)]
```

### 4. Temporal Pattern Decoding

Decode based on precise spike timing patterns:

```python
class TemplateMatching:
    def __init__(self, templates, time_precision=1.0):
        """
        templates: Dictionary of {class: spike_pattern}
        time_precision: Temporal tolerance (ms)
        """
        self.templates = templates
        self.precision = time_precision

    def decode(self, observed_spikes):
        """
        Find best matching template
        observed_spikes: Spike times to classify
        Returns: Best matching class
        """
        best_match = None
        best_score = -np.inf

        for class_name, template in self.templates.items():
            score = self._match_score(observed_spikes, template)
            if score > best_score:
                best_score = score
                best_match = class_name

        return best_match

    def _match_score(self, spikes1, spikes2):
        """
        Compute similarity between spike trains
        Uses Victor-Purpura distance
        """
        # Simplified: count coincident spikes within precision window
        coincidences = 0

        for s1 in spikes1:
            if np.any(np.abs(spikes2 - s1) < self.precision):
                coincidences += 1

        # Normalize
        return coincidences / max(len(spikes1), len(spikes2))
```

## Encoding Comparison

| Encoding | Latency | Energy | Accuracy | Noise Robustness | Use Case |
|----------|---------|--------|----------|------------------|----------|
| Rate | High (50-200ms) | High | Good | Excellent | Non-real-time |
| Latency | Very Low (1-10ms) | Very Low | Good | Poor | Real-time |
| Population | Medium (20-50ms) | Medium | Excellent | Excellent | Motor control |
| Phase | Low (10-20ms) | Low | Good | Medium | Navigation |

## Practical Guidelines

**Choose Rate Coding when:**
- You have abundant time for integration
- Noise is significant
- Converting from traditional ML models
- Hardware doesn't support precise timing

**Choose Latency Coding when:**
- Latency is critical (<10ms)
- Energy budget is tight
- Using event-based sensors (DVS)
- Implementing real-time control

**Choose Population Coding when:**
- Robustness is paramount
- You have many neurons available
- Implementing cognitive functions
- Need graceful degradation

**Choose Phase Coding when:**
- Working with oscillatory systems
- Multiplexing information
- Implementing attention mechanisms
- Modeling biological systems

## Summary

This chapter covered:
- **Rate coding:** Frequency-based, robust but slow
- **Temporal coding:** Fast, efficient, but noise-sensitive
- **Population coding:** Distributed, robust, many neurons
- **Phase coding:** Oscillation-relative, multiplexing
- **DVS/Cochlea:** Event-based sensor encoding
- **Decoding:** Count, population vector, Bayesian, template matching

Choosing the right encoding is crucial for neuromorphic system performance!

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
