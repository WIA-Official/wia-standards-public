# Chapter 7: Deployment Strategies and Integration

## From Research to Production

Deploying neuromorphic systems in production requires careful consideration of hardware constraints, software interfaces, and integration with existing infrastructure. This chapter provides practical guidance for taking SNNs from development to real-world deployment.

## Deployment Platforms

### Edge Devices

**Intel Loihi 2 on USB:**
```python
from lava.magma.core.run_configs import Loihi2HwCfg
from lava.magma.core.run_conditions import RunSteps

# Train network on GPU
model = train_snn_model(dataset)

# Convert to Lava processes
lava_model = convert_to_lava(model)

# Deploy to Loihi USB device
lava_model.run(condition=RunSteps(num_steps=100),
              run_cfg=Loihi2HwCfg())

# Inference
result = lava_model.get_output()
lava_model.stop()
```

**BrainChip Akida PCIe Card:**
```python
import akida

# Load trained model
model = akida.Model("model.h5")

# Connect to Akida device
devices = akida.devices()
print(f"Found {len(devices)} Akida device(s)")

device = devices[0]
device.soc.power_measurement_enabled = True

# Map model to hardware
model.map(device)

# Inference
prediction = model.predict(test_image)

# Get power consumption
power_mw = device.soc.power_meter.average
print(f"Average power: {power_mw:.2f} mW")
```

### Embedded Systems

**Microcontroller Deployment:**
```c
// Simplified SNN inference on ARM Cortex-M
typedef struct {
    int16_t v;           // Membrane potential
    uint16_t threshold;  // Firing threshold
    uint8_t refractory;  // Refractory counter
} LIFNeuron;

typedef struct {
    uint16_t source;
    uint16_t target;
    int8_t weight;       // 8-bit quantized weight
} Synapse;

// Network configuration
#define N_NEURONS 256
#define N_SYNAPSES 10000

LIFNeuron neurons[N_NEURONS];
Synapse synapses[N_SYNAPSES];

// Inference function
void snn_forward(uint8_t* input_spikes, uint8_t* output_spikes) {
    // Reset output
    memset(output_spikes, 0, N_NEURONS);

    // Process input spikes
    for (int i = 0; i < N_SYNAPSES; i++) {
        if (input_spikes[synapses[i].source]) {
            neurons[synapses[i].target].v += synapses[i].weight;
        }
    }

    // Update neurons
    for (int i = 0; i < N_NEURONS; i++) {
        if (neurons[i].refractory > 0) {
            neurons[i].refractory--;
            continue;
        }

        // Check threshold
        if (neurons[i].v >= neurons[i].threshold) {
            output_spikes[i] = 1;
            neurons[i].v = 0;  // Reset
            neurons[i].refractory = 5;  // 5ms refractory period
        }

        // Leak
        neurons[i].v = (neurons[i].v * 95) / 100;  // 5% decay per ms
    }
}

// Energy-efficient main loop
int main() {
    while (1) {
        // Wait for event (spike or sensor data)
        wait_for_event();

        // Process
        snn_forward(input_buffer, output_buffer);

        // Act on output
        if (output_buffer[TARGET_NEURON]) {
            trigger_action();
        }

        // Sleep until next event (save power)
        enter_low_power_mode();
    }
}
```

**Quantization for Embedded:**
```python
def quantize_snn_for_embedded(model, weight_bits=8, activation_bits=8):
    """
    Quantize SNN model for microcontroller deployment
    """
    import torch.quantization as quant

    # Quantization config
    model.qconfig = quant.get_default_qconfig('fbgemm')

    # Prepare for quantization
    quant.prepare(model, inplace=True)

    # Calibrate with representative dataset
    with torch.no_grad():
        for data, _ in calibration_loader:
            model(data)

    # Convert to quantized model
    quant.convert(model, inplace=True)

    return model

# Export quantized weights
quantized_model = quantize_snn_for_embedded(snn_model)

# Extract weights for C deployment
for name, param in quantized_model.named_parameters():
    if 'weight' in name:
        weights_int8 = param.int_repr().numpy()
        save_c_array(weights_int8, f"{name}.h")
```

### Cloud/Server Deployment

**TensorFlow Serving with SNNs:**
```python
import tensorflow as tf
from tensorflow_serving.apis import predict_pb2
from tensorflow_serving.apis import prediction_service_pb2_grpc

# Convert SNN to TensorFlow SavedModel
class SNNServingModel(tf.Module):
    def __init__(self, snn_model):
        super().__init__()
        self.snn_model = snn_model

    @tf.function(input_signature=[tf.TensorSpec(shape=[None, 784], dtype=tf.float32)])
    def __call__(self, x):
        # Encode input as spikes
        spike_train = rate_encode_tf(x, time_steps=50)

        # Run SNN
        output_spikes = self.snn_model(spike_train)

        # Decode to class probabilities
        probabilities = tf.reduce_mean(output_spikes, axis=0)

        return {
            'probabilities': probabilities,
            'predicted_class': tf.argmax(probabilities)
        }

# Save for serving
model = SNNServingModel(trained_snn)
tf.saved_model.save(model, '/models/snn_mnist/1')

# Deploy with TensorFlow Serving
# docker run -p 8501:8501 --name=tf_serving \
#   --mount type=bind,source=/models/snn_mnist,target=/models/snn_mnist \
#   -e MODEL_NAME=snn_mnist -t tensorflow/serving
```

**FastAPI REST Service:**
```python
from fastapi import FastAPI, File, UploadFile
import torch
import numpy as np
from PIL import Image

app = FastAPI(title="Neuromorphic Inference API")

# Load model at startup
model = None

@app.on_event("startup")
async def load_model():
    global model
    model = torch.load("snn_model.pth")
    model.eval()
    print("SNN model loaded successfully")

@app.post("/predict")
async def predict(file: UploadFile = File(...)):
    """
    Neuromorphic image classification endpoint
    """
    # Read image
    image = Image.open(file.file).convert('L')
    image = np.array(image.resize((28, 28))) / 255.0

    # Encode as spikes (rate coding)
    spike_input = (np.random.rand(100, 784) < image.flatten()).astype(float)

    # Inference
    with torch.no_grad():
        output = model(torch.FloatTensor(spike_input))

    # Decode
    spike_counts = output.sum(dim=0)
    predicted_class = int(torch.argmax(spike_counts))
    confidence = float(spike_counts[predicted_class] / spike_counts.sum())

    return {
        "predicted_class": predicted_class,
        "confidence": confidence,
        "spike_counts": spike_counts.tolist(),
        "encoding": "rate_100ms",
        "platform": "CPU-SNN"
    }

@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "model_loaded": model is not None,
        "device": "cpu"
    }

# Run: uvicorn app:app --host 0.0.0.0 --port 8000
```

## Integration Patterns

### Event-Based Camera Integration

**DVS Camera Pipeline:**
```python
import dv
import cv2

class DVSProcessor:
    def __init__(self, snn_model):
        self.model = snn_model
        self.event_buffer = []

    def process_stream(self, camera_name='DVS'):
        """Process DVS camera stream in real-time"""
        camera = dv.io.CameraCapture(camera_name)

        while True:
            # Get events
            events = camera.getNextEventBatch()
            if events is None:
                continue

            # Accumulate events into frame
            frame = self.events_to_frame(events, accumulation_time=10000)  # 10ms

            # SNN inference
            spikes = self.model.forward_spike_frame(frame)

            # Take action
            if self.detect_gesture(spikes):
                print("Gesture detected!")

    def events_to_frame(self, events, accumulation_time):
        """Convert event stream to spike frame"""
        # Create 2D histogram of events
        frame = np.zeros((events.height(), events.width()), dtype=np.float32)

        for event in events:
            if event.timestamp() < accumulation_time:
                frame[event.y(), event.x()] += event.polarity()

        return frame

    def detect_gesture(self, spikes):
        """Decode gesture from output spikes"""
        return torch.sum(spikes) > threshold
```

### Sensor Fusion

**Multi-Modal Integration:**
```python
class MultimodalNeuromorphicSystem:
    def __init__(self):
        self.vision_snn = load_vision_model()
        self.audio_snn = load_audio_model()
        self.fusion_snn = load_fusion_model()

    def process(self, dvs_events, audio_spikes):
        """
        Fuse visual and auditory inputs
        """
        # Process each modality
        vision_features = self.vision_snn(dvs_events)
        audio_features = self.audio_snn(audio_spikes)

        # Concatenate spike trains
        fused_input = torch.cat([vision_features, audio_features], dim=1)

        # Multi-modal decision
        output = self.fusion_snn(fused_input)

        return output

# Example: Audio-visual speech recognition
system = MultimodalNeuromorphicSystem()

while True:
    dvs_frame = capture_dvs_frame()
    audio_spikes = capture_audio_spikes()

    result = system.process(dvs_frame, audio_spikes)

    if result['confidence'] > 0.8:
        print(f"Detected: {result['class']}")
```

### Robot Control Integration

**ROS Integration:**
```python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class NeuromorphicRobotController:
    def __init__(self):
        rospy.init_node('neuromorphic_controller')

        # Load SNN model
        self.model = load_snn_model()

        # ROS subscribers/publishers
        self.camera_sub = rospy.Subscriber('/camera/events',
                                          Image,
                                          self.event_callback)

        self.action_pub = rospy.Publisher('/robot/action',
                                         Int32,
                                         queue_size=10)

        # State
        self.event_buffer = []

    def event_callback(self, msg):
        """Process incoming events"""
        # Convert ROS message to numpy
        events = np.frombuffer(msg.data, dtype=np.uint8)

        # Add to buffer
        self.event_buffer.append(events)

        # Process when buffer full
        if len(self.event_buffer) >= 10:  # 10ms buffer
            self.process_buffer()

    def process_buffer(self):
        """Run SNN inference on event buffer"""
        # Stack events
        event_tensor = torch.FloatTensor(self.event_buffer)

        # SNN inference (very low latency)
        with torch.no_grad():
            action_spikes = self.model(event_tensor)

        # Decode action
        action = torch.argmax(torch.sum(action_spikes, dim=0))

        # Publish action
        self.action_pub.publish(Int32(data=int(action)))

        # Clear buffer
        self.event_buffer = []

    def run(self):
        rospy.spin()

# Launch: rosrun neuromorphic_pkg controller.py
```

## Production Considerations

### Model Versioning

```python
from datetime import datetime

class SNNModelRegistry:
    def __init__(self, registry_path='./models'):
        self.registry_path = registry_path

    def save_model(self, model, metadata):
        """
        Save model with metadata for reproducibility
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        model_id = f"snn_v{metadata['version']}_{timestamp}"

        save_path = f"{self.registry_path}/{model_id}"
        os.makedirs(save_path, exist_ok=True)

        # Save model weights
        torch.save(model.state_dict(), f"{save_path}/weights.pth")

        # Save architecture
        with open(f"{save_path}/architecture.json", 'w') as f:
            json.dump(model.get_config(), f, indent=2)

        # Save metadata
        full_metadata = {
            **metadata,
            'timestamp': timestamp,
            'model_id': model_id,
            'pytorch_version': torch.__version__,
            'wia_standard': 'WIA-SEMI-007 v1.0'
        }

        with open(f"{save_path}/metadata.json", 'w') as f:
            json.dump(full_metadata, f, indent=2)

        print(f"Model saved: {model_id}")
        return model_id

# Usage
registry = SNNModelRegistry()
model_id = registry.save_model(
    model=trained_snn,
    metadata={
        'version': '1.0',
        'dataset': 'N-MNIST',
        'accuracy': 99.1,
        'energy_per_inference': 0.1,  # μJ
        'platform': 'Loihi 2',
        'author': 'team@example.com'
    }
)
```

### A/B Testing

```python
class ABTestController:
    def __init__(self, model_a, model_b, split_ratio=0.5):
        self.model_a = model_a
        self.model_b = model_b
        self.split_ratio = split_ratio
        self.metrics = {'a': [], 'b': []}

    def predict(self, input_data, user_id):
        """Route request to A or B based on user_id"""
        # Deterministic assignment based on user_id
        use_model_a = (hash(user_id) % 100) < (self.split_ratio * 100)

        start = time.time()

        if use_model_a:
            result = self.model_a(input_data)
            model_version = 'a'
        else:
            result = self.model_b(input_data)
            model_version = 'b'

        latency = time.time() - start

        # Log metrics
        self.metrics[model_version].append({
            'latency': latency,
            'timestamp': time.time(),
            'user_id': user_id
        })

        return result, model_version

    def get_stats(self):
        """Compare A vs B performance"""
        stats_a = {
            'mean_latency': np.mean([m['latency'] for m in self.metrics['a']]),
            'requests': len(self.metrics['a'])
        }
        stats_b = {
            'mean_latency': np.mean([m['latency'] for m in self.metrics['b']]),
            'requests': len(self.metrics['b'])
        }

        return {'model_a': stats_a, 'model_b': stats_b}
```

### Monitoring and Logging

```python
import logging
from prometheus_client import Counter, Histogram, Gauge

# Prometheus metrics
inference_counter = Counter('snn_inferences_total',
                           'Total SNN inferences')
inference_latency = Histogram('snn_inference_latency_seconds',
                             'SNN inference latency')
spike_count = Gauge('snn_spikes_current',
                   'Current spike count')
energy_consumption = Counter('snn_energy_microjoules',
                            'Total energy consumed')

class MonitoredSNN:
    def __init__(self, model):
        self.model = model
        self.logger = logging.getLogger('neuromorphic')

    def predict(self, input_data):
        """Instrumented prediction"""
        inference_counter.inc()

        with inference_latency.time():
            # Inference
            output = self.model(input_data)

            # Count spikes
            n_spikes = torch.sum(output > 0).item()
            spike_count.set(n_spikes)

            # Estimate energy
            energy = n_spikes * 5e-12 * 1e6  # Convert to μJ
            energy_consumption.inc(energy)

        # Log
        self.logger.info(f"Inference: {n_spikes} spikes, {energy:.3f} μJ")

        return output

# Expose metrics for Prometheus
from prometheus_client import start_http_server
start_http_server(8000)  # Metrics at http://localhost:8000/metrics
```

## Performance Optimization

### Batch Processing

```python
def batched_inference(model, inputs, batch_size=32):
    """
    Process inputs in batches for higher throughput
    """
    n_samples = len(inputs)
    outputs = []

    for i in range(0, n_samples, batch_size):
        batch = inputs[i:i+batch_size]

        # Batch encoding
        batch_spikes = [rate_encode(x) for x in batch]
        batch_tensor = torch.stack(batch_spikes)

        # Batch inference (parallelism on neuromorphic hardware)
        with torch.no_grad():
            batch_output = model(batch_tensor)

        outputs.extend(batch_output)

    return outputs
```

### Caching Strategies

```python
from functools import lru_cache

class CachedNeuromorphicInference:
    def __init__(self, model, cache_size=1000):
        self.model = model
        self.cache = {}
        self.max_cache_size = cache_size

    def predict(self, input_hash):
        """
        Cache predictions for repeated inputs
        (useful for video where frames are similar)
        """
        if input_hash in self.cache:
            return self.cache[input_hash]

        # Compute
        result = self.model(input_data)

        # Cache
        if len(self.cache) < self.max_cache_size:
            self.cache[input_hash] = result

        return result
```

## Summary

This chapter covered:
- **Edge Deployment:** Loihi USB, Akida PCIe, embedded microcontrollers
- **Cloud Deployment:** TensorFlow Serving, FastAPI, containerization
- **Integration:** DVS cameras, sensor fusion, ROS robotics
- **Production:** Model versioning, A/B testing, monitoring
- **Optimization:** Batch processing, caching

Next: Future directions and emerging technologies in neuromorphic computing.

© 2025 WIA-Official · Part of WIA-SEMI-007 Standard
