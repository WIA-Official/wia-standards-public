# WIA-AI-008 PHASE 4: Integration & Model Serving

**Standard ID**: WIA-AI-008-P4
**Version**: 1.0.0
**Status**: Active
**Last Updated**: 2025-01-15

## 弘益人間 - Benefit All Humanity

---

## 1. Overview

Phase 4 defines standards for model serving infrastructure, deployment patterns, monitoring, and production operations. This phase ensures models run reliably and efficiently in production environments.

## 2. Serving Infrastructure

### 2.1 Standard Serving Interface

```python
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

class ModelServer(ABC):
    """Base interface for model servers"""

    @abstractmethod
    def load_model(self, model_path: str, **config) -> None:
        """Load model into memory"""
        pass

    @abstractmethod
    def predict(self, inputs: Any) -> Any:
        """Run inference on inputs"""
        pass

    @abstractmethod
    def batch_predict(self, inputs: List[Any]) -> List[Any]:
        """Run batch inference"""
        pass

    @abstractmethod
    def health_check(self) -> Dict[str, Any]:
        """Check server health"""
        pass

    @abstractmethod
    def model_metadata(self) -> Dict[str, Any]:
        """Get model metadata"""
        pass
```

### 2.2 Health Check Specification

All servers MUST implement `/health` endpoint:

```json
GET /health

Response:
{
  "status": "healthy",
  "model": {
    "name": "resnet50",
    "version": "1.0.0",
    "loaded": true,
    "memory_mb": 245.7
  },
  "server": {
    "uptime_seconds": 3600,
    "requests_total": 15234,
    "requests_per_second": 4.23,
    "avg_latency_ms": 23.5
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

## 3. Deployment Patterns

### 3.1 Real-Time Serving

**Requirements:**
- Latency: < 100ms (p95)
- Availability: 99.9%+
- Auto-scaling support

**Example (TorchServe):**
```bash
torchserve --start \
  --model-store ./model-store \
  --models resnet50=resnet50.mar \
  --ncs \
  --ts-config config.properties
```

### 3.2 Batch Inference

**Requirements:**
- Throughput: 1000+ samples/second
- Efficient resource utilization
- Progress tracking

**Example (PySpark):**
```python
from pyspark.sql.functions import pandas_udf
import pandas as pd

@pandas_udf("array<float>")
def predict_udf(batch: pd.Series) -> pd.Series:
    # Load model once per executor
    model = load_model()

    # Batch inference
    results = model.predict(batch.values)
    return pd.Series(results.tolist())

df.withColumn("predictions", predict_udf("features")).write.parquet("output/")
```

### 3.3 Streaming Inference

**Requirements:**
- Latency: < 1 second
- Handle event streams
- State management

**Example (Kafka + ONNX Runtime):**
```python
from kafka import KafkaConsumer, KafkaProducer
import onnxruntime as ort

consumer = KafkaConsumer('input-topic')
producer = KafkaProducer('output-topic')
session = ort.InferenceSession("model.onnx")

for message in consumer:
    input_data = deserialize(message.value)
    output = session.run(None, {'input': input_data})
    producer.send('output-topic', serialize(output))
```

## 4. Monitoring & Observability

### 4.1 Required Metrics

Servers MUST expose Prometheus-compatible metrics:

```python
from prometheus_client import Counter, Histogram, Gauge

# Request metrics
REQUEST_COUNT = Counter(
    'model_requests_total',
    'Total inference requests',
    ['model_name', 'model_version', 'status']
)

REQUEST_LATENCY = Histogram(
    'model_request_latency_seconds',
    'Request latency in seconds',
    ['model_name', 'model_version']
)

# Model metrics
MODEL_LOADED = Gauge(
    'model_loaded',
    'Whether model is loaded',
    ['model_name', 'model_version']
)

ACTIVE_REQUESTS = Gauge(
    'model_active_requests',
    'Number of active requests',
    ['model_name']
)

# Resource metrics
GPU_MEMORY_MB = Gauge(
    'model_gpu_memory_mb',
    'GPU memory usage in MB',
    ['model_name', 'gpu_id']
)

CPU_USAGE_PERCENT = Gauge(
    'model_cpu_usage_percent',
    'CPU usage percentage',
    ['model_name']
)
```

### 4.2 Logging Standards

Structured logging in JSON format:

```json
{
  "timestamp": "2025-01-15T10:30:00Z",
  "level": "INFO",
  "event": "inference_request",
  "model": {
    "name": "resnet50",
    "version": "1.0.0"
  },
  "request": {
    "id": "req-123456",
    "input_shape": [1, 3, 224, 224],
    "batch_size": 1
  },
  "performance": {
    "latency_ms": 23.5,
    "preprocessing_ms": 2.1,
    "inference_ms": 18.3,
    "postprocessing_ms": 3.1
  },
  "user_id": "user-789",
  "trace_id": "trace-abc123"
}
```

### 4.3 Distributed Tracing

Support OpenTelemetry for distributed tracing:

```python
from opentelemetry import trace
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.exporter.jaeger.thrift import JaegerExporter

# Configure tracing
trace.set_tracer_provider(TracerProvider())
jaeger_exporter = JaegerExporter(
    agent_host_name="localhost",
    agent_port=6831,
)
trace.get_tracer_provider().add_span_processor(
    BatchSpanProcessor(jaeger_exporter)
)

tracer = trace.get_tracer(__name__)

def predict_with_tracing(input_data):
    with tracer.start_as_current_span("model_inference") as span:
        span.set_attribute("model.name", "resnet50")
        span.set_attribute("model.version", "1.0.0")

        with tracer.start_as_current_span("preprocessing"):
            preprocessed = preprocess(input_data)

        with tracer.start_as_current_span("inference"):
            output = model(preprocessed)

        with tracer.start_as_current_span("postprocessing"):
            result = postprocess(output)

        return result
```

## 5. Auto-Scaling

### 5.1 Horizontal Pod Autoscaler (Kubernetes)

```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: model-server-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: model-server
  minReplicas: 2
  maxReplicas: 20
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Pods
    pods:
      metric:
        name: model_requests_per_second
      target:
        type: AverageValue
        averageValue: "100"
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 50
        periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 0
      policies:
      - type: Percent
        value: 100
        periodSeconds: 30
      - type: Pods
        value: 4
        periodSeconds: 30
      selectPolicy: Max
```

## 6. A/B Testing & Canary Deployments

### 6.1 Traffic Splitting (Istio)

```yaml
apiVersion: networking.istio.io/v1alpha3
kind: VirtualService
metadata:
  name: model-service
spec:
  hosts:
  - model-service
  http:
  - match:
    - headers:
        canary:
          exact: "true"
    route:
    - destination:
        host: model-service
        subset: v2
  - route:
    - destination:
        host: model-service
        subset: v1
      weight: 90
    - destination:
        host: model-service
        subset: v2
      weight: 10
```

## 7. Model Performance Optimization

### 7.1 Batching Configuration

```python
class DynamicBatcher:
    """Dynamic request batching"""

    def __init__(
        self,
        max_batch_size: int = 32,
        max_delay_ms: int = 100,
        timeout_ms: int = 5000
    ):
        self.max_batch_size = max_batch_size
        self.max_delay_ms = max_delay_ms
        self.timeout_ms = timeout_ms
        self.pending_requests = []

    async def add_request(self, request):
        self.pending_requests.append(request)

        # Trigger batch if full
        if len(self.pending_requests) >= self.max_batch_size:
            return await self.process_batch()

        # Wait for more requests or timeout
        await asyncio.sleep(self.max_delay_ms / 1000)
        return await self.process_batch()

    async def process_batch(self):
        if not self.pending_requests:
            return

        batch = self.pending_requests[:self.max_batch_size]
        self.pending_requests = self.pending_requests[self.max_batch_size:]

        # Run batch inference
        results = await self.model.predict(batch)
        return results
```

### 7.2 Model Caching

```python
from functools import lru_cache
import hashlib

class ModelCache:
    """Cache inference results"""

    def __init__(self, max_size=1000):
        self.cache = {}
        self.max_size = max_size

    def get_cache_key(self, input_data):
        """Generate cache key from input"""
        return hashlib.sha256(
            str(input_data).encode()
        ).hexdigest()

    def get(self, input_data):
        key = self.get_cache_key(input_data)
        return self.cache.get(key)

    def set(self, input_data, result):
        key = self.get_cache_key(input_data)

        if len(self.cache) >= self.max_size:
            # Remove oldest entry
            self.cache.pop(next(iter(self.cache)))

        self.cache[key] = result
```

## 8. Disaster Recovery

### 8.1 Rollback Strategy

```python
class ModelRollback:
    """Automated rollback on errors"""

    def __init__(self, error_threshold=0.05, window_seconds=300):
        self.error_threshold = error_threshold
        self.window_seconds = window_seconds

    def should_rollback(self, error_rate: float) -> bool:
        """Check if rollback is needed"""
        return error_rate > self.error_threshold

    def rollback(self, previous_version: str):
        """Rollback to previous version"""
        # Update deployment
        # Switch traffic
        # Alert operators
        pass
```

### 8.2 Circuit Breaker

```python
from enum import Enum
import time

class CircuitState(Enum):
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Failing, reject requests
    HALF_OPEN = "half_open"  # Testing recovery

class CircuitBreaker:
    """Prevent cascade failures"""

    def __init__(
        self,
        failure_threshold=5,
        timeout_seconds=60,
        success_threshold=2
    ):
        self.failure_threshold = failure_threshold
        self.timeout_seconds = timeout_seconds
        self.success_threshold = success_threshold

        self.state = CircuitState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = None

    def call(self, func, *args, **kwargs):
        if self.state == CircuitState.OPEN:
            if time.time() - self.last_failure_time > self.timeout_seconds:
                self.state = CircuitState.HALF_OPEN
                self.success_count = 0
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = func(*args, **kwargs)

            if self.state == CircuitState.HALF_OPEN:
                self.success_count += 1
                if self.success_count >= self.success_threshold:
                    self.state = CircuitState.CLOSED
                    self.failure_count = 0

            return result

        except Exception as e:
            self.failure_count += 1
            self.last_failure_time = time.time()

            if self.failure_count >= self.failure_threshold:
                self.state = CircuitState.OPEN

            raise e
```

## 9. Security Best Practices

### 9.1 Input Validation

```python
def validate_input(input_data):
    """Validate input before inference"""

    # Check shape
    if input_data.shape != (1, 3, 224, 224):
        raise ValueError("Invalid input shape")

    # Check dtype
    if input_data.dtype != np.float32:
        raise ValueError("Invalid dtype")

    # Check range
    if np.min(input_data) < 0 or np.max(input_data) > 1:
        raise ValueError("Input values out of range")

    # Check for NaN/Inf
    if np.isnan(input_data).any() or np.isinf(input_data).any():
        raise ValueError("Input contains NaN or Inf")

    return True
```

### 9.2 Rate Limiting

```python
from collections import defaultdict
import time

class RateLimiter:
    """Token bucket rate limiter"""

    def __init__(self, rate=100, capacity=100):
        self.rate = rate  # tokens per second
        self.capacity = capacity
        self.tokens = defaultdict(lambda: capacity)
        self.last_update = defaultdict(lambda: time.time())

    def allow(self, user_id: str) -> bool:
        now = time.time()
        elapsed = now - self.last_update[user_id]

        # Add tokens based on elapsed time
        self.tokens[user_id] = min(
            self.capacity,
            self.tokens[user_id] + elapsed * self.rate
        )
        self.last_update[user_id] = now

        # Check if request allowed
        if self.tokens[user_id] >= 1:
            self.tokens[user_id] -= 1
            return True

        return False
```

## 10. Compliance Checklist

Phase 4 compliant deployments MUST:
- [ ] Implement health check endpoint
- [ ] Expose Prometheus metrics
- [ ] Provide structured JSON logging
- [ ] Support auto-scaling (cloud deployments)
- [ ] Implement input validation
- [ ] Include rate limiting
- [ ] Support A/B testing or canary deployments
- [ ] Have rollback strategy
- [ ] Implement circuit breaker
- [ ] Provide distributed tracing

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
Licensed under Apache 2.0
