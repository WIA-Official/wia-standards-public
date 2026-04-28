# Chapter 8: Production Deployment

## Introduction

Deploying speech recognition systems in production requires careful consideration of scalability, reliability, security, and cost. This chapter explores best practices for production ASR deployment that serves 弘益人間 (Benefit All Humanity) through robust, accessible systems.

## System Architecture

### Microservices Architecture

```python
from flask import Flask, request, jsonify
import torch
import queue
import threading

class ASRMicroservice:
    """ASR as a microservice"""

    def __init__(self, model_path, num_workers=4):
        self.app = Flask(__name__)
        self.model = self.load_model(model_path)
        self.request_queue = queue.Queue()
        self.num_workers = num_workers

        # Start workers
        self.workers = []
        for _ in range(num_workers):
            worker = threading.Thread(target=self._worker)
            worker.daemon = True
            worker.start()
            self.workers.append(worker)

        # Register endpoints
        self._register_routes()

    def load_model(self, model_path):
        """Load ASR model"""
        model = torch.load(model_path)
        model.eval()
        return model

    def _register_routes(self):
        """Register API endpoints"""

        @self.app.route('/health', methods=['GET'])
        def health():
            return jsonify({'status': 'healthy', 'workers': self.num_workers})

        @self.app.route('/transcribe', methods=['POST'])
        def transcribe():
            # Get audio data
            audio_data = request.files['audio'].read()

            # Queue for processing
            result_queue = queue.Queue()
            self.request_queue.put((audio_data, result_queue))

            # Wait for result
            result = result_queue.get(timeout=30)

            return jsonify(result)

        @self.app.route('/transcribe/stream', methods=['POST'])
        def transcribe_stream():
            # Streaming endpoint
            return self._handle_streaming()

    def _worker(self):
        """Worker thread for processing requests"""
        while True:
            try:
                audio_data, result_queue = self.request_queue.get(timeout=1)

                # Process
                result = self._process_audio(audio_data)

                # Return result
                result_queue.put(result)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Worker error: {e}")

    def _process_audio(self, audio_data):
        """Process audio data"""
        # Load audio
        audio = self._load_audio_from_bytes(audio_data)

        # Extract features
        features = self._extract_features(audio)

        # Run model
        with torch.no_grad():
            logits = self.model(features)

        # Decode
        text = self._decode(logits)

        return {
            'text': text,
            'confidence': self._calculate_confidence(logits),
            'duration': len(audio) / 16000
        }

    def run(self, host='0.0.0.0', port=5000):
        """Run microservice"""
        self.app.run(host=host, port=port, threaded=True)
```

### Containerization

```dockerfile
# Dockerfile for ASR service
FROM python:3.9-slim

# Install dependencies
RUN apt-get update && apt-get install -y \
    libsndfile1 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirements
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Download model
RUN python download_model.py

# Expose port
EXPOSE 5000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:5000/health || exit 1

# Run service
CMD ["python", "app.py"]
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  asr-service:
    build: .
    ports:
      - "5000:5000"
    environment:
      - MODEL_PATH=/models/asr_model.pt
      - NUM_WORKERS=4
      - LOG_LEVEL=INFO
    volumes:
      - ./models:/models
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 8G
        reservations:
          cpus: '2'
          memory: 4G
    restart: unless-stopped

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - asr-service
    restart: unless-stopped
```

### Kubernetes Deployment

```yaml
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: asr-service
  labels:
    app: asr
spec:
  replicas: 3
  selector:
    matchLabels:
      app: asr
  template:
    metadata:
      labels:
        app: asr
    spec:
      containers:
      - name: asr
        image: your-registry/asr-service:latest
        ports:
        - containerPort: 5000
        env:
        - name: MODEL_PATH
          value: "/models/asr_model.pt"
        - name: NUM_WORKERS
          value: "4"
        resources:
          requests:
            memory: "4Gi"
            cpu: "2"
          limits:
            memory: "8Gi"
            cpu: "4"
        livenessProbe:
          httpGet:
            path: /health
            port: 5000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 5000
          initialDelaySeconds: 10
          periodSeconds: 5
        volumeMounts:
        - name: models
          mountPath: /models
      volumes:
      - name: models
        persistentVolumeClaim:
          claimName: models-pvc

---
apiVersion: v1
kind: Service
metadata:
  name: asr-service
spec:
  type: LoadBalancer
  selector:
    app: asr
  ports:
  - protocol: TCP
    port: 80
    targetPort: 5000

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: asr-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: asr-service
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

## Model Optimization

### Quantization

```python
import torch
import torch.quantization as quantization

def quantize_model(model, calibration_data):
    """Quantize model to INT8"""

    # 1. Fuse operations
    model = quantization.fuse_modules(model, [['conv', 'bn', 'relu']])

    # 2. Prepare for quantization
    model.qconfig = quantization.get_default_qconfig('fbgemm')
    quantization.prepare(model, inplace=True)

    # 3. Calibrate
    model.eval()
    with torch.no_grad():
        for data in calibration_data:
            model(data)

    # 4. Convert to quantized model
    quantization.convert(model, inplace=True)

    return model

# Dynamic quantization (no calibration needed)
def dynamic_quantize(model):
    """Dynamic quantization"""
    return quantization.quantize_dynamic(
        model,
        {torch.nn.Linear, torch.nn.LSTM},
        dtype=torch.qint8
    )

# Compare sizes
original_size = os.path.getsize('model.pt') / (1024 * 1024)  # MB
torch.save(quantized_model.state_dict(), 'model_quantized.pt')
quantized_size = os.path.getsize('model_quantized.pt') / (1024 * 1024)

print(f"Size reduction: {original_size:.2f}MB -> {quantized_size:.2f}MB")
print(f"Compression ratio: {original_size / quantized_size:.2f}x")
```

### Model Distillation

```python
class ModelDistillation:
    """Knowledge distillation for model compression"""

    def __init__(self, teacher_model, student_model, temperature=3.0):
        self.teacher = teacher_model
        self.student = student_model
        self.temperature = temperature

        # Freeze teacher
        for param in self.teacher.parameters():
            param.requires_grad = False

        self.teacher.eval()

    def distillation_loss(self, student_logits, teacher_logits, targets, alpha=0.5):
        """
        Compute distillation loss

        alpha: Weight between hard targets and soft targets
        """
        # Soft targets (from teacher)
        teacher_probs = torch.nn.functional.softmax(
            teacher_logits / self.temperature, dim=-1
        )
        student_log_probs = torch.nn.functional.log_softmax(
            student_logits / self.temperature, dim=-1
        )

        distillation_loss = torch.nn.functional.kl_div(
            student_log_probs,
            teacher_probs,
            reduction='batchmean'
        ) * (self.temperature ** 2)

        # Hard targets (ground truth)
        student_log_probs_hard = torch.nn.functional.log_softmax(
            student_logits, dim=-1
        )
        hard_loss = torch.nn.functional.nll_loss(
            student_log_probs_hard.view(-1, student_log_probs_hard.size(-1)),
            targets.view(-1)
        )

        # Combined loss
        total_loss = alpha * hard_loss + (1 - alpha) * distillation_loss

        return total_loss

    def train_step(self, features, targets, optimizer):
        """Single training step"""
        self.student.train()

        # Teacher predictions
        with torch.no_grad():
            teacher_logits = self.teacher(features)

        # Student predictions
        student_logits = self.student(features)

        # Compute loss
        loss = self.distillation_loss(
            student_logits, teacher_logits, targets
        )

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        return loss.item()
```

### ONNX Export

```python
def export_to_onnx(model, output_path, input_shape=(1, 100, 80)):
    """Export PyTorch model to ONNX"""

    # Create dummy input
    dummy_input = torch.randn(*input_shape)

    # Export
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=['audio_features'],
        output_names=['transcription'],
        dynamic_axes={
            'audio_features': {0: 'batch_size', 1: 'time'},
            'transcription': {0: 'batch_size', 1: 'length'}
        }
    )

    print(f"Model exported to {output_path}")

# Use ONNX Runtime for inference
import onnxruntime as ort

def inference_with_onnx(onnx_path, features):
    """Run inference with ONNX model"""
    session = ort.InferenceSession(onnx_path)

    # Run inference
    outputs = session.run(
        None,
        {'audio_features': features.numpy()}
    )

    return outputs[0]
```

## Monitoring and Logging

### Metrics Collection

```python
from prometheus_client import Counter, Histogram, Gauge, start_http_server
import time

class ASRMetrics:
    """Prometheus metrics for ASR service"""

    def __init__(self):
        # Request metrics
        self.requests_total = Counter(
            'asr_requests_total',
            'Total ASR requests',
            ['endpoint', 'status']
        )

        self.request_duration = Histogram(
            'asr_request_duration_seconds',
            'Request duration in seconds',
            ['endpoint']
        )

        self.audio_duration = Histogram(
            'asr_audio_duration_seconds',
            'Audio duration in seconds'
        )

        # Model metrics
        self.inference_time = Histogram(
            'asr_inference_seconds',
            'Model inference time'
        )

        self.wer = Histogram(
            'asr_word_error_rate',
            'Word error rate'
        )

        # System metrics
        self.active_requests = Gauge(
            'asr_active_requests',
            'Number of active requests'
        )

        self.queue_size = Gauge(
            'asr_queue_size',
            'Request queue size'
        )

    def track_request(self, endpoint):
        """Context manager for tracking requests"""
        return RequestTracker(self, endpoint)

class RequestTracker:
    """Track individual request metrics"""

    def __init__(self, metrics, endpoint):
        self.metrics = metrics
        self.endpoint = endpoint
        self.start_time = None

    def __enter__(self):
        self.start_time = time.time()
        self.metrics.active_requests.inc()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        duration = time.time() - self.start_time

        # Record metrics
        status = 'success' if exc_type is None else 'error'
        self.metrics.requests_total.labels(
            endpoint=self.endpoint,
            status=status
        ).inc()

        self.metrics.request_duration.labels(
            endpoint=self.endpoint
        ).observe(duration)

        self.metrics.active_requests.dec()

# Usage
metrics = ASRMetrics()
start_http_server(8000)  # Expose metrics on port 8000

def transcribe_with_metrics(audio):
    with metrics.track_request('transcribe') as tracker:
        # Process audio
        result = process_audio(audio)

        # Record audio duration
        metrics.audio_duration.observe(audio.duration)

        return result
```

### Logging

```python
import logging
import json
from datetime import datetime

class StructuredLogger:
    """Structured JSON logging for ASR service"""

    def __init__(self, service_name='asr-service'):
        self.service_name = service_name
        self.logger = logging.getLogger(service_name)
        self.logger.setLevel(logging.INFO)

        # JSON formatter
        handler = logging.StreamHandler()
        handler.setFormatter(self.JSONFormatter())
        self.logger.addHandler(handler)

    class JSONFormatter(logging.Formatter):
        """Format logs as JSON"""

        def format(self, record):
            log_data = {
                'timestamp': datetime.utcnow().isoformat(),
                'level': record.levelname,
                'message': record.getMessage(),
                'module': record.module,
                'function': record.funcName,
                'line': record.lineno
            }

            # Add extra fields
            if hasattr(record, 'request_id'):
                log_data['request_id'] = record.request_id

            if hasattr(record, 'duration'):
                log_data['duration_ms'] = record.duration

            if hasattr(record, 'audio_duration'):
                log_data['audio_duration_s'] = record.audio_duration

            return json.dumps(log_data)

    def log_request(self, request_id, audio_duration, result, duration):
        """Log ASR request"""
        self.logger.info(
            'ASR request completed',
            extra={
                'request_id': request_id,
                'audio_duration': audio_duration,
                'text_length': len(result['text']),
                'confidence': result.get('confidence', 0),
                'duration': duration
            }
        )

    def log_error(self, request_id, error, traceback):
        """Log error"""
        self.logger.error(
            f'ASR request failed: {error}',
            extra={
                'request_id': request_id,
                'error': str(error),
                'traceback': traceback
            }
        )
```

## Security

### Authentication and Authorization

```python
from functools import wraps
import jwt
from flask import request, jsonify

class APIKeyAuth:
    """API key authentication"""

    def __init__(self, valid_keys):
        self.valid_keys = set(valid_keys)

    def require_api_key(self, f):
        """Decorator for API key authentication"""
        @wraps(f)
        def decorated_function(*args, **kwargs):
            api_key = request.headers.get('X-API-Key')

            if not api_key or api_key not in self.valid_keys:
                return jsonify({'error': 'Invalid API key'}), 401

            return f(*args, **kwargs)

        return decorated_function

class JWTAuth:
    """JWT authentication"""

    def __init__(self, secret_key):
        self.secret_key = secret_key

    def create_token(self, user_id, expiration_hours=24):
        """Create JWT token"""
        import datetime

        payload = {
            'user_id': user_id,
            'exp': datetime.datetime.utcnow() + datetime.timedelta(hours=expiration_hours)
        }

        return jwt.encode(payload, self.secret_key, algorithm='HS256')

    def verify_token(self, token):
        """Verify JWT token"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=['HS256'])
            return payload['user_id']
        except jwt.ExpiredSignatureError:
            return None
        except jwt.InvalidTokenError:
            return None

    def require_jwt(self, f):
        """Decorator for JWT authentication"""
        @wraps(f)
        def decorated_function(*args, **kwargs):
            token = request.headers.get('Authorization', '').replace('Bearer ', '')

            user_id = self.verify_token(token)
            if not user_id:
                return jsonify({'error': 'Invalid or expired token'}), 401

            return f(user_id=user_id, *args, **kwargs)

        return decorated_function
```

### Data Privacy

```python
class PrivacyProtection:
    """Privacy-preserving ASR"""

    def __init__(self):
        self.pii_patterns = {
            'email': r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b',
            'phone': r'\b\d{3}[-.]?\d{3}[-.]?\d{4}\b',
            'ssn': r'\b\d{3}-\d{2}-\d{4}\b',
            'credit_card': r'\b\d{4}[-\s]?\d{4}[-\s]?\d{4}[-\s]?\d{4}\b'
        }

    def redact_pii(self, text):
        """Redact personally identifiable information"""
        import re

        redacted = text
        for pii_type, pattern in self.pii_patterns.items():
            redacted = re.sub(pattern, f'[{pii_type.upper()}]', redacted)

        return redacted

    def encrypt_audio(self, audio_data, key):
        """Encrypt audio data"""
        from cryptography.fernet import Fernet

        f = Fernet(key)
        return f.encrypt(audio_data)

    def decrypt_audio(self, encrypted_audio, key):
        """Decrypt audio data"""
        from cryptography.fernet import Fernet

        f = Fernet(key)
        return f.decrypt(encrypted_audio)
```

## Performance Testing

### Load Testing

```python
import asyncio
import aiohttp
import time

class LoadTester:
    """Load testing for ASR service"""

    def __init__(self, service_url):
        self.service_url = service_url

    async def send_request(self, session, audio_file):
        """Send single request"""
        start = time.time()

        try:
            with open(audio_file, 'rb') as f:
                data = aiohttp.FormData()
                data.add_field('audio', f, filename='audio.wav')

                async with session.post(
                    f'{self.service_url}/transcribe',
                    data=data
                ) as response:
                    result = await response.json()
                    duration = time.time() - start

                    return {
                        'success': response.status == 200,
                        'duration': duration,
                        'result': result
                    }

        except Exception as e:
            return {
                'success': False,
                'duration': time.time() - start,
                'error': str(e)
            }

    async def run_load_test(self, audio_file, num_requests=100, concurrency=10):
        """Run load test"""
        print(f"Starting load test: {num_requests} requests, {concurrency} concurrent")

        results = []

        async with aiohttp.ClientSession() as session:
            tasks = []

            for i in range(num_requests):
                task = self.send_request(session, audio_file)
                tasks.append(task)

                # Limit concurrency
                if len(tasks) >= concurrency:
                    batch_results = await asyncio.gather(*tasks)
                    results.extend(batch_results)
                    tasks = []

            # Process remaining
            if tasks:
                batch_results = await asyncio.gather(*tasks)
                results.extend(batch_results)

        # Analyze results
        self._analyze_results(results)

    def _analyze_results(self, results):
        """Analyze load test results"""
        successful = [r for r in results if r['success']]
        failed = [r for r in results if not r['success']]

        durations = [r['duration'] for r in successful]

        print(f"\nResults:")
        print(f"  Total requests: {len(results)}")
        print(f"  Successful: {len(successful)}")
        print(f"  Failed: {len(failed)}")
        print(f"  Success rate: {len(successful)/len(results)*100:.2f}%")

        if durations:
            print(f"\nLatency:")
            print(f"  Min: {min(durations):.3f}s")
            print(f"  Max: {max(durations):.3f}s")
            print(f"  Mean: {sum(durations)/len(durations):.3f}s")
            print(f"  P50: {sorted(durations)[len(durations)//2]:.3f}s")
            print(f"  P95: {sorted(durations)[int(len(durations)*0.95)]:.3f}s")
            print(f"  P99: {sorted(durations)[int(len(durations)*0.99)]:.3f}s")

# Run test
async def main():
    tester = LoadTester('http://localhost:5000')
    await tester.run_load_test('test_audio.wav', num_requests=100, concurrency=10)

asyncio.run(main())
```

## Summary

Production deployment requires:

- **Architecture**: Microservices, containerization, orchestration
- **Optimization**: Quantization, distillation, ONNX export
- **Monitoring**: Metrics, logging, alerting
- **Security**: Authentication, encryption, PII protection
- **Testing**: Load testing, performance benchmarking

These practices ensure ASR systems serve 弘익人間 through reliable, scalable, secure production deployments.

## Review Questions

1. What are the benefits of microservices architecture for ASR?
2. How does quantization reduce model size and latency?
3. Explain knowledge distillation for model compression
4. What metrics should be monitored in production ASR?
5. How do you implement API key authentication?
6. Why is PII redaction important in ASR?
7. What is the purpose of load testing?
8. Compare Docker and Kubernetes for deployment
9. How does ONNX improve portability?
10. What are the key considerations for horizontal scaling?

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

Production deployment brings ASR technology to billions of users worldwide, making natural voice interaction accessible, reliable, and secure for everyone.

---

*Previous: [Streaming ASR](chapter7-streaming-asr.md)*

---

**Congratulations!** You've completed the WIA-AI-022 Speech Recognition eBook. You now have comprehensive knowledge of speech recognition from fundamentals to production deployment. Use this knowledge to build systems that benefit all humanity through natural, accessible voice interaction.
