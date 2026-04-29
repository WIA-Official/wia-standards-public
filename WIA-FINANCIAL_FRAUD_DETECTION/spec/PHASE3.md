# WIA-FINANCIAL_FRAUD_DETECTION - PHASE 3: Security, Performance & Compliance
**Version**: 1.0
**Status**: Production Ready
**Last Updated**: 2026-01-11

---

## 📋 Table of Contents

1. [Security Architecture](#security-architecture)
2. [Data Privacy & Protection](#data-privacy--protection)
3. [Performance Optimization](#performance-optimization)
4. [Scalability & High Availability](#scalability--high-availability)
5. [Model Security & Robustness](#model-security--robustness)
6. [Compliance & Audit](#compliance--audit)
7. [Testing & Quality Assurance](#testing--quality-assurance)
8. [Disaster Recovery](#disaster-recovery)

---

## 1. Security Architecture

### 1.1 Defense in Depth

**Multi-Layer Security Model**:

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 7: Application Security                              │
│ - Input validation, output encoding, CSRF protection       │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 6: API Security                                       │
│ - OAuth 2.0, JWT, API key rotation, rate limiting          │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 5: Data Security                                      │
│ - Encryption at rest (AES-256), in transit (TLS 1.3)       │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 4: Network Security                                   │
│ - VPC isolation, security groups, NACLs, WAF               │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 3: Container Security                                 │
│ - Image scanning, runtime protection, Pod Security         │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 2: Identity & Access Management                       │
│ - RBAC, MFA, SSO, privileged access management             │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────┴──────────────────────────────────────────┐
│ Layer 1: Physical/Infrastructure Security                   │
│ - Data center security (cloud provider responsibility)     │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Authentication Security

#### 1.2.1 API Key Management

**Key Generation**:
```python
import secrets
import hashlib

def generate_api_key():
    # Generate 32-byte random key
    key_bytes = secrets.token_bytes(32)
    # Prefix for key type identification
    prefix = "EXAMPLE_API_KEY_REPLACE_ME" if is_production else "sk_test_"
    # Base64 encode
    key = prefix + base64.urlsafe_b64encode(key_bytes).decode()
    # Store ONLY hashed version in database
    key_hash = hashlib.sha256(key.encode()).hexdigest()
    return key, key_hash
```

**Key Rotation Policy**:
- Automatic rotation every 90 days
- Manual rotation on security events
- Grace period: 7 days (both old and new keys valid)
- Email notification 14 days before expiration

**Key Storage**:
- **Application**: Environment variables or secrets manager (Vault, AWS Secrets Manager)
- **Database**: Hashed with SHA-256 (never store plaintext)
- **Transmission**: HTTPS only, never in URL parameters

#### 1.2.2 OAuth 2.0 Implementation

**Supported Flows**:
1. **Client Credentials** (machine-to-machine)
   ```
   POST /oauth/token
   grant_type=client_credentials&client_id=...&client_secret=...
   ```

2. **Authorization Code** (user consent)
   ```
   GET /oauth/authorize?response_type=code&client_id=...
   POST /oauth/token
   grant_type=authorization_code&code=...&redirect_uri=...
   ```

**Token Security**:
- **Access Token**: Short-lived (1 hour), JWT with RS256 signature
- **Refresh Token**: Long-lived (30 days), opaque token, one-time use
- **Token Rotation**: New refresh token issued on each refresh
- **Token Revocation**: Immediate invalidation on logout or security event

**JWT Structure**:
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "key_20260111"
  },
  "payload": {
    "iss": "https://auth.wia-fraud.io",
    "sub": "merchant_abc123",
    "aud": "api.wia-fraud.io",
    "exp": 1736597400,
    "iat": 1736593800,
    "scope": "fraud:read fraud:write",
    "merchant_id": "merch_abc123"
  },
  "signature": "..."
}
```

#### 1.2.3 Multi-Factor Authentication (MFA)

**Supported Methods**:
- **TOTP** (Time-based One-Time Password): Google Authenticator, Authy
- **SMS**: Fallback option (less secure, avoid for high-risk accounts)
- **Hardware Tokens**: YubiKey, FIDO2 keys (recommended for admins)
- **Backup Codes**: One-time recovery codes (10 codes, single-use)

**Enforcement Policy**:
- **Mandatory** for admin roles
- **Optional but recommended** for operators
- **Challenge-based** for sensitive operations (rule changes, model deployment)

### 1.3 Network Security

#### 1.3.1 VPC Architecture (AWS Example)

```
┌─────────────────────── VPC (10.0.0.0/16) ───────────────────────┐
│                                                                  │
│  ┌────────────── Public Subnets (10.0.1.0/24) ───────────────┐ │
│  │                                                             │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌─────────────┐ │ │
│  │  │   ALB (API   │    │  NAT Gateway │    │   Bastion   │ │ │
│  │  │   Gateway)   │    │              │    │    Host     │ │ │
│  │  └──────────────┘    └──────────────┘    └─────────────┘ │ │
│  │                                                             │ │
│  └─────────────────────────────┬───────────────────────────────┘ │
│                                │                                 │
│  ┌──────────── Private Subnets (10.0.2.0/24) ──────────────────┐│
│  │                            │                                 ││
│  │  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ││
│  │  │  EKS Nodes   │   │  EKS Nodes   │   │  EKS Nodes   │   ││
│  │  │ (Fraud API)  │   │ (ML Models)  │   │ (Streaming)  │   ││
│  │  └──────────────┘   └──────────────┘   └──────────────┘   ││
│  │                                                             ││
│  └─────────────────────────────┬───────────────────────────────┘│
│                                │                                 │
│  ┌────────── Database Subnets (10.0.3.0/24) ────────────────── ││
│  │                            │                                 ││
│  │  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ││
│  │  │  PostgreSQL  │   │   DynamoDB   │   │    Redis     │   ││
│  │  │   (Aurora)   │   │   (NoSQL)    │   │  (Features)  │   ││
│  │  └──────────────┘   └──────────────┘   └──────────────┘   ││
│  │                                                             ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

**Security Groups**:
```yaml
# ALB Security Group
ALB-SG:
  Inbound:
    - Port 443 (HTTPS) from 0.0.0.0/0
  Outbound:
    - Port 8080 to EKS-SG

# EKS Nodes Security Group
EKS-SG:
  Inbound:
    - Port 8080 from ALB-SG
    - Port 9090 (metrics) from Monitoring-SG
  Outbound:
    - Port 5432 to DB-SG (PostgreSQL)
    - Port 6379 to Cache-SG (Redis)
    - Port 443 to 0.0.0.0/0 (external APIs)

# Database Security Group
DB-SG:
  Inbound:
    - Port 5432 from EKS-SG
  Outbound:
    - None (no outbound connections)
```

#### 1.3.2 Web Application Firewall (WAF)

**AWS WAF Rules**:
```json
{
  "Name": "FraudDetectionWAF",
  "Rules": [
    {
      "Name": "RateLimitRule",
      "Priority": 1,
      "Statement": {
        "RateBasedStatement": {
          "Limit": 2000,
          "AggregateKeyType": "IP"
        }
      },
      "Action": {"Block": {}}
    },
    {
      "Name": "SQLInjectionRule",
      "Priority": 2,
      "Statement": {
        "ManagedRuleGroupStatement": {
          "VendorName": "AWS",
          "Name": "AWSManagedRulesSQLiRuleSet"
        }
      },
      "Action": {"Block": {}}
    },
    {
      "Name": "GeoBlockingRule",
      "Priority": 3,
      "Statement": {
        "GeoMatchStatement": {
          "CountryCodes": ["KP", "IR", "SY"]  # Block high-risk countries
        }
      },
      "Action": {"Block": {}}
    },
    {
      "Name": "BotDetectionRule",
      "Priority": 4,
      "Statement": {
        "ManagedRuleGroupStatement": {
          "VendorName": "AWS",
          "Name": "AWSManagedRulesBotControlRuleSet"
        }
      },
      "Action": {"Block": {}}
    }
  ]
}
```

#### 1.3.3 DDoS Protection

**Mitigation Strategies**:
- **AWS Shield Standard**: Automatic protection against common DDoS attacks
- **AWS Shield Advanced**: 24/7 DDoS response team, cost protection
- **Rate Limiting**: API-level rate limiting (see PHASE 2)
- **CDN**: CloudFront for static assets, caching
- **Auto-Scaling**: Absorb traffic spikes with horizontal scaling

### 1.4 Container Security

#### 1.4.1 Image Scanning

**Tools**: Trivy, Aqua Security, Twistlock

**Scan Policy**:
```yaml
# .gitlab-ci.yml
container_scanning:
  stage: test
  image: aquasec/trivy:latest
  script:
    - trivy image --severity HIGH,CRITICAL fraud-detection-api:$CI_COMMIT_SHA
    - if [ $? -ne 0 ]; then exit 1; fi  # Fail pipeline on HIGH/CRITICAL vulnerabilities
  only:
    - merge_requests
    - main
```

**Vulnerability Management**:
- **Critical**: Fix within 24 hours, hotfix deployment
- **High**: Fix within 7 days, include in next release
- **Medium**: Fix within 30 days
- **Low**: Fix opportunistically

#### 1.4.2 Runtime Protection

**Pod Security Standards** (Kubernetes):
```yaml
apiVersion: policy/v1beta1
kind: PodSecurityPolicy
metadata:
  name: fraud-detection-psp
spec:
  privileged: false
  allowPrivilegeEscalation: false
  requiredDropCapabilities:
    - ALL
  runAsUser:
    rule: MustRunAsNonRoot
  seLinux:
    rule: RunAsAny
  fsGroup:
    rule: RunAsAny
  volumes:
    - configMap
    - secret
    - emptyDir
```

**Network Policies**:
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: fraud-api-netpol
spec:
  podSelector:
    matchLabels:
      app: fraud-detection-api
  policyTypes:
    - Ingress
    - Egress
  ingress:
    - from:
      - namespaceSelector:
          matchLabels:
            name: ingress-nginx
      ports:
        - protocol: TCP
          port: 8080
  egress:
    - to:
      - namespaceSelector:
          matchLabels:
            name: databases
      ports:
        - protocol: TCP
          port: 5432
```

---

## 2. Data Privacy & Protection

### 2.1 Encryption

#### 2.1.1 Encryption at Rest

**Database Encryption** (PostgreSQL):
```sql
-- Enable Transparent Data Encryption (TDE)
ALTER SYSTEM SET encryption = on;
ALTER SYSTEM SET encryption_algorithm = 'AES-256';

-- Encrypted columns for sensitive data
CREATE TABLE transactions (
    id UUID PRIMARY KEY,
    customer_id UUID NOT NULL,
    amount NUMERIC(12,2) NOT NULL,
    card_pan BYTEA NOT NULL,  -- Encrypted PAN
    card_cvv BYTEA,            -- Encrypted CVV
    created_at TIMESTAMP DEFAULT NOW()
);

-- Application-level encryption (before INSERT)
INSERT INTO transactions (card_pan, card_cvv)
VALUES (
    pgp_sym_encrypt('4242424242424242', 'encryption_key'),
    pgp_sym_encrypt('123', 'encryption_key')
);
```

**File Storage Encryption** (AWS S3):
```python
import boto3

s3_client = boto3.client('s3')

# Server-Side Encryption (SSE-KMS)
s3_client.put_object(
    Bucket='fraud-detection-data',
    Key='models/xgboost_v2.3.1.json',
    Body=model_data,
    ServerSideEncryption='aws:kms',
    SSEKMSKeyId='arn:aws:kms:us-east-1:123456789:key/abc-123'
)
```

**Key Management** (AWS KMS):
```yaml
# Encryption keys
- DataKey: Envelope encryption for application data
- MasterKey: Encrypts data keys (automatic rotation every 365 days)
- KeyPolicy: IAM-based access control

# Key rotation
AutomaticRotation: Enabled (yearly)
ManualRotation: On security events
```

#### 2.1.2 Encryption in Transit

**TLS Configuration**:
```nginx
# Nginx configuration
server {
    listen 443 ssl http2;
    server_name api.wia-fraud.io;

    # TLS 1.3 only (most secure)
    ssl_protocols TLSv1.3;
    ssl_ciphers 'TLS_AES_256_GCM_SHA384:TLS_CHACHA20_POLY1305_SHA256';
    ssl_prefer_server_ciphers on;

    # HSTS (force HTTPS)
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains; preload" always;

    # Certificate
    ssl_certificate /etc/ssl/certs/wia-fraud.io.crt;
    ssl_certificate_key /etc/ssl/private/wia-fraud.io.key;

    # OCSP stapling
    ssl_stapling on;
    ssl_stapling_verify on;

    location / {
        proxy_pass http://fraud-detection-api:8080;
        proxy_ssl_verify on;
    }
}
```

**Certificate Management**:
- **Provider**: Let's Encrypt (automated renewal) or AWS ACM
- **Validity**: 90 days (Let's Encrypt) or 13 months (ACM)
- **Renewal**: Automated via certbot or AWS
- **Monitoring**: Alert 30 days before expiration

### 2.2 Data Tokenization

**PCI DSS Requirement**: Never store full PAN (Primary Account Number)

**Tokenization Service**:
```python
import hashlib
import secrets

class TokenizationService:
    def tokenize_card(self, card_pan: str) -> str:
        """
        Tokenize card number (PCI DSS compliant)
        Returns: token (e.g., tok_abc123def456)
        """
        # Generate random token
        token = f"tok_{secrets.token_urlsafe(16)}"

        # Store mapping in secure vault (NOT in main database)
        vault_client.store(
            key=token,
            value=card_pan,
            ttl=86400 * 365  # 1 year
        )

        return token

    def detokenize_card(self, token: str) -> str:
        """Retrieve original PAN (restricted access)"""
        return vault_client.retrieve(token)
```

**Token Storage**:
- **Location**: Separate vault (HashiCorp Vault, CyberArk)
- **Access Control**: Only payment processing service can detokenize
- **Audit**: All detokenization events logged
- **Expiration**: Tokens expire after 1 year (PCI DSS compliance)

### 2.3 Data Minimization

**Collect Only Necessary Data**:
```python
# ❌ BAD: Collecting unnecessary data
transaction = {
    "amount": 100.00,
    "customer_ssn": "123-45-6789",  # NOT needed for fraud detection
    "customer_salary": 75000,        # NOT needed
    "card_cvv": "123",               # NEVER store CVV after authorization
}

# ✅ GOOD: Minimal data collection
transaction = {
    "amount": 100.00,
    "card_bin": "424242",            # First 6 digits (sufficient for fraud detection)
    "card_last4": "4242",            # Last 4 digits
    "card_brand": "visa",
    "device_fingerprint": "fp_abc123",
}
```

**Data Retention Policy**:
| Data Type | Retention Period | Reason |
|-----------|------------------|--------|
| Transaction Records | 7 years | Legal/regulatory requirement |
| Card PAN (tokenized) | 1 year | Payment processing need |
| CVV | NEVER stored | PCI DSS prohibition |
| Device Fingerprints | 2 years | Fraud pattern analysis |
| IP Addresses | 90 days | GDPR privacy requirement |
| ML Model Training Data | 2 years | Model retraining |
| Audit Logs | 5 years | Compliance requirement |

### 2.4 Differential Privacy (Federated Learning)

**Use Case**: Train fraud detection models across multiple banks without sharing raw transaction data

**ε-Differential Privacy**:
```python
import numpy as np

def add_laplace_noise(value, epsilon, sensitivity):
    """
    Add Laplace noise for ε-differential privacy

    Args:
        value: Original value
        epsilon: Privacy budget (smaller = more privacy)
        sensitivity: Maximum change in output from single record
    """
    scale = sensitivity / epsilon
    noise = np.random.laplace(loc=0, scale=scale)
    return value + noise

# Example: Aggregate fraud statistics with privacy
true_fraud_count = 1250
epsilon = 1.0  # Privacy budget
sensitivity = 1  # Adding/removing one transaction changes count by 1

noisy_fraud_count = add_laplace_noise(true_fraud_count, epsilon, sensitivity)
# Result: 1253 (true value 1250 + noise)
```

**Federated Learning Architecture**:
```
┌────────────┐       ┌────────────┐       ┌────────────┐
│  Bank A    │       │  Bank B    │       │  Bank C    │
│            │       │            │       │            │
│ Local Model│       │ Local Model│       │ Local Model│
│ Training   │       │ Training   │       │ Training   │
└─────┬──────┘       └─────┬──────┘       └─────┬──────┘
      │                    │                    │
      │  Encrypted         │  Encrypted         │  Encrypted
      │  Gradients         │  Gradients         │  Gradients
      │                    │                    │
      └────────────────────┼────────────────────┘
                           │
                           ▼
                  ┌────────────────┐
                  │ Central Server │
                  │ (Aggregation)  │
                  └────────────────┘
                           │
                           │ Global Model
                           │ Update
                           ▼
          ┌────────────────┴────────────────┐
          │                                  │
    Distribute updated model to all banks
```

**Benefits**:
- No raw data sharing between institutions
- Collective fraud detection without privacy compromise
- GDPR compliant (data stays on-premise)

---

## 3. Performance Optimization

### 3.1 Latency Optimization

#### 3.1.1 Target Latencies

| Metric | Target | Measurement |
|--------|--------|-------------|
| **API Response (p50)** | ≤50ms | Median response time |
| **API Response (p95)** | ≤100ms | 95th percentile |
| **API Response (p99)** | ≤200ms | 99th percentile |
| **ML Inference** | ≤30ms | Model prediction time |
| **Feature Retrieval** | ≤10ms | Online feature store lookup |
| **Database Query** | ≤20ms | PostgreSQL/DynamoDB query |

#### 3.1.2 Caching Strategy

**Multi-Level Caching**:

```
┌─────────────────────────────────────────────────────────┐
│ L1: Application Cache (In-Memory)                      │
│ - Feature vectors: 60 seconds TTL                      │
│ - Model predictions: 5 minutes TTL                     │
│ - Tool: Guava Cache, Caffeine                         │
└────────────────────┬────────────────────────────────────┘
                     │ Cache Miss
                     ▼
┌─────────────────────────────────────────────────────────┐
│ L2: Distributed Cache (Redis)                          │
│ - User profiles: 15 minutes TTL                        │
│ - Device fingerprints: 1 hour TTL                      │
│ - Transaction history: 5 minutes TTL                   │
└────────────────────┬────────────────────────────────────┘
                     │ Cache Miss
                     ▼
┌─────────────────────────────────────────────────────────┐
│ L3: Database (PostgreSQL, DynamoDB)                    │
│ - Source of truth                                       │
└─────────────────────────────────────────────────────────┘
```

**Redis Configuration**:
```conf
# redis.conf
maxmemory 8gb
maxmemory-policy allkeys-lru  # Evict least recently used keys
tcp-keepalive 60
timeout 300

# Persistence (for durability)
save 900 1       # Save if 1 key changed in 15 minutes
save 300 10      # Save if 10 keys changed in 5 minutes
save 60 10000    # Save if 10000 keys changed in 1 minute
```

**Cache Invalidation**:
```python
# Event-driven cache invalidation
def on_transaction_feedback(transaction_id, feedback):
    # Invalidate user profile cache (risk score may change)
    redis_client.delete(f"user_profile:{transaction.customer_id}")

    # Invalidate device cache
    redis_client.delete(f"device:{transaction.device.fingerprint}")

    # Publish cache invalidation event
    redis_client.publish("cache_invalidation", {
        "type": "user_profile",
        "user_id": transaction.customer_id
    })
```

#### 3.1.3 Database Optimization

**PostgreSQL Indexing**:
```sql
-- Index on frequently queried columns
CREATE INDEX idx_transactions_customer_timestamp
ON transactions (customer_id, timestamp DESC);

CREATE INDEX idx_transactions_device_timestamp
ON transactions (device_fingerprint, timestamp DESC);

-- Partial index for fraud cases only
CREATE INDEX idx_transactions_fraud
ON transactions (timestamp DESC)
WHERE is_fraud = true;

-- GiST index for geolocation queries
CREATE INDEX idx_transactions_geolocation
ON transactions USING GIST (location);
```

**Query Optimization**:
```sql
-- ❌ BAD: Full table scan
SELECT * FROM transactions
WHERE customer_id = 'cust_123'
ORDER BY timestamp DESC;

-- ✅ GOOD: Index usage + column selection
SELECT id, amount, timestamp, merchant_id
FROM transactions
WHERE customer_id = 'cust_123'
ORDER BY timestamp DESC
LIMIT 100;

-- Execution plan
EXPLAIN ANALYZE ...;
-- Result: Index Scan using idx_transactions_customer_timestamp
```

**Connection Pooling**:
```python
# PgBouncer configuration
[databases]
fraud_detection = host=postgres.internal port=5432 dbname=fraud_detection

[pgbouncer]
pool_mode = transaction  # Fast, suitable for REST APIs
max_client_conn = 1000
default_pool_size = 25   # 25 connections per database
```

#### 3.1.4 Asynchronous Processing

**Celery Task Queue** (for non-critical operations):
```python
from celery import Celery

celery_app = Celery('fraud_detection', broker='redis://localhost:6379/0')

@celery_app.task
def update_user_risk_profile(customer_id):
    """Background task: Update user risk profile (not time-critical)"""
    transactions = get_recent_transactions(customer_id, days=30)
    risk_profile = calculate_risk_profile(transactions)
    save_risk_profile(customer_id, risk_profile)

# API endpoint
@app.post("/fraud/analyze")
async def analyze_transaction(transaction: Transaction):
    # Synchronous: Real-time fraud detection (must be fast)
    fraud_score = await fraud_detector.analyze(transaction)

    # Asynchronous: Update risk profile in background
    update_user_risk_profile.delay(transaction.customer_id)

    return {"fraud_score": fraud_score}
```

### 3.2 Throughput Optimization

#### 3.2.1 Batch Processing

**Kafka Streams** (micro-batching):
```java
StreamsBuilder builder = new StreamsBuilder();
KStream<String, Transaction> transactions = builder.stream("transactions");

transactions
    .groupByKey()
    .windowedBy(TimeWindows.of(Duration.ofSeconds(1)))  // 1-second windows
    .aggregate(
        TransactionBatch::new,
        (key, transaction, batch) -> batch.add(transaction)
    )
    .toStream()
    .foreach((windowedKey, batch) -> {
        // Process batch of transactions
        List<FraudScore> scores = fraudDetector.analyzeBatch(batch);
    });
```

**Benefits**:
- **Higher Throughput**: Process 1000 transactions/batch vs. 1 transaction/call
- **Resource Efficiency**: Amortize overhead (network, serialization)
- **Cost Reduction**: Fewer API calls to external services

#### 3.2.2 Model Optimization

**Model Quantization** (reduce model size and inference time):
```python
import tensorflow as tf

# Original model: 8 MB, 30ms inference
model = tf.keras.models.load_model('fraud_model.h5')

# Quantize to INT8 (8-bit integers)
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.int8]

tflite_model = converter.convert()

# Result: 2 MB model, 10ms inference (3x faster)
```

**Model Distillation** (student-teacher learning):
```python
# Large teacher model (200 trees, slow but accurate)
teacher_model = XGBoostClassifier(n_estimators=200)

# Small student model (50 trees, fast)
student_model = XGBoostClassifier(n_estimators=50)

# Train student to mimic teacher predictions
X_train, y_train = load_training_data()
teacher_predictions = teacher_model.predict_proba(X_train)[:, 1]

# Soft labels (weighted combination of true labels and teacher predictions)
alpha = 0.8  # Weight for teacher predictions
soft_labels = alpha * teacher_predictions + (1 - alpha) * y_train

student_model.fit(X_train, soft_labels)

# Result: 90% of teacher accuracy, 4x faster inference
```

**ONNX Runtime** (cross-platform model optimization):
```python
import onnxruntime as ort

# Convert PyTorch/TensorFlow model to ONNX format
# ... (conversion code)

# Load optimized model
session = ort.InferenceSession(
    "fraud_model.onnx",
    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
)

# Inference
inputs = {"input": transaction_features.numpy()}
outputs = session.run(None, inputs)
fraud_score = outputs[0][0]

# Result: 2-3x faster inference with ONNX Runtime optimizations
```

---

## 4. Scalability & High Availability

### 4.1 Horizontal Scaling

#### 4.1.1 Kubernetes Auto-Scaling

**Horizontal Pod Autoscaler (HPA)**:
```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: fraud-detection-api-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: fraud-detection-api
  minReplicas: 3
  maxReplicas: 50
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
    - type: Pods
      pods:
        metric:
          name: request_latency_p95
        target:
          type: AverageValue
          averageValue: "100m"  # 100ms
  behavior:
    scaleUp:
      stabilizationWindowSeconds: 60
      policies:
        - type: Percent
          value: 50  # Scale up by 50% per minute
          periodSeconds: 60
    scaleDown:
      stabilizationWindowSeconds: 300  # 5-minute cooldown
      policies:
        - type: Pods
          value: 2  # Scale down by 2 pods per 5 minutes
          periodSeconds: 300
```

**Cluster Autoscaler** (scale nodes):
```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: cluster-autoscaler-config
data:
  min-nodes: "3"
  max-nodes: "20"
  scale-down-delay-after-add: "10m"
  scale-down-unneeded-time: "10m"
```

#### 4.1.2 Database Sharding

**Shard Key Selection**: `customer_id` (most queries filter by customer)

**Sharding Strategy**:
```python
import hashlib

def get_shard_id(customer_id: str, num_shards: int = 8) -> int:
    """Consistent hashing for shard assignment"""
    hash_value = int(hashlib.md5(customer_id.encode()).hexdigest(), 16)
    return hash_value % num_shards

# Example: Route query to correct shard
customer_id = "cust_123"
shard_id = get_shard_id(customer_id)
db_connection = get_database_connection(f"shard_{shard_id}")

transactions = db_connection.query(
    "SELECT * FROM transactions WHERE customer_id = %s",
    (customer_id,)
)
```

**Shard Distribution** (8 shards):
```
Shard 0: customers with hash % 8 == 0 (12.5% of data)
Shard 1: customers with hash % 8 == 1 (12.5% of data)
...
Shard 7: customers with hash % 8 == 7 (12.5% of data)
```

**Benefits**:
- **Linear Scalability**: Add more shards to handle more data
- **Fault Isolation**: One shard failure doesn't affect others
- **Parallel Processing**: Query multiple shards concurrently

### 4.2 High Availability

#### 4.2.1 Multi-Region Deployment

```
┌─────────────────────────────────────────────────────────────┐
│                    Route 53 (Global DNS)                    │
│              Latency-based routing + Health checks          │
└───────────┬─────────────────────────────────┬───────────────┘
            │                                 │
            ▼                                 ▼
┌───────────────────────┐         ┌───────────────────────┐
│   Region: us-east-1   │         │   Region: eu-west-1   │
│  (Primary - Active)   │         │ (Secondary - Active)  │
│                       │         │                       │
│  ┌─────────────────┐ │         │  ┌─────────────────┐  │
│  │  EKS Cluster    │ │         │  │  EKS Cluster    │  │
│  │  (API + ML)     │ │         │  │  (API + ML)     │  │
│  └─────────────────┘ │         │  └─────────────────┘  │
│                       │         │                       │
│  ┌─────────────────┐ │         │  ┌─────────────────┐  │
│  │  Aurora (RW)    │◄┼─────────┼──┤  Aurora (RO)    │  │
│  │  Multi-AZ       │ │         │  │  Read Replica   │  │
│  └─────────────────┘ │         │  └─────────────────┘  │
│                       │         │                       │
└───────────────────────┘         └───────────────────────┘
```

**Failover Strategy**:
1. **Health Check Failure** (us-east-1 down):
   - Route 53 detects unhealthy endpoint (30-second checks)
   - Routes 100% traffic to eu-west-1
   - Promote Aurora read replica to read-write
   - Time to failover: <2 minutes

2. **Recovery** (us-east-1 back online):
   - Gradually shift traffic back (50% → 80% → 100%)
   - Re-establish Aurora replication
   - Monitor for consistency issues

**RTO/RPO Targets**:
- **RTO** (Recovery Time Objective): <5 minutes
- **RPO** (Recovery Point Objective): <1 minute (Aurora continuous replication)

#### 4.2.2 Circuit Breaker Pattern

**Prevent Cascading Failures**:
```python
from circuitbreaker import circuit

@circuit(failure_threshold=5, recovery_timeout=60)
def call_external_api(url, data):
    """
    Circuit breaker for external API calls

    - Trips open after 5 consecutive failures
    - Stays open for 60 seconds
    - Half-open state: Try 1 request to test recovery
    """
    response = requests.post(url, json=data, timeout=2)
    response.raise_for_status()
    return response.json()

# Usage
try:
    result = call_external_api("https://external-api.com/verify", data)
except CircuitBreakerError:
    # Circuit is open, use fallback
    result = fallback_verification(data)
```

**Benefits**:
- **Fast Failure**: Don't wait for timeout if service is known to be down
- **Resource Protection**: Prevent thread pool exhaustion
- **Graceful Degradation**: Use cached data or simplified logic

---

## 5. Model Security & Robustness

### 5.1 Adversarial Attack Defense

#### 5.1.1 Attack Vectors

**Evasion Attacks**:
- **Feature Manipulation**: Attacker tweaks transaction features to avoid detection
  - Example: Split large transaction into multiple small ones (below threshold)
  - Example: Use VPN to appear from low-risk country

**Model Inversion**:
- **Goal**: Reverse-engineer training data from model predictions
- **Risk**: Expose sensitive transaction patterns

**Poisoning Attacks**:
- **Goal**: Inject malicious data into training set to degrade model
- **Example**: Submit false feedback ("not fraud") for actual fraud transactions

#### 5.1.2 Defense Mechanisms

**Feature Obfuscation**:
```python
def add_feature_noise(features, epsilon=0.1):
    """
    Add small random noise to features (adversarial training)
    Makes model robust to small perturbations
    """
    noise = np.random.normal(0, epsilon, size=features.shape)
    return features + noise

# During training
X_train_augmented = add_feature_noise(X_train)
model.fit(X_train_augmented, y_train)
```

**Ensemble Diversity**:
- Train models on different subsets of features
- Attacker must evade ALL models simultaneously (harder)

**Input Validation**:
```python
def validate_transaction(transaction):
    """Detect suspicious feature values"""

    # Check for impossible values
    if transaction.amount < 0:
        raise ValueError("Negative amount")

    if transaction.amount > 10000 and transaction.customer.account_age_days < 1:
        # Suspicious: Large transaction on brand new account
        transaction.risk_flags.append("high_amount_new_account")

    # Check for feature engineering attacks
    if transaction.velocity_1h > 100:
        # Impossible: 100 transactions in 1 hour
        raise ValueError("Anomalous velocity")

    return transaction
```

**Rate Limiting by Entity**:
```python
# Limit fraud checks per customer (prevent feature probing)
if rate_limiter.is_exceeded(f"fraud_check:{customer_id}", limit=10, window=60):
    raise RateLimitError("Too many fraud checks for this customer")
```

### 5.2 Model Monitoring

#### 5.2.1 Performance Drift Detection

**Metrics to Track**:
```python
import numpy as np
from scipy.stats import ks_2samp

def detect_concept_drift(recent_scores, baseline_scores):
    """
    Detect distribution shift in fraud scores (concept drift)
    Uses Kolmogorov-Smirnov test
    """
    statistic, p_value = ks_2samp(recent_scores, baseline_scores)

    if p_value < 0.05:
        # Significant drift detected
        alert(f"Concept drift detected: p={p_value:.4f}")
        trigger_model_retraining()
```

**Monitoring Dashboard**:
```
┌──────────────────────────────────────────────────────────┐
│ Fraud Detection Model Performance (Last 7 Days)         │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  Precision:  ██████████████░░░░ 87% (↓ from 92%)       │
│  Recall:     ████████████████░░ 93% (stable)            │
│  F1-Score:   ███████████████░░░ 90% (↓ from 92%)       │
│  AUC-ROC:    ██████████████████ 0.985 (stable)          │
│                                                          │
│  False Positive Rate: 0.8% (↑ from 0.5%) ⚠️            │
│  False Negative Rate: 0.15% (stable) ✅                 │
│                                                          │
│  ⚠️ ALERT: Precision dropped 5% in last 24 hours       │
│      Recommendation: Investigate feature drift          │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

#### 5.2.2 Feature Drift Detection

**Monitor Feature Distributions**:
```python
# Compare recent feature distribution to baseline
baseline_avg_amount = 150.00  # Historical average
recent_avg_amount = 210.00    # Last 24 hours

drift_ratio = recent_avg_amount / baseline_avg_amount
if drift_ratio > 1.5 or drift_ratio < 0.5:
    alert(f"Feature drift: avg_amount changed by {(drift_ratio-1)*100:.1f}%")
```

**Example Alert**:
```
⚠️ Feature Drift Alert
Feature: transaction_amount
Baseline Mean: $150.00
Recent Mean: $210.00 (+40%)
Likely Cause: Holiday shopping season
Recommended Action: Retrain model with recent data
```

---

## 6. Compliance & Audit

### 6.1 Audit Logging

#### 6.1.1 What to Log

**Security Events**:
```json
{
  "event_type": "authentication_failure",
  "timestamp": "2026-01-11T10:30:00Z",
  "user_id": "user_123",
  "ip_address": "203.0.113.42",
  "user_agent": "Mozilla/5.0...",
  "reason": "Invalid API key",
  "severity": "warning"
}
```

**Fraud Decisions**:
```json
{
  "event_type": "fraud_decision",
  "timestamp": "2026-01-11T10:30:00.095Z",
  "transaction_id": "txn_1234567890",
  "decision": "block",
  "risk_score": 0.92,
  "model_version": "v2.3.1",
  "top_features": [
    {"feature": "device_fraud_rate", "value": 0.85},
    {"feature": "impossible_travel", "value": 1}
  ],
  "triggered_rules": ["impossible_travel_rule"],
  "reviewed_by": null,
  "severity": "critical"
}
```

**Data Access**:
```json
{
  "event_type": "data_access",
  "timestamp": "2026-01-11T10:30:00Z",
  "user_id": "analyst_456",
  "resource": "transactions.customer_id=cust_123",
  "action": "read",
  "reason": "Chargeback investigation",
  "approved_by": "manager_789"
}
```

#### 6.1.2 Log Storage & Retention

**Architecture**:
```
Application Logs
  ↓
Fluentd (log aggregation)
  ↓
Elasticsearch (search & analysis)
  ↓
Kibana (visualization)
  ↓
S3 Glacier (long-term archive)
```

**Retention Policy**:
- **Hot Tier** (Elasticsearch): 90 days (fast search)
- **Warm Tier** (S3 Standard): 1 year (occasional access)
- **Cold Tier** (S3 Glacier): 7 years (compliance requirement)

**Access Control**:
```yaml
# Elasticsearch role
fraud_analyst:
  indices:
    - names: ['audit-logs-*']
      privileges: ['read']
      field_security:
        grant: ['*']
        except: ['customer.email', 'customer.phone']  # PII redacted

fraud_admin:
  indices:
    - names: ['audit-logs-*']
      privileges: ['read', 'write', 'delete']
      field_security:
        grant: ['*']  # Full access
```

### 6.2 Regulatory Reporting

#### 6.2.1 PCI DSS Quarterly Reports

**Automated Report Generation**:
```python
def generate_pci_dss_report(quarter):
    """Generate PCI DSS compliance report"""

    report = {
        "quarter": quarter,
        "compliance_status": "compliant",
        "requirements": [
            {
                "req_id": "3.4",
                "description": "Render PAN unreadable",
                "status": "compliant",
                "evidence": "All PANs tokenized, 0 plaintext storage instances"
            },
            {
                "req_id": "10.2",
                "description": "Audit trail for all access to cardholder data",
                "status": "compliant",
                "evidence": f"100% of data access logged, {log_count} entries"
            },
            {
                "req_id": "11.3",
                "description": "Penetration testing",
                "status": "compliant",
                "evidence": "External pentest conducted 2026-01-05, 0 critical findings"
            }
        ],
        "vulnerabilities": {
            "critical": 0,
            "high": 0,
            "medium": 2,
            "low": 5
        },
        "generated_at": datetime.now().isoformat()
    }

    return report
```

#### 6.2.2 GDPR Data Subject Rights

**Right to Access** (Article 15):
```python
@app.get("/v1/gdpr/data-export")
async def export_user_data(customer_id: str):
    """Export all data related to a customer (GDPR Article 15)"""

    data_export = {
        "customer_profile": get_customer_profile(customer_id),
        "transactions": get_transactions(customer_id, all=True),
        "fraud_assessments": get_fraud_assessments(customer_id),
        "device_fingerprints": get_devices(customer_id),
        "consent_records": get_consent_history(customer_id)
    }

    # Format as human-readable JSON
    return JSONResponse(content=data_export)
```

**Right to Erasure** (Article 17):
```python
@app.delete("/v1/gdpr/delete-data")
async def delete_user_data(customer_id: str):
    """Delete all customer data (GDPR Right to be Forgotten)"""

    # Check if legal obligation to retain (e.g., ongoing fraud investigation)
    if has_legal_hold(customer_id):
        raise HTTPException(
            status_code=403,
            detail="Cannot delete: legal hold due to fraud investigation"
        )

    # Pseudonymize instead of delete (for ML model integrity)
    pseudonymize_customer_data(customer_id)

    # Mark for deletion in 30 days (cooling-off period)
    schedule_data_deletion(customer_id, days=30)

    return {"status": "scheduled_for_deletion", "deletion_date": "2026-02-10"}
```

---

## 7. Testing & Quality Assurance

### 7.1 Model Testing

#### 7.1.1 Offline Evaluation

**Backtesting**:
```python
def backtest_model(model, test_data, dates):
    """Test model on historical data (time-series split)"""

    results = []
    for date in dates:
        # Use only data available before this date for prediction
        train_data = test_data[test_data['timestamp'] < date]
        eval_data = test_data[test_data['timestamp'] == date]

        model.fit(train_data)
        predictions = model.predict(eval_data)

        precision = precision_score(eval_data['is_fraud'], predictions)
        recall = recall_score(eval_data['is_fraud'], predictions)

        results.append({
            'date': date,
            'precision': precision,
            'recall': recall
        })

    return results

# Example results
# Date: 2025-12-01, Precision: 0.92, Recall: 0.94
# Date: 2025-12-15, Precision: 0.91, Recall: 0.95
# Date: 2026-01-01, Precision: 0.88, Recall: 0.93 (holiday season, more fraud)
```

#### 7.1.2 Online A/B Testing

**Shadow Mode** (safe deployment):
```python
# Deploy new model v2.4.0 in shadow mode
# - Both old (v2.3.1) and new (v2.4.0) models score transactions
# - Only v2.3.1 decisions are used (production)
# - Compare performance after 48 hours

def score_transaction_ab_test(transaction):
    # Production model (v2.3.1)
    prod_score = model_v231.predict(transaction)
    production_decision = make_decision(prod_score)

    # Candidate model (v2.4.0) - shadow mode
    candidate_score = model_v240.predict(transaction)
    candidate_decision = make_decision(candidate_score)

    # Log both for comparison
    log_ab_test_result({
        'transaction_id': transaction.id,
        'prod_score': prod_score,
        'candidate_score': candidate_score,
        'prod_decision': production_decision,
        'candidate_decision': candidate_decision
    })

    # Return production decision only
    return production_decision
```

**Canary Deployment** (gradual rollout):
```yaml
# Kubernetes Canary Deployment
---
apiVersion: v1
kind: Service
metadata:
  name: fraud-detection-model
spec:
  selector:
    app: fraud-detection
  # Routes traffic to both stable and canary versions
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: fraud-detection-stable
spec:
  replicas: 9  # 90% traffic
  template:
    metadata:
      labels:
        app: fraud-detection
        version: v2.3.1
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: fraud-detection-canary
spec:
  replicas: 1  # 10% traffic
  template:
    metadata:
      labels:
        app: fraud-detection
        version: v2.4.0
```

### 7.2 Security Testing

#### 7.2.1 Penetration Testing

**Scope** (Annual):
- API security (authentication, authorization, input validation)
- Infrastructure (network segmentation, firewall rules)
- Application vulnerabilities (OWASP Top 10)
- Social engineering (phishing simulations)

**Tools**:
- **Burp Suite**: Web application security testing
- **OWASP ZAP**: Automated vulnerability scanning
- **Metasploit**: Exploitation framework
- **Nmap**: Network reconnaissance

**Example Findings** (from 2026-01-05 pentest):
```
✅ No Critical vulnerabilities
✅ No High vulnerabilities
⚠️ 2 Medium vulnerabilities:
  - M1: Missing rate limiting on /oauth/token endpoint
    → Fixed: Implemented 10 requests/minute per IP
  - M2: Verbose error messages expose stack traces
    → Fixed: Sanitized error responses in production
📝 5 Low vulnerabilities:
  - L1-L5: Security headers improvements (HSTS, CSP)
```

#### 7.2.2 Chaos Engineering

**Resilience Testing**:
```python
import chaosengine

# Test 1: Database connection failure
@chaosengine.experiment
def database_failure_test():
    # Simulate PostgreSQL connection loss
    chaosengine.kill_process("postgres")

    # Verify: API should use Redis cache fallback
    response = requests.post("/v1/fraud/analyze", json=test_transaction)
    assert response.status_code == 200
    assert response.json()["data_source"] == "cache"

# Test 2: High latency
@chaosengine.experiment
def latency_injection_test():
    # Add 200ms latency to ML model inference
    chaosengine.add_latency("fraud-detection-model", ms=200)

    # Verify: API should timeout and use rule-based fallback
    response = requests.post("/v1/fraud/analyze", json=test_transaction)
    assert response.status_code == 200
    assert response.json()["decision_method"] == "rules_only"
```

---

## 8. Disaster Recovery

### 8.1 Backup Strategy

**Database Backups** (Aurora):
- **Automated Snapshots**: Daily, retained for 35 days
- **Continuous Backup**: Point-in-time recovery (up to 5 minutes ago)
- **Cross-Region Replication**: Asynchronous replication to DR region

**Model Backups**:
- **Versioned Storage**: S3 with versioning enabled
- **Retention**: Last 10 model versions (6 months history)
- **Cross-Region Copy**: Replicate to secondary region

### 8.2 Recovery Procedures

**RTO/RPO Summary**:
| Failure Scenario | RTO | RPO | Recovery Procedure |
|------------------|-----|-----|-------------------|
| Single pod failure | <1 min | 0 | Kubernetes auto-restart |
| Node failure | <2 min | 0 | Pod rescheduling to healthy node |
| Availability Zone failure | <5 min | 0 | Multi-AZ failover (Aurora, EKS) |
| Region failure | <30 min | <5 min | Route 53 failover + Aurora promotion |
| Data corruption | <2 hours | <1 hour | Restore from snapshot |
| Ransomware attack | <4 hours | <1 day | Restore from immutable backup |

**Recovery Steps** (Regional Failure):
1. **Detection** (5 minutes): CloudWatch alarms detect us-east-1 outage
2. **Route 53 Failover** (2 minutes): Route traffic to eu-west-1
3. **Aurora Promotion** (5 minutes): Promote read replica to primary
4. **Verification** (10 minutes): Test critical paths, monitor error rates
5. **Communication** (ongoing): Update status page, notify customers

---

**Document Status**: ✅ **Approved for PHASE 4 Development**
**Next Phase**: [PHASE4.md](./PHASE4.md) - Deployment, Operations & Monitoring

---

© 2026 WIA (World Certification Industry Association)
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
