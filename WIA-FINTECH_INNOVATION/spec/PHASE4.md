# WIA-FINTECH_INNOVATION Specification - PHASE 4: Deployment, Operations & Monitoring

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2026-01-11

## Table of Contents
1. [Deployment Architecture](#deployment-architecture)
2. [CI/CD Pipeline](#cicd-pipeline)
3. [Monitoring & Observability](#monitoring--observability)
4. [Incident Response](#incident-response)
5. [Operations & SRE](#operations--sre)
6. [Cost Optimization](#cost-optimization)

---

## 1. Deployment Architecture

### 1.1 Kubernetes-Based Infrastructure

**Cluster Configuration:**
```yaml
apiVersion: v1
kind: Cluster
metadata:
  name: wia-fintech-production
spec:
  regions:
    - us-east-1
    - eu-west-1
    - ap-southeast-1

  node_pools:
    - name: general-purpose
      machine_type: n2-standard-8  # 8 vCPU, 32GB RAM
      min_nodes: 3
      max_nodes: 50
      autoscaling: true

    - name: high-memory
      machine_type: n2-highmem-16  # 16 vCPU, 128GB RAM
      min_nodes: 2
      max_nodes: 20
      labels:
        workload: fraud-detection

    - name: gpu-enabled
      machine_type: n1-standard-8-gpu  # For ML inference
      accelerator: nvidia-tesla-t4
      min_nodes: 0
      max_nodes: 10

  network:
    vpc_native: true
    private_cluster: true
    authorized_networks:
      - "10.0.0.0/8"  # Internal only
```

**Service Mesh (Istio):**
```
┌─────────────────────────────────────────────────────────────┐
│                    Istio Service Mesh                        │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Ingress Gateway (TLS termination, L7 routing)        │  │
│  └────────────────┬───────────────────────────────────────┘  │
│                   │                                          │
│  ┌────────────────▼──────┐    ┌─────────────────────────┐   │
│  │  Payment Service      │    │  Open Banking Service   │   │
│  │  - Envoy Sidecar      │◄───┤  - Envoy Sidecar        │   │
│  │  - mTLS               │    │  - Circuit Breaker      │   │
│  │  - Retry logic        │    │  - Request tracing      │   │
│  └────────┬──────────────┘    └────────┬────────────────┘   │
│           │                             │                    │
│  ┌────────▼─────────────────────────────▼────────────────┐   │
│  │  BNPL Service                                         │   │
│  │  - Rate limiting (100 RPS per customer)               │   │
│  │  - Canary deployment (5% traffic to new version)     │   │
│  └───────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Microservices Deployment

**Payment Service:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: payment-service
  namespace: fintech-prod
spec:
  replicas: 10
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 2
      maxUnavailable: 1

  selector:
    matchLabels:
      app: payment-service

  template:
    metadata:
      labels:
        app: payment-service
        version: v1.2.3
    spec:
      serviceAccountName: payment-service-sa

      containers:
      - name: payment-service
        image: gcr.io/wia-fintech/payment-service:v1.2.3
        ports:
        - containerPort: 8080
          name: http
        - containerPort: 9090
          name: metrics

        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-credentials
              key: connection-string
        - name: REDIS_URL
          value: "redis://redis-cluster:6379"
        - name: KAFKA_BROKERS
          value: "kafka-0:9092,kafka-1:9092,kafka-2:9092"

        resources:
          requests:
            cpu: "1000m"
            memory: "2Gi"
          limits:
            cpu: "2000m"
            memory: "4Gi"

        livenessProbe:
          httpGet:
            path: /health/live
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
          timeoutSeconds: 5
          failureThreshold: 3

        readinessProbe:
          httpGet:
            path: /health/ready
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 5
          timeoutSeconds: 3
          failureThreshold: 2

---
apiVersion: v1
kind: Service
metadata:
  name: payment-service
  namespace: fintech-prod
spec:
  type: ClusterIP
  selector:
    app: payment-service
  ports:
  - name: http
    port: 80
    targetPort: 8080
  - name: metrics
    port: 9090
    targetPort: 9090

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: payment-service-hpa
  namespace: fintech-prod
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: payment-service
  minReplicas: 10
  maxReplicas: 100
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
        name: http_requests_per_second
      target:
        type: AverageValue
        averageValue: "1000"

  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 10
        periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 0
      policies:
      - type: Percent
        value: 50
        periodSeconds: 30
      - type: Pods
        value: 5
        periodSeconds: 30
      selectPolicy: Max
```

### 1.3 Database Deployment

**PostgreSQL High Availability:**
```yaml
# Using CloudNativePG operator
apiVersion: postgresql.cnpg.io/v1
kind: Cluster
metadata:
  name: fintech-db-cluster
spec:
  instances: 3  # 1 primary + 2 hot standbys

  postgresql:
    parameters:
      max_connections: "500"
      shared_buffers: "16GB"
      effective_cache_size: "48GB"
      work_mem: "64MB"
      maintenance_work_mem: "2GB"
      random_page_cost: "1.1"  # SSD optimization
      effective_io_concurrency: "200"
      wal_level: "logical"
      max_wal_senders: "10"
      max_replication_slots: "10"

  bootstrap:
    initdb:
      database: fintech_prod
      owner: fintech_user
      encoding: UTF8
      localeCType: en_US.UTF-8
      localeCollate: en_US.UTF-8

  storage:
    size: 500Gi
    storageClass: ssd-retain

  backup:
    barmanObjectStore:
      destinationPath: gs://wia-fintech-backups/postgres
      wal:
        compression: gzip
        encryption: AES256
      retentionPolicy: "30d"
    schedule: "0 */6 * * *"  # Every 6 hours

  monitoring:
    enablePodMonitor: true

  resources:
    requests:
      cpu: "4000m"
      memory: "16Gi"
    limits:
      cpu: "8000m"
      memory: "32Gi"
```

### 1.4 Deployment Strategies

**Blue-Green Deployment:**
```yaml
# Blue environment (current)
apiVersion: v1
kind: Service
metadata:
  name: payment-service
spec:
  selector:
    app: payment-service
    environment: blue
  ports:
  - port: 80

---
# Green environment (new version)
apiVersion: apps/v1
kind: Deployment
metadata:
  name: payment-service-green
spec:
  replicas: 10
  selector:
    matchLabels:
      app: payment-service
      environment: green
  template:
    metadata:
      labels:
        app: payment-service
        environment: green
        version: v1.3.0
    spec:
      containers:
      - name: payment-service
        image: gcr.io/wia-fintech/payment-service:v1.3.0

# Traffic switch (after validation)
# kubectl patch service payment-service -p '{"spec":{"selector":{"environment":"green"}}}'
```

**Canary Deployment (Istio):**
```yaml
apiVersion: networking.istio.io/v1beta1
kind: VirtualService
metadata:
  name: payment-service-canary
spec:
  hosts:
  - payment-service
  http:
  - match:
    - headers:
        x-canary:
          exact: "true"
    route:
    - destination:
        host: payment-service
        subset: v2
      weight: 100

  - route:
    - destination:
        host: payment-service
        subset: v1
      weight: 95
    - destination:
        host: payment-service
        subset: v2
      weight: 5  # 5% traffic to new version

---
apiVersion: networking.istio.io/v1beta1
kind: DestinationRule
metadata:
  name: payment-service
spec:
  host: payment-service
  subsets:
  - name: v1
    labels:
      version: v1.2.3
  - name: v2
    labels:
      version: v1.3.0
```

---

## 2. CI/CD Pipeline

### 2.1 Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Developer                                                   │
│  git push → GitHub/GitLab                                   │
└────────────────┬────────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────────┐
│  CI Pipeline (GitHub Actions / GitLab CI)                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │  Build   │→ │  Test    │→ │  Scan    │→ │  Package │    │
│  │  Docker  │  │  Unit    │  │  SAST    │  │  Push to │    │
│  │  Image   │  │  Integ.  │  │  SCA     │  │  Registry│    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
└────────────────┬────────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────────┐
│  CD Pipeline (ArgoCD / Flux)                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │  Deploy  │→ │  Smoke   │→ │  Canary  │→ │  Full    │    │
│  │  to      │  │  Tests   │  │  5%      │  │  Rollout │    │
│  │  Staging │  │          │  │  Traffic │  │  100%    │    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 GitHub Actions Workflow

```yaml
name: CI/CD Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

env:
  DOCKER_REGISTRY: gcr.io/wia-fintech
  SERVICE_NAME: payment-service

jobs:
  build-and-test:
    runs-on: ubuntu-latest

    steps:
    - name: Checkout code
      uses: actions/checkout@v4

    - name: Set up Node.js
      uses: actions/setup-node@v4
      with:
        node-version: '20'
        cache: 'npm'

    - name: Install dependencies
      run: npm ci

    - name: Run linter
      run: npm run lint

    - name: Run unit tests
      run: npm run test:unit -- --coverage

    - name: Run integration tests
      run: npm run test:integration
      env:
        DATABASE_URL: postgresql://test:test@localhost:5432/test
        REDIS_URL: redis://localhost:6379

    - name: Upload coverage to Codecov
      uses: codecov/codecov-action@v4
      with:
        token: ${{ secrets.CODECOV_TOKEN }}

  security-scan:
    runs-on: ubuntu-latest
    needs: build-and-test

    steps:
    - name: Checkout code
      uses: actions/checkout@v4

    - name: Run Snyk security scan
      uses: snyk/actions/node@master
      env:
        SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}

    - name: Run Trivy vulnerability scanner
      uses: aquasecurity/trivy-action@master
      with:
        scan-type: 'fs'
        scan-ref: '.'
        format: 'sarif'
        output: 'trivy-results.sarif'

    - name: Upload Trivy results to GitHub Security
      uses: github/codeql-action/upload-sarif@v3
      with:
        sarif_file: 'trivy-results.sarif'

    - name: SonarQube scan
      uses: SonarSource/sonarcloud-github-action@master
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
        SONAR_TOKEN: ${{ secrets.SONAR_TOKEN }}

  build-and-push-docker:
    runs-on: ubuntu-latest
    needs: [build-and-test, security-scan]
    if: github.ref == 'refs/heads/main'

    steps:
    - name: Checkout code
      uses: actions/checkout@v4

    - name: Set up Docker Buildx
      uses: docker/setup-buildx-action@v3

    - name: Authenticate to Google Cloud
      uses: google-github-actions/auth@v2
      with:
        credentials_json: ${{ secrets.GCP_SA_KEY }}

    - name: Configure Docker for GCR
      run: gcloud auth configure-docker

    - name: Extract metadata
      id: meta
      uses: docker/metadata-action@v5
      with:
        images: ${{ env.DOCKER_REGISTRY }}/${{ env.SERVICE_NAME }}
        tags: |
          type=ref,event=branch
          type=sha,prefix={{branch}}-
          type=semver,pattern={{version}}

    - name: Build and push Docker image
      uses: docker/build-push-action@v5
      with:
        context: .
        push: true
        tags: ${{ steps.meta.outputs.tags }}
        labels: ${{ steps.meta.outputs.labels }}
        cache-from: type=gha
        cache-to: type=gha,mode=max
        build-args: |
          NODE_ENV=production
          BUILD_DATE=${{ github.event.head_commit.timestamp }}
          VCS_REF=${{ github.sha }}

  deploy-staging:
    runs-on: ubuntu-latest
    needs: build-and-push-docker
    environment: staging

    steps:
    - name: Checkout GitOps repo
      uses: actions/checkout@v4
      with:
        repository: WIA-Official/fintech-gitops
        token: ${{ secrets.GITOPS_TOKEN }}

    - name: Update staging manifest
      run: |
        cd overlays/staging
        kustomize edit set image \
          ${{ env.DOCKER_REGISTRY }}/${{ env.SERVICE_NAME }}:${{ github.sha }}

    - name: Commit and push
      run: |
        git config user.name "GitHub Actions"
        git config user.email "actions@github.com"
        git add .
        git commit -m "Deploy ${{ env.SERVICE_NAME }}:${{ github.sha }} to staging"
        git push

    - name: Wait for deployment
      run: |
        kubectl rollout status deployment/${{ env.SERVICE_NAME }} \
          -n fintech-staging --timeout=5m

    - name: Run smoke tests
      run: npm run test:smoke
      env:
        API_URL: https://staging.wia-fintech.io

  deploy-production:
    runs-on: ubuntu-latest
    needs: deploy-staging
    environment: production
    if: github.ref == 'refs/heads/main'

    steps:
    - name: Checkout GitOps repo
      uses: actions/checkout@v4
      with:
        repository: WIA-Official/fintech-gitops
        token: ${{ secrets.GITOPS_TOKEN }}

    - name: Update production manifest (canary)
      run: |
        cd overlays/production
        kustomize edit set image \
          ${{ env.DOCKER_REGISTRY }}/${{ env.SERVICE_NAME }}:${{ github.sha }}
        # Update canary weight to 5%
        kubectl patch virtualservice ${{ env.SERVICE_NAME }}-canary \
          --type merge -p '{"spec":{"http":[{"route":[{"destination":{"subset":"v2"},"weight":5}]}]}}'

    - name: Monitor canary metrics (15 min)
      run: |
        ./scripts/monitor-canary.sh \
          --service ${{ env.SERVICE_NAME }} \
          --duration 15m \
          --error-rate-threshold 0.01 \
          --latency-p99-threshold 500

    - name: Full rollout
      run: |
        kubectl patch virtualservice ${{ env.SERVICE_NAME }}-canary \
          --type merge -p '{"spec":{"http":[{"route":[{"destination":{"subset":"v2"},"weight":100}]}]}}'

    - name: Verify deployment
      run: |
        kubectl rollout status deployment/${{ env.SERVICE_NAME }} \
          -n fintech-prod --timeout=10m
```

### 2.3 Rollback Strategy

**Automated Rollback Triggers:**
```yaml
# Flagger configuration for automated rollback
apiVersion: flagger.app/v1beta1
kind: Canary
metadata:
  name: payment-service
  namespace: fintech-prod
spec:
  targetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: payment-service

  service:
    port: 80

  analysis:
    interval: 1m
    threshold: 5  # Number of failed checks before rollback
    maxWeight: 50  # Max canary traffic percentage
    stepWeight: 10  # Traffic increment per step

    metrics:
    - name: request-success-rate
      thresholdRange:
        min: 99  # Rollback if success rate < 99%
      interval: 1m

    - name: request-duration-p99
      thresholdRange:
        max: 500  # Rollback if P99 latency > 500ms
      interval: 1m

    - name: error-rate-5xx
      thresholdRange:
        max: 0.01  # Rollback if 5xx errors > 1%
      interval: 1m

    webhooks:
    - name: load-test
      url: http://loadtester.fintech-test/
      timeout: 5s
      metadata:
        cmd: "hey -z 1m -q 10 -c 2 http://payment-service-canary/"
```

---

## 3. Monitoring & Observability

### 3.1 Metrics Collection

**Prometheus Configuration:**
```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-config
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
      evaluation_interval: 15s
      external_labels:
        cluster: wia-fintech-prod
        region: us-east-1

    scrape_configs:
    - job_name: 'payment-service'
      kubernetes_sd_configs:
      - role: pod
        namespaces:
          names:
          - fintech-prod
      relabel_configs:
      - source_labels: [__meta_kubernetes_pod_label_app]
        action: keep
        regex: payment-service
      - source_labels: [__meta_kubernetes_pod_name]
        target_label: pod
      metric_relabel_configs:
      - source_labels: [__name__]
        regex: 'go_.*'
        action: drop  # Drop Go runtime metrics to reduce cardinality
```

**Key Performance Indicators (KPIs):**
```typescript
// Application metrics
const metrics = {
  // Request metrics
  http_requests_total: new promClient.Counter({
    name: 'http_requests_total',
    help: 'Total HTTP requests',
    labelNames: ['method', 'path', 'status']
  }),

  http_request_duration_seconds: new promClient.Histogram({
    name: 'http_request_duration_seconds',
    help: 'HTTP request duration in seconds',
    labelNames: ['method', 'path', 'status'],
    buckets: [0.01, 0.05, 0.1, 0.2, 0.5, 1, 2, 5]
  }),

  // Business metrics
  payment_transactions_total: new promClient.Counter({
    name: 'payment_transactions_total',
    help: 'Total payment transactions',
    labelNames: ['status', 'payment_method', 'currency']
  }),

  payment_amount_usd: new promClient.Histogram({
    name: 'payment_amount_usd',
    help: 'Payment amount in USD',
    labelNames: ['status', 'payment_method'],
    buckets: [10, 50, 100, 500, 1000, 5000, 10000]
  }),

  bnpl_approval_rate: new promClient.Gauge({
    name: 'bnpl_approval_rate',
    help: 'BNPL approval rate (0-1)',
    labelNames: ['plan_type']
  }),

  open_banking_consent_active: new promClient.Gauge({
    name: 'open_banking_consent_active',
    help: 'Number of active open banking consents',
    labelNames: ['bank_id']
  }),

  // SLA metrics
  payment_sla_success: new promClient.Counter({
    name: 'payment_sla_success',
    help: 'Payments meeting SLA (<300ms)',
    labelNames: ['payment_method']
  }),

  payment_sla_violation: new promClient.Counter({
    name: 'payment_sla_violation',
    help: 'Payments violating SLA (>=300ms)',
    labelNames: ['payment_method']
  })
};
```

### 3.2 Distributed Tracing

**OpenTelemetry Instrumentation:**
```typescript
import { NodeTracerProvider } from '@opentelemetry/sdk-trace-node';
import { Resource } from '@opentelemetry/resources';
import { SemanticResourceAttributes } from '@opentelemetry/semantic-conventions';
import { JaegerExporter } from '@opentelemetry/exporter-jaeger';
import { BatchSpanProcessor } from '@opentelemetry/sdk-trace-base';

const provider = new NodeTracerProvider({
  resource: new Resource({
    [SemanticResourceAttributes.SERVICE_NAME]: 'payment-service',
    [SemanticResourceAttributes.SERVICE_VERSION]: '1.2.3',
    [SemanticResourceAttributes.DEPLOYMENT_ENVIRONMENT]: 'production'
  })
});

const exporter = new JaegerExporter({
  endpoint: 'http://jaeger-collector:14268/api/traces'
});

provider.addSpanProcessor(new BatchSpanProcessor(exporter));
provider.register();

// Trace payment flow
const tracer = trace.getTracer('payment-service');

async function processPayment(paymentData) {
  const span = tracer.startSpan('processPayment', {
    attributes: {
      'payment.amount': paymentData.amount,
      'payment.currency': paymentData.currency,
      'payment.method': paymentData.method,
      'customer.id': paymentData.customerId
    }
  });

  try {
    // Step 1: Validate payment
    const validationSpan = tracer.startSpan('validatePayment', {
      parent: span
    });
    await validatePayment(paymentData);
    validationSpan.end();

    // Step 2: Charge payment method
    const chargeSpan = tracer.startSpan('chargePaymentMethod', {
      parent: span,
      attributes: {
        'payment_processor': 'stripe'
      }
    });
    const charge = await stripeClient.charges.create(paymentData);
    chargeSpan.setAttribute('charge.id', charge.id);
    chargeSpan.end();

    // Step 3: Record transaction
    const dbSpan = tracer.startSpan('database.insert', {
      parent: span,
      attributes: {
        'db.system': 'postgresql',
        'db.statement': 'INSERT INTO payments ...'
      }
    });
    await db.payments.insert({...});
    dbSpan.end();

    span.setStatus({ code: SpanStatusCode.OK });
    return charge;
  } catch (error) {
    span.recordException(error);
    span.setStatus({
      code: SpanStatusCode.ERROR,
      message: error.message
    });
    throw error;
  } finally {
    span.end();
  }
}
```

### 3.3 Logging

**Structured Logging (JSON):**
```typescript
import winston from 'winston';

const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.json()
  ),
  defaultMeta: {
    service: 'payment-service',
    version: '1.2.3',
    environment: process.env.NODE_ENV
  },
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({
      filename: '/var/log/payment-service/error.log',
      level: 'error'
    }),
    new winston.transports.File({
      filename: '/var/log/payment-service/combined.log'
    })
  ]
});

// Log examples
logger.info('Payment processed successfully', {
  payment_id: 'pmt_abc123',
  amount: 149.99,
  currency: 'USD',
  customer_id: 'cus_xyz789',
  processing_time_ms: 247,
  trace_id: 'a1b2c3d4e5f6',
  span_id: 'f6e5d4c3b2a1'
});

logger.error('Payment processing failed', {
  payment_id: 'pmt_def456',
  error: {
    type: 'CardDeclinedError',
    code: 'card_declined',
    message: 'Insufficient funds',
    decline_code: 'insufficient_funds'
  },
  customer_id: 'cus_abc123',
  trace_id: 'g7h8i9j0k1l2'
});
```

**Log Aggregation (ELK Stack):**
```yaml
# Filebeat configuration
filebeat.inputs:
- type: log
  enabled: true
  paths:
    - /var/log/payment-service/*.log
  json.keys_under_root: true
  json.add_error_key: true
  fields:
    log_type: application
  processors:
    - add_kubernetes_metadata:
        host: ${NODE_NAME}
        matchers:
        - logs_path:
            logs_path: "/var/log/containers/"

output.elasticsearch:
  hosts: ["elasticsearch:9200"]
  index: "fintech-logs-%{+yyyy.MM.dd}"
  username: "${ELASTICSEARCH_USERNAME}"
  password: "${ELASTICSEARCH_PASSWORD}"
```

### 3.4 Alerting

**Prometheus Alert Rules:**
```yaml
groups:
- name: payment-service
  interval: 30s
  rules:
  - alert: HighErrorRate
    expr: |
      (
        sum(rate(http_requests_total{status=~"5.."}[5m]))
        /
        sum(rate(http_requests_total[5m]))
      ) > 0.01
    for: 5m
    labels:
      severity: critical
      service: payment-service
    annotations:
      summary: "High error rate (>1%) in payment service"
      description: "Error rate is {{ $value | humanizePercentage }} (threshold: 1%)"

  - alert: HighLatency
    expr: |
      histogram_quantile(0.99,
        sum(rate(http_request_duration_seconds_bucket[5m])) by (le, path)
      ) > 0.5
    for: 10m
    labels:
      severity: warning
      service: payment-service
    annotations:
      summary: "High P99 latency (>500ms) in payment service"
      description: "P99 latency is {{ $value }}s on path {{ $labels.path }}"

  - alert: LowBNPLApprovalRate
    expr: |
      bnpl_approval_rate < 0.60
    for: 15m
    labels:
      severity: warning
      service: payment-service
    annotations:
      summary: "BNPL approval rate dropped below 60%"
      description: "Current approval rate: {{ $value | humanizePercentage }}"

  - alert: DatabaseConnectionPoolExhausted
    expr: |
      pg_pool_size - pg_pool_available < 5
    for: 5m
    labels:
      severity: critical
      service: payment-service
    annotations:
      summary: "Database connection pool nearly exhausted"
      description: "Only {{ $value }} connections available"
```

---

## 4. Incident Response

### 4.1 Incident Classification

| Severity | Response Time | Examples |
|----------|---------------|----------|
| **SEV1 (Critical)** | 15 minutes | Complete service outage, data breach, payment processing down |
| **SEV2 (High)** | 1 hour | Partial outage, degraded performance (>5% error rate), security vulnerability |
| **SEV3 (Medium)** | 4 hours | Minor feature issue, non-critical API degradation |
| **SEV4 (Low)** | 1 business day | Cosmetic issues, minor bugs |

### 4.2 On-Call Rotation

**PagerDuty Escalation Policy:**
```yaml
escalation_policies:
  - name: Payment Service Escalation
    escalation_rules:
      - escalation_delay_in_minutes: 0
        targets:
          - type: schedule
            id: primary_on_call_schedule

      - escalation_delay_in_minutes: 15
        targets:
          - type: schedule
            id: secondary_on_call_schedule

      - escalation_delay_in_minutes: 30
        targets:
          - type: user
            id: engineering_manager

      - escalation_delay_in_minutes: 45
        targets:
          - type: user
            id: vp_engineering
```

### 4.3 Incident Response Runbook

**Payment Processing Failure Runbook:**
```markdown
# Runbook: Payment Processing Failures

## Symptoms
- Increased 5xx errors in payment API
- Timeouts on /v1/payments endpoint
- Customer reports of failed payments

## Investigation Steps

1. **Check Service Health**
   ```bash
   kubectl get pods -n fintech-prod | grep payment-service
   kubectl logs -n fintech-prod deployment/payment-service --tail=100
   ```

2. **Check Database**
   ```bash
   # Connection pool status
   kubectl exec -it -n fintech-prod payment-service-0 -- \
     psql -c "SELECT count(*) FROM pg_stat_activity WHERE datname='fintech_prod';"

   # Long-running queries
   kubectl exec -it -n fintech-prod postgresql-0 -- \
     psql -c "SELECT pid, now() - pg_stat_activity.query_start AS duration, query
              FROM pg_stat_activity
              WHERE state = 'active' AND now() - pg_stat_activity.query_start > interval '30 seconds';"
   ```

3. **Check Third-Party Services**
   ```bash
   # Stripe API status
   curl https://status.stripe.com/api/v2/status.json

   # Check circuit breaker state
   kubectl exec -it -n fintech-prod payment-service-0 -- \
     curl localhost:8080/admin/circuit-breakers
   ```

4. **Check Metrics**
   - Open Grafana dashboard: https://grafana.wia-fintech.io/d/payment-service
   - Review error rate, latency, throughput graphs

## Mitigation Steps

1. **If database connection pool exhausted:**
   ```bash
   # Increase pool size temporarily
   kubectl set env deployment/payment-service -n fintech-prod \
     DATABASE_POOL_MAX=100
   ```

2. **If third-party service down (e.g., Stripe):**
   ```bash
   # Switch to backup processor
   kubectl set env deployment/payment-service -n fintech-prod \
     PRIMARY_PROCESSOR=adyen
   ```

3. **If service unresponsive:**
   ```bash
   # Rolling restart
   kubectl rollout restart deployment/payment-service -n fintech-prod
   ```

4. **If recent deployment caused issue:**
   ```bash
   # Rollback to previous version
   kubectl rollout undo deployment/payment-service -n fintech-prod
   ```

## Communication

- Update status page: https://status.wia-fintech.io
- Post in #incidents Slack channel
- If SEV1, notify customers via email

## Post-Incident

- File incident report: https://wia.atlassian.net/incidents
- Schedule blameless postmortem
- Create follow-up tickets for preventive measures
```

---

## 5. Operations & SRE

### 5.1 SLOs (Service Level Objectives)

| Metric | SLO | Measurement Window |
|--------|-----|-------------------|
| **Availability** | 99.99% | 30 days |
| **Latency (P99)** | < 500ms | 7 days |
| **Error Rate** | < 0.1% | 7 days |
| **Payment Success Rate** | > 99.5% | 24 hours |
| **Data Durability** | 99.999999999% | N/A |

**Error Budget:**
```
Availability SLO: 99.99%
Downtime budget per month: 0.01% × 30 days = 4.32 minutes

If error budget exhausted:
- Freeze feature deployments
- Focus on reliability improvements
- Conduct incident review
```

### 5.2 Capacity Planning

**Resource Forecasting:**
```python
# Predictive model for capacity planning
import numpy as np
from sklearn.linear_model import LinearRegression

# Historical data: monthly payment volume
months = np.array([1, 2, 3, 4, 5, 6]).reshape(-1, 1)
payment_volume = np.array([1000000, 1200000, 1500000, 1800000, 2200000, 2600000])

model = LinearRegression()
model.fit(months, payment_volume)

# Forecast next 6 months
future_months = np.array([7, 8, 9, 10, 11, 12]).reshape(-1, 1)
forecast = model.predict(future_months)

# Calculate required infrastructure
# Assumption: 100 TPS per pod, peak traffic = 5x average
for month, volume in zip(future_months.flatten(), forecast):
    avg_tps = volume / (30 * 24 * 3600)
    peak_tps = avg_tps * 5
    required_pods = int(np.ceil(peak_tps / 100))
    print(f"Month {month}: {volume:,.0f} payments, {required_pods} pods needed")

# Output:
# Month 7: 3,080,000 payments, 18 pods needed
# Month 8: 3,540,000 payments, 21 pods needed
```

### 5.3 Cost Optimization

**Right-Sizing Pods:**
```bash
# Analyze actual resource usage
kubectl top pods -n fintech-prod --containers | grep payment-service

# Recommendations based on P95 usage
# Current: requests: 1000m CPU, 2Gi memory
# Actual usage: 450m CPU (45%), 1.2Gi memory (60%)
# Recommendation: requests: 500m CPU, 1.5Gi memory

# Savings: ~40% compute cost reduction
```

**Spot/Preemptible Instances:**
```yaml
# Node pool for non-critical workloads
- name: spot-instances
  machine_type: n2-standard-8
  preemptible: true  # 70-80% cost savings
  min_nodes: 5
  max_nodes: 50

  taints:
    - key: cloud.google.com/gke-preemptible
      value: "true"
      effect: NoSchedule

  # Deploy analytics, batch jobs, dev/test environments on spot
```

---

## 6. Cost Optimization

### 6.1 Cost Breakdown (Monthly)

| Category | Service | Cost (USD) | Optimization |
|----------|---------|------------|--------------|
| **Compute** | GKE Nodes (50 nodes) | $15,000 | Spot instances for 30% → Save $4,500 |
| **Database** | Cloud SQL (PostgreSQL) | $8,000 | Right-size instances → Save $2,000 |
| **Storage** | Persistent Disks (10TB) | $1,000 | Lifecycle policies (cold storage) → Save $300 |
| **Networking** | Egress traffic (50TB) | $4,500 | CDN caching → Save $1,500 |
| **Observability** | Prometheus, Grafana, Jaeger | $2,000 | Sampling (50% traces) → Save $800 |
| **Third-Party** | Stripe fees (1M transactions) | $29,000 | Volume discounts |
| **Total** | | **$59,500** | **Potential Savings: $9,100 (15%)** |

### 6.2 Cost Allocation

```yaml
# Kubernetes cost labels
metadata:
  labels:
    cost-center: "fintech"
    team: "payments"
    environment: "production"
    application: "payment-service"

# Query costs by team
SELECT
  labels.value as team,
  SUM(cost) as total_cost
FROM billing_export
WHERE labels.key = 'team'
  AND usage_date >= '2026-01-01'
GROUP BY team
ORDER BY total_cost DESC;
```

---

## Deployment Checklist

- [ ] Load testing completed (10K TPS sustained for 1 hour)
- [ ] Security scan passed (Snyk, Trivy, SonarQube)
- [ ] Database migrations tested on staging
- [ ] Canary deployment configured (5% → 25% → 50% → 100%)
- [ ] Rollback procedure documented and tested
- [ ] Monitoring dashboards updated
- [ ] Alerts configured for new features
- [ ] Runbooks updated
- [ ] On-call engineers notified
- [ ] Change advisory board (CAB) approval obtained
- [ ] Customer communication prepared (if user-facing changes)

---

## References

1. **Site Reliability Engineering** - Google SRE Book
2. **Kubernetes** - Production Best Practices
3. **GitOps** - Argo CD, Flux CD documentation
4. **Observability** - OpenTelemetry, Prometheus, Grafana
5. **FinOps** - Cloud Cost Optimization Best Practices

---

**© 2026 WIA | 弘益人間**
