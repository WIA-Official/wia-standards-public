# WIA-FLOOD_PREDICTION Specification - PHASE 4

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## Deployment, Operations & Monitoring

This phase defines the infrastructure architecture, deployment pipelines, ML model serving, monitoring systems, and operational procedures for the WIA-FLOOD_PREDICTION system.

---

## Table of Contents

1. [Infrastructure Architecture](#infrastructure-architecture)
2. [Deployment Strategy](#deployment-strategy)
3. [ML Model Serving](#ml-model-serving)
4. [Data Pipeline Operations](#data-pipeline-operations)
5. [Monitoring & Observability](#monitoring--observability)
6. [Incident Response](#incident-response)
7. [Operational Runbooks](#operational-runbooks)

---

## Infrastructure Architecture

### Cloud Architecture (Multi-Cloud)

```
┌─────────────────────────────────────────────────────────────────┐
│  CLOUDFLARE CDN + DDoS Protection                               │
│  - Global edge caching    - Bot protection    - SSL/TLS         │
└────────────────────────────┬────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│  AWS Route 53 (DNS) + Application Load Balancer                 │
└────────────────────────────┬────────────────────────────────────┘
                             │
                ┌────────────┴────────────┐
                │                         │
┌───────────────▼────────────┐  ┌────────▼───────────────────────┐
│  AWS EKS (us-east-1)       │  │  GCP GKE (europe-west1)        │
│  Primary Region            │  │  Secondary Region (DR)         │
│                            │  │                                │
│  ┌──────────────────────┐ │  │  ┌──────────────────────┐     │
│  │ API Gateway (Kong)   │ │  │  │ API Gateway (Kong)   │     │
│  └──────────┬───────────┘ │  │  └──────────┬───────────┘     │
│             │             │  │             │                 │
│  ┌──────────▼───────────┐ │  │  ┌──────────▼───────────┐     │
│  │ Prediction Service   │ │  │  │ Prediction Service   │     │
│  │ (FastAPI + uvicorn)  │ │  │  │ (FastAPI + uvicorn)  │     │
│  │ - 20 pods (autoscale)│ │  │  │ - 20 pods (autoscale)│     │
│  └──────────┬───────────┘ │  │  └──────────┬───────────┘     │
│             │             │  │             │                 │
│  ┌──────────▼───────────┐ │  │  ┌──────────▼───────────┐     │
│  │ ML Model Servers     │ │  │  │ ML Model Servers     │     │
│  │ (NVIDIA Triton)      │ │  │  │ (NVIDIA Triton)      │     │
│  │ - LSTM model         │ │  │  │ - LSTM model         │     │
│  │ - CNN model          │ │  │  │ - CNN model          │     │
│  │ - Ensemble           │ │  │  │ - Ensemble           │     │
│  │ GPU: 4x NVIDIA A100  │ │  │  │ GPU: 4x NVIDIA A100  │     │
│  └──────────┬───────────┘ │  │  └──────────┬───────────┘     │
└─────────────┼─────────────┘  └─────────────┼─────────────────┘
              │                               │
┌─────────────▼───────────────────────────────▼─────────────────┐
│  Data Layer (Multi-Region Replication)                         │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐ │
│  │ PostgreSQL   │  │ Redis Cache  │  │ S3 (Satellite Data) │ │
│  │ RDS Multi-AZ │  │ ElastiCache  │  │ Cross-region sync   │ │
│  │ Primary: us  │  │ 6 nodes      │  │ 50TB capacity       │ │
│  │ Replica: eu  │  │              │  │                     │ │
│  └──────────────┘  └──────────────┘  └─────────────────────┘ │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  Data Ingestion Layer (Apache Airflow on EKS)                  │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐ │
│  │ Sentinel-1/2   │  │ NOAA GFS       │  │ USGS Gauges     │ │
│  │ Downloader     │  │ Ingestor       │  │ Poller          │ │
│  │ (every 6h)     │  │ (every 6h)     │  │ (every 15min)   │ │
│  └────────────────┘  └────────────────┘  └─────────────────┘ │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  Monitoring & Logging                                           │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐ │
│  │ Prometheus     │  │ Grafana        │  │ ELK Stack       │ │
│  │ (metrics)      │  │ (dashboards)   │  │ (logs)          │ │
│  └────────────────┘  └────────────────┘  └─────────────────┘ │
└────────────────────────────────────────────────────────────────┘
```

### Compute Resources

#### Production Cluster (AWS EKS)

**Kubernetes Version**: 1.28

**Node Groups**:

1. **API Nodes** (CPU-optimized):
   - Instance Type: `c6i.2xlarge` (8 vCPU, 16GB RAM)
   - Count: 10 nodes (min 5, max 20)
   - Auto-scaling: Based on CPU/memory utilization

2. **GPU Nodes** (ML inference):
   - Instance Type: `p4d.24xlarge` (96 vCPU, 1152GB RAM, 8x NVIDIA A100 80GB)
   - Count: 2 nodes (min 1, max 4)
   - Cost: ~$32/hour per node

3. **Data Processing Nodes** (memory-optimized):
   - Instance Type: `r6i.4xlarge` (16 vCPU, 128GB RAM)
   - Count: 5 nodes (min 3, max 10)
   - Purpose: Satellite imagery preprocessing (GDAL)

**Total Cost**: ~$15,000/month (baseline), ~$30,000/month (peak)

### Storage

| Type | Service | Capacity | Redundancy | Cost |
|------|---------|----------|------------|------|
| **Database** | AWS RDS PostgreSQL 15 | 2TB (IOPS: 10,000) | Multi-AZ | $1,200/mo |
| **Cache** | AWS ElastiCache Redis | 128GB | 6 nodes | $800/mo |
| **Object Storage** | AWS S3 (Intelligent-Tiering) | 50TB | Cross-region | $1,000/mo |
| **Time Series** | InfluxDB (self-hosted) | 5TB | HA cluster | $300/mo |
| **Backup** | AWS S3 Glacier | 100TB | Multi-region | $400/mo |

### Networking

- **VPC**: 10.0.0.0/16 (65,536 IPs)
- **Subnets**:
  - Public (3 AZs): 10.0.1.0/24, 10.0.2.0/24, 10.0.3.0/24
  - Private (3 AZs): 10.0.11.0/24, 10.0.12.0/24, 10.0.13.0/24
  - Data (3 AZs): 10.0.21.0/24, 10.0.22.0/24, 10.0.23.0/24
- **NAT Gateway**: 3 (one per AZ) for outbound traffic
- **VPC Peering**: AWS ↔ GCP for disaster recovery
- **Direct Connect**: 10 Gbps link to NOAA for weather data

---

## Deployment Strategy

### CI/CD Pipeline

**Tools**: GitHub Actions + ArgoCD

**Workflow**:

```
┌──────────────┐
│ Git Push     │
│ (main branch)│
└──────┬───────┘
       │
┌──────▼───────────────────────────────────────┐
│ GitHub Actions (CI)                          │
│  1. Lint (ruff, pylint)                      │
│  2. Unit tests (pytest, coverage >80%)       │
│  3. Integration tests (Docker Compose)       │
│  4. Build Docker images                      │
│  5. Scan images (Trivy, Snyk)                │
│  6. Push to AWS ECR                          │
│  7. Update Helm chart versions               │
└──────┬───────────────────────────────────────┘
       │
┌──────▼───────────────────────────────────────┐
│ ArgoCD (CD)                                  │
│  1. Detect new Helm chart version            │
│  2. Deploy to Staging (auto)                 │
│  3. Run smoke tests                          │
│  4. Wait for approval (manual gate)          │
│  5. Deploy to Production (canary)            │
│  6. Monitor metrics (5 min)                  │
│  7. Rollout or Rollback                      │
└──────────────────────────────────────────────┘
```

**Deployment Frequency**: 3-5 times per week

**Rollback Time**: <5 minutes (automated)

### Canary Deployment

**Strategy**: Progressive rollout with traffic shifting

**Steps**:
1. Deploy new version to 10% of pods (2 out of 20)
2. Monitor for 5 minutes:
   - Error rate <1%
   - P95 latency <500ms
   - No ML model accuracy degradation
3. If healthy: Shift to 50% (10 pods) → Wait 5 min
4. If healthy: Shift to 100% (20 pods)
5. If unhealthy at any stage: Automatic rollback

**Configuration** (Argo Rollouts):
```yaml
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: prediction-service
spec:
  replicas: 20
  strategy:
    canary:
      steps:
        - setWeight: 10
        - pause: {duration: 5m}
        - setWeight: 50
        - pause: {duration: 5m}
        - setWeight: 100
      trafficRouting:
        istio:
          virtualService:
            name: prediction-service
      analysis:
        templates:
          - templateName: error-rate
          - templateName: latency
        startingStep: 1
```

### Blue-Green Deployment (ML Models)

**Purpose**: Zero-downtime model updates

**Process**:
1. Train new model (v2.2) on GPU cluster
2. Validate accuracy on holdout set (must be ≥85%)
3. Deploy to "green" environment (parallel to "blue" v2.1)
4. Route 5% of inference traffic to green
5. Compare predictions: If accuracy drop <2% → Switch
6. Update routing: 100% traffic to green (v2.2)
7. Keep blue (v2.1) for 24 hours, then decommission

---

## ML Model Serving

### NVIDIA Triton Inference Server

**Architecture**:

```
┌─────────────────────────────────────────────────────┐
│  Triton Inference Server (GPU Pod)                  │
│  ┌─────────────────────────────────────────────┐   │
│  │ Model Repository (S3 Mount)                 │   │
│  │  - lstm_model/2.1/model.savedmodel         │   │
│  │  - cnn_model/1.8/model.savedmodel          │   │
│  │  - ensemble/1.0/config.pbtxt               │   │
│  └─────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────┐   │
│  │ Triton Server Process                       │   │
│  │  - gRPC endpoint (port 8001)                │   │
│  │  - HTTP endpoint (port 8000)                │   │
│  │  - Metrics endpoint (port 8002)             │   │
│  └─────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────┐   │
│  │ GPU: NVIDIA A100 80GB                       │   │
│  │  - Concurrent requests: 64                  │   │
│  │  - Batch size: 32 (dynamic batching)        │   │
│  │  - Mixed precision (FP16)                   │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

**Model Configuration** (ensemble):
```protobuf
name: "flood_prediction_ensemble"
platform: "ensemble"
max_batch_size: 32

input [
  {
    name: "time_series_input"
    data_type: TYPE_FP32
    dims: [30, 8]  # 30 days, 8 features
  },
  {
    name: "spatial_input"
    data_type: TYPE_FP32
    dims: [256, 256, 8]  # 256x256 image, 8 channels
  }
]

output [
  {
    name: "flood_probability"
    data_type: TYPE_FP32
    dims: [7]  # 7-day forecast
  },
  {
    name: "flood_depth_map"
    data_type: TYPE_FP32
    dims: [256, 256]
  }
]

ensemble_scheduling {
  step [
    {
      model_name: "lstm_model"
      model_version: -1  # Latest
      input_map { key: "input" value: "time_series_input" }
      output_map { key: "output" value: "lstm_prediction" }
    },
    {
      model_name: "cnn_model"
      model_version: -1
      input_map { key: "input" value: "spatial_input" }
      output_map { key: "output" value: "cnn_prediction" }
    },
    {
      model_name: "ensemble_fusion"
      input_map { key: "lstm_input" value: "lstm_prediction" }
      input_map { key: "cnn_input" value: "cnn_prediction" }
      output_map { key: "probability" value: "flood_probability" }
      output_map { key: "depth_map" value: "flood_depth_map" }
    }
  ]
}

instance_group [
  {
    count: 2
    kind: KIND_GPU
    gpus: [0]
  }
]

dynamic_batching {
  preferred_batch_size: [8, 16, 32]
  max_queue_delay_microseconds: 5000
}
```

**Performance**:
- **Throughput**: 500 predictions/second (per GPU)
- **Latency**: 12ms (P50), 25ms (P99)
- **GPU Utilization**: 75-85%

### Model Registry (MLflow)

**Purpose**: Version control for ML models

**Metadata Stored**:
- Model artifacts (TensorFlow SavedModel format)
- Training metrics (accuracy, loss, F1 score)
- Hyperparameters
- Training dataset ID
- Validation results

**Example Model Log**:
```python
import mlflow
import tensorflow as tf

with mlflow.start_run():
    # Train model
    model = train_lstm_model(X_train, y_train)

    # Log parameters
    mlflow.log_param("lstm_units", 128)
    mlflow.log_param("dropout", 0.2)
    mlflow.log_param("learning_rate", 0.001)

    # Log metrics
    mlflow.log_metric("accuracy", 0.87)
    mlflow.log_metric("f1_score", 0.83)
    mlflow.log_metric("val_loss", 0.12)

    # Log model
    mlflow.tensorflow.log_model(
        model,
        "lstm_model",
        registered_model_name="flood_lstm",
        signature=mlflow.models.signature.infer_signature(X_train, y_train)
    )
```

### Model Retraining Pipeline

**Frequency**: Monthly (automated), or triggered by accuracy drop

**Pipeline** (Apache Airflow DAG):

```python
from airflow import DAG
from airflow.operators.python import PythonOperator
from datetime import datetime, timedelta

dag = DAG(
    'flood_model_retraining',
    start_date=datetime(2026, 1, 1),
    schedule_interval='@monthly',
    catchup=False
)

def fetch_new_data():
    # Download last 30 days of observed flood events
    # Pull satellite imagery, weather data, gauge readings
    pass

def preprocess_data():
    # Clean, normalize, augment data
    pass

def train_lstm():
    # Train LSTM model on new + historical data
    pass

def train_cnn():
    # Train CNN model
    pass

def validate_models():
    # Evaluate on holdout set
    # If accuracy < 85%, alert team and abort
    pass

def deploy_to_staging():
    # Push models to staging Triton server
    pass

def integration_test():
    # Run end-to-end prediction tests
    pass

def promote_to_production():
    # Blue-green deployment to prod
    pass

fetch_task = PythonOperator(task_id='fetch_data', python_callable=fetch_new_data, dag=dag)
preprocess_task = PythonOperator(task_id='preprocess', python_callable=preprocess_data, dag=dag)
train_lstm_task = PythonOperator(task_id='train_lstm', python_callable=train_lstm, dag=dag)
train_cnn_task = PythonOperator(task_id='train_cnn', python_callable=train_cnn, dag=dag)
validate_task = PythonOperator(task_id='validate', python_callable=validate_models, dag=dag)
deploy_staging_task = PythonOperator(task_id='deploy_staging', python_callable=deploy_to_staging, dag=dag)
test_task = PythonOperator(task_id='integration_test', python_callable=integration_test, dag=dag)
promote_task = PythonOperator(task_id='promote_to_prod', python_callable=promote_to_production, dag=dag)

fetch_task >> preprocess_task >> [train_lstm_task, train_cnn_task] >> validate_task >> deploy_staging_task >> test_task >> promote_task
```

---

## Data Pipeline Operations

### Satellite Data Ingestion

**Airflow DAG**: `sentinel_downloader`

**Schedule**: Every 6 hours (00:00, 06:00, 12:00, 18:00 UTC)

**Steps**:
1. Query Copernicus Hub API for new Sentinel-1/2 products
   - Area of Interest (AOI): Global, prioritize high-risk zones
   - Date range: Last 12 hours
   - Cloud cover: <30% (Sentinel-2 only)
2. Download products (parallel, 5 concurrent downloads)
3. Store raw data in S3: `s3://wia-flood-data/raw/sentinel-1/2026/01/11/`
4. Trigger preprocessing pipeline

**Error Handling**:
- Retry 3 times with exponential backoff
- If Copernicus Hub is down: Use Google Earth Engine as fallback
- Alert on-call engineer if data gap > 24 hours

### Weather Data Ingestion

**Airflow DAG**: `noaa_gfs_ingestor`

**Schedule**: Every 6 hours (synced with NOAA GFS release)

**Steps**:
1. Download NOAA GFS GRIB2 files (precipitation, temperature, humidity)
   - Resolution: 0.25° (~28km)
   - Forecast horizon: 16 days
   - Variables: APCP (precipitation), TMP (temperature), RH (humidity)
2. Convert GRIB2 to NetCDF using `wgrib2`
3. Extract points of interest (POIs) near rivers/urban areas
4. Store in InfluxDB (time series database)

**Data Volume**: ~5GB per forecast cycle

### River Gauge Polling

**Service**: `gauge_poller` (Go microservice)

**Schedule**: Every 15 minutes

**Steps**:
1. Query USGS Water Services API for 9,000+ gauges
2. Parse JSON response (stage, discharge, timestamp)
3. Write to InfluxDB and PostgreSQL
4. Check for anomalies:
   - Level increase >0.5m in 15 min → Alert
   - Missing data >1 hour → Alert

**Example USGS API Call**:
```bash
curl "https://waterservices.usgs.gov/nwis/iv/?format=json&sites=01646500,01647850&parameterCd=00065,00060&siteStatus=all"
```

### Data Preprocessing Pipeline

**Trigger**: New satellite imagery or weather data

**Pipeline** (runs on data processing nodes):

1. **Sentinel-1 Preprocessing** (~10 minutes):
   ```bash
   gpt Apply-Orbit-File -Ssource=S1A_IW_GRDH.zip -Toutput=orbit_corrected.dim
   gpt Calibration -Ssource=orbit_corrected.dim -Toutput=calibrated.dim
   gpt Terrain-Correction -Ssource=calibrated.dim -Toutput=terrain_corrected.tif
   ```

2. **Water Mask Extraction** (~5 minutes):
   ```python
   import rasterio
   import numpy as np

   with rasterio.open('terrain_corrected.tif') as src:
       vh_band = src.read(2)  # VH polarization
       water_mask = (vh_band < -20)  # Threshold

       # Save as GeoTIFF
       with rasterio.open('water_mask.tif', 'w', **profile) as dst:
           dst.write(water_mask.astype(np.uint8), 1)
   ```

3. **Feature Engineering** (~15 minutes):
   - Calculate NDWI from Sentinel-2
   - Compute precipitation accumulation (3, 7, 14 days)
   - Extract distance to river using OSM data
   - Generate terrain features (slope, aspect, TWI)

4. **Store in Feature Store** (Feast):
   - Versioned feature sets
   - Low-latency serving for ML inference

---

## Monitoring & Observability

### Metrics Collection (Prometheus)

**Exporters**:
- **Kubernetes Metrics**: `kube-state-metrics`
- **Node Metrics**: `node_exporter`
- **API Metrics**: Custom FastAPI exporter
- **ML Model Metrics**: Triton metrics endpoint
- **Database Metrics**: `postgres_exporter`, `redis_exporter`

**Key Metrics**:

| Metric | Type | Description | Alert Threshold |
|--------|------|-------------|-----------------|
| `api_request_duration_seconds` | Histogram | API response time | P95 > 500ms |
| `api_request_total` | Counter | Total requests | - |
| `api_errors_total` | Counter | Error count | Rate > 5% |
| `prediction_accuracy` | Gauge | Current model accuracy | < 85% |
| `prediction_latency_ms` | Histogram | ML inference time | P99 > 100ms |
| `satellite_download_failures` | Counter | Failed downloads | > 3 in 1 hour |
| `usgs_gauge_missing_data` | Counter | Gauges with stale data | > 50 gauges |
| `database_connections` | Gauge | Active connections | > 80% of max |
| `gpu_utilization_percent` | Gauge | GPU usage | > 95% for 10 min |

**Prometheus Configuration** (excerpt):
```yaml
scrape_configs:
  - job_name: 'prediction-service'
    kubernetes_sd_configs:
      - role: pod
        namespaces:
          names: ['flood-prediction']
    relabel_configs:
      - source_labels: [__meta_kubernetes_pod_label_app]
        regex: prediction-service
        action: keep
    scrape_interval: 15s

  - job_name: 'triton'
    static_configs:
      - targets: ['triton-server:8002']
    scrape_interval: 10s
```

### Dashboards (Grafana)

**1. System Health Dashboard**:
- Cluster CPU/memory utilization
- Pod restart count
- Network I/O
- Disk usage

**2. API Performance Dashboard**:
- Request rate (QPS)
- Response time (P50, P95, P99)
- Error rate by endpoint
- Geographic distribution of requests

**3. ML Model Dashboard**:
- Inference throughput
- Model accuracy (daily validation)
- Prediction confidence distribution
- Model version tracking

**4. Data Pipeline Dashboard**:
- Satellite imagery download status
- Weather data ingestion lag
- River gauge data freshness
- Preprocessing job durations

**5. Business Metrics Dashboard**:
- Active alert subscriptions
- Predictions generated (daily)
- Flood events detected
- API usage by tier (Free, Developer, Pro, Enterprise)

### Logging (ELK Stack)

**Components**:
- **Elasticsearch**: Log storage and search
- **Logstash**: Log aggregation and parsing
- **Kibana**: Log visualization
- **Filebeat**: Log shipper (on each Kubernetes node)

**Log Levels**:
- `DEBUG`: Detailed diagnostic info (not in prod)
- `INFO`: General informational messages
- `WARNING`: Unexpected but handled (e.g., retries)
- `ERROR`: Failures requiring attention
- `CRITICAL`: System-wide failures

**Structured Logging** (JSON format):
```json
{
  "timestamp": "2026-01-11T14:30:15.234Z",
  "level": "INFO",
  "service": "prediction-service",
  "pod": "prediction-service-7d8f6c9b4-x2j9k",
  "request_id": "req_abc123",
  "user_id": "user_12345",
  "endpoint": "/v1/predictions",
  "method": "GET",
  "status_code": 200,
  "response_time_ms": 145,
  "message": "Prediction generated successfully"
}
```

**Log Retention**:
- Hot tier (Elasticsearch): 30 days
- Warm tier (S3): 90 days
- Cold tier (S3 Glacier): 1 year

### Distributed Tracing (Jaeger)

**Purpose**: Trace requests across microservices

**Example Trace**:
```
GET /v1/predictions?lat=38.9&lng=-77.0
  ├─ API Gateway (Kong)           [5ms]
  ├─ Prediction Service            [140ms]
  │   ├─ Feature Service           [20ms]
  │   │   ├─ PostgreSQL query      [8ms]
  │   │   └─ Redis cache miss      [3ms]
  │   ├─ Triton LSTM inference     [40ms]
  │   ├─ Triton CNN inference      [60ms]
  │   └─ Ensemble fusion           [20ms]
  └─ Response formatting           [5ms]
Total: 145ms
```

**Instrumentation** (OpenTelemetry):
```python
from opentelemetry import trace
from opentelemetry.instrumentation.fastapi import FastAPIInstrumentor

app = FastAPI()
FastAPIInstrumentor.instrument_app(app)

@app.get("/v1/predictions")
async def get_predictions(lat: float, lng: float):
    tracer = trace.get_tracer(__name__)

    with tracer.start_as_current_span("fetch_features"):
        features = await feature_service.get_features(lat, lng)

    with tracer.start_as_current_span("ml_inference"):
        prediction = await triton_client.infer(features)

    return prediction
```

---

## Incident Response

### On-Call Rotation

**Schedule**: 24/7 coverage, 1-week rotations

**Tiers**:
1. **Tier 1**: SRE on-call (first responder)
2. **Tier 2**: Senior SRE (escalation)
3. **Tier 3**: Engineering manager + CTO (critical outages)

**Escalation Policy**:
- **SEV-1** (Critical): Immediate page to Tier 1+2
- **SEV-2** (High): Page Tier 1, email Tier 2
- **SEV-3** (Medium): Ticket only (during business hours)

### Incident Severity Definitions

| Severity | Definition | Example | Response Time | Resolution Time |
|----------|------------|---------|---------------|-----------------|
| **SEV-1** | System down, critical functionality unavailable | API returning 500, no predictions generated | <15 minutes | <2 hours |
| **SEV-2** | Major feature broken, workaround available | Satellite data pipeline failing, using cached data | <1 hour | <8 hours |
| **SEV-3** | Minor issue, minimal customer impact | Single gauge offline, alerting delayed by 5 min | <4 hours | <48 hours |
| **SEV-4** | Cosmetic issue, no functional impact | Dashboard typo, log formatting issue | Best effort | N/A |

### Incident Response Workflow

1. **Detection**: Alert fires (PagerDuty, Opsgenie)
2. **Acknowledgment**: On-call engineer acks within 5 min
3. **Investigation**: Check runbooks, logs, metrics
4. **Communication**: Update status page, notify customers
5. **Mitigation**: Apply fix or rollback
6. **Verification**: Confirm system restored
7. **Post-mortem**: Write incident report (within 48 hours)

**Status Page**: `https://status.wia-flood.org` (powered by StatusPage.io)

---

## Operational Runbooks

### Runbook 1: API High Error Rate

**Trigger**: Error rate > 5% for 5 minutes

**Steps**:
1. Check Grafana "API Performance" dashboard
2. Identify failing endpoint: `kubectl logs -l app=prediction-service | grep ERROR`
3. Common causes:
   - Database connection pool exhausted → Scale up `max_connections`
   - ML model server timeout → Check Triton logs, restart if needed
   - External API down (USGS, NOAA) → Use cached data fallback
4. If unable to resolve in 15 min → Rollback to previous version
5. Update status page with incident details

### Runbook 2: Satellite Data Pipeline Failure

**Trigger**: No new Sentinel imagery downloaded in 12 hours

**Steps**:
1. Check Airflow DAG: `airflow dags list-runs -d sentinel_downloader`
2. If task failed:
   - **Copernicus Hub down**: Use Google Earth Engine fallback
   - **Network timeout**: Increase retry count, check NAT gateway
   - **Authentication error**: Rotate OAuth token in Vault
3. Manually trigger backfill: `airflow dags backfill -s 2026-01-10 -e 2026-01-11 sentinel_downloader`
4. Alert team if data gap > 24 hours (affects prediction accuracy)

### Runbook 3: Database Connection Pool Exhausted

**Trigger**: `database_connections > 80%` and API latency spiking

**Steps**:
1. Identify slow queries: `SELECT * FROM pg_stat_activity WHERE state='active' AND query_start < NOW() - INTERVAL '30 seconds';`
2. Kill long-running queries: `SELECT pg_terminate_backend(pid);`
3. Increase connection pool: Edit `DATABASE_MAX_CONNECTIONS` in ConfigMap, restart pods
4. Long-term fix: Optimize slow queries (add indexes, rewrite)

### Runbook 4: GPU Out of Memory (OOM)

**Trigger**: Triton pod restarting, `gpu_memory_usage=100%`

**Steps**:
1. Check batch size: `kubectl logs triton-server | grep "batch size"`
2. Reduce dynamic batching: Edit `config.pbtxt`, set `max_batch_size=16` (was 32)
3. Enable model instance sharing: Set `instance_group.count=1` (was 2)
4. If still OOM: Upgrade to larger GPU (A100 80GB → H100 80GB)

### Runbook 5: Prediction Accuracy Drop

**Trigger**: Daily validation shows accuracy < 85% for 3 consecutive days

**Steps**:
1. Investigate data quality:
   - Check if satellite imagery has gaps
   - Verify USGS gauges are reporting correctly
   - Confirm weather forecasts are recent
2. Validate model integrity:
   - Compare predictions to observed events (false positives/negatives)
   - Check for model drift (feature distribution changes)
3. If data is good but model is bad → Trigger retraining pipeline
4. Notify ML team for deep dive analysis

---

## Cost Optimization

### Current Monthly Costs

| Category | Service | Cost |
|----------|---------|------|
| **Compute** | EKS cluster (10 nodes) | $3,500 |
| **GPU** | 2x p4d.24xlarge (24/7) | $15,360 |
| **Database** | RDS PostgreSQL | $1,200 |
| **Cache** | ElastiCache Redis | $800 |
| **Storage** | S3 (50TB) | $1,000 |
| **Networking** | Data transfer (5TB/mo) | $450 |
| **Monitoring** | Datadog / New Relic | $1,200 |
| **CDN** | Cloudflare | $200 |
| **Other** | DNS, backups, etc. | $500 |
| **Total** | | **$24,210/month** |

### Optimization Strategies

1. **Spot Instances** for data processing nodes: Save 70% ($1,200/mo)
2. **GPU right-sizing**: Use A100 40GB instead of 80GB: Save 25% ($3,800/mo)
3. **S3 Intelligent-Tiering**: Automatic archiving: Save 40% ($400/mo)
4. **Reserved Instances** (1-year): Save 30% on compute: Save $1,050/mo

**Optimized Monthly Cost**: ~$17,760/month (27% reduction)

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
