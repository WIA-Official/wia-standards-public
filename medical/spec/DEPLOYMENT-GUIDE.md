# WIA Medical Device Accessibility: Deployment Guide

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Phase 4 - Ecosystem Integration
- **Standard**: WIA-MED-DEPLOY-001

## 1. Overview

This guide provides comprehensive deployment instructions for implementing WIA Medical Device Accessibility standards in production environments.

### 1.1 Deployment Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WIA MEDICAL DEPLOYMENT ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         Production Layer                             │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │    │
│  │  │ Load Balancer│  │   API GW    │  │   CDN      │                 │    │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                 │    │
│  └─────────┼────────────────┼────────────────┼──────────────────────────┘    │
│            │                │                │                               │
│  ┌─────────┼────────────────┼────────────────┼──────────────────────────┐    │
│  │         ▼                ▼                ▼     Application Layer    │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │    │
│  │  │  Profile    │  │   Device    │  │   Alert     │                 │    │
│  │  │  Service    │  │   Service   │  │   Service   │                 │    │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                 │    │
│  └─────────┼────────────────┼────────────────┼──────────────────────────┘    │
│            │                │                │                               │
│  ┌─────────┼────────────────┼────────────────┼──────────────────────────┐    │
│  │         ▼                ▼                ▼     Data Layer           │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │    │
│  │  │  PostgreSQL │  │    Redis    │  │   S3/Blob   │                 │    │
│  │  │  (Profiles) │  │   (Cache)   │  │  (Assets)   │                 │    │
│  │  └─────────────┘  └─────────────┘  └─────────────┘                 │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  弘益人間 (홍익인간) - Accessible Deployment for All                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Prerequisites

### 2.1 System Requirements

```yaml
infrastructure:
  compute:
    minimum:
      cpu: 4 cores
      memory: 8GB RAM
      storage: 100GB SSD
    recommended:
      cpu: 8+ cores
      memory: 16GB+ RAM
      storage: 500GB SSD
    high_availability:
      nodes: 3+
      regions: 2+

  network:
    bandwidth: 1Gbps minimum
    latency: <50ms to users
    ssl_tls: TLS 1.3 required

  database:
    postgresql: "14+"
    redis: "7+"
    backup: automated daily

software:
  runtime:
    rust: "1.75+"
    node: "20 LTS"
    python: "3.11+" (optional)

  containers:
    docker: "24+"
    kubernetes: "1.28+"

  monitoring:
    prometheus: required
    grafana: recommended
    jaeger: recommended
```

### 2.2 Security Requirements

```yaml
security_requirements:
  authentication:
    - OAuth 2.0 / OIDC
    - API key management
    - mTLS for service-to-service

  encryption:
    at_rest: AES-256
    in_transit: TLS 1.3
    key_management: HSM or KMS

  compliance:
    healthcare:
      - HIPAA (US)
      - GDPR (EU)
      - PIPEDA (Canada)
      - PIPA (Korea)
    accessibility:
      - WCAG 2.1 AA
      - Section 508
      - ADA
    medical_device:
      - FDA 21 CFR Part 11
      - IEC 62304
```

---

## 3. Installation

### 3.1 Rust SDK Installation

```bash
# Add to Cargo.toml
[dependencies]
wia-medical = "0.1.0"
tokio = { version = "1.0", features = ["full"] }
serde = { version = "1.0", features = ["derive"] }

# Or install from git
[dependencies]
wia-medical = { git = "https://github.com/WIA-Official/wia-standards", branch = "main" }
```

### 3.2 Docker Deployment

```dockerfile
# Dockerfile for WIA Medical Service
FROM rust:1.75-slim as builder

WORKDIR /app
COPY . .

RUN cargo build --release

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    ca-certificates \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/target/release/wia-medical-service /usr/local/bin/

# Create non-root user
RUN useradd -r -s /bin/false wia
USER wia

EXPOSE 8080
EXPOSE 8443

CMD ["wia-medical-service"]
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  wia-medical-api:
    build: .
    ports:
      - "8080:8080"
      - "8443:8443"
    environment:
      - DATABASE_URL=postgres://wia:password@db:5432/wia_medical
      - REDIS_URL=redis://redis:6379
      - WIA_CLOUD_API_KEY=${WIA_CLOUD_API_KEY}
      - RUST_LOG=info
    depends_on:
      - db
      - redis
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  db:
    image: postgres:14
    environment:
      POSTGRES_USER: wia
      POSTGRES_PASSWORD: password
      POSTGRES_DB: wia_medical
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init.sql:/docker-entrypoint-initdb.d/init.sql
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U wia -d wia_medical"]
      interval: 10s
      timeout: 5s
      retries: 5

  redis:
    image: redis:7-alpine
    command: redis-server --appendonly yes
    volumes:
      - redis_data:/data

volumes:
  postgres_data:
  redis_data:
```

### 3.3 Kubernetes Deployment

```yaml
# kubernetes/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: wia-medical
  labels:
    name: wia-medical
    compliance: hipaa

---
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-medical-api
  namespace: wia-medical
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-medical-api
  template:
    metadata:
      labels:
        app: wia-medical-api
    spec:
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
      containers:
        - name: api
          image: wia/medical-api:latest
          ports:
            - containerPort: 8080
            - containerPort: 8443
          env:
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: wia-secrets
                  key: database-url
            - name: REDIS_URL
              valueFrom:
                secretKeyRef:
                  name: wia-secrets
                  key: redis-url
          resources:
            requests:
              memory: "256Mi"
              cpu: "250m"
            limits:
              memory: "512Mi"
              cpu: "500m"
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 30
            periodSeconds: 10
          readinessProbe:
            httpGet:
              path: /ready
              port: 8080
            initialDelaySeconds: 5
            periodSeconds: 5
          volumeMounts:
            - name: config
              mountPath: /etc/wia
              readOnly: true
      volumes:
        - name: config
          configMap:
            name: wia-config

---
# kubernetes/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: wia-medical-api
  namespace: wia-medical
spec:
  selector:
    app: wia-medical-api
  ports:
    - name: http
      port: 80
      targetPort: 8080
    - name: https
      port: 443
      targetPort: 8443
  type: ClusterIP

---
# kubernetes/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: wia-medical-ingress
  namespace: wia-medical
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
    - hosts:
        - api.medical.wia.live
      secretName: wia-medical-tls
  rules:
    - host: api.medical.wia.live
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: wia-medical-api
                port:
                  number: 80
```

---

## 4. Configuration

### 4.1 Application Configuration

```toml
# config/production.toml

[server]
host = "0.0.0.0"
port = 8080
tls_port = 8443
tls_cert = "/etc/wia/tls/cert.pem"
tls_key = "/etc/wia/tls/key.pem"

[database]
url = "${DATABASE_URL}"
pool_size = 20
connection_timeout = 30

[redis]
url = "${REDIS_URL}"
pool_size = 10

[wia_cloud]
api_endpoint = "https://api.wia.live/v1"
api_key = "${WIA_CLOUD_API_KEY}"
sync_interval = 60  # seconds

[accessibility]
default_font_size = 16
default_contrast = "high"
default_modality = "multi"
emergency_escalation_timeout = 30  # seconds

[logging]
level = "info"
format = "json"
output = "stdout"

[metrics]
enabled = true
endpoint = "/metrics"
port = 9090

[tracing]
enabled = true
jaeger_endpoint = "http://jaeger:14268/api/traces"
sample_rate = 0.1

[security]
cors_origins = ["https://medical.wia.live", "https://app.wia.live"]
rate_limit_rps = 100
rate_limit_burst = 200
```

### 4.2 Environment Variables

```bash
# .env.production

# Database
DATABASE_URL=postgres://wia:${DB_PASSWORD}@db.wia.live:5432/wia_medical?sslmode=require

# Redis
REDIS_URL=redis://:${REDIS_PASSWORD}@redis.wia.live:6379/0

# WIA Cloud
WIA_CLOUD_API_KEY=${WIA_API_KEY}
WIA_CLOUD_REGION=us-east-1

# Security
JWT_SECRET=${JWT_SECRET}
ENCRYPTION_KEY=${ENCRYPTION_KEY}

# Logging
RUST_LOG=wia_medical=info,tower_http=debug
LOG_FORMAT=json

# Feature Flags
ENABLE_FHIR_INTEGRATION=true
ENABLE_BIONIC_EYE_SUPPORT=true
ENABLE_EXOSKELETON_SUPPORT=true
```

### 4.3 Database Schema

```sql
-- migrations/001_initial.sql

-- Profiles table
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    wia_id VARCHAR(255) UNIQUE NOT NULL,
    profile_data JSONB NOT NULL,
    accessibility_score DECIMAL(5,2),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    sync_version BIGINT DEFAULT 1
);

CREATE INDEX idx_user_profiles_wia_id ON user_profiles(wia_id);
CREATE INDEX idx_user_profiles_updated ON user_profiles(updated_at);

-- Device profiles table
CREATE TABLE device_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    device_id VARCHAR(255) UNIQUE NOT NULL,
    manufacturer VARCHAR(255) NOT NULL,
    model VARCHAR(255) NOT NULL,
    profile_data JSONB NOT NULL,
    accessibility_score DECIMAL(5,2),
    certification_level VARCHAR(50),
    certification_expires TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_device_profiles_cert ON device_profiles(certification_level);

-- Alerts table
CREATE TABLE alerts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES user_profiles(id),
    device_id UUID REFERENCES device_profiles(id),
    alert_type VARCHAR(50) NOT NULL,
    priority VARCHAR(20) NOT NULL,
    payload JSONB NOT NULL,
    delivery_status JSONB NOT NULL DEFAULT '{}',
    acknowledged_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_alerts_user ON alerts(user_id, created_at DESC);
CREATE INDEX idx_alerts_priority ON alerts(priority, created_at DESC);

-- Audit log table (HIPAA compliance)
CREATE TABLE audit_log (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID,
    action VARCHAR(100) NOT NULL,
    resource_type VARCHAR(100) NOT NULL,
    resource_id UUID,
    details JSONB,
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_audit_log_user ON audit_log(user_id, created_at DESC);
CREATE INDEX idx_audit_log_resource ON audit_log(resource_type, resource_id);

-- Enable row-level security
ALTER TABLE user_profiles ENABLE ROW LEVEL SECURITY;
ALTER TABLE device_profiles ENABLE ROW LEVEL SECURITY;
ALTER TABLE alerts ENABLE ROW LEVEL SECURITY;
```

---

## 5. Integration Guide

### 5.1 REST API Integration

```rust
use wia_medical::prelude::*;
use axum::{
    routing::{get, post},
    Router, Json,
    extract::{State, Path},
};

#[tokio::main]
async fn main() {
    // Initialize services
    let profile_service = ProfileService::new().await;
    let alert_service = AlertService::new().await;

    let app_state = AppState {
        profile_service,
        alert_service,
    };

    // Build router
    let app = Router::new()
        // Profile endpoints
        .route("/api/v1/profiles", post(create_profile))
        .route("/api/v1/profiles/:id", get(get_profile))
        .route("/api/v1/profiles/:id/accessibility", get(get_accessibility_score))

        // Device endpoints
        .route("/api/v1/devices", post(register_device))
        .route("/api/v1/devices/:id/compatibility/:user_id", get(check_compatibility))

        // Alert endpoints
        .route("/api/v1/alerts", post(create_alert))
        .route("/api/v1/alerts/:id/acknowledge", post(acknowledge_alert))

        // Health endpoints
        .route("/health", get(health_check))
        .route("/ready", get(readiness_check))

        .with_state(app_state);

    // Start server
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn create_profile(
    State(state): State<AppState>,
    Json(request): Json<CreateProfileRequest>,
) -> Result<Json<UserProfile>, ApiError> {
    let profile = state.profile_service
        .create_profile(request)
        .await?;

    Ok(Json(profile))
}

async fn get_accessibility_score(
    State(state): State<AppState>,
    Path(id): Path<String>,
) -> Result<Json<AccessibilityScore>, ApiError> {
    let profile = state.profile_service.get_profile(&id).await?;
    let score = AccessibilityScoreCalculator::calculate(&profile);

    Ok(Json(score))
}

async fn check_compatibility(
    State(state): State<AppState>,
    Path((device_id, user_id)): Path<(String, String)>,
) -> Result<Json<CompatibilityResult>, ApiError> {
    let device = state.profile_service.get_device(&device_id).await?;
    let user = state.profile_service.get_profile(&user_id).await?;

    let result = ProfileMatcher::check_compatibility(&device, &user);

    Ok(Json(result))
}
```

### 5.2 WebSocket Integration (Real-time Alerts)

```rust
use axum::{
    extract::{ws::{Message, WebSocket, WebSocketUpgrade}, State},
    response::Response,
};
use futures::{sink::SinkExt, stream::StreamExt};

async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> Response {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();

    // Authentication
    let auth_msg = receiver.next().await;
    let user_id = match authenticate_ws(auth_msg).await {
        Ok(id) => id,
        Err(_) => {
            let _ = sender.send(Message::Close(None)).await;
            return;
        }
    };

    // Subscribe to user's alert channel
    let mut alert_rx = state.alert_service.subscribe(&user_id).await;

    // Handle incoming messages and outgoing alerts
    loop {
        tokio::select! {
            // Receive alert from service
            Some(alert) = alert_rx.recv() => {
                let msg = serde_json::to_string(&alert).unwrap();
                if sender.send(Message::Text(msg)).await.is_err() {
                    break;
                }
            }

            // Receive message from client
            Some(Ok(msg)) = receiver.next() => {
                match msg {
                    Message::Text(text) => {
                        handle_client_message(&text, &user_id, &state).await;
                    }
                    Message::Close(_) => break,
                    _ => {}
                }
            }

            else => break,
        }
    }
}

async fn handle_client_message(text: &str, user_id: &str, state: &AppState) {
    if let Ok(msg) = serde_json::from_str::<ClientMessage>(text) {
        match msg {
            ClientMessage::Acknowledge { alert_id } => {
                state.alert_service.acknowledge(&alert_id, user_id).await;
            }
            ClientMessage::UpdatePreferences { preferences } => {
                state.profile_service.update_preferences(user_id, preferences).await;
            }
        }
    }
}
```

### 5.3 BLE Device Integration

```rust
use btleplug::api::{Central, Peripheral, WriteType};
use btleplug::platform::{Adapter, Manager};

const WIA_MEDICAL_SERVICE_UUID: Uuid = uuid!("00001850-0000-1000-8000-00805f9b34fb");
const ACCESSIBILITY_PROFILE_CHAR: Uuid = uuid!("00002a50-0000-1000-8000-00805f9b34fb");
const ALERT_CHAR: Uuid = uuid!("00002a51-0000-1000-8000-00805f9b34fb");

pub struct BLEDeviceManager {
    adapter: Adapter,
    connected_devices: HashMap<String, Peripheral>,
}

impl BLEDeviceManager {
    pub async fn new() -> Result<Self, BLEError> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let adapter = adapters.into_iter().next()
            .ok_or(BLEError::NoAdapter)?;

        Ok(Self {
            adapter,
            connected_devices: HashMap::new(),
        })
    }

    pub async fn scan_for_wia_devices(&self) -> Result<Vec<DeviceInfo>, BLEError> {
        self.adapter.start_scan(ScanFilter::default()).await?;
        tokio::time::sleep(Duration::from_secs(5)).await;

        let peripherals = self.adapter.peripherals().await?;
        let mut wia_devices = Vec::new();

        for peripheral in peripherals {
            if let Some(props) = peripheral.properties().await? {
                if props.services.contains(&WIA_MEDICAL_SERVICE_UUID) {
                    wia_devices.push(DeviceInfo {
                        id: peripheral.id().to_string(),
                        name: props.local_name.unwrap_or_default(),
                        rssi: props.rssi,
                    });
                }
            }
        }

        Ok(wia_devices)
    }

    pub async fn connect_device(&mut self, device_id: &str) -> Result<(), BLEError> {
        let peripheral = self.find_peripheral(device_id).await?;
        peripheral.connect().await?;
        peripheral.discover_services().await?;

        self.connected_devices.insert(device_id.to_string(), peripheral);
        Ok(())
    }

    pub async fn send_accessibility_profile(
        &self,
        device_id: &str,
        profile: &AccessibilityProfile,
    ) -> Result<(), BLEError> {
        let peripheral = self.connected_devices.get(device_id)
            .ok_or(BLEError::NotConnected)?;

        let char = self.find_characteristic(peripheral, ACCESSIBILITY_PROFILE_CHAR).await?;
        let data = profile.to_ble_format();

        peripheral.write(&char, &data, WriteType::WithResponse).await?;
        Ok(())
    }

    pub async fn subscribe_to_alerts(
        &self,
        device_id: &str,
        callback: impl Fn(Alert) + Send + 'static,
    ) -> Result<(), BLEError> {
        let peripheral = self.connected_devices.get(device_id)
            .ok_or(BLEError::NotConnected)?;

        let char = self.find_characteristic(peripheral, ALERT_CHAR).await?;
        peripheral.subscribe(&char).await?;

        let mut notifications = peripheral.notifications().await?;

        tokio::spawn(async move {
            while let Some(notification) = notifications.next().await {
                if notification.uuid == ALERT_CHAR {
                    if let Ok(alert) = Alert::from_ble_data(&notification.value) {
                        callback(alert);
                    }
                }
            }
        });

        Ok(())
    }
}
```

---

## 6. Monitoring & Observability

### 6.1 Metrics Configuration

```yaml
# prometheus/wia-medical-rules.yml
groups:
  - name: wia-medical-alerts
    rules:
      - alert: HighErrorRate
        expr: rate(wia_medical_errors_total[5m]) > 0.1
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: High error rate in WIA Medical API

      - alert: AlertDeliveryFailure
        expr: rate(wia_alert_delivery_failures_total[5m]) > 0.05
        for: 2m
        labels:
          severity: critical
        annotations:
          summary: Alert delivery failures detected

      - alert: ProfileSyncLag
        expr: wia_profile_sync_lag_seconds > 300
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: Profile sync lag exceeds 5 minutes

      - alert: AccessibilityScoreDropped
        expr: delta(wia_accessibility_score_avg[1h]) < -5
        for: 15m
        labels:
          severity: warning
        annotations:
          summary: Average accessibility score dropped significantly
```

### 6.2 Grafana Dashboard

```json
{
  "dashboard": {
    "title": "WIA Medical Accessibility",
    "panels": [
      {
        "title": "API Request Rate",
        "type": "graph",
        "targets": [
          {
            "expr": "rate(wia_medical_requests_total[5m])",
            "legendFormat": "{{method}} {{endpoint}}"
          }
        ]
      },
      {
        "title": "Alert Delivery Success Rate",
        "type": "gauge",
        "targets": [
          {
            "expr": "sum(rate(wia_alert_delivery_success_total[5m])) / sum(rate(wia_alert_delivery_total[5m])) * 100"
          }
        ],
        "options": {
          "thresholds": [
            {"value": 95, "color": "green"},
            {"value": 90, "color": "yellow"},
            {"value": 0, "color": "red"}
          ]
        }
      },
      {
        "title": "Accessibility Score Distribution",
        "type": "histogram",
        "targets": [
          {
            "expr": "histogram_quantile(0.5, wia_accessibility_score_bucket)"
          }
        ]
      },
      {
        "title": "WIA Device Connections",
        "type": "stat",
        "targets": [
          {
            "expr": "sum(wia_connected_devices) by (device_type)"
          }
        ]
      },
      {
        "title": "Profile Sync Status",
        "type": "table",
        "targets": [
          {
            "expr": "wia_profile_sync_status",
            "format": "table"
          }
        ]
      }
    ]
  }
}
```

### 6.3 Logging Configuration

```rust
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

fn setup_logging() {
    let fmt_layer = tracing_subscriber::fmt::layer()
        .json()
        .with_target(true)
        .with_thread_ids(true)
        .with_file(true)
        .with_line_number(true);

    let filter_layer = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "wia_medical=info,tower_http=debug".into());

    tracing_subscriber::registry()
        .with(filter_layer)
        .with(fmt_layer)
        .init();
}

// Structured logging example
#[instrument(skip(profile_service))]
async fn handle_alert(
    alert: Alert,
    profile_service: &ProfileService,
) -> Result<(), AlertError> {
    tracing::info!(
        alert_id = %alert.id,
        alert_type = %alert.alert_type,
        priority = %alert.priority,
        "Processing alert"
    );

    let user_profile = profile_service.get_profile(&alert.user_id).await?;

    tracing::debug!(
        user_id = %alert.user_id,
        preferred_modality = ?user_profile.output_preferences.primary_modality,
        "Retrieved user profile"
    );

    // ... delivery logic

    tracing::info!(
        alert_id = %alert.id,
        delivery_time_ms = %delivery_time.as_millis(),
        modalities_used = ?delivered_modalities,
        "Alert delivered successfully"
    );

    Ok(())
}
```

---

## 7. Security Hardening

### 7.1 API Security

```rust
use axum::{
    middleware,
    extract::Request,
    response::Response,
};
use tower::ServiceBuilder;
use tower_http::{
    cors::CorsLayer,
    trace::TraceLayer,
    compression::CompressionLayer,
};

fn build_secure_router(state: AppState) -> Router {
    Router::new()
        .route("/api/v1/*path", any(api_handler))
        .layer(
            ServiceBuilder::new()
                // Rate limiting
                .layer(RateLimitLayer::new(100, Duration::from_secs(60)))
                // CORS
                .layer(
                    CorsLayer::new()
                        .allow_origin(["https://medical.wia.live".parse().unwrap()])
                        .allow_methods([Method::GET, Method::POST, Method::PUT])
                        .allow_headers([AUTHORIZATION, CONTENT_TYPE])
                )
                // Request tracing
                .layer(TraceLayer::new_for_http())
                // Compression
                .layer(CompressionLayer::new())
                // Security headers
                .layer(middleware::from_fn(security_headers))
                // Authentication
                .layer(middleware::from_fn_with_state(state.clone(), authenticate))
                // Audit logging
                .layer(middleware::from_fn_with_state(state.clone(), audit_log))
        )
        .with_state(state)
}

async fn security_headers(request: Request, next: Next) -> Response {
    let mut response = next.run(request).await;
    let headers = response.headers_mut();

    headers.insert("X-Content-Type-Options", "nosniff".parse().unwrap());
    headers.insert("X-Frame-Options", "DENY".parse().unwrap());
    headers.insert("X-XSS-Protection", "1; mode=block".parse().unwrap());
    headers.insert(
        "Strict-Transport-Security",
        "max-age=31536000; includeSubDomains".parse().unwrap()
    );
    headers.insert(
        "Content-Security-Policy",
        "default-src 'self'".parse().unwrap()
    );

    response
}

async fn audit_log(
    State(state): State<AppState>,
    request: Request,
    next: Next,
) -> Response {
    let method = request.method().clone();
    let uri = request.uri().clone();
    let user_id = extract_user_id(&request);
    let ip_address = extract_ip(&request);

    let response = next.run(request).await;

    // Log to audit table
    state.audit_service.log(AuditEntry {
        user_id,
        action: format!("{} {}", method, uri.path()),
        resource_type: extract_resource_type(&uri),
        resource_id: extract_resource_id(&uri),
        ip_address,
        status_code: response.status().as_u16(),
        timestamp: chrono::Utc::now(),
    }).await;

    response
}
```

### 7.2 Data Encryption

```rust
use aes_gcm::{
    aead::{Aead, KeyInit},
    Aes256Gcm, Nonce,
};
use argon2::{Argon2, PasswordHasher};

pub struct EncryptionService {
    cipher: Aes256Gcm,
}

impl EncryptionService {
    pub fn new(key: &[u8; 32]) -> Self {
        let cipher = Aes256Gcm::new(key.into());
        Self { cipher }
    }

    pub fn encrypt_profile(&self, profile: &UserProfile) -> Result<EncryptedData, CryptoError> {
        let plaintext = serde_json::to_vec(profile)?;
        let nonce = generate_nonce();

        let ciphertext = self.cipher
            .encrypt(Nonce::from_slice(&nonce), plaintext.as_ref())
            .map_err(|_| CryptoError::EncryptionFailed)?;

        Ok(EncryptedData {
            nonce: nonce.to_vec(),
            ciphertext,
        })
    }

    pub fn decrypt_profile(&self, encrypted: &EncryptedData) -> Result<UserProfile, CryptoError> {
        let plaintext = self.cipher
            .decrypt(
                Nonce::from_slice(&encrypted.nonce),
                encrypted.ciphertext.as_ref()
            )
            .map_err(|_| CryptoError::DecryptionFailed)?;

        let profile = serde_json::from_slice(&plaintext)?;
        Ok(profile)
    }

    pub fn hash_sensitive_data(&self, data: &str) -> Result<String, CryptoError> {
        let salt = SaltString::generate(&mut OsRng);
        let argon2 = Argon2::default();

        let hash = argon2
            .hash_password(data.as_bytes(), &salt)
            .map_err(|_| CryptoError::HashFailed)?;

        Ok(hash.to_string())
    }
}
```

---

## 8. Disaster Recovery

### 8.1 Backup Strategy

```yaml
backup_strategy:
  database:
    type: continuous
    method: WAL archiving
    retention: 30 days
    point_in_time_recovery: true

    daily_snapshot:
      time: "02:00 UTC"
      retention: 90 days
      storage: S3/GCS cross-region

    weekly_snapshot:
      day: Sunday
      retention: 1 year
      storage: S3 Glacier/GCS Coldline

  redis:
    method: RDB + AOF
    frequency: every 5 minutes
    retention: 7 days

  configuration:
    method: git versioning
    storage: encrypted S3
    retention: indefinite

  audit_logs:
    retention: 7 years  # HIPAA requirement
    storage: immutable S3 bucket
    encryption: KMS
```

### 8.2 Recovery Procedures

```bash
#!/bin/bash
# disaster-recovery.sh

# 1. Database Recovery
recover_database() {
    echo "Starting database recovery..."

    # Stop application
    kubectl scale deployment wia-medical-api --replicas=0

    # Restore from backup
    pg_restore -h $DB_HOST -U $DB_USER -d wia_medical \
        --clean --if-exists \
        /backups/latest/wia_medical.dump

    # Verify integrity
    psql -h $DB_HOST -U $DB_USER -d wia_medical \
        -c "SELECT COUNT(*) FROM user_profiles;"

    # Restart application
    kubectl scale deployment wia-medical-api --replicas=3

    echo "Database recovery complete"
}

# 2. Redis Recovery
recover_redis() {
    echo "Starting Redis recovery..."

    # Stop Redis
    kubectl scale statefulset wia-redis --replicas=0

    # Restore RDB file
    aws s3 cp s3://wia-backups/redis/latest/dump.rdb /data/dump.rdb

    # Start Redis
    kubectl scale statefulset wia-redis --replicas=1

    echo "Redis recovery complete"
}

# 3. Full System Recovery
full_recovery() {
    echo "Starting full system recovery..."

    # Deploy infrastructure
    terraform apply -auto-approve

    # Deploy Kubernetes resources
    kubectl apply -f kubernetes/

    # Recover database
    recover_database

    # Recover Redis
    recover_redis

    # Verify services
    kubectl get pods -n wia-medical

    # Run smoke tests
    ./scripts/smoke-tests.sh

    echo "Full recovery complete"
}
```

---

## 9. Compliance Checklist

### 9.1 HIPAA Compliance

```markdown
## HIPAA Compliance Checklist

### Administrative Safeguards
- [x] Security Management Process
  - [x] Risk analysis documented
  - [x] Risk management plan
  - [x] Sanction policy for violations

- [x] Workforce Security
  - [x] Authorization procedures
  - [x] Workforce clearance
  - [x] Termination procedures

- [x] Information Access Management
  - [x] Access authorization
  - [x] Access establishment/modification
  - [x] Access termination

### Physical Safeguards
- [x] Facility Access Controls
  - [x] Contingency operations plan
  - [x] Facility security plan
  - [x] Access control procedures

- [x] Workstation Security
  - [x] Workstation use policy
  - [x] Workstation security measures

### Technical Safeguards
- [x] Access Control
  - [x] Unique user identification
  - [x] Emergency access procedure
  - [x] Automatic logoff
  - [x] Encryption/decryption

- [x] Audit Controls
  - [x] Audit logging enabled
  - [x] Log retention (6+ years)
  - [x] Log review procedures

- [x] Integrity Controls
  - [x] Data integrity verification
  - [x] Transmission integrity

- [x] Transmission Security
  - [x] TLS 1.3 for all transmissions
  - [x] End-to-end encryption
```

### 9.2 Accessibility Compliance

```markdown
## WIA Accessibility Compliance Checklist

### Core Accessibility
- [x] WCAG 2.1 AA conformance
- [x] Screen reader compatibility
- [x] Keyboard navigation
- [x] Color contrast (4.5:1 minimum)
- [x] Text resizing (up to 200%)

### Medical-Specific Accessibility
- [x] Multi-modal alert system
- [x] Emergency accessibility
- [x] Voice output for readings
- [x] Haptic feedback support
- [x] Sign language support

### WIA Integration
- [x] WIA profile support
- [x] Exoskeleton integration
- [x] Bionic eye integration
- [x] Voice-Sign integration
- [x] Smart wheelchair integration

### Documentation
- [x] Accessibility statement published
- [x] VPAT completed
- [x] User documentation accessible
- [x] Support channels accessible
```

---

## 10. Support & Maintenance

### 10.1 Support Channels

| Channel | Purpose | Response Time |
|---------|---------|---------------|
| Emergency Hotline | Critical accessibility failures | 15 minutes |
| Support Portal | General issues | 4 hours |
| Email | Non-urgent queries | 24 hours |
| Community Forum | Peer support | Community |
| Documentation | Self-service | Immediate |

### 10.2 Maintenance Windows

```yaml
maintenance_schedule:
  regular:
    day: Sunday
    time: "02:00-06:00 UTC"
    frequency: weekly
    notification: 72 hours advance

  emergency:
    notification: ASAP
    max_duration: 4 hours

  upgrade:
    notification: 2 weeks advance
    testing: staging environment first
    rollback_plan: required
```

---

## Document Information

- **Document ID**: WIA-MED-DEPLOY-001
- **Classification**: Public Standard
- **Maintainer**: WIA Medical Standards Committee
- **License**: Open Standard (CC BY 4.0)

弘益人間 (홍익인간) - Deploying Accessibility for All Humanity
